#!/usr/bin/env python3
"""
THE FAULT LAB - a board that is broken on purpose, and a console to find out why.

    python lab.py            play from the first level
    python lab.py 3          start at level 3
    python lab.py --list     what the levels are

Every level hands you the firmware's own console, talking to `fwsim.py` -
the same integer arithmetic the board runs, with one thing wrong. You have
the commands a person at the bench has: `s`, `v`, `x`, `get`, `set`, `cap`,
`dump`, `hall`, and the rest. `?` lists the lab's own commands.

Two rules, and they are the point of the thing:

  1. You win by proving the cause, not by naming it. Each level has one
     observation that shows what is wrong; diagnose without having made it
     and the level says so and does not count it.
  2. Every fault here really happened, to this project, on this bench. The
     debrief at the end of each level says when, and what it cost.

Nothing here touches a board. It cannot: the lab is the emulator.
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import fwsim      # noqa: E402

MS = 20                       # control periods in a millisecond


# --------------------------------------------------------------- the bench --

class Bench:
    """One emulated board, with the commands typed at it remembered."""

    def __init__(self):
        self.fw = fwsim.FwSim(realtime=False)
        self.typed = []
        self.captures = []        # (channel, [samples]) of every `dump`
        self.cap_ch = None
        self.cap_div = 1

    def say(self, cmd, settle_ms=300):
        """Type a line, let the motor live for a while, return what came back."""
        self.typed.append(cmd)
        if cmd.startswith("cap "):
            bits = cmd.split()
            self.cap_ch = bits[1] if len(bits) > 1 else None
            self.cap_div = int(bits[2]) if len(bits) > 2 and bits[2].isdigit() else 1
        self.fw.write_line(cmd)
        out = self._drain()
        self.fw.run(settle_ms * MS)
        out += self._drain()
        if cmd.startswith("dump") and self.cap_ch:
            data = []
            for line in out:
                if line.startswith("d "):
                    data += [int(v) for v in line.split()[1:]]
            if data:
                self.captures.append((self.cap_ch, data))
        return out

    def wait(self, ms):
        self.fw.run(ms * MS)
        return self._drain()

    def _drain(self):
        raw = self.fw.read().decode("ascii", "replace")
        return [l.strip() for l in raw.splitlines() if l.strip()]

    # ---- what a level looks at to decide whether you proved anything

    def ran(self, *words):
        """True if a command starting with any of these was typed."""
        return any(t.split()[0] in words for t in self.typed if t.split())

    def captured(self, channel):
        """The samples of the last `dump` of this channel, or None."""
        for ch, data in reversed(self.captures):
            if ch == channel:
                return data
        return None

    def field(self, cmd, name):
        """One value out of a reply of `word name value name value ...`."""
        word = cmd.split()[0]
        for line in self.say(cmd, settle_ms=50):
            parts = line.split()
            # the state line has no word in front of it: it opens with `ang`
            if word == "s" and parts and parts[0] == "ang":
                parts = ["s"] + parts
            if parts and parts[0] == word:
                pairs = dict(zip(parts[1::2], parts[2::2]))
                if name in pairs:
                    try:
                        return int(pairs[name])
                    except ValueError:
                        return pairs[name]
        return None


# ------------------------------------------------------------- the levels --

class Level:
    name = ""
    brief = ""
    cause = ""                 # the token that wins it
    wrong = {}                 # token -> what the bench says to that guess
    hint = ""
    debrief = ""

    def setup(self, bench):
        raise NotImplementedError

    def proved(self, bench):
        """Did the player make the observation that shows the cause?"""
        return False


class L1HallTable(Level):
    name = "มอเตอร์ครางแต่ไม่หมุน"
    brief = ("บอร์ดนี้ต่อ Hall sensor และคาลิเบรตมาแล้ว สั่ง `spd 2000` แล้ว\n"
             "   มอเตอร์สั่นอยู่กับที่ กินกระแสเต็มแต่ไม่ออกแรงบิด")
    cause = "hall-table"
    wrong = {
        "gains": "ลองดูก่อนว่ากระแสที่สั่งไปถึงไหม ถ้า iq เต็มแต่ไม่หมุน ปัญหาไม่ได้อยู่ที่ gain",
        "current-limit": "ถ้าติดลิมิตกระแส `x` จะบอกว่าแรงดันยังเหลือ ลองดูอีกที",
        "throttle-wire": "คันเร่งไม่เกี่ยว ด่านนี้สั่งด้วย `spd` ตรง ๆ",
    }
    hint = ("แรงบิดเกิดจากกระแสที่ตั้งฉากกับสนามแม่เหล็กของโรเตอร์\n"
            "   ถ้ากระแสมีครบแต่แรงบิดไม่มี แปลว่ามุมที่ลูปใช้ไม่ตรงกับโรเตอร์จริง\n"
            "   ช่อง `aerr` ของตัวบันทึกคือส่วนต่างนั้น")

    def setup(self, bench):
        bench.say("set mot 1", 50)
        bench.say("set sens 0", 50)
        bench.say("hallcal", 50)
        bench.wait(2500)                       # the calibration runs
        fw = bench.fw
        # One sector out: the table is right, but every entry belongs to the
        # state next door. This is what a swapped pair of Hall wires does.
        order = [c for c in range(8) if fw.h_wid[c]]
        rot = {c: order[(order.index(c) + 1) % len(order)] for c in order}
        fw.h_edge = [fw.h_edge[rot[c]] if c in rot else fw.h_edge[c] for c in range(8)]
        fw.h_wid = [fw.h_wid[rot[c]] if c in rot else fw.h_wid[c] for c in range(8)]
        bench.say("set sens 1", 50)
        bench.say("spd 2000", 50)
        bench.wait(800)
        bench.typed.clear()                    # the player starts clean
        bench.captures.clear()

    def proved(self, bench):
        data = bench.captured("aerr")
        return bool(data) and max(abs(v) for v in data) > 4000

    debrief = (
        "ตาราง Hall เลื่อนไปหนึ่ง sector มุมที่ลูปใช้จึงผิดไป ~60 องศาไฟฟ้า\n"
        "   กระแสที่ควรเป็นแรงบิดกลายเป็นกระแสที่ดึงโรเตอร์ไปด้านข้างแทน\n\n"
        "   ของจริง: สลับสาย Hall สองเส้น หรือคาลิเบรตตอนโรเตอร์ถูกล็อกไว้\n"
        "   โปรเจกต์นี้กันไว้ด้วยการวัดขอบเองทุกครั้ง (`hallcal`) แทนที่จะสมมติ\n"
        "   ว่าหกช่องกว้างเท่ากัน - 3144 เปิดแค่ 148 องศา ช่องจึงไม่เท่ากันเลย")


class L2Ringing(Level):
    name = "ความเร็วแกว่งไม่ยอมนิ่ง"
    brief = ("สั่ง `spd 1000` แล้วความเร็วแกว่งขึ้นลงไม่ยอมนิ่ง\n"
             "   คำถามคือ มันแกว่งจริง หรือแค่ตัววัดบอกว่าแกว่ง")
    cause = "gains"
    wrong = {
        "hall-table": "ด่านนี้ใช้ encoder อยู่ มุมมาจากตัวนับ ไม่ได้มาจาก Hall",
        "sensor": "ลองเทียบดูว่าเปลี่ยนไปใช้ตัววัดอื่นแล้วอาการหายไหม",
        "current-limit": "ถ้าติดลิมิต ความเร็วจะนิ่งที่ค่าต่ำ ไม่ใช่แกว่งขึ้นลง",
    }
    hint = ("ตัววัดที่สะอาดที่สุดในบอร์ดนี้คือ encoder (`set sens 0`)\n"
            "   ถ้าอาการยังอยู่ทั้งที่มุมสะอาด แปลว่าปัญหาอยู่ที่การควบคุม\n"
            "   แล้วลองดูว่า gain ตัวไหนใหญ่ผิดปกติ: `get`")

    def setup(self, bench):
        bench.say("set mot 1", 50)
        bench.say("set sens 0", 50)
        # Not simply "too stiff": a big integrator behind a small proportional
        # term is what makes a speed loop hunt, and it is the combination a
        # person reaches for when the drive feels sluggish.
        bench.say("set simax 29491", 50)
        bench.say("set skp 300000", 50)
        bench.say("set ski 131072", 50)
        bench.say("spd 1000", 50)
        bench.wait(1200)
        bench.typed.clear()
        bench.captures.clear()

    def proved(self, bench):
        data = bench.captured("spd")
        if not data:
            return False
        mean = sum(data) / len(data)
        return (max(data) - min(data)) > max(120, abs(mean) * 0.1)

    debrief = (
        "`ski` ถูกตั้งไว้สุดสเกลโดยที่ `skp` ไม่ได้ใหญ่ตาม ตัวสะสมความผิดพลาด\n"
        "   จึงตามไม่ทันและเลยเป้าทุกครั้ง กลายเป็นวงจรแกว่งที่มีคาบของตัวเอง\n"
        "   ราว 60 ms ซึ่งเห็นได้ชัดในภาพที่บันทึกมา\n"
        "   สังเกตว่าอาการอยู่แม้ใช้ encoder ที่ให้มุมใหม่ทุก 50 us - ตัววัดไม่ผิด\n\n"
        "   ของจริงเมื่อ 2026-11-11: อาการ 'ลูปแกว่งบน Hall' กลับไม่ใช่เรื่องนี้เลย\n"
        "   บันทึกคลื่นออกมาแล้วเป็นขั้นบันไดที่เปลี่ยนตามขอบเซนเซอร์ ไม่มีคาบของ\n"
        "   ตัวเอง = ตัววัดเพี้ยน ไม่ใช่ลูปแกว่ง การแยกสองอย่างนี้ให้ออกคือทั้งหมด\n"
        "   ของด่านนี้ ถ้าไปจูน gain ตั้งแต่แรกจะเสียเวลาทั้งวันกับสิ่งที่ไม่ใช่เหตุ")


class L3ThrottleWire(Level):
    name = "คันเร่งสั่งไม่ขึ้น"
    brief = ("โหมดคันเร่งเปิดไม่ติด สั่ง `drv 1` แล้วบอร์ดปฏิเสธทุกครั้ง\n"
             "   มอเตอร์เองยังสั่งด้วย `spd` ได้ตามปกติ")
    cause = "throttle-wire"
    wrong = {
        "gains": "gain ไม่เกี่ยว ด่านนี้บอร์ดปฏิเสธคำสั่งตั้งแต่ยังไม่ขับ",
        "hall-table": "ตัววัดมุมไม่เกี่ยว ลองอ่านสิ่งที่บอร์ดตอบกลับมาตอน `drv 1`",
        "current-limit": "ยังไม่ทันขับเลย ลิมิตกระแสจึงไม่เกี่ยว",
    }
    hint = ("บรรทัด `v` บอกสภาพฝั่งยานพาหนะทั้งหมดในบรรทัดเดียว\n"
            "   ดูค่า `th` (ค่าที่อ่านได้จากคันเร่ง) เทียบกับ `thmin`/`thmax` ที่ตั้งไว้\n"
            "   และดูธง `vflt`")

    def setup(self, bench):
        bench.say("set insrc 0", 50)
        bench.say("th 40", 50)                 # far below thmin: a broken wire
        bench.say("drv 1", 50)
        bench.wait(300)
        bench.typed.clear()
        bench.captures.clear()

    def proved(self, bench):
        return bench.field("v", "vflt") == 1 and bench.ran("v")

    debrief = (
        "ค่าที่อ่านจากคันเร่งต่ำกว่าครึ่งหนึ่งของ `thmin` คาไว้นานเกิน 50 ms\n"
        "   เฟิร์มแวร์ถือว่านั่นคือสายขาด ไม่ใช่ตำแหน่งคันเร่ง จึงล็อก `vflt`\n"
        "   และไม่ยอมออกตัวจนกว่าจะ `clear`\n\n"
        "   ของจริง: คันเร่ง hall ในรถไฟฟ้าขาดกลางทางคือเรื่องปกติ ถ้าเฟิร์มแวร์\n"
        "   อ่าน 0 โวลต์แล้วแปลว่า 'ปล่อยคันเร่ง' ก็ยังดี แต่ถ้าสายสัญญาณขาด\n"
        "   แล้วลอยไปแตะไฟเลี้ยง มันจะแปลว่า 'เร่งสุด' - จึงต้องเป็นความผิดพลาด\n"
        "   ที่ล็อกไว้ ไม่ใช่ค่าที่เอาไปใช้ต่อ")


class L4CurrentLimit(Level):
    name = "เร็วไม่ขึ้นตามสั่ง"
    brief = ("สั่ง `spd 5000` แต่มันค้างอยู่แถว ๆ ครึ่งเดียว ไม่มีอาการแกว่ง\n"
             "   ไม่มี fault ไม่มีอะไรร้อง มันแค่ไปไม่ถึง")
    cause = "current-limit"
    wrong = {
        "gains": "ถ้า gain น้อยไปมันจะช้าแต่ยังไปถึง ด่านนี้มันไปไม่ถึงเลย",
        "hall-table": "ด่านนี้ใช้ encoder มุมสะอาด",
        "throttle-wire": "ด่านนี้ไม่ได้ใช้คันเร่ง",
    }
    hint = ("`x` บอกแรงดันที่จ่ายจริง และ `s` บอกกระแสที่ไหลอยู่\n"
            "   ถ้ากระแสชนเพดานเท่า ๆ กันตลอดเวลา ให้ไปดูว่าเพดานนั้นตั้งไว้เท่าไร\n"
            "   `get` แล้วมองหาค่าที่จำกัดกระแสของลูปความเร็ว")

    def setup(self, bench):
        bench.say("set mot 1", 50)
        bench.say("set sens 0", 50)
        bench.say("set simax 2200", 50)        # the speed loop may ask for little
        bench.say("spd 5000", 50)
        bench.wait(2000)
        bench.typed.clear()
        bench.captures.clear()

    def proved(self, bench):
        iq = bench.field("s", "iq")
        return iq is not None and abs(iq) >= 2000 and bench.ran("get", "s")

    debrief = (
        "`simax` คือกระแสมากที่สุดที่ลูปความเร็วขอได้ ถูกตั้งไว้ต่ำ\n"
        "   ลูปจึงขอเต็มเพดานตลอดเวลาแล้วยังเร่งไม่พอจะไปถึงเป้า\n\n"
        "   ของจริง: ลิมิตแบบนี้คือสิ่งที่กันไม่ให้ไดรฟ์ดึงกระแสจนไหม้ตอนออกตัว\n"
        "   อาการ 'เร็วไม่ขึ้นแต่ไม่มีอะไรร้อง' จึงมักไม่ใช่ความผิดพลาด แต่เป็น\n"
        "   ลิมิตที่ทำงานถูกต้อง สิ่งที่ผิดคือมันถูกตั้งไว้ไม่ตรงกับมอเตอร์")


LEVELS = [L1HallTable(), L2Ringing(), L3ThrottleWire(), L4CurrentLimit()]

CAUSES = {
    "hall-table": "ตาราง Hall / มุมที่ลูปใช้ผิด",
    "gains": "gain ของลูปตั้งไว้ไม่เหมาะ",
    "throttle-wire": "สัญญาณคันเร่งออกนอกช่วงที่รับได้",
    "current-limit": "ชนลิมิตกระแสที่ตั้งไว้",
    "sensor": "ตัววัดมุมคนละตัวกับที่ต่ออยู่",
}


def render(channel, data, div):
    """256 numbers are evidence only if they can be read. Draw them."""
    lo, hi = min(data), max(data)
    mean = sum(data) / len(data)
    span = max(hi - lo, 1)
    bars = " .:-=+*#%@"
    shape = "".join(bars[min(9, (v - lo) * 9 // span)] for v in data)
    ms = div * 0.05
    out = [f"   {channel}: {len(data)} ตัวอย่าง ทุก {ms:.2f} ms "
           f"(รวม {len(data) * ms:.0f} ms)",
           f"   ต่ำสุด {lo}   สูงสุด {hi}   เฉลี่ย {mean:.0f}   "
           f"กว้าง {hi - lo}"]
    for i in range(0, len(shape), 64):
        out.append("   " + shape[i:i + 64])
    return out


# ---------------------------------------------------------------- the game --

HELP = """
คำสั่งของแล็บ (ขึ้นต้นด้วย ?)
  ?            คำสั่งทั้งหมด
  ?brief       อ่านโจทย์ด่านนี้อีกครั้ง
  ?causes      รายชื่อสาเหตุที่ตอบได้
  ?diagnose X  ตอบว่าสาเหตุคือ X
  ?hint        ขอคำใบ้ (เสียคะแนน)
  ?wait N      ปล่อยให้เวลาเดินไป N มิลลิวินาที
  ?skip        ข้ามด่านนี้
  ?quit        ออก

ที่เหลือพิมพ์ใส่คอนโซลของเฟิร์มแวร์ได้ตรง ๆ เช่น
  s  ·  v  ·  x  ·  t  ·  get  ·  set <ชื่อ> <ค่า>  ·  spd <n>  ·  off
  cap <ช่อง> <ตัวหาร>  แล้ว  dump      (ช่อง: id iq vd vq spd ang ol hall aerr idfw)
"""


def play(level, number, total):
    bench = Bench()
    print(f"\n{'=' * 68}")
    print(f"  ด่าน {number}/{total}: {level.name}")
    print(f"{'=' * 68}")
    print(f"   {level.brief}\n")
    print("   พิมพ์ ? เพื่อดูคำสั่งของแล็บ, ?diagnose <สาเหตุ> เมื่อพิสูจน์ได้แล้ว\n")
    level.setup(bench)

    used_hint = False
    started = time.perf_counter()
    while True:
        try:
            line = input("fault-lab> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return None
        if not line:
            continue

        if line.startswith("?"):
            word, _, rest = line[1:].partition(" ")
            word = word.lower()
            if word in ("", "help", "h"):
                print(HELP)
            elif word == "brief":
                print(f"   {level.brief}")
            elif word == "causes":
                for key, text in CAUSES.items():
                    print(f"   {key:<16} {text}")
            elif word == "hint":
                used_hint = True
                print(f"   คำใบ้: {level.hint}")
            elif word == "wait":
                ms = int(rest) if rest.strip().isdigit() else 500
                for out in bench.wait(ms):
                    print("   " + out)
                print(f"   (ปล่อยเวลาไป {ms} ms)")
            elif word == "skip":
                print(f"\n   ข้ามด่าน เฉลย: {level.cause}\n   {level.debrief}\n")
                return False
            elif word in ("quit", "q", "exit"):
                return None
            elif word in ("diagnose", "d"):
                guess = rest.strip().lower()
                if guess not in CAUSES:
                    print(f"   ไม่รู้จัก '{guess}' - ?causes เพื่อดูรายชื่อ")
                    continue
                if guess != level.cause:
                    print("   ไม่ใช่ " + level.wrong.get(guess, "ลองดูอีกที"))
                    continue
                if not level.proved(bench):
                    print("   ถูกสาเหตุ แต่ยังไม่ได้พิสูจน์ - ด่านนี้ยังไม่ผ่าน")
                    print("   หาการวัดที่แสดงให้เห็นก่อน แล้วตอบอีกครั้ง (?hint ได้)")
                    continue
                secs = time.perf_counter() - started
                print(f"\n   ผ่าน  ใช้ {len(bench.typed)} คำสั่ง "
                      f"{'(ใช้คำใบ้) ' if used_hint else ''}{secs:.0f} วินาที")
                print(f"\n   {level.debrief}\n")
                return True
            else:
                print("   ไม่รู้จักคำสั่งของแล็บนี้ - ? เพื่อดูทั้งหมด")
            continue

        before = len(bench.captures)
        for out in bench.say(line):
            if out.startswith("d ") or out == "cap end":
                continue                      # drawn below, not listed raw
            print("   " + out)
        if len(bench.captures) > before:
            ch, data = bench.captures[-1]
            for row in render(ch, data, bench.cap_div):
                print(row)


def main(argv):
    if "--list" in argv:
        for i, lv in enumerate(LEVELS, 1):
            print(f"  {i}. {lv.name}")
        return 0

    start = int(argv[0]) - 1 if argv and argv[0].isdigit() else 0
    print("\n   THE FAULT LAB - บอร์ดที่พังไว้ให้ตั้งใจ")
    print("   ทุกความผิดพลาดในนี้เคยเกิดขึ้นจริงกับโปรเจกต์นี้")
    print("   ชนะด้วยการพิสูจน์ ไม่ใช่ด้วยการเดา")

    done = 0
    for i in range(start, len(LEVELS)):
        result = play(LEVELS[i], i + 1, len(LEVELS))
        if result is None:
            print("\n   ไว้เจอกันใหม่\n")
            return 0
        done += bool(result)
    print(f"\n   จบแล้ว ผ่าน {done} จาก {len(LEVELS)} ด่าน\n")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
