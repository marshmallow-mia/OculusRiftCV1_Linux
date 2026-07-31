#!/usr/bin/env python3
"""Find what references an address in Rift.dll, when rizin's `axt` cannot.

`axt` resolves nothing in this binary (noted at decomp/rift-dll-ipd-iad.md:12),
which is why only ~15 of its 5585 functions have ever been named. Every mapping
in decomp/ was made by hand-scanning for RIP-relative operands; this is that
scan, written down so it stops being retyped.

x86-64 RIP-relative addressing is encoded as modrm with mod=00 and rm=101, i.e.
(modrm & 0xC7) == 0x05, followed by a signed 32-bit displacement. The target is
computed from the address of the NEXT instruction, so the scan has to know each
candidate's total length. Two forms cover essentially all string and constant
references emitted here:

    48/4c 8d /r disp32     lea r64, [rip+disp32]        7 bytes
    f3/f2 0f 10 /r disp32  movss/movsd xmm, [rip+d32]   8 bytes (+1 if REX)

The second form is what matters for pulling float constants out of .rdata and
attributing them to the function that uses them - the EKF's Q, R and gains live
there as raw doubles with no symbol attached.

Sections in this image map with a single uniform delta (vaddr = paddr +
0x180000C00), which the loader below derives rather than assumes.

  tools/rip_xref.py Rift.dll 0x180453500           what references this address
  tools/rip_xref.py Rift.dll --const 0x180134cd0   float constants a function reads
"""
import argparse
import struct
import subprocess


def sections(path):
    out = subprocess.run(["rizin", "-q", "-e", "scr.color=0", "-c", "iS", "-c", "q!",
                          path], capture_output=True, text=True).stdout
    secs = []
    for line in out.splitlines():
        f = line.split()
        if len(f) >= 7 and f[0].startswith("0x") and f[2].startswith("0x"):
            secs.append(dict(paddr=int(f[0], 16), size=int(f[1], 16),
                             vaddr=int(f[2], 16), perm=f[5], name=f[6]))
    return secs


def scan(blob, base, lo, hi):
    """Yield (vaddr_of_insn, target, kind) for RIP-relative operands in blob."""
    n = len(blob)
    for i in range(n - 8):
        b = blob[i]
        # lea r64, [rip+disp32]
        if b in (0x48, 0x4c) and blob[i + 1] == 0x8d and (blob[i + 2] & 0xC7) == 0x05:
            disp = struct.unpack_from("<i", blob, i + 3)[0]
            tgt = base + i + 7 + disp
            if lo <= tgt <= hi:
                yield base + i, tgt, "lea"
        # movss/movsd xmm, [rip+disp32], with or without a REX prefix
        for pre, extra in ((0, 0), (1, 1)):
            j = i + extra
            if j + 8 > n:
                continue
            if extra and not (0x40 <= blob[i] <= 0x4f):
                continue
            if blob[j] in (0xf3, 0xf2) and blob[j + 1] == 0x0f and \
               blob[j + 2] == 0x10 and (blob[j + 3] & 0xC7) == 0x05:
                disp = struct.unpack_from("<i", blob, j + 4)[0]
                tgt = base + i + 8 + extra + disp
                if lo <= tgt <= hi:
                    yield base + i, tgt, "movs"
                break


def func_at(path, vaddr):
    out = subprocess.run(["rizin", "-q", "-p", path.replace(".dll", ".rzdb"),
                          "-e", "scr.color=0", "-c", "s 0x%x" % vaddr,
                          "-c", "afi~^name", "-c", "q!", path],
                         capture_output=True, text=True).stdout.strip()
    return out.split()[-1] if out else "?"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dll")
    ap.add_argument("target", nargs="?", help="address referenced, e.g. 0x180453500")
    ap.add_argument("--const", help="function address: report the constants it reads")
    ap.add_argument("--span", type=lambda x: int(x, 0), default=0x2000,
                    help="bytes of the function to scan with --const")
    a = ap.parse_args()

    secs = sections(a.dll)
    text = next(s for s in secs if s["name"] == ".text")
    blob = open(a.dll, "rb").read()
    tb = blob[text["paddr"]:text["paddr"] + text["size"]]

    if a.const:
        fn = int(a.const, 0)
        off = fn - text["vaddr"]
        sub = tb[off:off + a.span]
        seen = {}
        for _, tgt, kind in scan(sub, fn, 0, 1 << 63):
            if kind != "movs":
                continue
            sec = next((s for s in secs if s["vaddr"] <= tgt < s["vaddr"] + s["size"]), None)
            if sec is None or "r" not in sec["perm"]:
                continue
            raw = blob[sec["paddr"] + tgt - sec["vaddr"]:][:8]
            if len(raw) < 8:
                continue
            f32 = struct.unpack("<f", raw[:4])[0]
            f64 = struct.unpack("<d", raw)[0]
            seen.setdefault(tgt, (f32, f64))
        print("float constants referenced by 0x%x (first %d bytes):" % (fn, a.span))
        for tgt, (f32, f64) in sorted(seen.items()):
            print("  0x%x  f32 %-18.9g  f64 %.9g" % (tgt, f32, f64))
        return

    tgt = int(a.target, 0)
    print("references to 0x%x:" % tgt)
    hits = [v for v, t, k in scan(tb, text["vaddr"], tgt, tgt) if t == tgt]
    for v in hits:
        print("  0x%x   in %s" % (v, func_at(a.dll, v)))
    if not hits:
        print("  none found")


if __name__ == "__main__":
    main()
