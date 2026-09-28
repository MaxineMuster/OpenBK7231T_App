#!/usr/bin/env python3
"""Compile the actual LN882H Wi-Fi HAL with host SDK fakes; no network required."""
from pathlib import Path
import argparse
import os
import re
import shutil
import subprocess
import tempfile


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default=os.environ.get("CC", "cc"))
    parser.add_argument("--sanitize", action="store_true")
    parser.add_argument("--unsigned-char", action="store_true")
    parser.add_argument("--case", help="Run just one named case")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    testdir = root / "tests/ln882h"
    with tempfile.TemporaryDirectory(prefix="obk-ln882h-test-") as temporary:
        work = Path(temporary)
        haldir = work / "src/hal/ln882h"
        haldir.mkdir(parents=True)
        for name in ("hal_wifi_ln882h.c", "ln882h_best_bssid.h"):
            source = root / "src/hal/ln882h" / name
            if source.is_file():
                shutil.copy2(source, haldir / source.name)
        # Preserve the production source. Supply empty include shims; the harness
        # provides the SDK declarations through mock_sdk.h before including it.
        for source in haldir.glob("*"):
            for include in re.findall(r'^\s*#include\s+["<]([^">]+)[">]', source.read_text(), re.M):
                if include in {"stdint.h", "stdbool.h", "string.h", "stddef.h"}:
                    continue
                path = (haldir / include).resolve()
                if path.exists():
                    continue
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("/* host SDK shim */\n")
        exe = work / "test_best_bssid"
        flags = ["-std=c99", "-O2", "-g", "-Wall", "-Wextra", "-Werror", "-pthread",
                 "-funsigned-char" if args.unsigned_char else "-fsigned-char"]
        if args.sanitize:
            flags += ["-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-no-pie"]
        subprocess.run([args.cc, *flags, "-I", str(testdir), "-I", str(haldir),
                        str(testdir / "test_best_bssid.c"), "-o", str(exe)], check=True)
        cases = [args.case] if args.case else subprocess.check_output([str(exe)], text=True).splitlines()
        failures = []
        for case in cases:
            result = subprocess.run([str(exe), case], check=False, timeout=30)
            if result.returncode:
                failures.append(case)
        print(f"{len(cases)-len(failures)}/{len(cases)} passed; compiler={args.cc}; "
              f"sanitize={args.sanitize}; unsigned_char={args.unsigned_char}")
        if failures:
            print("FAILED: " + ", ".join(failures))
        return bool(failures)


if __name__ == "__main__":
    raise SystemExit(main())
