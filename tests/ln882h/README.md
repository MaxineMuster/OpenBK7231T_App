# LN882H best-BSSID regression tests

Run from the repository root on a Linux host with Python 3 and GCC or Clang:

```sh
python3 tests/ln882h/run_tests.py --cc gcc --sanitize
python3 tests/ln882h/run_tests.py --cc gcc --sanitize --unsigned-char
python3 tests/ln882h/run_tests.py --cc clang --sanitize
python3 tests/ln882h/run_tests.py --cc clang --sanitize --unsigned-char
```

The runner copies the **actual** `hal_wifi_ln882h.c` and its private helper into a
scratch directory. It substitutes SDK include shims and links the hardware/OS
fakes in this directory; it does not rewrite the production function bodies.
Each named test runs in a separate process, so static HAL state begins as it
would at boot. `--case NAME` runs a single case. Assertions, compiler warnings
and AddressSanitizer/UndefinedBehaviorSanitizer failures fail the run.

The 40 cases cover strongest matching SSID selection, high-bit MAC bytes,
maximum-length and unterminated SSIDs, invalid BSSID/channel rejection, fresh
results on each scan, vanished APs, absent matches, allocation/start/list/wait/
signal errors, immediate and duplicate completion, callbacks arriving after a
timeout, stable BSSID storage, failed selected-AP fallback, repeated connections,
DHCP/static IP, open networks, explicit and stored fast-connect arguments.
They include 2,000 generated RSSI lists and 2,000 threaded completion/timeout
interleavings. Another threaded test pauses a callback inside semaphore release
while the main thread abandons the wait.

## Intended behavior

* Only ordinary LN882H association (no explicit BSSID or channel) performs the
  optional two-pass scan. Fast-connect arguments are not overwritten.
* The SDK AP list is a cache: each pass clears it first, and only the final
  completed pass supplies the candidate. The normal diagnostic callback and
  the selector serialize their access to the borrowed list.
* A permanent callback signals a persistent semaphore. No callback ever writes
  the chosen address. No semaphore/mutex is destroyed while Wi-Fi is running.
* A scan error or timeout disables this optional optimization until reboot.
  The SDK callback has no request ID or documented cancellation barrier, so
  rearming it after an ambiguous completion would be unsafe. Normal association
  and retries remain enabled; a busy SDK may reject the first fallback attempt.
* If a selected AP fails to connect, the next ordinary attempt is unpinned.
  Successful connection clears that fallback. A subsequent pre-scan also waits
  for the prior association's terminal SDK callback before being armed.
* LN8825 and other HAL paths, persisted configuration and shared Wi-Fi APIs are
  unchanged. The integration retains upstream's current fast-connect code.

## Scope and hardware checks

These are deterministic host tests of application logic using an SDK facade,
not an emulator of the Wi-Fi firmware. They do not prove radio association,
SDK task ordering or power-save behavior on a board. Also run the existing
firmware build matrix and simulator suite. On LN882H hardware, verify two APs
sharing an SSID, AP removal/channel changes between scans and association,
repeated reconnects, power save, fast-connect fallback and recovery AP mode.
Check actual BSSID/channel and heap stability over repeated attempts.

The policy assumes the SDK delivers one completion per accepted scan and that
association terminal callbacks follow that association's scan callbacks. Never
start another dedicated scan from an unrelated task while this HAL owns one.
