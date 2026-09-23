# open1V

`open1V` is a standalone Windows, macOS, and Linux command-line application for the independently
implemented UNO R4 WiFi/Minima camera bridge. It has no build-time dependency on the
Arduino source tree.

The repository now builds three targets:

- `open1v-core` — reusable connection/session library for CLI or GUI clients
- `open1V` — human-readable command-line client linked to `open1v-core`
- `open1V-debug` — protocol diagnostics, raw bridge commands, offline tests,
  and explicitly authorized diagnostic writes

See [LIBRARY.md](LIBRARY.md) for the public API and integration example.

## Boundaries

- `CameraRequest` is the business-neutral request layer. It validates EOS
  packets, handles bounded retries and asynchronous F4 traffic, and selects a
  bridge receive profile without knowing which setting or record is requested.
- `CameraProtocolSession` composes those requests into camera workflows. CLI calls
  `readOnce()` for handshake, one selected task, F2 shutdown, and error cleanup.
  A future GUI can instead call `beginSession()`, any number of `perform()`
  operations, and one `endSession()`. All paths return raw `CameraPacket`
  values without interpreting them.
- A long-lived session matches the captured Canon application behavior: the
  physical PC-mode connection remains active across logical actions. Each later
  action uses the in-session `FF/F4/F1` boundary; `F2` is reserved for the final
  `endSession()`. `sessionActive()` exposes that connection state.
- Readable commands and `*-debug` commands use that same protocol layer. The
  readable CLI decodes confirmed fields and omits fields the original
  application does not display; diagnostics print the raw packets. Diagnostic
  commands are exposed only by `open1V-debug`.
- `BridgeClient` owns the documented `O1` framing protocol.
- `ITransport` isolates platform transport from both layers.
- `SerialTransport` is the default adapter and auto-detects the
  official UNO R4 WiFi/Minima identity or the experimental Minima ES-E1-ID
  CDC identity (`04A9:3040`). Windows uses the native COM API; macOS and Linux
  share a POSIX termios/poll implementation, with IOKit or sysfs used only for
  USB identity discovery. The latter identity still carries O1 frames and is
  not the original KLSI/MCCI transport.
- `WinUsbTransport` remains available behind `--winusb` for future native USB
  firmware work. It also recognizes the Zadig interface GUID used by the
  experimental Minima `04A9:3040` identity build; this path must be selected
  explicitly and still transports O1 frames.
- The Arduino bridge owns camera-side electrical behavior: bit timing, the D5
  high-level assist circuit, stale-RX draining, line release, and concrete
  receive windows. New clients select a neutral receive profile; legacy bridge
  exchange remains supported for older firmware.

The first camera operation is deliberately read-only: `camera identify`.

## Build

Open `open1V.slnx` in Visual Studio, or run:

```powershell
MSBuild.exe open1V.slnx /p:Configuration=Release /p:Platform=x64
```

From a normal Command Prompt with Visual Studio installed, `build.cmd` locates the newest MSBuild automatically and performs the same Release build.

On macOS or Linux, use CMake with a C++20 compiler:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

On macOS, CMake links IOKit and CoreFoundation for USB device discovery. Linux
uses sysfs and needs no additional library. An explicit device may be selected
with `--port /dev/cu.usbmodem...` on macOS or `--port /dev/ttyACM0` on Linux.
The `--winusb` option remains Windows-only.

## User commands

`open1V` is the normal interactive and readable command-line application.
Running it without arguments opens `camera console`.

```text
open1V camera id
open1V camera cfn
open1V camera pfn
open1V camera clock
open1V camera info
open1V camera console
open1V camera set-id --id 12
open1V camera set-clock --clock 260922120000
```

`id`, `cfn`, `pfn`, and `clock` decode fields shown by the original application
into readable values and close PC mode after reading. `camera info` reads all
four groups in one physical PC-mode session and exits once.

Running `open1V` without arguments enters `camera console` automatically.
Startup performs only the handshake and displays `Camera: EOS-1V` plus the
camera ID, so the prompt appears quickly. `show` explicitly reads all current
settings. The console accepts `cfn`, `pfn`, `set id`, and `set clock` (`set
time` remains a compatibility alias). PC mode remains active until `exit` or
end-of-input, when the only F2 is sent.

`cfn` first asks for `current`, `1`, `2`, or `3`. Inside `cfn-edit`, any number
of options can be staged. `commit` writes and verifies all differences, while
`discard` or `q` returns without writing. `pfn` provides the same staged edit,
preview, commit, and discard flow for P.Fn settings.

`set clock` can synchronize with the local system clock or accept a complete
manual `YYMMDDhhmmss` value. EOS-1V supports calendar years 2000 through 2099,
so `YY` always means `20YY`; system-clock synchronization is rejected outside
that range. Dates are checked against the actual number of days in each month,
including leap years. `set time` remains a compatibility alias.

P.Fn-27 includes both its ON/OFF bit and a dial-selection value from the fifth
`DD/DE` payload byte: `0` Main Dial only, `1` Quick Control Dial only, and `2`
Both dials. The console displays and verifies both parts separately.
If the handshake cannot be established, the console reports that the camera is
not connected or is not in PC mode and does not enter the command prompt.

## Debug and diagnostics

`open1V-debug` is a separate diagnostic executable. It exposes raw bridge and
camera packets, offline tests, protocol experiments, and guarded diagnostic
writes. These commands are intended for development and hardware investigation.

Read-only and session-control commands:

```text
open1V-debug self-test
open1V-debug frame ping
open1V-debug bridge ping
open1V-debug bridge status
open1V-debug bridge link-status
open1V-debug camera identify
open1V-debug camera read-settings
open1V-debug camera handshake-debug
open1V-debug camera settings-debug
open1V-debug camera cfn-debug
open1V-debug camera pfn-debug
open1V-debug camera clock-debug
open1V-debug camera unknown-read-debug
open1V-debug camera film-header-debug
open1V-debug camera film-download-debug
open1V-debug camera continuous-debug
open1V-debug camera exit-debug
```

Diagnostic write commands require the explicit `--allow-write-debug` option:

```text
open1V-debug camera pfn4-roundtrip-debug --allow-write-debug
open1V-debug camera clock-set-debug --clock YYMMDDhhmmss --allow-write-debug
open1V-debug camera shooting-width16-roundtrip-debug --allow-write-debug
open1V-debug camera pfn-write-debug --block C3 --expect A010 --value 9810 --allow-write-debug
open1V-debug camera cfn-write-debug --block D1 --expect 2181122111111111110102 --value 2181122111111111110802 --allow-write-debug
```

Debug flows do not send F2 and therefore keep the camera's physical PC-mode
session open. Run `camera exit-debug` only when the session should end; entering
PC mode again after that is a manual camera operation.

`unknown-read-debug` is read-only. It samples `F3`, `A1`, `D1`, and `DD` three
times across separate logical actions while retaining one physical PC-mode
session. The raw output is intended to distinguish dynamic auxiliary bytes
from stable stored fields. It deliberately leaves PC mode open for follow-up
tests; use `camera exit-debug` when finished.

Setting writes treat an absent response as a busy interval: they wait up to 15
seconds, acknowledge asynchronous `F4`, and retry only the command phase. Once
the payload has been sent it is never resent on an ambiguous timeout. Round-trip
diagnostics therefore perform their test write and restoration in the same
physical PC-mode session without sending `F2` between them.

`self-test` is fully offline and is the first diagnostic check to run after
building.

## Transport options

Use `--port COM3` on Windows or a `/dev/...` path on macOS and Linux to select
a serial port explicitly. `--winusb` selects the reserved Windows-only WinUSB
transport. Camera protocol behavior remains independent of the transport.
