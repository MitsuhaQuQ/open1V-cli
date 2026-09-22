# open1V

`open1V` is a standalone Windows command-line application for the independently
implemented UNO R4 WiFi camera bridge. It has no build-time dependency on the
Arduino source tree.

The repository now builds three targets:

- `open1v-core.lib` — reusable connection/session library for CLI or GUI clients
- `open1V.exe` — human-readable command-line client linked to `open1v-core`
- `open1V-debug.exe` — protocol diagnostics, raw bridge commands, offline tests,
  and explicitly authorized diagnostic writes

See [LIBRARY.md](LIBRARY.md) for the public API and integration example.

## Boundaries

- `CameraProtocolSession` is the single camera communication layer. CLI calls
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
  commands are exposed only by `open1V-debug.exe`.
- `BridgeClient` owns the documented `O1` framing protocol.
- `ITransport` isolates platform transport from both layers.
- `SerialTransport` is the default Windows adapter and auto-detects the
  official UNO R4 WiFi COM port.
- `WinUsbTransport` remains available behind `--winusb` for future native USB
  firmware work.
- The Arduino project only forwards framed requests to its camera-side UART.

The first camera operation is deliberately read-only: `camera identify`.

## Build

Open `open1V.slnx` in Visual Studio, or run:

```powershell
MSBuild.exe open1V.slnx /p:Configuration=Release /p:Platform=x64
```

From a normal Command Prompt with Visual Studio installed, `build.cmd` locates the newest MSBuild automatically and performs the same Release build.

## Commands

```text
open1V-debug self-test
open1V-debug frame ping
open1V-debug bridge ping
open1V-debug bridge status
open1V-debug bridge link-status
open1V-debug camera identify
open1V camera id
open1V camera cfn
open1V camera pfn
open1V camera clock
open1V camera info
open1V camera console
open1V camera set-id --id 12
open1V camera set-clock --clock 260922120000
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

Use `--port COM3` after a command to select a port explicitly, or `--winusb`
to exercise the reserved WinUSB transport. Camera behavior remains independent
of either transport.

`self-test` is fully offline and is the first check to run after building.

`id`, `cfn`, `pfn`, and `clock` decode fields shown by the original application
into readable values and close PC mode after reading. They do not implement a
second communication path. Opaque fields remain available only in raw
`*-debug` output.
`camera info` reads all four groups in one physical PC-mode session and exits once.

Running `open1V.exe` without arguments enters `camera console` automatically.
`camera console` is the interactive human-readable mode. Startup performs only
the handshake and displays `Camera: EOS-1V` plus the camera ID, so the prompt
appears quickly. `show` explicitly reads all current settings. The console accepts
`cfn`, `pfn`, `set id`, and `set clock` (`set time` remains a compatibility alias).
`cfn` and `pfn` each open a guided read/modify prompt:
it refreshes the target's current value, asks for a field number where needed,
shows its available choices, then validates and verifies the write. Entering
`q` at any prompt returns to the main console. PC mode remains active until
`exit` or end-of-input, when the only F2 is sent.

`cfn` first asks for `current`, `1`, `2`, or `3`. It then displays only the
selected bank and writes through the matching current/registered command pair.
Inside `cfn-edit`, any number of options can be staged. The complete selected
bank is shown after every change; `commit` writes and verifies all differences,
while `discard` or `q` returns without writing.

`pfn` similarly enters `pfn-edit`. ON/OFF and supported sub-values are
staged in a complete P.Fn preview. `commit` is available only in that sub-state
and performs the verified writes; `discard` or `q` abandons them.

`set clock` offers two choices: `1) Sync with system`, which uses the Windows
local system clock, and `2) Set manually`, which accepts the complete
`YYMMDDhhmmss` value. `set time` remains accepted as a compatibility alias.

P.Fn-27 includes both its ON/OFF bit and a dial-selection value from the fifth
`DD/DE` payload byte: `0` Main Dial only, `1` Quick Control Dial only, and `2`
Both dials. The console displays and verifies both parts separately.
If the handshake cannot be established, the console reports that the camera is
not connected or is not in PC mode and does not enter the command prompt.
