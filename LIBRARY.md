# open1v-core library

`open1v-core` is the reusable camera connection library. It contains no CLI,
terminal input, or human-readable presentation code.

Public headers are under `include/open1v`:

- `camera_protocol.hpp` — camera session lifecycle, reads, and verified writes
- `bridge_client.hpp` — framed host-to-UNO bridge client
- `serial_transport.hpp` — Windows COM transport and UNO R4 auto-detection
- `winusb_transport.hpp` — reserved native WinUSB transport
- `transport.hpp` — transport interface for another host or tests

Add `open1v-core.vcxproj` as a Visual Studio project reference and add this
repository's `include` directory to the consumer's include path. Windows
applications must link `setupapi.lib` and `winusb.lib`.

```cpp
#include "open1v/camera_protocol.hpp"
#include "open1v/serial_transport.hpp"

open1v::SerialTransport transport; // auto-detect the UNO R4 port
open1v::BridgeClient bridge(transport);
open1v::CameraProtocolSession camera(bridge);

// Script or CLI style: one task and automatic F2 shutdown.
auto packets = camera.readOnce(open1v::CameraRead::cfn);

// GUI style: retain one physical PC-mode session across several actions.
camera.beginSession();
auto cfn = camera.perform(open1v::CameraRead::cfn);
auto pfn = camera.perform(open1v::CameraRead::pfn);
camera.setCurrentCfn(19, 3); // includes write read-back verification
camera.endSession();         // the only F2
```

The library returns raw `CameraPacket` values. Applications decide which
confirmed fields to decode or display. Unknown auxiliary fields remain
available without being assigned speculative meanings.
