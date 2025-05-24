# MAVLink Zig Library ✈️

[![Zig Version](https://img.shields.io/badge/Zig-0.14.0-%23f7a41d.svg)](https://ziglang.org/)
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Build Status](https://github.com/neelsani/mavlink-zig/actions/workflows/build.yaml/badge.svg)](https://github.com/neelsani/mavlink-zig/actions)

A lightweight, zero-dependency, zero-allocation MAVLink v2.0 protocol implementation in Zig, designed for embedded systems and high-performance applications.

## What is MAVLink? 🚁

MAVLink (Micro Air Vehicle Link) is a lightweight messaging protocol for communicating with drones, robotics systems, and other autonomous vehicles. Key features:

- **Binary protocol** optimized for limited bandwidth
- **Standardized message set** for common drone operations
- **Extensible** through custom message definitions
- **Widely adopted** by major autopilots (PX4, ArduPilot) and ground stations

This library brings MAVLink to Zig with full type safety and no runtime overhead.

## Features ✨

- 🚀 Full MAVLink v2.0 protocol support
- ⚡ Zero allocations during packet processing
- 🔒 End-to-end type safety with compile-time message validation
- 🔋 Suitable for resource-constrained systems (embedded friendly)
- 🔄 Automatic dialect generation from official XML definitions
- ✨ 100% Pure Zig - no C dependencies

## Installation

### As a Dependency

Add to your project using Zig's package manager:

```sh
# For stable Zig releases (recommended for production):
zig fetch --save https://github.com/neelsani/mavlink-zig/archive/refs/tags/vX.Y.Z.tar.gz

# For Zig master branch compatibility:
zig fetch --save git+https://github.com/neelsani/mavlink-zig
```
## Usage / Examples 🚀

### 📂 Where to Find Examples

- All sample code is located in the [`examples/`](examples/) directory.
- Each subfolder contains a self-contained example.

### ▶️ Running an Example

To run an individual example, use Zig's build system. Replace `<name of example folder>` with the specific example you want to run. For instance, to run the `net` example:

```sh
zig build net
```

### ✨ Minimal Example

Here’s a minimal example of how to parse a MAVLink heartbeat message using the library:

```zig
const std = @import("std"); // Import Zig’s standard library
const net = std.net; // Bring networking APIs into scope
const mavlink = @import("mavlink"); // Import the MAVLink library

const D = mavlink.dialects.common; // Alias the “common” dialect for convenience

pub fn main() !void {
    // Establish a TCP connection to the MAVLink endpoint (e.g., SITL at 127.0.0.1:8888)
    const conn = try net.tcpConnectToAddress(try net.Address.parseIp4("127.0.0.1", 8888));
    defer conn.close(); // Ensure the connection is closed on exit

    // -----------------------------------------------------------------------
    // Build a HEARTBEAT message, acting as a Ground Control Station (GCS)
    // -----------------------------------------------------------------------
    const hb = D.messages.HEARTBEAT{
        .type = .MAV_TYPE_GCS, // Identify as GCS
        .autopilot = .MAV_AUTOPILOT_INVALID, // Not a flight controller
        .base_mode = @intFromEnum(D.enums.MAV_MODE.MAV_MODE_AUTO_ARMED),
        // Armed in AUTO mode
        .custom_mode = 0, // No custom mode flags
        .system_status = .MAV_STATE_STANDBY, // Standby state
        .mavlink_version = 3, // MAVLink v2.0 identifier
    };

    // Prepare a buffer large enough for any v2 packet
    var buf: [mavlink.v2.MAVLINK_MAX_PACKET_SIZE]u8 = undefined;

    // Serialize the HEARTBEAT into the buffer slice, get the byte count
    const len = try mavlink.v2.writeMessageToSlice(buf[0..], hb);

    // Send the serialized packet over the TCP connection
    try conn.writer().writeAll(buf[0..len]);

    // -----------------------------------------------------------------------
    // Initialize the MAVLink v2 parser for incoming data
    // -----------------------------------------------------------------------
    var parser = mavlink.v2.init();

    // A temporary read buffer for incoming TCP data
    var readBuf: [512]u8 = undefined;

    // Continuously read incoming bytes and feed them to the parser
    while (true) {
        const n = try conn.reader().read(&readBuf); // Read up to `readBuf.len` bytes
        for (readBuf[0..n]) |b| { // Iterate each received byte
            if (parser.parseChar(b)) |msg| { // If a full MAVLink message is parsed
                // Check if it’s a HEARTBEAT response
                if (msg.msgid == D.messages.HEARTBEAT.MSG_ID) {
                    // Deserialize the payload into a HEARTBEAT struct
                    const reply = try mavlink.serde.deserialize(D.messages.HEARTBEAT, msg.payload[0..msg.len]);
                    // Print the received heartbeat in debug output
                    std.debug.print("GCS got HEARTBEAT: {any}\n", .{reply});
                }
            }
        }
    }
}
```

> **Note:**  
> For more complete and advanced examples, check out the [`examples/`](examples/) directory.

### 📖 API Reference

For detailed documentation and the full API reference, visit:  
[http://mavlink-zig.neels.dev/](http://mavlink-zig.neels.dev/)

---

## Acknowledgements
- [mavlink](https://mavlink.io/en/) — Protocol specification
- [zig-xml](https://github.com/ianprime0509/zig-xml) — the XML parsing library used by the code generator to transform MAVLink XML definitions into Zig code.  

## Support

For support, email neel@neels.dev.