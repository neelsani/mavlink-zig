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
                // Check if it’s a ATTITUDE response
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
