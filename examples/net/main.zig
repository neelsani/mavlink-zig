// main.zig — request data streams, send periodic HEARTBEAT, parse incoming packets
const std = @import("std");
const net = std.net;
const mavlink = @import("mavlink");

// ArduPilot‐Mega dialect
const dialect = mavlink.dialects.all;

pub fn main() !void {
    const addr = try net.Address.parseIp4("127.0.0.1", 8888);
    const conn = try net.tcpConnectToAddress(addr);
    defer conn.close();

    // Build REQUEST_DATA_STREAM once
    const req = dialect.messages.REQUEST_DATA_STREAM{
        .target_system = 1,
        .target_component = 0,
        .req_stream_id = @intFromEnum(dialect.enums.MAV_DATA_STREAM.MAV_DATA_STREAM_ALL),
        .req_message_rate = 100,
        .start_stop = 1,
    };

    // Build a GCS HEARTBEAT to send
    const hb = dialect.messages.HEARTBEAT{
        .autopilot = .MAV_AUTOPILOT_INVALID,
        .type = .MAV_TYPE_GCS,
        .system_status = .MAV_STATE_ACTIVE,
        .mavlink_version = 3,
        .base_mode = 0,
        .custom_mode = 0,
    };

    var outBuf: [mavlink.v2.MAVLINK_MAX_PACKET_SIZE]u8 = undefined;
    const reqLen = try mavlink.v2.writeMessageToSlice(outBuf[0..], req);
    const hbLen = try mavlink.v2.writeMessageToSlice(outBuf[0..], hb);

    try conn.writer().writeAll(outBuf[0..reqLen]);

    // Initialize a byte‐wise parser
    var parser = mavlink.v2.init();

    var readBuf: [1024]u8 = undefined;
    while (true) {
        const n = try conn.reader().read(&readBuf);
        if (n == 0) return; // closed

        // Feed each byte into parser until a full message appears
        for (readBuf[0..n]) |b| {
            if (parser.parseChar(b)) |msg| {
                switch (msg.msgid) {
                    // upon any HEARTBEAT reply, re‐send our GCS heartbeat
                    dialect.messages.HEARTBEAT.MSG_ID => {
                        try conn.writer().writeAll(outBuf[0..hbLen]);
                    },
                    // ATTITUDE: deserialize + print
                    dialect.messages.ATTITUDE.MSG_ID => {
                        const at = try mavlink.serde.deserialize(dialect.messages.ATTITUDE, msg.payload[0..msg.len]);
                        std.debug.print("Attitude: {any}\n", .{at});
                    },
                    // catch‐all for other known messages
                    else => |id| {
                        inline for (@typeInfo(dialect.messages).@"struct".decls) |decl| {
                            const T = @field(dialect.messages, decl.name);
                            if (id == T.MSG_ID) {
                                const msgde: T = mavlink.serde.deserialize(T, msg.payload[0..msg.len]) catch |e| {
                                    std.debug.print("Failed to deserialize {s}: {any}\n{any}\n", .{ @typeName(T), e, msg.payload[0..msg.len] });
                                    break;
                                };
                                _ = msgde;
                                //std.debug.print("Unhandled: {any}\n", .{msgde});
                            }
                        }
                    },
                }
            }
        }
    }
}
