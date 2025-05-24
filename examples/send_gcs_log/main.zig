const std = @import("std");
const net = std.net;
const mavlink = @import("mavlink");

const D = mavlink.dialects.common;

pub fn main() !void {
    const conn = try net.tcpConnectToAddress(try net.Address.parseIp4("127.0.0.1", 8888));
    defer conn.close();

    // Build HEARTBEAT as Ground Control Station
    const hb = D.messages.HEARTBEAT{
        .type = @intFromEnum(D.enums.MAV_TYPE.MAV_TYPE_GCS),
        .autopilot = @intFromEnum(D.enums.MAV_AUTOPILOT.MAV_AUTOPILOT_INVALID),
        .base_mode = 0,
        .custom_mode = 0,
        .system_status = @intFromEnum(D.enums.MAV_STATE.MAV_STATE_STANDBY),
        .mavlink_version = 3,
    };

    var buf: [mavlink.v2.MAVLINK_MAX_PACKET_SIZE]u8 = undefined;
    const len = try mavlink.v2.writeMessageToSlice(buf[0..], hb);
    try conn.writer().writeAll(buf[0..len]);

    var parser = mavlink.v2.init();
    var readBuf: [512]u8 = undefined;
    while (true) {
        const n = try conn.reader().read(&readBuf);
        for (readBuf[0..n]) |b| {
            if (parser.parseChar(b)) |msg| {
                if (msg.msgid == D.messages.HEARTBEAT.MSG_ID) {
                    const reply = try mavlink.serde.deserialize(D.messages.HEARTBEAT, msg.payload[0..msg.len]);
                    std.debug.print("GCS got HEARTBEAT: {any}\n", .{reply});
                }
            }
        }
    }
}
