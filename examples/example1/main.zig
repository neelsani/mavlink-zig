const std = @import("std");
const net = std.net;
const mavlink = @import("mavlink");

const D = mavlink.dialects.ardupilotmega;

pub fn main() !void {
    const conn = try net.tcpConnectToAddress(try net.Address.parseIp4("127.0.0.1", 8888));
    defer conn.close();

    // Send REQUEST_MESSAGE for PROTOCOL_VERSION
    const cmd = D.messages.COMMAND_LONG{
        .target_system = 1,
        .target_component = 0,
        .command = .MAV_CMD_REQUEST_MESSAGE,
        .confirmation = 0,
        .param1 = 0,
        .param2 = 0,
        .param3 = 0,
        .param4 = 0,
        .param5 = 0,
        .param6 = 0,
        .param7 = 0,
    };
    var buf: [mavlink.v2.MAVLINK_MAX_PACKET_SIZE]u8 = undefined;
    const clen = try mavlink.v2.writeMessageToSlice(buf[0..], cmd);
    try conn.writer().writeAll(buf[0..clen]);

    var parser = mavlink.v2.init();
    var rb: [256]u8 = undefined;
    while (true) {
        const n = try conn.reader().read(&rb);
        for (rb[0..n]) |b| {
            if (parser.parseChar(b)) |msg| {
                if (msg.msgid == D.messages.ATTITUDE.MSG_ID) {
                    const pv = try mavlink.serde.deserialize(D.messages.ATTITUDE, msg.payload[0..msg.len]);
                    std.debug.print("ATTITUDE {any}\n", .{pv});
                    return;
                }
            }
        }
    }
}
