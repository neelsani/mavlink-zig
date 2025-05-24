const std = @import("std");
const net = std.net;
const mavlink = @import("mavlink");

const D = mavlink.dialects.common;

pub fn main() !void {
    const conn = try net.tcpConnectToAddress(try net.Address.parseIp4("127.0.0.1", 8888));
    defer conn.close();

    // PARAM_REQUEST_LIST asks for every param
    const req = D.messages.PARAM_REQUEST_LIST{
        .target_system = 1,
        .target_component = 0,
    };

    var buf: [mavlink.v2.MAVLINK_MAX_PACKET_SIZE]u8 = undefined;
    const rlen = try mavlink.v2.writeMessageToSlice(buf[0..], req);
    try conn.writer().writeAll(buf[0..rlen]);

    var parser = mavlink.v2.init();
    var rb: [512]u8 = undefined;
    while (true) {
        const n = try conn.reader().read(&rb);
        for (rb[0..n]) |b| {
            if (parser.parseChar(b)) |msg| {
                if (msg.msgid == D.messages.PARAM_VALUE.MSG_ID) {
                    const pv = try mavlink.serde.deserialize(D.messages.PARAM_VALUE, msg.payload[0..msg.len]);
                    // Compute index+1 up front
                    const display_index = pv.param_index + 1;

                    // Use {} for both string, float, and integer placeholders:
                    std.debug.print("{s} = {e} ({d}/{d})\n", .{ pv.param_id, pv.param_value, display_index, pv.param_count });

                    // Safe check for last parameter
                    if (pv.param_count == 0) return; // handle case where there are no parameters
                    if (pv.param_index + 1 >= pv.param_count) return;
                }
            }
        }
    }
}
