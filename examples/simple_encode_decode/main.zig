const std = @import("std");
const mavlink = @import("mavlink");
const ardu = mavlink.dialects.ardupilotmega;
pub fn main() !void {
    var parser = mavlink.v2.init();
    const x: []const u8 = &.{ 253, 33, 0, 0, 0, 255, 255, 75, 0, 0, 0, 0, 128, 63, 10, 215, 35, 60, 10, 215, 35, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 179, 0, 1, 0, 3, 213, 122 };
    //std.debug.print("{d}\n", .{pak.calculateCrc()});
    for (x) |b| {
        if (parser.parseChar(b)) |pak| {
            std.debug.print("{d}\n", .{pak.msgid});
            if (pak.msgid == ardu.messages.COMMAND_INT.MSG_ID) {
                const req = try mavlink.serde.deserialize(ardu.messages.COMMAND_INT, pak.payload[0..pak.len]);
                std.debug.print("{any}\n", .{req});
            }
        }
    }
}
