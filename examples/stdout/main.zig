const std = @import("std");
const mavlink = @import("mavlink");

const dialect = mavlink.dialects.common;

pub fn main() !void {
    // Prepare a HEARTBEAT indicating this is a GCS
    const hb = dialect.messages.HEARTBEAT{
        .type = @intFromEnum(dialect.enums.MAV_TYPE.MAV_TYPE_GCS),
        .autopilot = @intFromEnum(dialect.enums.MAV_AUTOPILOT.MAV_AUTOPILOT_INVALID),
        .base_mode = @intFromEnum(dialect.enums.MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED),
        .custom_mode = 0,
        .system_status = @intFromEnum(dialect.enums.MAV_STATE.MAV_STATE_STANDBY),
        .mavlink_version = 3,
    };

    // Serialize to buffer
    var buf: [mavlink.v2.MAVLINK_MAX_PACKET_SIZE]u8 = undefined;
    const len = try mavlink.v2.writeMessageToSlice(buf[0..], hb);

    // Write raw packet to stdout
    const w = std.io.getStdOut().writer();
    try w.writeAll(buf[0..len]);
}
