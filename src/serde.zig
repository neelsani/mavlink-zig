const std = @import("std");

/// Deserialize a fixed-size value of type `T` from the provided buffer without allocations.
///
/// Supports:
/// - primitive types: integers, floats, bools
/// - fixed-size arrays
/// - packed and unpacked structs
///
/// Does NOT support:
/// - slices or dynamically sized collections
/// - allocations
/// - unions
/// - enums (except as part of struct fields)
///
/// # Parameters
/// - `T`: The compile-time type to deserialize into.
/// - `buf`: A byte slice containing serialized data in little-endian order.
///
/// # Returns
/// - `T` populated with data from `buf`.
///
/// # Errors
/// - Returns an error if reading beyond `buf` length or encountering an unsupported type.
pub fn deserialize(comptime T: type, buf: []const u8) !T {
    var fbs = std.io.fixedBufferStream(buf);
    const reader = fbs.reader();
    var out: T = undefined;
    try recursiveDeserialize(T, reader, &out);
    return out;
}

/// Recursively deserialize data of type `T` from `reader` into `target`.
///
/// Handles nested arrays and structs via compile-time reflection. Emits a compile error
/// for unsupported types like pointers (slices).
///
/// # Parameters
/// - `T`: The compile-time type to read.
/// - `reader`: A fixed-buffer reader implementing `readByte`, `readInt`, etc.
/// - `target`: Pointer to the output location where the deserialized value will be stored.
///
/// # Errors
/// - Propagates I/O errors from `reader`.
fn recursiveDeserialize(
    comptime T: type,
    reader: anytype,
    target: *T,
) !void {
    switch (@typeInfo(T)) {
        .void => {},
        .bool => target.* = (try reader.readByte() != 0),
        .int => |_| {
            target.* = try reader.readInt(T, .little);
        },
        .float => |_| {
            const IntType = std.meta.Int(.unsigned, @bitSizeOf(T));
            const bits = try reader.readInt(IntType, .little);
            target.* = @bitCast(bits);
        },
        .array => |arr| {
            if (arr.child == u8) {
                // Bulk-read for byte arrays
                try reader.readNoEof(target);
            } else {
                // Element-wise recursion for arrays of non-bytes
                for (&target.*) |*item| {
                    try recursiveDeserialize(arr.child, reader, item);
                }
            }
        },
        .@"struct" => |s| {
            inline for (s.fields) |fld| {
                if (s.layout == .@"packed") {
                    // Packed structs: read into a temp to avoid alignment issues
                    var tmp: fld.type = undefined;
                    try recursiveDeserialize(fld.type, reader, &tmp);
                    @field(target.*, fld.name) = tmp;
                } else {
                    // Aligned structs: deserialize directly into field
                    try recursiveDeserialize(fld.type, reader, &@field(target.*, fld.name));
                }
            }
        },
        .pointer => @compileError("Pointers/slices not supported in allocation-free deserializer"),
        // In your deserializer, add this case to the switch statement:
        .@"enum" => |enumInfo| {
            const IntType = enumInfo.tag_type;
            const raw = try reader.readInt(IntType, .little);

            // Special handling for bitmask enums
            if (@hasDecl(T, "is_bitmask") and T.is_bitmask) {
                target.* = @bitCast(raw);
                return;
            }

            // Regular enum handling
            inline for (std.meta.tags(T)) |tag| {
                if (@intFromEnum(tag) == raw) {
                    target.* = tag;
                    return;
                }
            }
            return error.InvalidEnumValue;
        },
        else => @compileError("Type not supported: " ++ @typeName(T)),
    }
}

/// Serialize a value `value` of any type to the provided `buffer` using little-endian format.
///
/// Internally writes to a fixed-buffer stream to avoid allocations.
///
/// # Parameters
/// - `value`: The value to serialize (integer, float, bool, array, struct).
/// - `buffer`: Byte slice to which the serialized data will be written.
///
/// # Returns
/// - `usize` Number of bytes written to `buffer`.
///
/// # Errors
/// - Propagates errors from the writer (e.g., buffer overflow).
pub fn serialize(
    value: anytype,
    buffer: []u8,
) !usize {
    var stream = std.io.fixedBufferStream(buffer);
    const writer = stream.writer();
    const T = @TypeOf(value);
    try recursiveSerialize(T, value, writer);
    return stream.pos;
}

/// Recursively serialize `value` of compile-time type `T` to `writer`.
///
/// Supports primitive types, fixed-size arrays, one-element pointers, and structs.
/// For packed structs, data is copied via `memcpy` to ensure correct layout.
///
/// # Parameters
/// - `T`: The compile-time type of `value`.
/// - `value`: The value to write.
/// - `writer`: A writer implementing `writeByte`, `writeInt`, etc.
///
/// # Errors
/// - Propagates I/O errors or compile-time errors for unsupported types.
fn recursiveSerialize(
    comptime T: type,
    value: T,
    writer: anytype,
) !void {
    const info = @typeInfo(T);

    switch (info) {
        .type, .void => {},
        .bool => try writer.writeByte(if (value) 1 else 0),
        .int => try writer.writeInt(T, value, .little),
        .float => {
            const IntType = std.meta.Int(.unsigned, @bitSizeOf(T));
            try writer.writeInt(IntType, @bitCast(value), .little);
        },
        .array => |arr| {
            if (arr.child == u8) {
                // Bulk-write for byte arrays
                try writer.writeAll(&value);
            } else {
                // Element-wise recursion for arrays of non-bytes
                for (value) |item| {
                    try recursiveSerialize(arr.child, item, writer);
                }
            }
        },
        .pointer => |ptr| {
            switch (ptr.size) {
                .slice => @compileError("Slices require known maximum size. Use array or provide a sufficiently large buffer"),
                .one => try recursiveSerialize(ptr.child, value.*, writer),
                else => @compileError("Unsupported pointer type: " ++ @typeName(T)),
            }
        },
        .@"struct" => |s| {
            inline for (s.fields) |fld| {
                const field_value = @field(value, fld.name);
                if (s.layout == .@"packed") {
                    // Copy to temp to maintain packed layout
                    var tmp: fld.type = undefined;
                    @memcpy(std.mem.asBytes(&tmp), std.mem.asBytes(&field_value)[0..@sizeOf(fld.type)]);
                    try recursiveSerialize(fld.type, tmp, writer);
                } else {
                    try recursiveSerialize(fld.type, field_value, writer);
                }
            }
        },
        .@"enum" => |enumInfo| {
            try writer.writeInt(enumInfo.tag_type, @intFromEnum(value), .little);
        },
        else => @compileError("Type not supported: " ++ @typeName(T)),
    }
}

test "serialize then deserialize equality" {
    const testing = std.testing;
    const mavlink = @import("root.zig");
    const dialect = mavlink.dialects.ardupilotmega;
    const data = dialect.messages.BATTERY_STATUS{
        .battery_function = .MAV_BATTERY_FUNCTION_ALL,
        .battery_remaining = 0,
        .charge_state = .MAV_BATTERY_CHARGE_STATE_EMERGENCY,
        .current_battery = 2,
        .current_consumed = 2,
        .energy_consumed = 3,
        .fault_bitmask = dialect.enums.MAV_BATTERY_FAULT.fromInt(1).bits,
        .id = 4,
        .mode = .MAV_BATTERY_MODE_AUTO_DISCHARGING,
        .temperature = 3,
        .time_remaining = 1,
        .type = .MAV_BATTERY_TYPE_LION,
        .voltages = [_]u16{
            1,
            4,
            1,
            2,
            4,
            1,
            4,
            1,
            2,
            4,
        },
        .voltages_ext = [_]u16{ 3, 4, 2, 4 },
    };
    var buf: [mavlink.v2.MAVLINK_MAX_PACKET_SIZE]u8 = undefined;
    const mlen = try serialize(data, buf[0..]);
    const pak = try deserialize(dialect.messages.BATTERY_STATUS, buf[0..mlen]);
    try testing.expectEqualDeep(pak, data);

    var buf1: [mavlink.v2.MAVLINK_MAX_PACKET_SIZE]u8 = undefined;
    const mlen1 = try serialize(pak, buf1[0..]);

    try testing.expectEqualSlices(u8, buf[0..mlen], buf1[0..mlen1]);
}

test "bool round-trip" {
    const testing = std.testing;
    const buf: [8]u8 = undefined;
    var writer_buf = buf;
    const v_true: bool = true;
    const wlen = try serialize(v_true, writer_buf[0..]);
    const r = try deserialize(bool, writer_buf[0..wlen]);
    try testing.expectEqual(r, v_true);

    const v_false: bool = false;
    const wlen2 = try serialize(v_false, writer_buf[0..]);
    const r2 = try deserialize(bool, writer_buf[0..wlen2]);
    try testing.expectEqual(r2, v_false);
}

test "integer array round-trip" {
    const testing = std.testing;
    const data: [5]u16 = .{ 10, 20, 30, 40, 50 };
    var buf: [16]u8 = undefined;
    const len = try serialize(data, buf[0..]);
    const out: [5]u16 = try deserialize([5]u16, buf[0..len]);
    try testing.expectEqualDeep(out, data);
}

test "nested struct serialize/deserialize" {
    const testing = std.testing;
    const Point = struct { x: i32, y: i32 };
    const Rect = struct { origin: Point, size: Point };
    const rect = Rect{ .origin = Point{ .x = -5, .y = 15 }, .size = Point{ .x = 20, .y = 30 } };

    var buf: [32]u8 = undefined;
    const len = try serialize(rect, buf[0..]);
    const got: Rect = try deserialize(Rect, buf[0..len]);
    try testing.expectEqualDeep(got, rect);
}

test "packed struct round-trip" {
    const testing = std.testing;
    const Packed = packed struct {
        a: u8,
        b: u16,
        c: f32,
    };
    const original = Packed{ .a = 1, .b = 0x1234, .c = 3.14 };
    var buf: [16]u8 = undefined;
    const len = try serialize(original, buf[0..]);
    const decoded: Packed = try deserialize(Packed, buf[0..len]);
    try testing.expectEqualDeep(decoded, original);
}
