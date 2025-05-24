const std = @import("std");
const Parser = @import("parser.zig");

pub fn generateMulti(allocator: std.mem.Allocator, profiles: []Parser.MavProfile, destination_dir: std.fs.Dir) !void {
    var dir = destination_dir;

    // Generate enums.zig
    var enums_file = try dir.createFile("enums.zig", .{});
    defer enums_file.close();
    var enums_writer = enums_file.writer();

    try enums_writer.writeAll(
        \\// Auto-generated MAVLink enums
        \\// DO NOT EDIT MANUALLY
        \\
        \\
    );

    // Generate messages.zig
    var messages_file = try dir.createFile("messages.zig", .{});
    defer messages_file.close();
    var messages_writer = messages_file.writer();

    try messages_writer.writeAll(
        \\// Auto-generated MAVLink messages
        \\// DO NOT EDIT MANUALLY
        \\
        \\const enums = @import("enums.zig");
        \\
        \\
    );

    // Process MAV_CMD separately
    var mavfields = std.ArrayList(Parser.MavEnumEntry).init(allocator);
    var mavcmd: Parser.MavEnum = .{
        .name = "MAV_CMD",
        .description = null,
        .entries = mavfields,
    };
    // Generate messages
    for (profiles) |profile| {
        var messages_iter = profile.messages.iterator();
        while (messages_iter.next()) |entry| {
            const msg = entry.value_ptr;

            if (msg.description) |desc| {
                try writeComment(messages_writer, desc, false);
            }

            try messages_writer.print("pub const {s} = struct {{\n", .{msg.name});
            try messages_writer.print("    pub const MSG_ID = {d};\n", .{msg.id});
            //try messages_writer.print("    pub const CRC_EXTRA = {d};\n\n", .{crc_extra});
            for (msg.fields.items) |field| {
                const zig_type = mavTypeToZigType(field.mavtypeinfo, allocator);

                // Handle field description comment
                if (field.description) |desc| {
                    try writeComment(messages_writer, desc, true);
                }

                // Handle reserved keyword 'error'
                const field_name = if (std.mem.eql(u8, field.name, "error"))
                    "mError"
                else
                    field.name;

                // Handle enum case
                if (field.mavtypeinfo.array_size == null and field.enumtype != null) {
                    const enum_found = blk: {
                        const enumval = field.enumtype.?;

                        for (profiles) |profenum| {
                            var enum_iter = profenum.enums.iterator();
                            while (enum_iter.next()) |enum_entry| {
                                if (std.mem.eql(u8, enumval, enum_entry.value_ptr.name)) {
                                    if (!enum_entry.value_ptr.bitmask) {
                                        enum_entry.value_ptr.enumtype = .{ .mavtype = field.mavtypeinfo.mavtype };
                                        try messages_writer.print("    {s}: enums.{s},\n\n", .{ field_name, enumval });
                                    } else {
                                        try messages_writer.print("    {s}: {s},\n\n", .{ field_name, zig_type });
                                    }
                                    break :blk true;
                                }
                            }
                        }
                        break :blk false;
                    };

                    if (!enum_found) {
                        @panic(try std.fmt.allocPrint(allocator, "    {s}: {s}, // Enum '{any}' not found in profiles\n\n", .{ field_name, zig_type, field.enumtype.? }));
                    }
                } else {
                    // Non-enum case
                    try messages_writer.print("    {s}: {s},\n\n", .{ field_name, zig_type });
                }
            }
            if (msg.extensions.items.len != 0) {
                try messages_writer.print("\n    //Extensions\n", .{});
            }
            for (msg.extensions.items) |field| {
                const zig_type = mavTypeToZigType(field.mavtypeinfo, allocator);
                if (field.description) |desc| {
                    try writeComment(messages_writer, desc, true);
                }

                const field_name = if (std.mem.eql(u8, field.name, "error"))
                    "mError"
                else
                    field.name;

                try messages_writer.print("    {s}: {s},\n\n", .{ field_name, zig_type });
            }

            try messages_writer.writeAll(
                \\};
            ++ "\n\n");
        }
    }
    // Generate enums
    for (profiles) |profile| {
        var enums_iter = profile.enums.iterator();
        while (enums_iter.next()) |entry| {
            const enm = entry.value_ptr;
            if (std.mem.eql(u8, enm.name, "MAV_CMD")) {
                if (enm.description) |desc| {
                    mavcmd.description = desc;
                }
                try mavfields.appendSlice(enm.entries.items);
                continue;
            }

            if (enm.description) |desc| {
                try writeComment(enums_writer, desc, false);
            }
            if (enm.enumtype) |etyp| {
                const zig_type = mavTypeToZigType(etyp, allocator);
                try enums_writer.print("pub const {s} = enum({s}) {{\n", .{ enm.name, zig_type });
            } else {
                try enums_writer.print("pub const {s} = enum(u32) {{\n", .{enm.name});
            }
            if (enm.bitmask) {
                try enums_writer.print("    pub const BITMASK = true;\n\n", .{});
            }
            for (enm.entries.items) |enum_entry| {
                if (enum_entry.description) |desc| {
                    try writeComment(enums_writer, desc, true);
                }
                try enums_writer.print("    {s} = {d},\n", .{ enum_entry.name, enum_entry.value orelse 0 });
            }
            try enums_writer.writeAll("};\n\n");
        }
    }

    // Generate MAV_CMD enum
    if (mavcmd.description) |desc| {
        try writeComment(enums_writer, desc, false);
    }
    try enums_writer.writeAll("pub const MAV_CMD = enum(u16) {\n");
    for (mavfields.items) |enum_entry| {
        if (enum_entry.description) |desc| {
            try writeComment(enums_writer, desc, true);
        }
        try enums_writer.print("    {s} = {d},\n", .{ enum_entry.name, enum_entry.value orelse 0 });
    }
    try enums_writer.writeAll("};\n\n");
}

fn mavTypeToZigType(mavtypeinfo: Parser.MavTypeInfo, allocator: std.mem.Allocator) []const u8 {
    return switch (mavtypeinfo.mavtype) {
        .UInt8 => "u8",
        .UInt16 => "u16",
        .UInt32 => "u32",
        .UInt64 => "u64",
        .Int8 => "i8",
        .Int16 => "i16",
        .Int32 => "i32",
        .Int64 => "i64",
        .Float => "f32",
        .Double => "f64",
        .FixedArray => blk: {
            const size = mavtypeinfo.array_size orelse @panic("FixedArray must have array_size");
            const child = mavtypeinfo.array_child_type orelse @panic("FixedArray must have array_child_type");
            break :blk switch (child) {
                .Float => std.fmt.allocPrint(allocator, "[{d}]f32", .{size}) catch unreachable,
                .Double => std.fmt.allocPrint(allocator, "[{d}]f64", .{size}) catch unreachable,
                .UInt8 => std.fmt.allocPrint(allocator, "[{d}]u8", .{size}) catch unreachable,
                .UInt16 => std.fmt.allocPrint(allocator, "[{d}]u16", .{size}) catch unreachable,
                .UInt32 => std.fmt.allocPrint(allocator, "[{d}]u32", .{size}) catch unreachable,
                .UInt64 => std.fmt.allocPrint(allocator, "[{d}]u64", .{size}) catch unreachable,
                .Int8 => std.fmt.allocPrint(allocator, "[{d}]i8", .{size}) catch unreachable,
                .Int16 => std.fmt.allocPrint(allocator, "[{d}]i16", .{size}) catch unreachable,
                .Int32 => std.fmt.allocPrint(allocator, "[{d}]i32", .{size}) catch unreachable,
                .Int64 => std.fmt.allocPrint(allocator, "[{d}]i64", .{size}) catch unreachable,
                else => @panic("Unsupported MAVLink type"),
            };
        },
        else => @panic("Unsupported MAVLink type"),
    };
}

fn writeComment(writer: anytype, comment: []const u8, indent: bool) !void {
    var it = std.mem.splitSequence(u8, comment, "\n");
    while (it.next()) |line| {
        if (indent) {
            try writer.print("    /// {s}\n", .{line});
        } else {
            try writer.print("/// {s}\n", .{line});
        }
    }
}
