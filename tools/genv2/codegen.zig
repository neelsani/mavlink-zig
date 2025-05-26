const std = @import("std");
const IR = @import("semantic_analysis.zig").IR;
const config = @import("config");

pub const OUTPUT_DIR: []const u8 = config.dialect_out_dir;

pub fn generate(allocator: std.mem.Allocator, ir: *const IR, name: []const u8) !void {
    // 1. Create full output path
    const full_path = try std.fs.path.join(allocator, &[_][]const u8{ OUTPUT_DIR, name[0 .. name.len - 4] });
    defer allocator.free(full_path);

    // 2. Delete folder if it exists, then recreate it
    const dir = std.fs.cwd();
    dir.deleteTree(full_path) catch {};
    try dir.makePath(full_path);

    // 3. Generate enums.zig file
    const enums_file = try dir.createFile(try std.fs.path.join(allocator, &[_][]const u8{ full_path, "enums.zig" }), .{});
    defer {
        enums_file.close();
    }

    // 4. Generate messages.zig file
    const messages_file = try dir.createFile(try std.fs.path.join(allocator, &[_][]const u8{ full_path, "messages.zig" }), .{});
    defer {
        messages_file.close();
    }

    const enums_writer = enums_file.writer();
    const messages_writer = messages_file.writer();

    try enums_writer.writeAll(
        \\// Auto-generated MAVLink enums
        \\// DO NOT EDIT MANUALLY
        \\
        \\
    );

    try messages_writer.writeAll(
        \\// Auto-generated MAVLink messages
        \\// DO NOT EDIT MANUALLY
        \\
        \\const enums = @import("enums.zig");
        \\
        \\
    );

    try generateEnums(allocator, ir, enums_writer);
    try generateMessages(allocator, ir, messages_writer);
}

pub fn generateEnums(allocator: std.mem.Allocator, ir: *const IR, writer: anytype) !void {
    var enum_iter = ir.enums.iterator();
    while (enum_iter.next()) |entry| {
        const enm = entry.value_ptr;

        if (enm.description) |desc| {
            try writeComment(writer, desc, false);
        }

        if (enm.bitmask) {
            // Generate bitmask enum with helper functions
            try writer.print(
                \\pub const {s} = packed struct {{
                \\    pub const is_bitmask = true;
                \\    bits: {s},
                \\
                \\    pub const Type = {s};
                \\
                \\    pub inline fn toInt(self: @This()) Type {{
                \\        return self.bits;
                \\    }}
                \\
                \\    pub inline fn fromInt(bits: Type) @This() {{
                \\        return @This(){{ .bits = bits }};
                \\    }}
                \\
                \\    pub inline fn isSet(self: @This(), comptime flag: @This()) bool {{
                \\        return (self.bits & flag.bits) != 0;
                \\    }}
                \\
                \\    pub inline fn set(self: *@This(), comptime flag: @This()) void {{
                \\        self.bits |= flag.bits;
                \\    }}
                \\
                \\    pub inline fn unset(self: *@This(), comptime flag: @This()) void {{
                \\        self.bits &= ~flag.bits;
                \\    }}
                \\
                \\    pub inline fn toggle(self: *@This(), comptime flag: @This()) void {{
                \\        self.bits ^= flag.bits;
                \\    }}
                \\
            , .{
                enm.name,
                enm.c_type.to_zig_primitive(allocator),
                enm.c_type.to_zig_primitive(allocator),
            });

            // Generate flag constants
            for (enm.entries) |enum_entry| {
                if (enum_entry.description) |desc| {
                    try writeComment(writer, desc, true);
                }
                try writer.print(
                    \\    pub const {s}: @This() = .{{ .bits = {d} }};
                    \\
                , .{
                    enum_entry.name,
                    enum_entry.value,
                });
            }

            try writer.writeAll("};\n\n");
        } else {
            // Regular enum generation
            try writer.print("pub const {s} = enum({s}) {{\n", .{
                enm.name,
                enm.c_type.to_zig_primitive(allocator),
            });

            for (enm.entries) |enum_entry| {
                if (enum_entry.description) |desc| {
                    try writeComment(writer, desc, true);
                }
                try writer.print("    {s} = {d},\n", .{
                    enum_entry.name,
                    enum_entry.value,
                });
            }
            try writer.writeAll("};\n\n");
        }
    }
}

pub fn generateMessages(allocator: std.mem.Allocator, ir: *const IR, writer: anytype) !void {
    var messages_iter = ir.messages.iterator();
    while (messages_iter.next()) |entry| {
        const msg = entry.value_ptr;

        if (msg.description) |desc| {
            try writeComment(writer, desc, false);
        }
        try writer.print("pub const {s} = struct {{\n", .{msg.name});
        try writer.print("    pub const MSG_ID = {d};\n", .{msg.id});

        for (msg.fields.items) |field| {
            const ztype = field.c_type.to_zig_primitive(allocator);
            if (field.is_extension) {
                try writer.writeAll("    //Extension Field\n");
            }
            if (field.description) |desc| {
                try writeComment(writer, desc, true);
            }

            if (field.enum_ref) |enm| {
                if (enm.bitmask) {
                    try writer.print("    {s}: enums.{s}.Type,\n\n", .{
                        field.name,
                        enm.name,
                    });
                } else {
                    try writer.print("    {s}: enums.{s},\n\n", .{
                        field.name,
                        enm.name,
                    });
                }
            } else {
                try writer.print("    {s}: {s},\n\n", .{
                    field.name,
                    ztype,
                });
            }
        }
        try writer.writeAll("};\n\n");
    }
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
