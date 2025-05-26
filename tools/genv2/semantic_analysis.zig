const std = @import("std");
const Ast = @import("ast.zig");

const reserved_keywords = [_][]const u8{
    "align",       "allowzero",      "and",     "anyframe", "anytype", "asm",         "async",  "await",    "break",
    "catch",       "comptime",       "const",   "continue", "defer",   "else",        "enum",   "errdefer", "error",
    "export",      "extern",         "false",   "fn",       "for",     "if",          "inline", "noalias",  "noinline",
    "nosuspend",   "null",           "opaque",  "or",       "orelse",  "packed",      "pub",    "resume",   "return",
    "linksection", "struct",         "suspend", "switch",   "test",    "threadlocal", "true",   "try",      "union",
    "unreachable", "usingnamespace", "var",     "volatile", "while",
};

fn escapeZigIdent(ident: []const u8, allocator: std.mem.Allocator) ![]const u8 {
    for (reserved_keywords) |kw| {
        if (std.mem.eql(u8, kw, ident)) {
            return try std.fmt.allocPrint(allocator, "@\"{s}\"", .{ident});
        }
    }
    return ident;
}

pub const IR = struct {
    enums: std.StringHashMap(IR_Enum),
    messages: std.StringHashMap(IR_Message),

    pub fn deinit(self: *IR) void {
        self.enums.deinit();
        self.messages.deinit();
    }
};

pub const IR_Enum = struct {
    name: []const u8,
    description: ?[]const u8,
    c_type: Ast.c_type,
    entries: []const Ast.EnumEntry,
    bitmask: bool,
};

pub const IR_Field = struct {
    name: []const u8,
    c_type: Ast.c_type,
    enum_ref: ?*IR_Enum,
    description: ?[]const u8,
    is_extension: bool,
    display: ?[]const u8,
};

pub const IR_Message = struct {
    name: []const u8,
    id: u24,
    description: ?[]const u8,
    fields: std.ArrayList(IR_Field),
};

fn getPrimitiveFromCtype(ctype: Ast.c_type) Ast.PrimitiveType {
    return switch (ctype) {
        .primitive => |p| p,
        .array => |arr| arr.ctype,
    };
}

pub fn buildIR(ast: *Ast.Dialect, allocator: std.mem.Allocator) !IR {
    var ir = IR{
        .enums = std.StringHashMap(IR_Enum).init(allocator),
        .messages = std.StringHashMap(IR_Message).init(allocator),
    };

    // 1. Prepare a table to collect enum primitive types as we discover them
    var enum_types = std.StringHashMap(Ast.PrimitiveType).init(allocator);

    // 2. First pass: scan all fields, collect primitive types for enums
    var msg_iter = ast.messages.iterator();
    while (msg_iter.next()) |entry| {
        const ast_msg = entry.value_ptr;
        for (ast_msg.fields.items) |field| {
            if (field.enumname) |enum_name| {
                const field_prim = getPrimitiveFromCtype(field.ctype);
                if (enum_types.get(enum_name)) |existing_prim| {
                    if (existing_prim != field_prim) {
                        std.debug.print(
                            "[semantic] INFO: Enum '{s}' for message {s} referenced with conflicting primitive types: '{any}' vs '{any}'\n[semantic] [INFO]: Will Replace...\n",
                            .{ enum_name, ast_msg.name, existing_prim, field_prim },
                        );
                    }
                } else {
                    try enum_types.put(enum_name, field_prim);
                }
            }
        }
    }

    // 3. Build IR enums, assigning types from collected table and escaping names
    var enum_iter = ast.enums.iterator();
    while (enum_iter.next()) |entry| {
        const ast_enum = entry.value_ptr;
        const enum_prim = enum_types.get(ast_enum.name) orelse blk: {
            std.debug.print(
                "[semantic] WARNING: Enum '{s}' is never referenced by any field, leaving type unset (defaulting to uint32_t)\n",
                .{ast_enum.name},
            );
            break :blk Ast.PrimitiveType.uint32_t;
        };
        const safe_enum_name = try escapeZigIdent(ast_enum.name, allocator);
        try ir.enums.put(
            safe_enum_name,
            IR_Enum{
                .name = safe_enum_name,
                .description = ast_enum.description,
                .c_type = .{ .primitive = enum_prim },
                .entries = ast_enum.entries.items,
                .bitmask = ast_enum.bitmask,
            },
        );
    }

    // 4. Build IR messages with resolved fields and escaped names
    msg_iter = ast.messages.iterator();
    while (msg_iter.next()) |entry| {
        const ast_msg = entry.value_ptr;
        var ir_fields = std.ArrayList(IR_Field).init(allocator);

        for (ast_msg.fields.items) |field| {
            var enum_ref: ?*IR_Enum = null;

            if (field.enumname) |enum_name| {
                const safe_enum_name = try escapeZigIdent(enum_name, allocator);
                if (ir.enums.getPtr(safe_enum_name)) |found_enum| {
                    if (field.display) |disp| {
                        if (std.mem.eql(u8, disp, "bitmask")) {
                            found_enum.*.bitmask = true;
                        }
                        if (found_enum.bitmask) {
                            found_enum.c_type = switch (field.ctype) {
                                .array => |arr| Ast.c_type{ .primitive = arr.ctype },
                                .primitive => field.ctype,
                            };
                        }
                    }

                    enum_ref = found_enum;
                } else {
                    {
                        std.debug.print(
                            "[semantic] ERROR: Field '{s}.{s}' references undefined enum '{s}'\n",
                            .{ ast_msg.name, field.name, enum_name },
                        );
                        return error.UndefinedEnumReference;
                    }
                }
            }

            const safe_field_name = try escapeZigIdent(field.name, allocator);

            //temp fix will do later causes problems with serde
            // if (switch (field.ctype) {
            //     .array => true,
            //     else => false, // Handles all other cases
            //  } or (enum_ref != null and enum_ref.?.bitmask)) {
            //      enum_ref = null;
            //  }

            try ir_fields.append(IR_Field{
                .name = safe_field_name,
                .c_type = field.ctype,
                .enum_ref = enum_ref,
                .description = field.description,
                .is_extension = field.is_extension,
                .display = field.display,
            });
        }

        const safe_msg_name = try escapeZigIdent(ast_msg.name, allocator);

        try ir.messages.put(safe_msg_name, IR_Message{
            .name = safe_msg_name,
            .id = ast_msg.id,
            .description = ast_msg.description,
            .fields = ir_fields,
        });
    }

    return ir;
}
