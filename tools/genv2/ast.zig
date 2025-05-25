const std = @import("std");

pub const DialectEnum = enum {
    @"ASLUAV.xml",
    @"AVSSUAS.xml",
    @"all.xml",
    @"ardupilotmega.xml",
    @"common.xml",
    @"csAirLink.xml",
    @"cubepilot.xml",
    @"development.xml",
    @"icarous.xml",
    @"loweheiser.xml",
    @"matrixpilot.xml",
    @"minimal.xml",
    @"paparazzi.xml",
    @"python_array_test.xml",
    @"standard.xml",
    @"storm32.xml",
    @"test.xml",
    @"uAvionix.xml",
    @"ualberta.xml",
};
pub const PrimitiveType = enum {
    uint8_t,
    uint16_t,
    uint32_t,
    uint64_t,
    int8_t,
    int16_t,
    int32_t,
    int64_t,
    float,
    double,
    char,
    uint8_t_mavlink_version,
};

pub const c_type = union(enum) {
    primitive: PrimitiveType,
    array: struct {
        ctype: PrimitiveType,
        len: usize,
    },

    pub fn to_zig_primitive(self: c_type, allocator: std.mem.Allocator) []const u8 {
        return switch (self) {
            .primitive => |prim| switch (prim) {
                .uint8_t, .uint8_t_mavlink_version, .char => "u8",
                .uint16_t => "u16",
                .uint32_t => "u32",
                .uint64_t => "u64",
                .int8_t => "i8",
                .int16_t => "i16",
                .int32_t => "i32",
                .int64_t => "i64",
                .float => "f32",
                .double => "f64",
            },
            .array => |arr| blk: {
                const rT: c_type = .{ .primitive = arr.ctype };
                const elem_type = rT.to_zig_primitive(allocator);
                break :blk std.fmt.allocPrint(allocator, "[{d}]{s}", .{ arr.len, elem_type }) catch unreachable;
            },
        };
    }
};

pub const Dialect = struct {
    allocator: std.mem.Allocator,
    includePaths: std.ArrayList(DialectEnum),
    version: usize = 0,
    enums: std.StringHashMap(Enum),
    messages: std.StringHashMap(Message),

    pub fn init(
        allocator: std.mem.Allocator,
    ) Dialect {
        return .{
            .allocator = allocator,
            .includePaths = std.ArrayList(DialectEnum).init(allocator),
            .enums = std.StringHashMap(Enum).init(allocator),
            .messages = std.StringHashMap(Message).init(allocator),
        };
    }

    pub fn addMessage(self: *Dialect, msg: Message, incDialect: DialectEnum) !void {
        if (self.messages.get(msg.name) != null) {
            std.debug.print("duplicate message attempted to be added: {s} from {s} panicing...\n", .{ msg.name, @tagName(incDialect) });
            @panic(msg.name);
        } else {
            try self.messages.put(msg.name, msg);
        }
    }

    pub fn addEnum(self: *Dialect, enm: Enum, incDialect: DialectEnum) !void {
        if (self.enums.getEntry(enm.name)) |entry| {
            std.debug.print("duplicate enum attempted to be added: {s} from {s}\nAttempting merge...\n", .{ enm.name, @tagName(incDialect) });
            try entry.value_ptr.merge(enm);
        } else {
            try self.enums.put(enm.name, enm);
        }
    }

    pub fn deinit(self: *Dialect) void {
        // No need to free enum values (DialectEnum) as they're not heap-allocated
        self.includePaths.deinit();

        for (self.enums.items) |*en| {
            en.deinit();
        }
        self.enums.deinit();

        for (self.messages.items) |*msg| {
            msg.deinit();
        }
        self.messages.deinit();
    }
};

pub const Enum = struct {
    name: []const u8,
    description: ?[]const u8 = null,
    entries: std.ArrayList(EnumEntry),
    c_type: ?c_type = null,
    bitmask: bool = false,

    pub fn init(name: []const u8, allocator: std.mem.Allocator) Enum {
        // Copy name to heap to own it
        const name_copy = allocator.dupe(u8, name) catch unreachable;
        return Enum{
            .name = name_copy,
            .entries = std.ArrayList(EnumEntry).init(allocator),
        };
    }

    pub fn deinit(self: *Enum, allocator: std.mem.Allocator) void {
        allocator.free(self.name);

        for (self.entries.items) |*entry| {
            entry.deinit(allocator);
        }
        self.entries.deinit();
    }

    /// Attempts to combine two enums with the same name, checking for duplicate entries
    /// Returns error if enums have different names or duplicate entries are found
    pub fn merge(self: *Enum, other: Enum) !void {
        std.debug.print("[enum merge] Attempting to merge enum '{s}' with '{s}'\n", .{ self.name, other.name });

        if (!std.mem.eql(u8, self.name, other.name)) {
            std.debug.print("[enum merge] ERROR: Name mismatch ('{s}' != '{s}')\n", .{ self.name, other.name });
            return error.EnumNameMismatch;
        }

        std.debug.print("[enum merge] Checking {} entries against {} existing entries\n", .{ other.entries.items.len, self.entries.items.len });

        for (other.entries.items) |other_entry| {
            std.debug.print("[enum merge] Processing entry '{s}' (value: {d})\n", .{ other_entry.name, other_entry.value });

            // Check for duplicate entries
            for (self.entries.items) |self_entry| {
                if (std.mem.eql(u8, self_entry.name, other_entry.name) and
                    self_entry.value == other_entry.value)
                {
                    std.debug.print("[enum merge] ERROR: Duplicate entry '{s}' with value {d} already exists\n", .{ other_entry.name, other_entry.value });
                    return error.DuplicateEntry;
                }
            }

            // If no duplicates found, add the entry
            std.debug.print("[enum merge] Adding new entry '{s}' (value: {d})\n", .{ other_entry.name, other_entry.value });
            try self.entries.append(other_entry);
        }

        // Merge descriptions if one is missing
        if (self.description == null and other.description != null) {
            std.debug.print("[enum merge] Adding description from other enum\n", .{});
            self.description = other.description;
        } else if (self.description != null and other.description != null) {
            std.debug.print("[enum merge] Both enums have descriptions, keeping original\n", .{});
        }

        // Merge c_type if one is missing
        if (self.c_type == null and other.c_type != null) {
            std.debug.print("[enum merge] Adding c_type from other enum\n", .{});
            self.c_type = other.c_type;
        } else if (self.c_type != null and other.c_type != null) {
            std.debug.print("[enum merge] Both enums have c_types, keeping original\n", .{});
        }

        // Combine bitmask flags
        const old_bitmask = self.bitmask;
        self.bitmask = self.bitmask or other.bitmask;
        if (self.bitmask != old_bitmask) {
            std.debug.print("[enum merge] Updated bitmask flag from {} to {}\n", .{ old_bitmask, self.bitmask });
        }

        std.debug.print("[enum merge] Successfully merged enum '{s}'. Total entries now: {}\n", .{ self.name, self.entries.items.len });
    }
};

pub const EnumEntry = struct {
    name: []const u8,
    value: usize,
    description: ?[]const u8 = null,
    params: ?std.ArrayList([]const u8) = null,

    pub fn init(name: []const u8, value: usize, description: ?[]const u8, allocator: std.mem.Allocator) EnumEntry {
        const name_copy = allocator.dupe(u8, name) catch unreachable;
        var desc_copy: ?[]const u8 = null;
        if (description) |desc| {
            desc_copy = allocator.dupe(u8, desc) catch unreachable;
        }
        return EnumEntry{
            .name = name_copy,
            .value = value,
            .description = desc_copy,
        };
    }

    pub fn deinit(self: *EnumEntry, allocator: std.mem.Allocator) void {
        allocator.free(self.name);
        if (self.description) |desc| {
            allocator.free(desc);
        }
    }
};

pub const Message = struct {
    id: u24,
    name: []const u8,
    description: ?[]const u8 = null,
    fields: std.ArrayList(Field),

    allocator: std.mem.Allocator,

    pub fn init(id: u24, name: []const u8, allocator: std.mem.Allocator) Message {
        // Copy name to heap to own it
        return Message{
            .id = id,
            .name = name,
            .fields = std.ArrayList(Field).init(allocator),
            .allocator = allocator,
        };
    }
};

pub const Field = struct {
    ctype: c_type,
    name: []const u8,
    enumname: ?[]const u8 = null,
    description: ?[]const u8 = null,
    is_extension: bool = false,
    display: ?[]const u8 = null,
};
