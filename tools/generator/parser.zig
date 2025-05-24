const std = @import("std");
const xml = @import("xml");
const log = std.log;
const fetch = @import("fetch.zig");
const Generator = @import("generator.zig");

pub const MavProfile = struct {
    name: []const u8,
    allocator: std.mem.Allocator,
    messages: std.StringHashMap(MavMessage),
    enums: std.StringHashMap(MavEnum),
    directdep: std.ArrayList(MavProfile),
    pub fn deinit(self: *MavProfile) void {
        self.messages.deinit();
        self.enums.deinit();
        for (self.directdep.items) |*x| {
            x.*.deinit();
        }
    }
    pub fn checkDepPulled(self: *MavProfile, depName: []const u8) !bool {
        for (self.directdep.items) |*x| {
            if (std.mem.eql(u8, depName, x.*.name)) {
                return true;
            }
            if (try x.*.checkDepPulled(depName)) {
                return true;
            }
        }
        return false;
    }
    pub fn getAllWithDep(self: *MavProfile) ![]MavProfile {
        var arrList = std.ArrayList(MavProfile).init(self.allocator);
        for (self.directdep.items) |*x| {
            try arrList.appendSlice(try x.*.getAllWithDep());
        }
        try arrList.append(self.*);
        return try arrList.toOwnedSlice();
    }
};

pub const MavMessage = struct {
    id: u32,
    name: []const u8,
    fields: std.ArrayList(MavField),
    extensions: std.ArrayList(MavField), // New field to store extension fields
    description: ?[]const u8,
    has_extensions: bool, // Flag to indicate if message has extensions
};

pub const MavField = struct {
    mavtypeinfo: MavTypeInfo,
    type_str: []const u8,
    name: []const u8,
    enumtype: ?[]const u8,
    description: ?[]const u8,
};

pub const MavEnum = struct {
    name: []const u8,
    entries: std.ArrayList(MavEnumEntry),
    description: ?[]const u8,
    enumtype: ?MavTypeInfo = null,
    bitmask: bool = false,
};

pub const MavEnumEntry = struct {
    value: ?u32,
    name: []const u8,
    description: ?[]const u8,
};

pub const MavType = enum {
    UInt8,
    UInt16,
    UInt32,
    UInt64,
    Int8,
    Int16,
    Int32,
    Int64,
    Float,
    Double,
    Array,
    FixedArray,
};

pub const MavTypeInfo = struct {
    mavtype: MavType,
    array_size: ?usize = null,
    array_child_type: ?MavType = null,
};

pub fn parseProfile(allocator: std.mem.Allocator, xml_data: []const u8, nameXml: []const u8) !MavProfile {
    var processed_deps = std.StringHashMap(void).init(allocator);
    defer processed_deps.deinit();
    return parseProfileWithRegistry(allocator, xml_data, nameXml, &processed_deps);
}

pub fn parseProfileWithRegistry(
    allocator: std.mem.Allocator,
    xml_data: []const u8,
    nameXml: []const u8,
    processed_deps: *std.StringHashMap(void),
) !MavProfile {
    // Check if we've already processed this file
    if (processed_deps.contains(nameXml)) {
        return MavProfile{
            .name = try allocator.dupe(u8, nameXml),
            .allocator = allocator,
            .messages = std.StringHashMap(MavMessage).init(allocator),
            .enums = std.StringHashMap(MavEnum).init(allocator),
            .directdep = std.ArrayList(MavProfile).init(allocator),
        };
    }

    // Mark this file as processed immediately to prevent circular dependencies
    try processed_deps.put(try allocator.dupe(u8, nameXml), {});

    var profile = MavProfile{
        .name = try allocator.dupe(u8, nameXml),
        .allocator = allocator,
        .messages = std.StringHashMap(MavMessage).init(allocator),
        .enums = std.StringHashMap(MavEnum).init(allocator),
        .directdep = std.ArrayList(MavProfile).init(allocator),
    };

    var fb = std.io.fixedBufferStream(xml_data);
    var doc = xml.streamingDocument(allocator, fb.reader());
    defer doc.deinit();
    var reader = doc.reader(allocator, .{});
    defer reader.deinit();
    var current_message: ?MavMessage = null;
    var current_enum: ?MavEnum = null;
    var current_field: ?MavField = null;
    var current_entry: ?MavEnumEntry = null;
    var current_dep: ?[]const u8 = null;
    var in_extensions: bool = false; // Track if we're inside extensions section

    // Flags to track which description we're in
    var description_context: enum {
        None,
        Message,
        Field,
        Enum,
        Entry,
    } = .None;

    while (true) {
        const node = reader.read() catch |err| switch (err) {
            error.MalformedXml => {
                const loc = reader.errorLocation();
                log.err("{}:{}: {}", .{ loc.line, loc.column, reader.errorCode() });
                return error.MalformedXml;
            },
            else => |other| return other,
        };

        switch (node) {
            .eof => break,
            .element_start => {
                const element_name = reader.elementNameNs();
                if (std.mem.eql(u8, element_name.local, "message")) {
                    current_message = MavMessage{
                        .id = 0,
                        .name = "",
                        .fields = std.ArrayList(MavField).init(allocator),
                        .extensions = std.ArrayList(MavField).init(allocator), // Initialize extensions list
                        .description = null,
                        .has_extensions = false,
                    };
                    for (0..reader.reader.attributeCount()) |i| {
                        const attribute_name = reader.attributeNameNs(i);
                        if (std.mem.eql(u8, attribute_name.local, "id")) {
                            current_message.?.id = try std.fmt.parseInt(u32, try reader.attributeValue(i), 10);
                        } else if (std.mem.eql(u8, attribute_name.local, "name")) {
                            current_message.?.name = try profile.allocator.dupe(u8, try reader.attributeValue(i));
                        }
                    }
                } else if (std.mem.eql(u8, element_name.local, "field")) {
                    current_field = MavField{
                        .mavtypeinfo = .{ .mavtype = .UInt8 },
                        .name = "",
                        .enumtype = null,
                        .description = null,
                        .type_str = "",
                    };

                    for (0..reader.reader.attributeCount()) |i| {
                        const attribute_name = reader.attributeNameNs(i);
                        if (std.mem.eql(u8, attribute_name.local, "type")) {
                            const type_str = try reader.attributeValue(i);
                            const type_info = parseMavType(type_str);
                            current_field.?.mavtypeinfo.mavtype = type_info.mavtype;
                            current_field.?.mavtypeinfo.array_size = type_info.array_size;
                            current_field.?.mavtypeinfo.array_child_type = type_info.array_child_type;
                            current_field.?.type_str = try profile.allocator.dupe(u8, type_str);
                        } else if (std.mem.eql(u8, attribute_name.local, "name")) {
                            current_field.?.name = try profile.allocator.dupe(u8, try reader.attributeValue(i));
                        } else if (std.mem.eql(u8, attribute_name.local, "enum")) {
                            current_field.?.enumtype = try profile.allocator.dupe(u8, try reader.attributeValue(i));
                        }
                    }
                } else if (std.mem.eql(u8, element_name.local, "enum")) {
                    current_enum = MavEnum{
                        .name = "",
                        .entries = std.ArrayList(MavEnumEntry).init(allocator),
                        .description = null,
                    };
                    for (0..reader.reader.attributeCount()) |i| {
                        const attribute_name = reader.attributeNameNs(i);
                        if (std.mem.eql(u8, attribute_name.local, "name")) {
                            current_enum.?.name = try profile.allocator.dupe(u8, try reader.attributeValue(i));
                        } else if (std.mem.eql(u8, attribute_name.local, "bitmask")) {
                            current_enum.?.bitmask = std.mem.eql(u8, try reader.attributeValue(i), "true");
                        }
                    }
                } else if (std.mem.eql(u8, element_name.local, "entry")) {
                    current_entry = MavEnumEntry{
                        .value = null,
                        .name = "",
                        .description = null,
                    };
                    for (0..reader.reader.attributeCount()) |i| {
                        const attribute_name = reader.attributeNameNs(i);
                        if (std.mem.eql(u8, attribute_name.local, "name")) {
                            current_entry.?.name = try profile.allocator.dupe(u8, try reader.attributeValue(i));
                        } else if (std.mem.eql(u8, attribute_name.local, "value")) {
                            const x = try reader.attributeValue(i);
                            current_entry.?.value = try parseIntAuto(x);
                        }
                    }
                } else if (std.mem.eql(u8, element_name.local, "description")) {
                    // Set the description context based on the current element we're in
                    if (current_entry != null) {
                        description_context = .Entry;
                    } else if (current_field != null) {
                        description_context = .Field;
                    } else if (current_enum != null) {
                        description_context = .Enum;
                    } else if (current_message != null) {
                        description_context = .Message;
                    }
                } else if (std.mem.eql(u8, element_name.local, "include")) {
                    current_dep = "valed";
                } else if (std.mem.eql(u8, element_name.local, "extensions")) {
                    in_extensions = true;
                    if (current_message) |*msg| {
                        msg.has_extensions = true;
                    }
                }
            },
            .element_end => {
                const element_name = reader.elementNameNs();
                if (std.mem.eql(u8, element_name.local, "message")) {
                    try profile.messages.put(current_message.?.name, current_message.?);
                    current_message = null;
                    in_extensions = false; // Reset extensions flag when message ends
                } else if (std.mem.eql(u8, element_name.local, "field")) {
                    if (in_extensions) {
                        try current_message.?.extensions.append(current_field.?);
                    } else {
                        try current_message.?.fields.append(current_field.?);
                    }
                    current_field = null;
                } else if (std.mem.eql(u8, element_name.local, "enum")) {
                    try profile.enums.put(current_enum.?.name, current_enum.?);
                    current_enum = null;
                } else if (std.mem.eql(u8, element_name.local, "entry")) {
                    try current_enum.?.entries.append(current_entry.?);
                    current_entry = null;
                } else if (std.mem.eql(u8, element_name.local, "description")) {
                    description_context = .None;
                } else if (std.mem.eql(u8, element_name.local, "include")) {
                    current_dep = null;
                }
            },
            .text => {
                const text_content = try reader.text();
                var no_tabs = std.ArrayList(u8).init(allocator);
                defer no_tabs.deinit();

                for (text_content) |c| {
                    if (c != '\t') {
                        try no_tabs.append(c);
                    }
                }

                const trimmed_content = std.mem.trim(u8, no_tabs.items, " \n\r");
                if (trimmed_content.len == 0) {
                    continue;
                }

                if (current_dep != null) {
                    // Check if we've already processed this dependency
                    if (processed_deps.contains(trimmed_content)) {
                        continue;
                    }

                    // Process the dependency
                    const dep_contents = try fetch.fetchXmlFile(allocator, trimmed_content);
                    defer allocator.free(dep_contents);

                    const dep_profile = try parseProfileWithRegistry(
                        allocator,
                        dep_contents,
                        trimmed_content,
                        processed_deps,
                    );
                    try profile.directdep.append(dep_profile);
                }
                // Handle descriptions based on our current context
                switch (description_context) {
                    .Message => {
                        current_message.?.description = try profile.allocator.dupe(u8, trimmed_content);
                    },
                    .Field => {
                        current_field.?.description = try profile.allocator.dupe(u8, trimmed_content);
                    },
                    .Enum => {
                        current_enum.?.description = try profile.allocator.dupe(u8, trimmed_content);
                    },
                    .Entry => {
                        current_entry.?.description = try profile.allocator.dupe(u8, trimmed_content);
                    },
                    .None => {
                        // Text content outside of a description tag but within another element
                        if (current_field != null and current_field.?.description == null) {
                            current_field.?.description = try profile.allocator.dupe(u8, trimmed_content);
                        }
                    },
                }
            },
            else => {},
        }
    }
    std.debug.print("{s} pulled {d} messages and {d} enums and needs {d}\n\n", .{
        nameXml,
        profile.messages.count(),
        profile.enums.count(),
        profile.directdep.items.len,
    });

    return profile;
}

fn parseIntAuto(s: []const u8) !u32 {
    if (std.mem.startsWith(u8, s, "0x")) {
        return std.fmt.parseInt(u32, s[2..], 16);
    } else {
        return std.fmt.parseInt(u32, s, 10);
    }
}

fn parseMavType(type_str: []const u8) MavTypeInfo {
    if (std.mem.eql(u8, type_str, "uint8_t")) {
        return .{
            .mavtype = .UInt8,
        };
    } else if (std.mem.eql(u8, type_str, "uint16_t")) {
        return .{
            .mavtype = .UInt16,
        };
    } else if (std.mem.eql(u8, type_str, "uint32_t")) {
        return .{
            .mavtype = .UInt32,
        };
    } else if (std.mem.eql(u8, type_str, "uint64_t")) {
        return .{
            .mavtype = .UInt64,
        };
    } else if (std.mem.eql(u8, type_str, "int8_t")) {
        return .{
            .mavtype = .Int8,
        };
    } else if (std.mem.eql(u8, type_str, "int16_t")) {
        return .{
            .mavtype = .Int16,
        };
    } else if (std.mem.eql(u8, type_str, "int32_t")) {
        return .{
            .mavtype = .Int32,
        };
    } else if (std.mem.eql(u8, type_str, "int64_t")) {
        return .{
            .mavtype = .Int64,
        };
    } else if (std.mem.eql(u8, type_str, "float")) {
        return .{
            .mavtype = .Float,
        };
    } else if (std.mem.eql(u8, type_str, "double")) {
        return .{
            .mavtype = .Double,
        };
    } else if (std.mem.endsWith(u8, type_str, "]")) {
        // Handle fixed-size arrays like "char[25]" or "float[4]"
        const open_bracket_index = std.mem.indexOf(u8, type_str, "[").?;
        const base_type = type_str[0..open_bracket_index];
        const close_bracket_index = std.mem.indexOf(u8, type_str, "]").?;
        const array_size_str = type_str[open_bracket_index + 1 .. close_bracket_index];
        const array_size = std.fmt.parseInt(usize, array_size_str, 10) catch unreachable;

        // Determine the base type of the array
        const base_mavtype: MavType = if (std.mem.eql(u8, base_type, "float"))
            .Float
        else if (std.mem.eql(u8, base_type, "double"))
            .Double
        else if (std.mem.eql(u8, base_type, "uint8_t"))
            .UInt8
        else if (std.mem.eql(u8, base_type, "uint16_t"))
            .UInt16
        else if (std.mem.eql(u8, base_type, "uint32_t"))
            .UInt32
        else if (std.mem.eql(u8, base_type, "uint64_t"))
            .UInt64
        else if (std.mem.eql(u8, base_type, "int8_t"))
            .Int8
        else if (std.mem.eql(u8, base_type, "int16_t"))
            .Int16
        else if (std.mem.eql(u8, base_type, "int32_t"))
            .Int32
        else if (std.mem.eql(u8, base_type, "int64_t"))
            .Int64
        else if (std.mem.eql(u8, base_type, "char")) // char arrays are typically uint8_t in MAVLink
            .UInt8
        else
            @panic(base_type);

        return .{ .mavtype = .FixedArray, .array_size = array_size, .array_child_type = base_mavtype };
    } else {
        return .{
            .mavtype = .UInt8,
        };
    }
}
