const std = @import("std");
const xml = @import("xml");
const Ast = @import("ast.zig");
const Self = @This();
doc: xml.StaticDocument,
reader: xml.GenericReader(xml.StaticDocument.Error),
allocator: std.mem.Allocator,

const MavTag = enum {
    include,
    dialect,
    enums,
    messages,
    mavlink,
    @"enum",
    entry,
    description,
    param,
    message,
    field,
    extensions,
    version,
    deprecated,
    wip,
};

pub fn init(xml_bytes: []const u8, allocator: std.mem.Allocator) !*Self {
    // Allocate memory for Self
    var self = try allocator.create(Self);

    // Initialize the document (owns the XML data)
    self.doc = xml.StaticDocument.init(xml_bytes);

    // Initialize the reader (references memory inside doc)
    self.reader = self.doc.reader(allocator, .{});

    self.allocator = allocator;
    return self;
}

pub fn parse(self: *Self) !Ast.Dialect {
    var ast = Ast.Dialect.init(self.allocator);
    while (true) {
        const node = self.reader.read() catch |err| switch (err) {
            error.MalformedXml => {
                const loc = self.reader.errorLocation();
                std.log.err("{}:{}: {}", .{ loc.line, loc.column, self.reader.errorCode() });
                return error.MalformedXml;
            },
            else => |other| {
                return other;
            },
        };
        //std.debug.print("{s}\n", .{@tagName(node)});
        switch (node) {
            .element_start => {
                const elName = self.reader.elementName();
                switch (std.meta.stringToEnum(MavTag, elName) orelse @panic(elName)) {
                    .include => {
                        const thed = try self.reader.readElementTextAlloc(self.allocator);
                        const path = std.meta.stringToEnum(Ast.DialectEnum, thed) orelse @panic(thed);
                        try ast.includePaths.append(path);
                    },
                    .dialect => {
                        const verText = try self.reader.readElementTextAlloc(self.allocator);
                        ast.version = try std.fmt.parseInt(u32, verText, 10);
                        self.allocator.free(verText);
                    },
                    .enums => {
                        try self.parseEnums(&ast);
                    },

                    .messages => {
                        try self.parseMessages(&ast);
                    },
                    else => {},
                }
            },

            .element_end => {
                //if (std.mem.eql(u8, self.reader.elementName(), "mavlink")) break;
            },
            .eof => break,
            else => {},
        }
    }
    return ast;
}

fn parseEnums(self: *Self, ast: *Ast.Dialect) !void {
    while (true) {
        const node = try self.reader.read();
        switch (node) {
            .element_start => {
                const elName = self.reader.elementName();
                switch (std.meta.stringToEnum(MavTag, elName) orelse @panic("unhandled type")) {
                    .@"enum" => {
                        const theEnum = try self.parseEnum();
                        try ast.enums.put(theEnum.name, theEnum);
                    },
                    else => {
                        try self.reader.skipElement();
                    },
                }
            },
            .element_end => {
                if (std.mem.eql(u8, self.reader.elementName(), "enums")) break;
            },
            .eof => break,
            else => {},
        }
    }
}

// Parse a single <enum name="...">...</enum>
fn parseEnum(self: *Self) !Ast.Enum {
    const name = self.getAttr("name") orelse @panic("name should be here");
    var enm = Ast.Enum.init(name, self.allocator);
    if (self.getAttr("bitmask")) |bm| {
        if (std.mem.eql(u8, bm, "true")) {
            enm.bitmask = true;
        }
    }

    if (self.getAttr("display")) |bm| {
        if (std.mem.eql(u8, bm, "bitmask")) {
            enm.bitmask = true;
        }
    }

    while (true) {
        const node = try self.reader.read();
        switch (node) {
            .element_start => {
                switch (std.meta.stringToEnum(MavTag, self.reader.elementName()) orelse @panic("unhandled type")) {
                    .entry => {
                        if (try self.parseEnumEntry()) |enEnt| {
                            try enm.entries.append(enEnt);
                        }
                    },
                    .description => {
                        if (try clean(self.allocator, try self.reader.readElementTextAlloc(self.allocator))) |desc| {
                            enm.description = desc;
                        }
                    },
                    else => {
                        try self.reader.skipElement();
                    },
                }
            },
            .element_end => {
                switch (std.meta.stringToEnum(MavTag, self.reader.elementName()) orelse @panic("unhandled type")) {
                    .@"enum" => {
                        break;
                    },
                    else => {},
                }
            },
            .eof => break,
            else => {},
        }
    }
    return enm;
}
// Parse a single <entry value="..." name="..."> with optional <description> and <param> children
fn parseEnumEntry(self: *Self) !?Ast.EnumEntry {
    const name = self.getAttr("name") orelse @panic("");
    const valueText = self.getAttr("value") orelse @panic("");

    const value = try parseIntAuto(valueText);

    var entry = Ast.EnumEntry.init(name, value, null, self.allocator);

    while (true) {
        const node = try self.reader.read();
        switch (node) {
            .element_start => {
                const tag = self.reader.elementName();
                switch (std.meta.stringToEnum(MavTag, tag) orelse @panic(tag)) {
                    .description => {
                        if (try clean(self.allocator, try self.reader.readElementTextAlloc(self.allocator))) |desc| {
                            entry.description = desc;
                        }
                    },
                    .param => {
                        //implement later
                    },
                    .wip => {
                        entry.deinit(self.allocator);
                        return null;
                    },
                    else => {
                        try self.reader.skipElement();
                    },
                }
            },
            .element_end => {
                if (std.mem.eql(u8, self.reader.elementName(), "entry")) break;
            },
            .eof => break,
            else => {},
        }
    }
    return entry;
}

// Parse the <messages> section
fn parseMessages(self: *Self, ast: *Ast.Dialect) !void {
    while (true) {
        const node = try self.reader.read();
        switch (node) {
            .element_start => {
                switch (std.meta.stringToEnum(MavTag, self.reader.elementName()) orelse @panic("unhandled type")) {
                    .message => {
                        if (try parseMessage(self)) |msg| {
                            try ast.messages.put(msg.name, msg);
                        }
                    },
                    else => {
                        try self.reader.skipElement();
                    },
                }
            },
            .element_end => {
                if (std.mem.eql(u8, self.reader.elementName(), "messages")) break;
            },
            .eof => break,
            else => {},
        }
    }
}

// Parse a single <message id="..." name="..."> with multiple <field> children
fn parseMessage(self: *Self) !?Ast.Message {
    const idText = self.getAttr("id") orelse @panic("");
    const id = try std.fmt.parseInt(u24, idText, 10);
    const name = self.getAttr("name") orelse @panic("");
    var msg = Ast.Message.init(id, name, self.allocator);
    var in_extensions = false;
    while (true) {
        const node = try self.reader.read();
        switch (node) {
            .element_start => {
                const tag = self.reader.elementName();
                switch (std.meta.stringToEnum(MavTag, tag) orelse @panic("unhandled type")) {
                    .field => {
                        var field = try self.parseField();
                        field.is_extension = in_extensions;
                        try msg.fields.append(field);
                    },
                    .description => {
                        if (try clean(self.allocator, try self.reader.readElementTextAlloc(self.allocator))) |desc| {
                            msg.description = desc;
                        }
                    },
                    .wip => {
                        return null;
                    },
                    .extensions => {
                        in_extensions = true;
                    },
                    else => {
                        try self.reader.skipElement();
                    },
                }
            },
            .element_end => {
                if (std.mem.eql(u8, self.reader.elementName(), "message")) break;
            },
            .eof => break,
            else => {},
        }
    }
    var basefields = std.ArrayList(Ast.Field).init(self.allocator);
    var extfields = std.ArrayList(Ast.Field).init(self.allocator);

    for (msg.fields.items) |fld| {
        if (fld.is_extension) {
            try extfields.append(fld);
        } else {
            try basefields.append(fld);
        }
    }
    const baseslice = try basefields.toOwnedSlice();
    msg.fields.clearRetainingCapacity();
    std.mem.sort(Ast.Field, baseslice, {}, Ast.Field.compareByCtype);
    try msg.fields.appendSlice(baseslice);
    try msg.fields.appendSlice(try extfields.toOwnedSlice());
    return msg;
}

// Parse a <field type="..." name="..." enum="...?" units="...?">Description text</field>
fn parseField(self: *Self) !Ast.Field {
    var theField = Ast.Field{
        .ctype = .{ .primitive = .uint8_t },
        .name = "",
    };
    const typ_str = self.getAttr("type") orelse @panic("");
    const typeEnum: Ast.c_type = blk: {
        if (std.meta.stringToEnum(Ast.PrimitiveType, typ_str)) |val| {
            break :blk Ast.c_type{
                .primitive = val,
            };
        }
        if (std.mem.endsWith(u8, typ_str, "]")) {
            // Handle fixed-size arrays like "char[25]" or "float[4]"
            const open_bracket_index = std.mem.indexOf(u8, typ_str, "[").?;
            const base_type = typ_str[0..open_bracket_index];
            const close_bracket_index = std.mem.indexOf(u8, typ_str, "]").?;
            const array_size_str = typ_str[open_bracket_index + 1 .. close_bracket_index];
            const array_size = std.fmt.parseInt(usize, array_size_str, 10) catch unreachable;
            const arrType = std.meta.stringToEnum(Ast.PrimitiveType, base_type) orelse @panic(base_type);
            const theType: Ast.c_type = .{ .array = .{ .ctype = arrType, .len = array_size } };
            break :blk theType;
        }
        // Should not reach here
        @panic(typ_str);
    };

    theField.ctype = typeEnum;
    theField.name = self.getAttr("name") orelse @panic("");

    if (self.getAttr("enum")) |enmname| {
        theField.enumname = enmname;
    }

    if (self.getAttr("display")) |disp| {
        theField.display = disp;
    }

    theField.description = try self.reader.readElementTextAlloc(self.allocator);
    return theField;
}

// Helper: get an attribute or fail
fn getAttr(self: *Self, name: []const u8) ?[]const u8 {
    const idx = self.reader.attributeIndex(name) orelse return null;
    return self.reader.attributeValue(idx) catch @panic("read error");
}

fn parseIntAuto(s: []const u8) !usize {
    if (std.mem.startsWith(u8, s, "0x")) {
        return std.fmt.parseInt(usize, s[2..], 16);
    } else {
        return std.fmt.parseInt(usize, s, 10);
    }
}
pub fn clean(allocator: std.mem.Allocator, input: []const u8) !?[]u8 {
    // Allocate a temporary buffer to strip out tabs
    var tmp = try allocator.alloc(u8, input.len);

    var count: usize = 0;
    for (input) |c| {
        if (c != '\t') {
            tmp[count] = c;
            count += 1;
        }
    }

    // Trim leading/trailing spaces, newlines, carriage returns
    const noTabs = tmp[0..count];
    const trimmed = std.mem.trim(u8, noTabs, " \n\r");

    // If nothing left after trimming, return null
    if (trimmed.len == 0) {
        return null;
    }

    // Allocate exact-sized result buffer and copy trimmed content
    const result = try allocator.alloc(u8, trimmed.len);
    std.mem.copyForwards(u8, result, trimmed);
    return result;
}
