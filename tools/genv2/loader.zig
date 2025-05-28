/// Loader for MAVLink dialects: handles recursion and AST merging
const std = @import("std");
const Parser = @import("parser.zig");
const Ast = @import("ast.zig");
const config = @import("config");
const Self = @This();

allocator: std.mem.Allocator,
parsed: std.StringHashMap(*Ast.Dialect),
depTree: std.ArrayList(u8),
/// Initialize a loader with the given allocator
pub fn init(allocator: std.mem.Allocator) Self {
    return .{
        .allocator = allocator,
        .parsed = std.StringHashMap(*Ast.Dialect).init(allocator),
        .depTree = std.ArrayList(u8).init(allocator),
    };
}
/// Load and merge a dialect from `path`. Returns a pointer to the merged AST.
pub fn load(self: *Self, path: []const u8) !?*Ast.Dialect {
    // Return cached if already parsed
    if (self.parsed.get(path) != null) {
        std.debug.print("[loader] Using cached AST for '{s}'\n", .{path});
        return null; //no need to return dep if already exists
    }
    try self.depTree.appendSlice(try std.fmt.allocPrint(self.allocator, "{s}->", .{path}));
    std.debug.print("[loader] Opening file: {s}\n", .{path});

    const totPath = try std.fs.path.join(self.allocator, &.{ config.mavlink_xml_def_dir, path });
    var file = try std.fs.cwd().openFile(totPath, .{});
    defer file.close();

    // Use a large limit (e.g., 16 MB) for big dialects
    const max_file_size = 1024 * 1024;
    const stat = try file.stat();
    if (stat.size > max_file_size) {
        std.debug.print("[loader] File too large: {d} bytes (limit: {d})\n", .{ stat.size, max_file_size });
        return error.FileTooBig;
    }

    std.debug.print("[loader] Reading {d} bytes from '{s}'\n", .{ stat.size, path });
    const xmlData = try file.readToEndAlloc(self.allocator, max_file_size);
    std.debug.print("[loader] Read {d} bytes from '{s}'\n", .{ xmlData.len, path });

    // Parse into raw AST
    std.debug.print("[loader] Parsing XML for '{s}'\n", .{path});
    var parser: Parser = undefined;
    parser.init(xmlData, self.allocator);
    const dialect = try parser.parse();

    // Allocate storage for this dialect
    const dialectPtr = try self.allocator.create(Ast.Dialect);
    dialectPtr.* = dialect;

    // Resolve includes relative to this file's directory
    const dir = std.fs.path.dirname(path) orelse ".";
    if (dialectPtr.includePaths.items.len > 0) {
        std.debug.print("[loader] Resolving includes for '{s}' in dir '{s}'\n", .{ path, dir });
    }
    for (dialectPtr.includePaths.items) |incPath| {
        const incName = @tagName(incPath);
        const incFullPath = try std.fs.path.join(self.allocator, &[_][]const u8{ dir, incName });
        std.debug.print("[loader] Including dialect: {s}\n", .{incFullPath});
        if (try self.load(incFullPath)) |incDialect| {

            // Merge enums (StringHashMap version)
            var enum_it = incDialect.enums.iterator();
            while (enum_it.next()) |entry| {
                try dialectPtr.addEnum(entry.value_ptr.*, incPath);
            }

            // Merge messages (StringHashMap version)
            var msg_it = incDialect.messages.iterator();
            while (msg_it.next()) |entry| {
                try dialectPtr.addMessage(entry.value_ptr.*, incPath);
            }
        }
    }

    std.debug.print("[loader] Caching AST for '{s}'\n", .{path});
    try self.parsed.put(path, dialectPtr);
    return dialectPtr;
}
