const std = @import("std");
const Parser = @import("parser.zig");
const Generator = @import("generator.zig");
const Error = @import("error.zig");
const fetch = @import("fetch.zig");

const names = [_][]const u8{
    "ASLUAV.xml",      "AVSSUAS.xml",   "all.xml",         "ardupilotmega.xml",     "common.xml",
    "csAirLink.xml",   "cubepilot.xml", "development.xml", "icarous.xml",           "loweheiser.xml",
    "matrixpilot.xml", "minimal.xml",   "paparazzi.xml",   "python_array_test.xml", "standard.xml",
    "storm32.xml",     "test.xml",      "uAvionix.xml",    "ualberta.xml",
};

pub fn main() !void {
    var gpa = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    const args = try std.process.argsAlloc(allocator);
    defer std.process.argsFree(allocator, args);

    if (args.len < 2) {
        std.debug.print("Usage: {s} <xml_file> <out_dir>\n", .{args[0]});
        return;
    }

    const xml_file = args[1];
    const out_dir = args[2];

    // Prepare output directory
    std.fs.cwd().makeDir(out_dir) catch {
        try std.fs.cwd().deleteTree(out_dir);
        try std.fs.cwd().makeDir(out_dir);
    };

    var outd = try std.fs.cwd().openDir(out_dir, .{});
    defer outd.close();

    var dFile = try outd.createFile("dialects.zig", .{});
    defer dFile.close();
    const writer = dFile.writer();

    if (std.mem.eql(u8, "all", xml_file)) {
        for (names) |name| {
            try processXmlFile(allocator, outd, writer, name);
        }
    } else if (containsSubstring(names[0..], xml_file)) {
        try processXmlFile(allocator, outd, writer, xml_file);
    } else {
        std.debug.print("invalid name {s}", .{xml_file});
        return;
    }
}

fn processXmlFile(allocator: std.mem.Allocator, outd: std.fs.Dir, writer: anytype, xml_file: []const u8) !void {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    const alloc1 = arena.allocator();

    const base_name = xml_file[0 .. xml_file.len - 4];
    const dir_name = if (std.mem.eql(u8, xml_file, "test.xml")) "Test" else base_name;

    try outd.makeDir(base_name);
    var nameDir = try outd.openDir(base_name, .{});
    defer nameDir.close();

    const xml_data = try fetch.fetchXmlFile(alloc1, xml_file);
    defer alloc1.free(xml_data);

    var profile = try Parser.parseProfile(alloc1, xml_data, xml_file);
    defer profile.deinit();
    const profiles = try profile.getAllWithDep();

    try Generator.generateMulti(allocator, profiles, nameDir);

    try writer.print(
        \\pub const {s} = struct {{
        \\    pub const enums    = @import("{s}/enums.zig");
        \\    pub const messages = @import("{s}/messages.zig");
        \\}};
        \\
        \\
        \\
    ,
        .{ dir_name, base_name, base_name },
    );
}

fn containsSubstring(namesP: []const []const u8, search_string: []const u8) bool {
    for (namesP) |name| {
        if (std.mem.eql(u8, name, search_string)) return true;
    }
    return false;
}
