const std = @import("std");

pub fn fetchXmlFile(allocator: std.mem.Allocator, xmlFile: []const u8) ![]const u8 {
    const url = try std.fmt.allocPrint(allocator, "https://raw.githubusercontent.com/mavlink/mavlink/refs/heads/master/message_definitions/v1.0/{s}", .{xmlFile});
    // Initialize HTTP client
    var client = std.http.Client{ .allocator = allocator };
    defer client.deinit();
    var arrList = std.ArrayList(u8).init(allocator);
    defer arrList.deinit();
    // Make GET requestc
    const res = try client.fetch(.{
        .method = .GET,
        .location = .{ .uri = try std.Uri.parse(url) },
        .response_storage = .{ .dynamic = &arrList },
    });
    std.debug.print("{s}   -  {d}\n", .{ xmlFile, res.status });
    return arrList.toOwnedSlice();
}
