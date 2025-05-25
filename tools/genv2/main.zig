const std = @import("std");
const IR = @import("semantic_analysis.zig");
const Loader = @import("loader.zig");
const Codegen = @import("codegen.zig");
const config = @import("config");
const DialectEnum = @import("ast.zig").DialectEnum;
pub fn main() anyerror!void {
    // Prepare output directory
    std.fs.cwd().makeDir(config.dialect_out_dir) catch {
        try std.fs.cwd().deleteTree(config.dialect_out_dir);
        try std.fs.cwd().makeDir(config.dialect_out_dir);
    };

    // Open directory
    var outd = try std.fs.cwd().openDir(config.dialect_out_dir, .{});

    // Create file
    var dFile = try outd.createFile("dialects.zig", .{});
    defer dFile.close();

    const writer = dFile.writer();
    if (std.mem.eql(u8, config.dialect_to_use, "all")) {
        for (std.meta.tags(DialectEnum)) |tag| {
            const path = @tagName(tag);
            try mavgen(path);
            if (std.mem.eql(u8, path[0 .. path.len - 4], "test")) {}
            try writer.print(
                \\pub const {s} = struct {{
                \\    pub const enums    = @import("{s}/enums.zig");
                \\    pub const messages = @import("{s}/messages.zig");
                \\}};
                \\
                \\
                \\
            ,
                .{
                    if (std.mem.eql(u8, path[0 .. path.len - 4], "test")) "Test" else path[0 .. path.len - 4],
                    path[0 .. path.len - 4],
                    path[0 .. path.len - 4],
                },
            );
        }
    } else {
        try mavgen(config.dialect_to_use);
    }
}

pub fn mavgen(path: []const u8) !void {
    var gpa = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    const stdout = std.io.getStdOut().writer();
    const xmlPath = path;

    var loader = Loader.init(allocator);
    // Load and merge includes
    if (try loader.load(xmlPath)) |dialAst| {
        try stdout.print("Parsed AST: version={d}, includes={d}, enums={d}, messages={d}\nDep Tree: {s}\n", .{
            dialAst.version,
            dialAst.includePaths.items.len,
            dialAst.enums.count(),
            dialAst.messages.count(),
            loader.depTree.items,
        });

        // Semantic analysis and IR
        const ir = try IR.buildIR(dialAst, allocator);
        try stdout.print("IR Gen complete: enums={d}, messages={d}\n", .{
            ir.enums.count(),
            ir.messages.count(),
        });

        try Codegen.generate(allocator, &ir, xmlPath);

        // Example: list first 5 enums and messages

        // All done
        try stdout.print("\nSuccess!\n", .{});
    } else {
        try stdout.print("\nError!\n", .{});
    }
}
