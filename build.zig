const std = @import("std");

pub fn build(b: *std.Build) void {
    // Standard build options
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    // Dependencies
    const xml = b.dependency("xml", .{
        .target = target,
        .optimize = optimize,
    }).module("xml");
    const buildExamples: bool = b.option(bool, "examples", "Build examples?") orelse true;

    // Main module
    const mavlink = b.addModule("mavlink", .{
        .root_source_file = b.path("src/root.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{.{
            .name = "xml",
            .module = xml,
        }},
    });

    const slib = b.addStaticLibrary(.{
        .name = "mavlink",
        .root_source_file = b.path("src/root.zig"),
        .target = target,
        .optimize = optimize,
    });

    b.installArtifact(slib);
    const install_docs = b.addInstallDirectory(.{
        .source_dir = slib.getEmittedDocs(),
        .install_dir = .prefix,
        .install_subdir = "docs",
    });

    const docs_step = b.step("docs", "Install docs into zig-out/docs");
    docs_step.dependOn(&install_docs.step);

    const shlib = b.addSharedLibrary(.{
        .name = "mavlink",
        .root_source_file = b.path("src/root.zig"),
        .target = target,
        .optimize = optimize,
    });
    b.installArtifact(shlib);

    // Test configuration
    const test_step = b.step("test", "Run all tests");
    const test_files = [_][]const u8{
        "src/serde.zig",
        "src/crc.zig",
        "src/v2.zig",
        // Add more test files as needed
    };

    for (test_files) |test_file| {
        const test_exe = b.addTest(.{
            .root_source_file = b.path(test_file),
            .target = target,
            .optimize = optimize,
        });
        test_exe.root_module.addImport("mavlink", mavlink);
        test_step.dependOn(&b.addRunArtifact(test_exe).step);
    }

    { // The dialect zig generator
        const genoptions = b.addOptions();
        const mav_def_dir_opt = b.option([]const u8, "mavlink_xml_def_dir", "The output directory of the definitions") orelse b.path("mavlink/message_definitions/v1.0").getPath(b);
        genoptions.addOption([]const u8, "dialect_to_use", blk: {
            if (b.option([]const u8, "dialect_to_use", "The dialect to use or (all) for all of them")) |val| {
                if (!std.mem.eql(u8, val, "all") and !std.mem.endsWith(u8, val, ".xml")) {
                    break :blk b.fmt("{s}.xml", .{val});
                } else {
                    break :blk val;
                }
            }
            break :blk "all";
        });
        genoptions.addOption([]const u8, "dialect_out_dir", b.option([]const u8, "dialect_out_dir", "The output directory of the generated dialects") orelse b.path("src/dialects").getPath(b));
        genoptions.addOption([]const u8, "mavlink_xml_def_dir", mav_def_dir_opt);
        genoptions.addOption([]const [:0]const u8, "available_dialects", getDialects(b, mav_def_dir_opt));
        const exe = b.addExecutable(.{
            .name = "genv2",
            .root_source_file = b.path("tools/genv2/main.zig"),
            .target = target,
            .optimize = optimize,
        });
        exe.root_module.addImport("mavlink", mavlink);
        exe.root_module.addImport("xml", xml);
        exe.root_module.addOptions("config", genoptions);
        b.installArtifact(exe);

        const run_cmd = b.addRunArtifact(exe);
        const tool_step = b.step("genv2", "Generate MAVLink code from XML definitions");
        tool_step.dependOn(&run_cmd.step);
    }

    // Examples configuration
    const examples = [_]struct {
        name: []const u8,
        file: []const u8,
        desc: []const u8,
    }{
        .{
            .name = "net",
            .file = "examples/net/main.zig",
            .desc = "Network example",
        },
        .{
            .name = "send_gcs_log",
            .file = "examples/send_gcs_log/main.zig",
            .desc = "GCS log sender example",
        },
        .{
            .name = "stdout",
            .file = "examples/stdout/main.zig",
            .desc = "Stdout example",
        },
        .{
            .name = "example1",
            .file = "examples/example1/main.zig",
            .desc = "Example 1",
        },
        .{
            .name = "params",
            .file = "examples/params/main.zig",
            .desc = "Parameters example",
        },
    };
    if (buildExamples) {
        for (examples) |example| {
            const exe = b.addExecutable(.{
                .name = example.name,
                .root_source_file = b.path(example.file),
                .target = target,
                .optimize = optimize,
            });
            exe.root_module.addImport("mavlink", mavlink);
            b.installArtifact(exe);

            const run_cmd = b.addRunArtifact(exe);
            const example_step = b.step(example.name, example.desc);
            example_step.dependOn(&run_cmd.step);
        }
    }
}

fn getDialects(b: *std.Build, defPath: []const u8) []const [:0]const u8 {
    var dir = b.build_root.handle.openDir(defPath, .{ .iterate = true }) catch |err| {
        std.debug.print("Failed to open directory: {}\n", .{err});
        return &.{};
    };
    defer dir.close();

    var file_names = std.ArrayList([:0]const u8).init(b.allocator);
    var it = dir.iterate();
    while (it.next() catch null) |entry| {
        if (!std.mem.endsWith(u8, entry.name, ".xml")) continue;

        // Create null-terminated copy
        const name = b.allocator.dupeZ(u8, entry.name) catch @panic("Allocation failed");
        file_names.append(name) catch @panic("Allocation failed");
    }

    if (file_names.items.len == 0) {
        std.debug.print("Warning: No XML files found in directory: {s}\n", .{defPath});
        // Create null-terminated fallback
        return &.{};
    }

    return file_names.toOwnedSlice() catch @panic("Allocation failed");
}
