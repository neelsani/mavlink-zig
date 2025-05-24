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
        const exe = b.addExecutable(.{
            .name = "generate",
            .root_source_file = b.path("tools/generator/main.zig"),
            .target = target,
            .optimize = optimize,
        });
        exe.root_module.addImport("mavlink", mavlink);
        exe.root_module.addImport("xml", xml);
        b.installArtifact(exe);

        const run_cmd = b.addRunArtifact(exe);
        run_cmd.addArg(b.option([]const u8, "xml", "XML file to process (default: 'ardupilotmega.xml')") orelse "all");
        run_cmd.addArg(b.option([]const u8, "out", "Output directory (default: 'src/dialects')") orelse "src/dialects");

        const tool_step = b.step("generate", "Generate MAVLink code from XML definitions");
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
