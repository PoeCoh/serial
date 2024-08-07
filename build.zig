const std = @import("std");
const log = std.log.scoped(.serial_lib__build);

const example_files = [_][]const u8{
    "echo",
    "list",
    "list_port_info",
};

pub fn build(b: *std.Build) void {
    const optimize = b.standardOptimizeOption(.{});
    const target = b.standardTargetOptions(.{});

    const serial_mod = b.addModule("SerialPort", .{
        .root_source_file = b.path("src/SerialPort.zig"),
    });

    const unit_tests = b.addTest(.{
        .root_source_file = b.path("src/tests.zig"),
        .target = target,
        .optimize = optimize,
    });
    const run_unit_tests = b.addRunArtifact(unit_tests);
    const test_step = b.step("test", "Run unit tests");
    test_step.dependOn(&run_unit_tests.step);

    const example_step = b.step("examples", "Build examples");
    {
        for (example_files) |example_name| {
            const example = b.addExecutable(.{
                .name = example_name,
                .root_source_file = b.path(
                    b.fmt("examples/{s}.zig", .{example_name}),
                ),
                .target = target,
                .optimize = optimize,
            });

            // port info only works on Windows!
            // TODO: Linux and MacOS port info support
            if (std.mem.eql(u8, example_name, "list_port_info") and
                example.rootModuleTarget().os.tag != .windows)
            {
                log.warn("skipping example 'list_port_info' - only supported on Windows", .{});
                continue;
            }

            example.root_module.addImport("SerialPort", serial_mod);
            const install_example = b.addInstallArtifact(example, .{});
            example_step.dependOn(&example.step);
            example_step.dependOn(&install_example.step);
        }
    }
}
