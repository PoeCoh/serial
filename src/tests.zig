const std = @import("std");
const SerialPort = @import("SerialPort.zig");
const native_os = @import("builtin").os.tag;

fn getFirstPort() ![]const u8 {
    const iterator = try SerialPort.iterate();
    while (try iterator.next()) |port| {
        return port.display_name;
    }
    return error.NoPortFound;
}

test "basic configuration test" {
    var port = try SerialPort.init("COM3", .{});
    defer port.deinit();
}

test "basic flush test" {
    var port = try SerialPort.init("COM3", .{});
    defer port.deinit();
    try port.flush(.input);
    try port.flush(.output);
    try port.flush(.both);
}

test "iterator test" {
    var iterator = try SerialPort.iterate();
    defer iterator.deinit();
    while (try iterator.next()) |port| {
        std.debug.print("{s}\n", .{port.display_name});
    }
}