const std = @import("std");
const SerialPort = @import("SerialPort.zig");
const builtin = @import("builtin");

test "iterator" {
    var iterator = SerialPort.iterator();
    while (try iterator.next()) |port| {
        _ = port;
    }
}

test "configuration" {
    var iterator = SerialPort.iterator();
    var port_info: SerialPort = undefined;
    while (try iterator.next()) |port| {
        port_info = port;
        break;
    }
    const name = port_info.file;
    var sp = try SerialPort.init(name, .{});
    defer sp.deinit();
}

test "control pins" {
    var iterator = SerialPort.iterator();
    var port_info: SerialPort = undefined;
    // get first port
    while (try iterator.next()) |port| {
        port_info = port;
        break;
    }
    const name = port_info.file;
    var sp = try SerialPort.init(name, .{});
    defer sp.deinit();
    _ = sp.controlPins;
}

test "com0com loopback" {
    var sp1 = try SerialPort.init("COM10", .{});
    defer sp1.deinit();
    var sp2 = try SerialPort.init("COM11", .{});
    defer sp2.deinit();
    try sp1.flush(.both);
    const hello = "hello";
    _ = try sp1.write(hello);
    var buf = [_]u8{0} ** 100;
    _ = try sp2.read(&buf);
    std.debug.print("read: {s}\n", .{buf[0..]});
    try std.testing.expectEqualStrings(hello, buf[0..]);
}
