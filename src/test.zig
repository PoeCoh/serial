const std = @import("std");
const serial = @import("serial.zig");

test "com0com loopback" {
    var sp1 = try std.fs.cwd().openFile("\\\\.\\COM10", .{ .mode = .read_write });
    defer sp1.close();
    var sp2 = try std.fs.cwd().openFile("\\\\.\\COM11", .{ .mode = .read_write });
    defer sp2.close();

    try serial.configureSerialPort(sp1, .{
        .baud_rate = 115200,
        .handshake = .none,
        .parity = .none,
        .stop_bits = .one,
        .word_size = .CS8,
    });
    try serial.configureSerialPort(sp2, .{
        .baud_rate = 115200,
        .handshake = .none,
        .parity = .none,
        .stop_bits = .one,
        .word_size = .CS8,
    });

    serial.flushSerialPort(sp1, true, true) catch {};
    serial.flushSerialPort(sp2, true, true) catch {};
    const hello = "hello";
    _ = try sp1.write(hello);
    var buf = [_]u8{0} ** 100;
    _ = try sp2.read(&buf);
    std.debug.print("read: {s}\n", .{buf[0..]});
    try std.testing.expectEqualStrings(hello, buf[0..]);
}
