const std = @import("std");
const serial = @import("serial.zig");
const builtin = @import("builtin");

test "com0com loopback" {
    var sp1 = try serial.init("COM10", .{});
    defer sp1.close();
    var sp2 = try serial.init("COM11", .{});
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

    try serial.flushSerialPort(sp1, true, true);
    try serial.flushSerialPort(sp2, true, true);
    const hello = "hello";
    _ = try sp1.write(hello);
    var buf = [_]u8{0} ** 100;
    _ = try sp2.read(&buf);
    std.debug.print("read: {s}\n", .{buf[0..]});
    try std.testing.expectEqualStrings(hello, buf[0..]);
}
