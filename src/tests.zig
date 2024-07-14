const std = @import("std");
const SerialPort = @import("SerialPort");
const native_os = @import("builtin").os.tag;

pub fn getPortName() []const u8 {
    return switch (native_os) {
        .windows => "\\\\.\\COM3",
        .macos => "/dev/cu.usbmodem101",
        else => "/dev/ttyUSB0",
    };
}

test "iterate ports" {
    var it = try SerialPort.iterator();
    while (try it.next()) |port| {
        std.debug.print("{s} (file: {s}, driver: {s})\n", .{ port.display_name, port.file_name, port.driver });
    }
}

test "basic configuration test" {
    var sp = SerialPort.init(getPortName(), .{
        .baud_rate = 115200,
        .word_size = .CS8,
        .parity = .none,
        .stop_bits = .one,
        .handshake = .none,
    });
    try sp.open();
    sp.close();
}

test "basic flush test" {
    var sp = SerialPort.init(getPortName(), .{
        .baud_rate = 115200,
        .word_size = .CS8,
        .parity = .none,
        .stop_bits = .one,
        .handshake = .none,
    });
    try sp.open();
    defer sp.close();

    try sp.flush(.input);
    try sp.flush(.output);
    try sp.flush(.both);
}

test "change control pins" {
    var sp = SerialPort.init(getPortName(), .{
        .baud_rate = 115200,
        .word_size = .CS8,
        .parity = .none,
        .stop_bits = .one,
        .handshake = .none,
    });
    try sp.open();
    defer sp.close();

    try sp.controlPins(.{
        .rts = true,
        .dtr = true,
    });
}