const std = @import("std");
const serial = @import("../serial.zig");
const c = @cImport(@cInclude("termios.h"));

pub fn pflush(file: std.fs.File, buffers: Buffers) !void {
    const mode = switch (buffers) {
        .input => c.TCIFLUSH,
        .output => c.TCOFLUSH,
        .both => c.TCIOFLUSH,
    };
    const result = c.tcflush(file.handle, @as(c_int, @intCast(mode)));
    if (result != 0) return error.FlushError;
}

pub const Buffers = enum {
    input,
    output,
    both,
};

pub fn controlPins(file: std.fs.File, pins: serial.ControlPins) !void {
    std.debug.print("control pins: {any}{any}\n", .{pins, file});
}