const std = @import("std");
const serial = @import("../serial.zig");
const termios = @cImport(@cInclude("termios.h"));
pub const Iterator = @import("Iterator.zig");
pub const InformationIterator = @import("InformationIterator.zig");

pub const Buffers = enum(c_int) {
    input = termios.TCIFLUSH,
    output = termios.TCOFLUSH,
    both = termios.TCIOFLUSH,
};

pub fn pFlush(file: std.fs.File, buffer: c_int) !void {
    const result = termios.tcflush(file.handle, buffer);
    if (result != 0) return error.FlushError;
}

pub fn pControlPins(file: std.fs.File, pins: serial.ControlPins) !void {
    std.debug.print("Not implemented for mac {any} {any}\n", .{file, pins}); // to make zls shut up
}