pub const Iterator = @import("darwin/Iterator.zig");
pub const InformationIterator = @import("darwin/InfoIterator.zig");
const std = @import("std");
const SerialPort = @import("SerialPort.zig");
const serial = @import("serial.zig");
const c = @cImport(@cInclude("termios.h"));
const enumType = @TypeOf(c.TCIOFLUSH);

pub usingnamespace @import("posix.zig");

pub fn flush(self: *SerialPort, buffer: Buffer) !void {
    const result = c.tcflush(self.file.handle, @as(c_int, @intCast(@intFromEnum(buffer))));
    if (result != 0) return error.FlushError;
}

const Buffer = enum(enumType) {
    input = c.TCIFLUSH,
    output = c.TCOFLUSH,
    both = c.TCIOFLUSH,
};

pub fn controlPins(self: *SerialPort, pins: serial.ControlPins) !void {
    std.debug.print("Not Implemented\n{any}{any}\n", .{ self, pins });
}

