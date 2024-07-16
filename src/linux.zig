pub const Iterator = @import("linux/Iterator.zig");
pub const InformationIterator = @import("linux/InfoIterator.zig");
const std = @import("std");
const SerialPort = @import("SerialPort.zig");
const serial = @import("serial.zig");
const TIOCMGET = @as(c_int, 0x5415);
const TIOCM_DTR = @as(c_int, 0x002);
const TIOCM_RTS = @as(c_int, 0x004);
const TIOCMSET = @as(c_int, 0x5418);

pub usingnamespace @import("posix.zig");

pub fn flush(self: *SerialPort, buffer: Buffer) !void {
    const result = std.os.linux.syscall3(.ioctl, @as(usize, @bitCast(@as(isize, self.file.handle))), 0x540B, @intFromEnum(buffer));
    if (result != 0) return error.FlushError;
}

const Buffer = enum(usize) {
    input = 0,
    output = 1,
    both = 2,
};

pub fn controlPins(self: *SerialPort, pins: serial.ControlPins) !void {
    var flags: c_int = 0;
    if (std.os.linux.ioctl(self.file.handle, TIOCMGET, @intFromPtr(&flags) != 0))
        return error.Unexpected;
    if (pins.dtr) |pin| {
        if (pin) flags |= TIOCM_DTR else flags &= ~TIOCM_DTR;
    }
    if (pins.rts) |pin| {
        if (pin) flags |= TIOCM_RTS else flags &= ~TIOCM_RTS;
    }
    if (std.os.linux.ioctl(self.file.handle, TIOCMSET, @intFromPtr(&flags)) != 0)
        return error.Unexpected;
}