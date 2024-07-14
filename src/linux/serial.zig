const std = @import("std");
const serial = @import("../serial.zig");
pub const Iterator = @import("Iterator.zig");
pub const InformationIterator = @import("InformationIterator.zig");

pub const Buffers = enum(comptime_int) {
    input = 0,
    output = 1,
    both = 3,
};

pub fn pFlush(file: std.fs.File, buffer: comptime_int) !void {
    const result = std.os.linux.syscall3(.ioctl, @as(usize, @bitCast(@as(isize, file.handle))), 0x540B, buffer);
    if (result != 0) return error.FlushError;
}

pub fn pControlPins(file: std.fs.File, pins: serial.ControlPins) !void {
    const TIOCM_RTS: c_int = 0x004;
    const TIOCM_DTR: c_int = 0x002;

    // from /usr/include/asm-generic/ioctls.h
    const TIOCMGET: u32 = 0x5415;
    const TIOCMSET: u32 = 0x5418;

    var flags: c_int = 0;
    if (std.os.linux.ioctl(file.handle, TIOCMGET, @intFromPtr(&flags)) != 0)
        return error.Unexpected;

    if (pins.dtr) |dtr| {
        if (dtr) flags |= TIOCM_DTR else flags &= ~TIOCM_DTR;
    }
    if (pins.rts) |rts| {
        if (rts) flags |= TIOCM_RTS else flags &= ~TIOCM_RTS;
    }

    if (std.os.linux.ioctl(file.handle, TIOCMSET, @intFromPtr(&flags)) != 0)
        return error.Unexpected;
}
