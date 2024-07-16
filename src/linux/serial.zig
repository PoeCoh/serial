const std = @import("std");
const serial = @import("../serial.zig");

pub fn flush(file: std.fs.File, buffers: Buffers) !void {
    const TCFLSH = 0x540B;
    const result = std.os.linux.syscall3(.ioctl, @as(usize, @bitCast(@as(isize, file.handle))), TCFLSH, @intFromEnum(buffers));
    if (result != 0) return error.FlushError;
}

pub const Buffers = enum(comptime_int) {
    input = 0,
    output = 1,
    both = 2,
};

pub fn controlPins(file: std.fs.File, pins: serial.ControlPins) !void {
    const TIOCM_RTS: c_int = 0x004;
    const TIOCM_DTR: c_int = 0x002;

    // from /usr/include/asm-generic/ioctls.h
    // const TIOCMBIS: u32 = 0x5416;
    // const TIOCMBIC: u32 = 0x5417;
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
