const std = @import("std");
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
