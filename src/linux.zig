const std = @import("std");
const SerialPort = @import("Serialport.zig");
const Iterator = @import("linux/Iterator.zig");
const serial = @import("serial.zig");

pub fn init(port_name: []const u8, config: serial.Config) !SerialPort {
    var buffer = [_]u8{0} ** std.fs.max_path_bytes;
    const file_name = try std.fmt.bufPrint(&buffer, "/dev/{s}", .{port_name});
    const file = try std.fs.cwd().openFile(file_name, .{ .mode = .read_write });
    errdefer file.close();
    try serial.configure(file, config);
    return .{ .file = file };
}

pub fn flush(self: *SerialPort, buffers: Buffers) !void {
    const result = std.os.linux.syscall3(.ioctl, @as(usize, @bitCast(@as(isize, self.file.handle))), 0x540B, @intFromEnum(buffers));
    if (result != 0) return error.FlushError;
}

pub fn iterate() !Iterator {
    return Iterator.init();
}

pub fn controlPins(self: *SerialPort, pins: serial.ControlPins) !void {
    const TIOCM_RTS: c_int = 0x004;
    const TIOCM_DTR: c_int = 0x002;
    const TIOCMGET: u32 = 0x5415;
    const TIOCMSET: u32 = 0x5418;

    var flags: c_int = 0;
    if (std.os.linux.ioctl(self.file.handle, TIOCMGET, @intFromPtr(&flags)) != 0)
        return error.Unexpected;

    if (pins.dtr) |dtr| {
        if (dtr)  flags |= TIOCM_DTR else flags &= ~TIOCM_DTR;
    }
    if (pins.rts) |rts| {
        if (rts) flags |= TIOCM_RTS else flags &= ~TIOCM_RTS;
    }

    if (std.os.linux.ioctl(self.file.handle, TIOCMSET, @intFromPtr(&flags)) != 0)
        return error.Unexpected;
}

const Buffers = enum(usize) {
    input = 0,
    output = 1,
    both = 2,
};
