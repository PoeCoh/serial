const std = @import("std");
const SerialPort = @import("Serialport.zig");
const serial = @import("serial.zig");
const c = @cImport(@cInclude("termios.h"));

pub fn init(port_name: []const u8, config: serial.Config) !SerialPort {
    var buffer = [_]u8{0} ** std.fs.max_path_bytes;
    const file_name = try std.fmt.bufPrint(&buffer, "/dev/{s}", .{port_name});
    const file = try std.fs.cwd().openFile(file_name, .{ .mode = .read_write });
    errdefer file.close();
    try serial.configure(file, config);
    return .{ .file = file };
}

pub fn flush(self: *SerialPort, buffers: Buffers) !void {
    const result = c.tcflush(self.file.handle, @intFromEnum(buffers));
    if (result != 0) return error.FlushError;
}

const Buffers = enum(c_int) {
    input = 1,
    output = 2,
    both = 3,
};