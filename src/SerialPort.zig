file: std.fs.File,

pub fn init(port_name: []const u8, config: Config) !SerialPort {
    const prefix = switch (native_os) {
        .windows => "\\\\.\\",
        else => "/dev/",
    };
    var buffer = [_]u8{0} ** std.fs.max_path_bytes;
    const file_name = try std.fmt.bufPrint(&buffer, "{s}{s}", .{ prefix, port_name });
    const file = try std.fs.cwd().openFile(file_name, .{ .mode = .read_write });
    errdefer file.close();
    try serial.configureSerialPort(file, config);
    return .{ .file = file };
}

pub fn deinit(self: *SerialPort) void {
    self.file.close();
}

pub fn write(self: *SerialPort, bytes: []const u8) !usize {
    return self.file.write(bytes);
}

pub fn read(self: *SerialPort, buffer: []u8) !usize {
    return self.file.read(buffer);
}

pub fn iterator() SerialPort.Iterator {
    return SerialPort.Iterator.init();
}

pub fn infoIterator() SerialPort.InformationIterator {
    return SerialPort.InformationIterator.init();
}

pub usingnamespace switch (native_os) {
    .windows => @import("windows.zig"),
    .macos, .tvos, .watchos, .ios => @import("darwin.zig"),
    else => @import("linux.zig"),
};

const std = @import("std");
const serial = @import("serial.zig");
const native_os = @import("builtin").os.tag;
const Config = serial.SerialConfig;
const SerialPort = @This();