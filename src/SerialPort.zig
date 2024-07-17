file: std.fs.File,

pub usingnamespace switch (native_os) {
    .windows => @import("windows.zig"),
    .macos => @import("darwin.zig"),
    else => @import("linux.zig"),
};

pub fn deinit(self: *SerialPort) void {
    self.file.close();
    self.* = undefined;
}

pub fn read(self: *SerialPort, buffer: []u8) !usize {
    return self.file.read(buffer);
}

pub fn write(self: *SerialPort, bytes: []const u8) !usize {
    return self.file.write(bytes);
}

const SerialPort = @This();
const std = @import("std");
const serial = @import("serial.zig");
const native_os = @import("builtin").os.tag;
