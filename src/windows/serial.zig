const std = @import("std");

pub fn flush(file: std.fs.File, buffers: Buffers) !void {
    const result = PurgeComm(file.handle, @intFromEnum(buffers));
    if (result == 0) return error.FlushError;
}

pub const Buffers = enum(comptime_int) {
    input = 0x0008,
    output = 0x0004,
    both = 0x000C,
};

extern "kernel32" fn PurgeComm(hFile: std.os.windows.HANDLE, dwFlags: std.os.windows.DWORD) callconv(std.os.windows.WINAPI) std.os.windows.BOOL;