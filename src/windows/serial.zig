const std = @import("std");
const serial = @import("../serial.zig");

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

pub fn controlPins(file: std.fs.File, pins: serial.ControlPins) !void {
    const CLRDTR = 6;
    const CLRRTS = 4;
    const SETDTR = 5;
    const SETRTS = 3;
    if (pins.dtr) |pin| {
        const dtr = EscapeCommFunction(file.handle, if (pin) SETDTR else CLRDTR);
        if (dtr == 0) return error.WindowsError;
    }
    if (pins.rts) |pin| {
        const rts = EscapeCommFunction(file.handle, if (pin) SETRTS else CLRRTS);
        if (rts == 0) return error.WindowsError;
    }
}

extern "kernel32" fn EscapeCommFunction(hFile: std.os.windows.HANDLE, dwFunc: std.os.windows.DWORD) callconv(std.os.windows.WINAPI) std.os.windows.BOOL;