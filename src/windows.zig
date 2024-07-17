const std = @import("std");
const SerialPort = @import("Serialport.zig");
const serial = @import("serial.zig");
const Iterator = @import("windows/Iterator.zig");
const InformationIterator = @import("windows/InformationIterator.zig");
const HANDLE = std.os.windows.HANDLE;
const DWORD = std.os.windows.DWORD;
const BOOL = std.os.windows.BOOL;
const WORD = std.os.windows.WORD;
const BYTE = std.os.windows.BYTE;
const WINAPI = std.os.windows.WINAPI;

pub fn init(port_name: []const u8, config: serial.Config) !SerialPort {
    // max_path_bytes
    var buffer = [_]u8{0} ** std.fs.max_path_bytes;
    const file_name = try std.fmt.bufPrint(&buffer, "\\\\.\\{s}", .{port_name});
    const file = try std.fs.cwd().openFile(file_name, .{ .mode = .read_write });
    errdefer file.close();
    try configure(file, config);
    return .{ .file = file };
}

pub fn flush(self: *SerialPort, buffers: Buffers) !void {
    const result = PurgeComm(self.file.handle, @intFromEnum(buffers));
    if (result == 0) return error.FlushError;
}

pub fn controlPins(self: *SerialPort, pins: serial.ControlPins) !void {
    if (pins.dtr) |pin| {
        const result = EscapeCommFunction(self.file.handle, if (pin) 5 else 6);
        if (result == 0) return error.ControlPinsError;
    }
    if (pins.rts) |pin| {
        const result = EscapeCommFunction(self.file.handle, if (pin) 3 else 4);
        if (result == 0) return error.ControlPinsError;
    }
}

pub fn iterate() !Iterator {
    return Iterator.init();
}

pub fn informationIterator() !InformationIterator {
    return InformationIterator.init();
}

const Buffers = enum(u8) {
    input = 0x0008,
    output = 0x0004,
    both = 0x000C,
};

fn configure(file: std.fs.File, config: serial.Config) !void {
    var dcb = std.mem.zeroes(DCB);
    dcb.DCBlength = @sizeOf(DCB);

    if (GetCommState(file.handle, &dcb) == 0)
        return error.WindowsError;

    // std.log.err("{s} {s}", .{ dcb, flags });

    dcb.BaudRate = config.baud_rate;

    dcb.flags = @bitCast(DCBFlags{
        .fParity = config.parity != .none,
        .fOutxCtsFlow = config.handshake == .hardware,
        .fOutX = config.handshake == .software,
        .fInX = config.handshake == .software,
        .fRtsControl = @as(u2, if (config.handshake == .hardware) 1 else 0),
    });

    dcb.wReserved = 0;
    dcb.ByteSize = @intFromEnum(config.word_size);
    dcb.Parity = switch (config.parity) {
        .none => @as(u8, 0),
        .even => @as(u8, 2),
        .odd => @as(u8, 1),
        .mark => @as(u8, 3),
        .space => @as(u8, 4),
    };
    dcb.StopBits = switch (config.stop_bits) {
        .one => @as(u2, 0),
        .two => @as(u2, 2),
    };
    dcb.XonChar = 0x11;
    dcb.XoffChar = 0x13;
    dcb.wReserved1 = 0;

    if (SetCommState(file.handle, &dcb) == 0)
        return error.WindowsError;
}

const DCBFlags = packed struct(u32) {
    fBinary: bool = true, // u1
    fParity: bool = false, // u1
    fOutxCtsFlow: bool = false, // u1
    fOutxDsrFlow: bool = false, // u1
    fDtrControl: u2 = 1, // u2
    fDsrSensitivity: bool = false, // u1
    fTXContinueOnXoff: bool = false, // u1
    fOutX: bool = false, // u1
    fInX: bool = false, // u1
    fErrorChar: bool = false, // u1
    fNull: bool = false, // u1
    fRtsControl: u2 = 0, // u2
    fAbortOnError: bool = false, // u1
    fDummy2: u17 = 0, // u17
};

/// Configuration for the serial port
///
/// Details: https://learn.microsoft.com/es-es/windows/win32/api/winbase/ns-winbase-dcb
const DCB = extern struct {
    DCBlength: DWORD,
    BaudRate: DWORD,
    flags: u32,
    wReserved: WORD,
    XonLim: WORD,
    XoffLim: WORD,
    ByteSize: BYTE,
    Parity: BYTE,
    StopBits: BYTE,
    XonChar: u8,
    XoffChar: u8,
    ErrorChar: u8,
    EofChar: u8,
    EvtChar: u8,
    wReserved1: WORD,
};

extern "kernel32" fn GetCommState(hFile: HANDLE, lpDCB: *DCB) callconv(WINAPI) BOOL;
extern "kernel32" fn SetCommState(hFile: HANDLE, lpDCB: *DCB) callconv(WINAPI) BOOL;
extern "kernel32" fn PurgeComm(hFile: HANDLE, dwFlags: DWORD) callconv(WINAPI) BOOL;
extern "kernel32" fn EscapeCommFunction(hFile: HANDLE, dwFunc: DWORD) callconv(WINAPI) BOOL;
