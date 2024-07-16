pub const Iterator = @import("windows/Iterator.zig");
pub const InformationIterator = @import("windows/InfoIterator.zig");
const std = @import("std");
const SerialPort = @import("SerialPort.zig");
const serial = @import("serial.zig");
const MAXDWORD = std.math.maxInt(std.os.windows.DWORD);

pub fn flush(self: *SerialPort, buffer: Buffer) !void {
    const result = PurgeComm(self.file.handle, @intFromEnum(buffer));
    if (result == 0) return error.FlushError;
}

const Buffer = enum(u8) {
    input = 0x0008,
    output = 0x0004,
    both = 0x000C,
};

extern "kernel32" fn PurgeComm(hFile: std.os.windows.HANDLE, dwFlags: std.os.windows.DWORD) callconv(std.os.windows.WINAPI) std.os.windows.BOOL;

pub fn controlPins(self: *SerialPort, pins: serial.ControlPins) !void {
    if (pins.rts) |pin| {
        const rts = EscapeCommFunction(self.file.handle, if (pin) 3 else 4);
        if (rts == 0) return error.WindowsError;
    }
    if (pins.dtr) |pin| {
        const dtr = EscapeCommFunction(self.file.handle, if (pin) 5 else 6);
        if (dtr == 0) return error.WindowsError;
    }
}

extern "kernel32" fn EscapeCommFunction(hFile: std.os.windows.HANDLE, dwFunc: std.os.windows.DWORD) callconv(std.os.windows.WINAPI) std.os.windows.BOOL;

pub fn configure(file: std.fs.File, config: serial.Config) !void {
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
    var timeout = COMMTIMEOUTS.init(config.timeout);
    if (SetCommTimeouts(file.handle, &timeout) == 0)
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
    DCBlength: std.os.windows.DWORD,
    BaudRate: std.os.windows.DWORD,
    flags: u32,
    wReserved: std.os.windows.WORD,
    XonLim: std.os.windows.WORD,
    XoffLim: std.os.windows.WORD,
    ByteSize: std.os.windows.BYTE,
    Parity: std.os.windows.BYTE,
    StopBits: std.os.windows.BYTE,
    XonChar: u8,
    XoffChar: u8,
    ErrorChar: u8,
    EofChar: u8,
    EvtChar: u8,
    wReserved1: std.os.windows.WORD,
};

const COMMTIMEOUTS = extern struct {
    ReadIntervalTimeout: std.os.windows.DWORD = MAXDWORD,
    ReadTotalTimeoutMultiplier: std.os.windows.DWORD = 0,
    ReadTotalTimeoutConstant: std.os.windows.DWORD = 0,
    WriteTotalTimeoutMultiplier: std.os.windows.DWORD = 0,
    WriteTotalTimeoutConstant: std.os.windows.DWORD = 0,

    pub fn init(timeout: u32) COMMTIMEOUTS {
        // This mimics the default behavior of posix systems.
        if (timeout == 0) return .{};
        // This is the god awful way windows does things. Waits until data is available, or timeout.
        // Practically, this means it waits until either the supplied buffer is full or timeout occures.
        return .{
            .ReadTotalTimeoutMultiplier = MAXDWORD,
            .ReadTotalTimeoutConstant = timeout,
        };
    }
};

extern "kernel32" fn GetCommState(hFile: std.os.windows.HANDLE, lpDCB: *DCB) callconv(std.os.windows.WINAPI) std.os.windows.BOOL;
extern "kernel32" fn SetCommState(hFile: std.os.windows.HANDLE, lpDCB: *DCB) callconv(std.os.windows.WINAPI) std.os.windows.BOOL;
extern "kernel32" fn SetCommTimeouts(in_hFile: std.os.windows.HANDLE, in_lpCommTimeouts: *COMMTIMEOUTS) callconv(std.os.windows.WINAPI) std.os.windows.BOOL;
