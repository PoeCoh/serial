const std = @import("std");
const serial = @import("../serial.zig");
pub const Iterator = @import("Iterator.zig");
pub const InformationIterator = @import("InformationIterator.zig");
const MAXDWORD = std.math.maxInt(std.os.windows.DWORD);
const DWORD = std.os.windows.DWORD;
const BOOL = std.os.windows.BOOL;
const HANDLE = std.os.windows.HANDLE;
const WORD = std.os.windows.WORD;
const BYTE = std.os.windows.BYTE;
const WINAPI = std.os.windows.WINAPI;

pub const Buffers = enum(u8) {
    input = 0x0008,
    output = 0x0004,
    both = 0x000C,
};

pub fn pFlush(file: *std.fs.File, buffer: u8) !void {
    const result = PurgeComm(file.handle, buffer);
    if (result == 0) return error.FlushError;
}

extern "kernel32" fn PurgeComm(hFile: HANDLE, dwFlags: DWORD) callconv(WINAPI) BOOL;

pub fn pControlPins(file: *std.fs.File, pins: serial.ControlPins) !void {
    const CLRDTR = 6;
    const CLRRTS = 4;
    const SETDTR = 5;
    const SETRTS = 3;
    if (pins.dtr) |dtr| {
        if (EscapeCommFunction(file.handle, if (dtr) SETDTR else CLRDTR) == 0)
            return error.WindowsError;
    }
    if (pins.rts) |rts| {
        if (EscapeCommFunction(file.handle, if (rts) SETRTS else CLRRTS) == 0)
            return error.WindowsError;
    }
}

extern "kernel32" fn EscapeCommFunction(hFile: HANDLE, dwFunc: DWORD) callconv(WINAPI) BOOL;

pub fn configure(file: *std.fs.File, config: serial.Config) !void {
    var dcb = std.mem.zeroes(DCB);
    dcb.DCBlength = @sizeOf(DCB);

    if (GetCommState(file.handle, &dcb) == 0)
        return error.WindowsError;

    dcb.BaudRate = config.baud_rate;

    dcb.flags = @bitCast(
        DCBFlags{
            .fParity = config.parity != .none,
            .fOutxCtsFlow = config.handshake == .hardware,
            .fOutX = config.handshake == .software,
            .fInX = config.handshake == .software,
            .fRtsControl = @as(u2, if (config.handshake == .hardware) 1 else 0),
        },
    );

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
    const timeout = COMMTIMEOUTS.init(config.timeout);
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

const COMMTIMEOUTS = extern struct {
    ReadIntervalTimeout: DWORD = MAXDWORD,
    ReadTotalTimeoutMultiplier: DWORD = 0,
    ReadTotalTimeoutConstant: DWORD = 0,
    WriteTotalTimeoutMultiplier: DWORD = 0,
    WriteTotalTimeoutConstant: DWORD = 0,

    pub fn init(timeout: u32) COMMTIMEOUTS {
        // This mimics the default behavior of posix systems.
        if (timeout == 0 ) return .{};
        // This is the god awful way windows does things. Waits until data is available, or timeout.
        // Practically, this means it waits until either the supplied buffer is full or timeout occures.
        return .{
            .ReadTotalTimeoutMultiplier = MAXDWORD,
            .ReadTotalTimeoutConstant = timeout,
        };
    }
};

extern "kernel32" fn GetCommState(hFile: HANDLE, lpDCB: *DCB) callconv(WINAPI) BOOL;
extern "kernel32" fn SetCommState(hFile: HANDLE, lpDCB: *DCB) callconv(WINAPI) BOOL;
extern "kernel32" fn SetCommTimeouts(in_hFile: HANDLE, in_lpCommTimeouts: *COMMTIMEOUTS) callconv(WINAPI) BOOL;
