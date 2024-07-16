const std = @import("std");
const builtin = @import("builtin");
const MAXDWORD = std.math.maxInt(std.os.windows.DWORD);
const native_os = @import("builtin").os.tag;
const serial = @This();

pub usingnamespace switch (native_os) {
    .windows => @import("windows/serial.zig"),
    .macos, .ios, .tvos, .watchos => @import("darwin/serial.zig"),
    else => @import("linux/serial.zig"),
};

pub fn list() !PortIterator {
    return try PortIterator.init();
}

pub fn list_info() !InformationIterator {
    return try InformationIterator.init();
}

pub const PortIterator = switch (builtin.os.tag) {
    .windows => @import("windows/Iterator.zig"),
    .linux => @import("linux/Iterator.zig"),
    .macos => @import("darwin/Iterator.zig"),
    else => @compileError("OS is not supported for port iteration"),
};

pub const InformationIterator = switch (builtin.os.tag) {
    .windows => @import("windows/InformationIterator.zig"),
    .linux, .macos => @panic("'Port Information' not yet implemented for this OS"),
    else => @compileError("OS is not supported for information iteration"),
};

pub const SerialPortDescription = struct {
    file_name: []const u8,
    display_name: []const u8,
    driver: ?[]const u8,
};

pub const PortInformation = struct {
    port_name: []const u8,
    system_location: []const u8,
    friendly_name: []const u8,
    description: []const u8,
    manufacturer: []const u8,
    serial_number: []const u8,
    // TODO: review whether to remove `hw_id`.
    // Is this useless/being used in a Windows-only way?
    hw_id: []const u8,
    vid: u16,
    pid: u16,
};

const HKEY = std.os.windows.HKEY;

pub const Parity = enum {
    /// No parity bit is used
    none,
    /// Parity bit is `0` when an even number of bits is set in the data.
    even,
    /// Parity bit is `0` when an odd number of bits is set in the data.
    odd,
    /// Parity bit is always `1`
    mark,
    /// Parity bit is always `0`
    space,
};

pub const StopBits = enum {
    /// The length of the stop bit is 1 bit
    one,
    /// The length of the stop bit is 2 bits
    two,
};

pub const Handshake = enum {
    /// No handshake is used
    none,
    /// XON-XOFF software handshake is used.
    software,
    /// Hardware handshake with RTS/CTS is used.
    hardware,
};

pub const WordSize = switch (builtin.os.tag) {
    .windows => enum(u8) {
        CS5 = 5,
        CS6 = 6,
        CS7 = 7,
        CS8 = 8,
    },
    else => std.posix.CSIZE,
};

pub const SerialConfig = struct {
    /// Symbol rate in bits/second. Not that these
    /// include also parity and stop bits.
    baud_rate: u32,

    /// Parity to verify transport integrity.
    parity: Parity = .none,

    /// Number of stop bits after the data
    stop_bits: StopBits = .one,

    /// Number of data bits per word.
    word_size: WordSize = .CS8,

    /// Defines the handshake protocol used.
    handshake: Handshake = .none,

    /// Timeout in milliseconds. Only used on windows.
    timeout: u32 = 0,
};

const CBAUD = 0o000000010017; //Baud speed mask (not in POSIX).
const CMSPAR = 0o010000000000;
const CRTSCTS = 0o020000000000;

const VTIME = 5;
const VMIN = 6;
const VSTART = 8;
const VSTOP = 9;

/// This function configures a serial port with the given config.
/// `port` is an already opened serial port, on windows these
/// are either called `\\.\COMxx\` or `COMx`, on unixes the serial
/// port is called `/dev/ttyXXX`.
pub fn configureSerialPort(port: std.fs.File, config: SerialConfig) !void {
    switch (builtin.os.tag) {
        .windows => {
            var dcb = std.mem.zeroes(DCB);
            dcb.DCBlength = @sizeOf(DCB);

            if (GetCommState(port.handle, &dcb) == 0)
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

            if (SetCommState(port.handle, &dcb) == 0)
                return error.WindowsError;
            var timeout = COMMTIMEOUTS.init(config.timeout);
            if (SetCommTimeouts(port.handle, &timeout) == 0)
                return error.WindowsError;
        },
        else => {
            var settings = try std.posix.tcgetattr(port.handle);

            settings.iflag = .{};
            settings.oflag = .{};
            settings.cflag = .{ .CREAD = true };
            settings.lflag = .{};
            settings.ispeed = .B0;
            settings.ospeed = .B0;

            switch (config.parity) {
                .none => {},
                .odd => settings.cflag.PARODD = true,
                .even => {}, // even parity is default when parity is enabled
                .mark => {
                    settings.cflag.PARODD = true;
                    // settings.cflag.CMSPAR = true;
                    settings.cflag._ |= (1 << 14);
                },
                .space => settings.cflag._ |= 1,
            }
            if (config.parity != .none) {
                settings.iflag.INPCK = true; // enable parity checking
                settings.cflag.PARENB = true; // enable parity generation
            }

            switch (config.handshake) {
                .none => settings.cflag.CLOCAL = true,
                .software => {
                    settings.iflag.IXON = true;
                    settings.iflag.IXOFF = true;
                },
                // .hardware => settings.cflag.CRTSCTS = true,
                .hardware => settings.cflag._ |= 1 << 15,
            }

            switch (config.stop_bits) {
                .one => {},
                .two => settings.cflag.CSTOPB = true,
            }

            settings.cflag.CSIZE = config.word_size;

            const baud: std.posix.speed_t = @enumFromInt(config.baud_rate);

            // settings.cflag &= ~@as(os.tcflag_t, CBAUD);
            // settings.cflag |= baudmask;
            settings.ispeed = baud;
            settings.ospeed = baud;

            settings.cc[VMIN] = 1;
            settings.cc[VSTOP] = 0x13; // XOFF
            settings.cc[VSTART] = 0x11; // XON
            settings.cc[VTIME] = 0;

            try std.posix.tcsetattr(port.handle, .NOW, settings);
        },
    }
}

pub const ControlPins = struct {
    rts: ?bool = null,
    dtr: ?bool = null,
};

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
        if (timeout == 0 ) return .{};
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

test "iterate ports" {
    var it = try list();
    while (try it.next()) |port| {
        _ = port;
        // std.debug.print("{s} (file: {s}, driver: {s})\n", .{ port.display_name, port.file_name, port.driver });
    }
}

test "basic configuration test" {
    const cfg = SerialConfig{
        .handshake = .none,
        .baud_rate = 115200,
        .parity = .none,
        .word_size = .eight,
        .stop_bits = .one,
    };

    var tty: []const u8 = undefined;

    switch (builtin.os.tag) {
        .windows => tty = "\\\\.\\COM3",
        .linux => tty = "/dev/ttyUSB0",
        .macos => tty = "/dev/cu.usbmodem101",
        else => unreachable,
    }

    var port = try std.fs.cwd().openFile(tty, .{ .mode = .read_write });
    defer port.close();

    try configureSerialPort(port, cfg);
}

test "basic flush test" {
    var tty: []const u8 = undefined;
    // if any, these will likely exist on a machine
    switch (builtin.os.tag) {
        .windows => tty = "\\\\.\\COM3",
        .linux => tty = "/dev/ttyUSB0",
        .macos => tty = "/dev/cu.usbmodem101",
        else => unreachable,
    }
    var port = try std.fs.cwd().openFile(tty, .{ .mode = .read_write });
    defer port.close();

    try serial.flush(port, .input);
    try serial.flush(port, .output);
    try serial.flush(port, .both);
}

test "change control pins" {
    _ = serial.controlPins;
}
