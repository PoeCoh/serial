const std = @import("std");
const builtin = @import("builtin");
const native_os = @import("builtin").os.tag;
const c = @cImport(@cInclude("termios.h"));
const serial = @This();

pub usingnamespace switch (native_os) {
    .windows => @import("windows/serial.zig"),
    .macos, .ios, .tvos, .watchos, .visionos => @import("darwin/serial.zig"),
    else => @import("linux/serial.zig"), // just assume everything else is linux and let it error out if it's not compatible
};

pub fn list() !PortIterator {
    return try PortIterator.init();
}

pub fn infoIterator() !serial.InformationIterator {
    return serial.InformationIterator.init();
}

pub const PortIterator = switch (builtin.os.tag) {
    .windows => WindowsPortIterator,
    .linux => LinuxPortIterator,
    .macos => DarwinPortIterator,
    else => @compileError("OS is not supported for port iteration"),
};

pub const SerialPortDescription = struct {
    file_name: []const u8,
    display_name: []const u8,
    driver: ?[]const u8,
};

pub const Information = struct {
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
const HWND = std.os.windows.HANDLE;
const HDEVINFO = std.os.windows.HANDLE;
const DEVINST = std.os.windows.DWORD;

const WindowsPortIterator = struct {
    const Self = @This();

    key: HKEY,
    index: u32,

    name: [256:0]u8 = undefined,
    name_size: u32 = 256,

    data: [256]u8 = undefined,
    filepath_data: [256]u8 = undefined,
    data_size: u32 = 256,

    pub fn init() !Self {
        const HKEY_LOCAL_MACHINE = @as(HKEY, @ptrFromInt(0x80000002));
        const KEY_READ = 0x20019;

        var self: Self = undefined;
        self.index = 0;
        if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM\\", 0, KEY_READ, &self.key) != 0)
            return error.WindowsError;

        return self;
    }

    pub fn deinit(self: *Self) void {
        _ = RegCloseKey(self.key);
        self.* = undefined;
    }

    pub fn next(self: *Self) !?SerialPortDescription {
        defer self.index += 1;

        self.name_size = 256;
        self.data_size = 256;

        return switch (RegEnumValueA(self.key, self.index, &self.name, &self.name_size, null, null, &self.data, &self.data_size)) {
            0 => SerialPortDescription{
                .file_name = try std.fmt.bufPrint(&self.filepath_data, "\\\\.\\{s}", .{self.data[0 .. self.data_size - 1]}),
                .display_name = self.data[0 .. self.data_size - 1],
                .driver = self.name[0..self.name_size],
            },
            259 => null,
            else => error.WindowsError,
        };
    }
};

extern "advapi32" fn RegOpenKeyExA(
    key: HKEY,
    lpSubKey: std.os.windows.LPCSTR,
    ulOptions: std.os.windows.DWORD,
    samDesired: std.os.windows.REGSAM,
    phkResult: *HKEY,
) callconv(std.os.windows.WINAPI) std.os.windows.LSTATUS;
extern "advapi32" fn RegCloseKey(key: HKEY) callconv(std.os.windows.WINAPI) std.os.windows.LSTATUS;
extern "advapi32" fn RegEnumValueA(
    hKey: HKEY,
    dwIndex: std.os.windows.DWORD,
    lpValueName: std.os.windows.LPSTR,
    lpcchValueName: *std.os.windows.DWORD,
    lpReserved: ?*std.os.windows.DWORD,
    lpType: ?*std.os.windows.DWORD,
    lpData: [*]std.os.windows.BYTE,
    lpcbData: *std.os.windows.DWORD,
) callconv(std.os.windows.WINAPI) std.os.windows.LSTATUS;


const LinuxPortIterator = struct {
    const Self = @This();

    const root_dir = "/sys/class/tty";

    // ls -hal /sys/class/tty/*/device/driver

    dir: std.fs.Dir,
    iterator: std.fs.Dir.Iterator,

    full_path_buffer: [std.fs.max_path_bytes]u8 = undefined,
    driver_path_buffer: [std.fs.max_path_bytes]u8 = undefined,

    pub fn init() !Self {
        var dir = try std.fs.cwd().openDir(root_dir, .{ .iterate = true });
        errdefer dir.close();

        return Self{
            .dir = dir,
            .iterator = dir.iterate(),
        };
    }

    pub fn deinit(self: *Self) void {
        self.dir.close();
        self.* = undefined;
    }

    pub fn next(self: *Self) !?SerialPortDescription {
        while (true) {
            if (try self.iterator.next()) |entry| {
                // not a dir => we don't care
                var tty_dir = self.dir.openDir(entry.name, .{}) catch continue;
                defer tty_dir.close();

                // we need the device dir
                // no device dir =>  virtual device
                var device_dir = tty_dir.openDir("device", .{}) catch continue;
                defer device_dir.close();

                // We need the symlink for "driver"
                const link = device_dir.readLink("driver", &self.driver_path_buffer) catch continue;

                // full_path_buffer
                // driver_path_buffer

                var fba = std.heap.FixedBufferAllocator.init(&self.full_path_buffer);

                const path = try std.fs.path.join(fba.allocator(), &.{
                    "/dev/",
                    entry.name,
                });

                return SerialPortDescription{
                    .file_name = path,
                    .display_name = path,
                    .driver = std.fs.path.basename(link),
                };
            } else {
                return null;
            }
        }
        return null;
    }
};

const DarwinPortIterator = struct {
    const Self = @This();

    const root_dir = "/dev/";

    dir: std.fs.IterableDir,
    iterator: std.fs.IterableDir.Iterator,

    full_path_buffer: [std.fs.max_path_bytes]u8 = undefined,
    driver_path_buffer: [std.fs.max_path_bytes]u8 = undefined,

    pub fn init() !Self {
        var dir = try std.fs.cwd().openIterableDir(root_dir, .{});
        errdefer dir.close();

        return Self{
            .dir = dir,
            .iterator = dir.iterate(),
        };
    }

    pub fn deinit(self: *Self) void {
        self.dir.close();
        self.* = undefined;
    }

    pub fn next(self: *Self) !?SerialPortDescription {
        while (true) {
            if (try self.iterator.next()) |entry| {
                if (!std.mem.startsWith(u8, entry.name, "cu.")) {
                    continue;
                } else {
                    var fba = std.heap.FixedBufferAllocator.init(&self.full_path_buffer);

                    const path = try std.fs.path.join(fba.allocator(), &.{
                        "/dev/",
                        entry.name,
                    });

                    return SerialPortDescription{
                        .file_name = path,
                        .display_name = path,
                        .driver = "darwin",
                    };
                }
            } else {
                return null;
            }
        }
        return null;
    }
};

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

pub const WordSize = switch (native_os) {
    .windows => enum(u8) {
        CS5 = 5,
        CS6 = 6,
        CS7 = 7,
        CS8 = 8,
    },
    else => std.c.CSIZE,
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
    /// Allowed values are 5, 6, 7, 8
    word_size: WordSize = .CS8,

    /// Defines the handshake protocol used.
    handshake: Handshake = .none,
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

            // let std lib handle baudrate
            const baudrate: std.posix.speed_t = @enumFromInt(config.baud_rate);

            // settings.cflag &= ~@as(os.tcflag_t, CBAUD);
            // settings.cflag |= baudmask;
            settings.ispeed = baudrate;
            settings.ospeed = baudrate;

            settings.cc[VMIN] = 1;
            settings.cc[VSTOP] = 0x13; // XOFF
            settings.cc[VSTART] = 0x11; // XON
            settings.cc[VTIME] = 0;

            try std.posix.tcsetattr(port.handle, .NOW, settings);
        },
    }
}

/// Flushes the serial port `port`. If `input` is set, all pending data in
/// the receive buffer is flushed, if `output` is set all pending data in
/// the send buffer is flushed.
pub fn flushSerialPort(port: std.fs.File, input: bool, output: bool) !void {
    if (!input and !output)
        return;

    switch (builtin.os.tag) {
        .windows => {
            const success = if (input and output)
                PurgeComm(port.handle, PURGE_TXCLEAR | PURGE_RXCLEAR)
            else if (input)
                PurgeComm(port.handle, PURGE_RXCLEAR)
            else if (output)
                PurgeComm(port.handle, PURGE_TXCLEAR)
            else
                @as(std.os.windows.BOOL, 0);
            if (success == 0)
                return error.FlushError;
        },

        .linux => if (input and output)
            try tcflush(port.handle, TCIOFLUSH)
        else if (input)
            try tcflush(port.handle, TCIFLUSH)
        else if (output)
            try tcflush(port.handle, TCOFLUSH),

        .macos => if (input and output)
            try tcflush(port.handle, c.TCIOFLUSH)
        else if (input)
            try tcflush(port.handle, c.TCIFLUSH)
        else if (output)
            try tcflush(port.handle, c.TCOFLUSH),

        else => @compileError("unsupported OS, please implement!"),
    }
}

pub const ControlPins = struct {
    rts: ?bool = null,
    dtr: ?bool = null,
};

pub fn changeControlPins(port: std.fs.File, pins: ControlPins) !void {
    switch (builtin.os.tag) {
        .windows => {
            const CLRDTR = 6;
            const CLRRTS = 4;
            const SETDTR = 5;
            const SETRTS = 3;

            if (pins.dtr) |dtr| {
                if (EscapeCommFunction(port.handle, if (dtr) SETDTR else CLRDTR) == 0)
                    return error.WindowsError;
            }
            if (pins.rts) |rts| {
                if (EscapeCommFunction(port.handle, if (rts) SETRTS else CLRRTS) == 0)
                    return error.WindowsError;
            }
        },
        .linux => {
            const TIOCM_RTS: c_int = 0x004;
            const TIOCM_DTR: c_int = 0x002;

            // from /usr/include/asm-generic/ioctls.h
            // const TIOCMBIS: u32 = 0x5416;
            // const TIOCMBIC: u32 = 0x5417;
            const TIOCMGET: u32 = 0x5415;
            const TIOCMSET: u32 = 0x5418;

            var flags: c_int = 0;
            if (std.os.linux.ioctl(port.handle, TIOCMGET, @intFromPtr(&flags)) != 0)
                return error.Unexpected;

            if (pins.dtr) |dtr| {
                if (dtr) {
                    flags |= TIOCM_DTR;
                } else {
                    flags &= ~TIOCM_DTR;
                }
            }
            if (pins.rts) |rts| {
                if (rts) {
                    flags |= TIOCM_RTS;
                } else {
                    flags &= ~TIOCM_RTS;
                }
            }

            if (std.os.linux.ioctl(port.handle, TIOCMSET, @intFromPtr(&flags)) != 0)
                return error.Unexpected;
        },

        .macos => {},

        else => @compileError("changeControlPins not implemented for " ++ @tagName(builtin.os.tag)),
    }
}

const PURGE_RXABORT = 0x0002;
const PURGE_RXCLEAR = 0x0008;
const PURGE_TXABORT = 0x0001;
const PURGE_TXCLEAR = 0x0004;

extern "kernel32" fn PurgeComm(hFile: std.os.windows.HANDLE, dwFlags: std.os.windows.DWORD) callconv(std.os.windows.WINAPI) std.os.windows.BOOL;
extern "kernel32" fn EscapeCommFunction(hFile: std.os.windows.HANDLE, dwFunc: std.os.windows.DWORD) callconv(std.os.windows.WINAPI) std.os.windows.BOOL;

const TCIFLUSH = 0;
const TCOFLUSH = 1;
const TCIOFLUSH = 2;
const TCFLSH = 0x540B;

fn tcflush(fd: std.os.linux.fd_t, mode: usize) !void {
    switch (builtin.os.tag) {
        .linux => {
            if (std.os.linux.syscall3(.ioctl, @as(usize, @bitCast(@as(isize, fd))), TCFLSH, mode) != 0)
                return error.FlushError;
        },
        .macos => {
            const err = c.tcflush(fd, @as(c_int, @intCast(mode)));
            if (err != 0) {
                std.debug.print("tcflush failed: {d}\r\n", .{err});
                return error.FlushError;
            }
        },
        else => @compileError("unsupported OS, please implement!"),
    }
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

extern "kernel32" fn GetCommState(hFile: std.os.windows.HANDLE, lpDCB: *DCB) callconv(std.os.windows.WINAPI) std.os.windows.BOOL;
extern "kernel32" fn SetCommState(hFile: std.os.windows.HANDLE, lpDCB: *DCB) callconv(std.os.windows.WINAPI) std.os.windows.BOOL;

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
        .word_size = .CS8,
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

    try flushSerialPort(port, true, true);
    try flushSerialPort(port, true, false);
    try flushSerialPort(port, false, true);
    try flushSerialPort(port, false, false);
}

test "change control pins" {
    _ = changeControlPins;
}
