const std = @import("std");
const native_os = @import("builtin").os.tag;

pub const Description = struct {
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
    else => std.posix.CSIZE,
};

pub const Config = struct {
    /// Symbol rate in bits/second. Not that these
    /// include also parity and stop bits.
    baud_rate: u32 = 9600,

    /// Parity to verify transport integrity.
    parity: Parity = .none,

    /// Number of stop bits after the data
    stop_bits: StopBits = .one,

    /// Number of data bits per word.
    word_size: WordSize = .CS8,

    /// Defines the handshake protocol used.
    handshake: Handshake = .none,
};

pub const ControlPins = struct {
    rts: ?bool = null,
    dtr: ?bool = null,
};

/// This function configures a serial port with the given config.
/// `port` is an already opened serial port, on windows these
/// are either called `\\.\COMxx\` or `COMx`, on unixes the serial
/// port is called `/dev/ttyXXX`.
pub fn configure(file: std.fs.File, config: Config) !void {
    const VTIME = 5;
    const VMIN = 6;
    const VSTART = 8;
    const VSTOP = 9;
    var settings = try std.posix.tcgetattr(file.handle);

    settings.iflag = .{};
    settings.oflag = .{};
    settings.cflag = .{ .CREAD = true };
    settings.lflag = .{};

    settings.cflag.PARODD = (config.parity == .odd) or (config.parity == .mark);
    if (config.parity == .mark) settings.cflag._ |= (1 << 14);
    if (config.parity == .space) settings.cflag._ |= 1;
    settings.iflag.INPCK = config.parity != .none;
    settings.cflag.PARENB = config.parity != .none;
    settings.cflag.LOCAL = config.handshake == .none;
    settings.iflag.IXON = config.handshake == .software;
    settings.iflag.IXOFF = config.handshake == .software;
    if (config.handshake == .hardware) settings.cflag._ |= 1 << 15;
    settings.cflag.CSTOPB = config.stop_bits == .two;
    settings.cflag.CSIZE = config.word_size;

    settings.ispeed = @intFromEnum(config.baud_rate);
    settings.ospeed = @intFromEnum(config.baud_rate);

    settings.cc[VMIN] = 1;
    settings.cc[VSTOP] = 0x13; // XOFF
    settings.cc[VSTART] = 0x11; // XON
    settings.cc[VTIME] = 0;

    try std.posix.tcsetattr(file.handle, .NOW, settings);
}
