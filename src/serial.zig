const std = @import("std");
const native_os = @import("builtin").os.tag;

pub const Config = struct {
    baud_rate: u32 = 115200,
    parity: Parity = .none,
    stop_bits: StopBits = .one,
    word_size: WordSize = .CS8,
    handshake: Handshake = .none,
    /// Timeout in milliseconds. Only used on windows.
    timeout: u32 = 0,
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

pub const ControlPins = struct {
    rts: ?bool = null,
    dtr: ?bool = null,
};

const CBAUD = 0o000000010017; //Baud speed mask (not in POSIX).
const CMSPAR = 0o010000000000;
const CRTSCTS = 0o020000000000;