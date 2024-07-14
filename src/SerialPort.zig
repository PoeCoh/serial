p: Private,

pub fn init(port: []const u8, config: SerialPort.Config) SerialPort {
    return .{
        .p = .{
            .config = config,
            .name = port,
        },
    };
}

pub fn open(self: *SerialPort) !void {
    self.p.file = try std.fs.cwd().openFile(self.p.name, .{ .mode = .read_write });
    const file = self.p.file orelse return error.FileOpenError;
    errdefer {
        file.close();
        self.p.file = null;
    }
    return switch (native_os) {
        .windows => SerialPort.configure(file, self.p.config),
        else => posixConfigure(file, self.p.config),
    };
}

pub fn close(self: *SerialPort) void {
    const file = self.p.file orelse return;
    file.close();
    self.p.file = null;
}

pub fn read(self: *SerialPort, buffer: []u8) !usize {
    const file = self.p.file orelse return error.PortClosed;
    return file.read(buffer);
}

pub fn write(self: *SerialPort, buffer: []const u8) !usize {
    const file = self.p.file orelse return error.PortClosed;
    return file.write(buffer);
}

pub fn flush(self: *SerialPort, buffers: SerialPort.Buffers) !void {
    const file = self.p.file orelse return error.PortClosed;
    const buffer = @intFromEnum(buffers);
    return SerialPort.pFlush(file, buffer);
}

pub fn controlPins(self: *SerialPort, pins: SerialPort.ControlPins) !void {
    const file = self.p.file orelse return error.PortClosed;
    return SerialPort.pControlPins(file, pins);
}

pub fn iterator() !SerialPort.Iterator {
    return SerialPort.Iterator.init();
}

pub fn infoIterator() !SerialPort.InformationIterator {
    return SerialPort.InformationIterator.init();
}

pub fn isOpen(self: *SerialPort) bool {
    return self.p.file != null;
}

usingnamespace @import("serial.zig");
usingnamespace switch (native_os) {
    .windows => @import("windows/serial.zig"),
    .macos, .ios, .tvos, .watchos, .visonos => @import("darwin/serial.zig"),
    else => @import("linux/serial.zig"),
};

const SerialPort = @This();
const std = @import("std");
const native_os = @import("builtin").os.tag;

const Private = struct {
    config: SerialPort.Config,
    name: []const u8,
    file: ?std.fs.File = null,
};

fn posixConfigure(file: std.fs.File, config: SerialPort.Config) !void {
    const VTIME = 5;
    const VMIN = 6;
    const VSTART = 8;
    const VSTOP = 9;

    var settings = try std.posix.tcgetattr(file.handle);
    settings.iflag = .{};
    settings.oflag = .{};
    settings.cflag = .{ .CREAD = true };
    settings.lflag = .{};
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

    try std.posix.tcsetattr(file.handle, .NOW, settings);
}
