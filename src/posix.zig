const SerialPort = @import("SerialPort.zig");
const serial = @import("serial.zig");
const std = @import("std");
const VTIME = 5;
const VMIN = 6;
const VSTART = 8;
const VSTOP = 9;

pub fn configure(file: std.fs.File, config: serial.Config) !void {
    var settings = try std.posix.tcgetattr(file.handle);

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

    try std.posix.tcsetattr(file.handle, .NOW, settings);
}
