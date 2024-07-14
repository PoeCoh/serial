const std = @import("std");
const SerialPort = @import("SerialPort");

pub fn main() !u2 {
    // const port_name = if (@import("builtin").os.tag == .windows) "\\\\.\\COM1" else "/dev/ttyUSB0";

    var sp = SerialPort.init("COM3", .{
        .baud_rate = 115200,
        .word_size = .CS8,
        .parity = .none,
        .stop_bits = .one,
        .handshake = .none,
    });
    try sp.open();
    defer sp.close();
    try sp.write("Hello, World!\r\n");
    var buffer: [100]u8 = undefined;
    try sp.read(&buffer);

    return 0;
}
