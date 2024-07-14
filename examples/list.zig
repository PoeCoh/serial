const std = @import("std");
const SerialPort = @import("SerialPort");

pub fn main() !u8 {
    var iterator = try SerialPort.iterator();
    defer iterator.deinit();

    while (try iterator.next()) |port| {
        std.debug.print("path={s},\tname={s},\tdriver={s}\n", .{ port.file_name, port.display_name, port.driver orelse "<no driver recognized>" });
    }

    return 0;
}
