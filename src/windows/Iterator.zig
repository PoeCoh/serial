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

pub fn next(self: *Self) !?Description {
    defer self.index += 1;

    self.name_size = 256;
    self.data_size = 256;

    return switch (RegEnumValueA(self.key, self.index, &self.name, &self.name_size, null, null, &self.data, &self.data_size)) {
        0 => Description{
            .file_name = try std.fmt.bufPrint(&self.filepath_data, "\\\\.\\{s}", .{self.data[0 .. self.data_size - 1]}),
            .display_name = self.data[0 .. self.data_size - 1],
            .driver = self.name[0..self.name_size],
        },
        259 => null,
        else => error.WindowsError,
    };
}

const Self = @This();
const std = @import("std");
const Description = @import("../serial.zig").Description;
const HKEY = std.os.windows.HKEY;
const DWORD = std.os.windows.DWORD;
const LPSTR = std.os.windows.LPSTR;
const BYTE = std.os.windows.BYTE;
const LSTATUS = std.os.windows.LSTATUS;
const LPCSTR = std.os.windows.LPCSTR;
const REGSAM = std.os.windows.REGSAM;
const WINAPI = std.os.windows.WINAPI;

extern "advapi32" fn RegEnumValueA(
    hKey: HKEY,
    dwIndex: DWORD,
    lpValueName: LPSTR,
    lpcchValueName: *DWORD,
    lpReserved: ?*DWORD,
    lpType: ?*DWORD,
    lpData: [*]BYTE,
    lpcbData: *DWORD,
) callconv(WINAPI) LSTATUS;

extern "advapi32" fn RegOpenKeyExA(
    key: HKEY,
    lpSubKey: LPCSTR,
    ulOptions: DWORD,
    samDesired: REGSAM,
    phkResult: *HKEY,
) callconv(WINAPI) LSTATUS;

extern "advapi32" fn RegCloseKey(key: HKEY) callconv(WINAPI) LSTATUS;
