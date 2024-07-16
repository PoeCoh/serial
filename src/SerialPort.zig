file: std.fs.File,

pub fn iterator() Iterator {
    return Iterator.init();
}

const std = @import("std");
const native_os = @import("builtin").os.tag;

const Iterator = switch (native_os) {
    .macosx, .ios, .watchos, .tvos => @import("darwin/Iterator.zig"),
    else => @import("linux/Iterator.zig"),
};
