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

pub fn next(self: *Self) !?Description {
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

                return Description{
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

const Self = @This();
const std = @import("std");
const Description = @import("serial").Description;
const root_dir = "/dev/";

