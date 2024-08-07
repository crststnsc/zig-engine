const std = @import("std");
const rl: type = @cImport(@cInclude("raylib.h"));
const obj: type = @import("obj");
const assert = std.debug.assert;

const Vec3D = struct { 
    x: f32 = 0, 
    y: f32 = 0, 
    z: f32 = 0
};

const Triangle = struct {
    vertices: [3]Vec3D,
    color: rl.Color = rl.Color{ .r = 255, .g = 255, .b = 255, .a = 255 },
};

const Mesh = struct {
    triangles: std.ArrayList(Triangle),

    pub fn init(allocator: std.mem.Allocator) Mesh {
        const triangles = std.ArrayList(Triangle).init(allocator);
        return Mesh{ .triangles = triangles };
    }

    pub fn deinit(self: Mesh) void {
        self.triangles.deinit();
    }
};

const Mat4x4 = struct {
    m: [4][4]f32 = std.mem.zeroes([4][4]f32),
};

fn multiplyVec3D(input_vector: Vec3D, output_vector: *Vec3D, matrix: Mat4x4) !void {
    output_vector.x = input_vector.x * matrix.m[0][0] + input_vector.y * matrix.m[1][0] + input_vector.z * matrix.m[2][0] + matrix.m[3][0];
    output_vector.y = input_vector.x * matrix.m[0][1] + input_vector.y * matrix.m[1][1] + input_vector.z * matrix.m[2][1] + matrix.m[3][1];
    output_vector.z = input_vector.x * matrix.m[0][2] + input_vector.y * matrix.m[1][2] + input_vector.z * matrix.m[2][2] + matrix.m[3][2];

    const w: f32 = input_vector.x * matrix.m[0][3] + input_vector.y * matrix.m[1][3] + input_vector.z * matrix.m[2][3] + matrix.m[3][3];
    if (w != 0.0) {
        output_vector.x /= w;
        output_vector.y /= w;
        output_vector.z /= w;
    }
}

fn dotProduct(v1: Vec3D, v2: Vec3D) f32 {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

fn drawTriangle(tri: Triangle, color: rl.Color) !void {
    rl.DrawTriangle(rl.Vector2{ .x = tri.vertices[0].x, .y = tri.vertices[0].y }, rl.Vector2{ .x = tri.vertices[1].x, .y = tri.vertices[1].y }, rl.Vector2{ .x = tri.vertices[2].x, .y = tri.vertices[2].y }, color);
}

fn compareTrianglesZ(_: void, t1: Triangle, t2: Triangle) bool {
    const z1: f32 = (t1.vertices[0].z + t1.vertices[1].z + t1.vertices[2].z) / 3.0;
    const z2: f32 = (t2.vertices[0].z + t2.vertices[1].z + t2.vertices[2].z) / 3.0;
    return z1 > z2;
}

fn makeMeshFromObj(comptime file_path: []const u8, allocator: std.mem.Allocator) !Mesh {
    var mesh_to_return = Mesh.init(allocator);

    const model = try obj.parseObj(allocator, @embedFile(file_path));
    const vertices = model.vertices;

    for (model.meshes) |mesh| {
        const indices = mesh.indices;
        const num_vertices = mesh.num_vertices;

        var index_offset: usize = 0;
        for (num_vertices) |num_verts| {
            if (num_verts == 3) { // Handle triangles directly
                const first_vertex_index: usize = @intCast(indices[index_offset].vertex.?);
                const second_vertex_index: usize = @intCast(indices[index_offset + 1].vertex.?);
                const third_vertex_index: usize = @intCast(indices[index_offset + 2].vertex.?);

                const triangle = Triangle{
                    .vertices = [3]Vec3D{
                        Vec3D{
                            .x = vertices[first_vertex_index * 3],
                            .y = vertices[first_vertex_index * 3 + 1],
                            .z = vertices[first_vertex_index * 3 + 2],
                        },
                        Vec3D{
                            .x = vertices[second_vertex_index * 3],
                            .y = vertices[second_vertex_index * 3 + 1],
                            .z = vertices[second_vertex_index * 3 + 2],
                        },
                        Vec3D{
                            .x = vertices[third_vertex_index * 3],
                            .y = vertices[third_vertex_index * 3 + 1],
                            .z = vertices[third_vertex_index * 3 + 2],
                        },
                    },
                };
                try mesh_to_return.triangles.append(triangle);
            } else if (num_verts > 3) {
                // Handle polygons with more than 3 vertices by triangulating them
                for (0..(num_verts - 2)) |i| {
                    const first_vertex_index: usize = @intCast(indices[index_offset].vertex.?);
                    const second_vertex_index: usize = @intCast(indices[index_offset + i + 1].vertex.?);
                    const third_vertex_index: usize = @intCast(indices[index_offset + i + 2].vertex.?);

                    const triangle = Triangle{
                        .vertices = [3]Vec3D{
                            Vec3D{
                                .x = vertices[first_vertex_index * 3],
                                .y = vertices[first_vertex_index * 3 + 1],
                                .z = vertices[first_vertex_index * 3 + 2],
                            },
                            Vec3D{
                                .x = vertices[second_vertex_index * 3],
                                .y = vertices[second_vertex_index * 3 + 1],
                                .z = vertices[second_vertex_index * 3 + 2],
                            },
                            Vec3D{
                                .x = vertices[third_vertex_index * 3],
                                .y = vertices[third_vertex_index * 3 + 1],
                                .z = vertices[third_vertex_index * 3 + 2],
                            },
                        },
                    };
                    try mesh_to_return.triangles.append(triangle);
                }
            }

            index_offset += num_verts;
        }
    }

    return mesh_to_return;
}

fn renderTriangles(triangles: []Triangle) !void {
    for (triangles) |triangle| {
        try drawTriangle(triangle, triangle.color);
    }
}

fn applyRotationX(triangle: *Triangle, rotation_value: f32) void {
    const rotation_matrix_x: Mat4x4 = Mat4x4{ 
        .m = [4][4]f32{
            [4]f32{ 1.0, 0.0, 0.0, 0.0 },
            [4]f32{ 0.0, std.math.cos(rotation_value), std.math.sin(rotation_value), 0.0 },
            [4]f32{ 0.0, -std.math.sin(rotation_value), std.math.cos(rotation_value), 0.0 },
            [4]f32{ 0.0, 0.0, 0.0, 1.0 },
        } 
    };

    try multiplyVec3D(triangle.vertices[0], &triangle.vertices[0], rotation_matrix_x);
    try multiplyVec3D(triangle.vertices[1], &triangle.vertices[1], rotation_matrix_x);
    try multiplyVec3D(triangle.vertices[2], &triangle.vertices[2], rotation_matrix_x);
}

fn applyRotationZ(triangle: *Triangle, rotation_value: f32) void {
    const rotation_matrix_z: Mat4x4 = Mat4x4{
        .m = [4][4]f32{
            [4]f32{ std.math.cos(rotation_value), std.math.sin(rotation_value), 0.0, 0.0 },
            [4]f32{ -std.math.sin(rotation_value), std.math.cos(rotation_value), 0.0, 0.0 },
            [4]f32{ 0.0, 0.0, 1.0, 0.0 },
            [4]f32{ 0.0, 0.0, 0.0, 1.0 },
        }   
    };

    try multiplyVec3D(triangle.vertices[0], &triangle.vertices[0], rotation_matrix_z);
    try multiplyVec3D(triangle.vertices[1], &triangle.vertices[1], rotation_matrix_z);
    try multiplyVec3D(triangle.vertices[2], &triangle.vertices[2], rotation_matrix_z);
}

pub fn main() !void {
    const window_width = 800;
    const window_height = 600;

    rl.InitWindow(window_width, window_height, "Zig Engine");
    defer rl.CloseWindow();

    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const allocator = gpa.allocator();

    // rendering mesh
    var mesh_to_render: Mesh = try makeMeshFromObj("tree.obj", allocator);
    defer mesh_to_render.deinit();

    const number_of_triangles = mesh_to_render.triangles.items.len;
    std.debug.print("Number of triangles: {d}\n", .{number_of_triangles});

    // Projection matrix
    const near: f16 = 0.001;
    const far: f16 = 1000.0;
    const fov: f16 = 90.0;
    const aspect_ratio: f16 = window_width / window_height;

    const fov_rad: f16 = 1.0 / std.math.tan(std.math.degreesToRadians(fov / 2.0));
    const projection_matrix: Mat4x4 = Mat4x4{ .m = [4][4]f32{
        [4]f32{ fov_rad / aspect_ratio, 0.0, 0.0, 0.0 },
        [4]f32{ 0.0, fov_rad, 0.0, 0.0 },
        [4]f32{ 0.0, 0.0, far / (far - near), 1.0 },
        [4]f32{ 0.0, 0.0, (-far * near) / (far - near), 0.0 },
    } };

    const camera: Vec3D = Vec3D{ .x = 0.0, .y = 0.0, .z = -50.0 };

    var triangles_to_rasterize = std.ArrayList(Triangle).init(allocator);
    defer triangles_to_rasterize.deinit();

    while (!rl.WindowShouldClose()) {
        const time: f32 = @floatCast(rl.GetTime());

        for (mesh_to_render.triangles.items) |mesh_triangle| {
            var triangle_to_process = mesh_triangle;

            applyRotationX(&triangle_to_process, time);
            applyRotationZ(&triangle_to_process, time / 2);

            triangle_to_process.vertices[0].z += 50.0;
            triangle_to_process.vertices[1].z += 50.0;
            triangle_to_process.vertices[2].z += 50.0;

            var normal: Vec3D = undefined;
            var line1: Vec3D = undefined;
            var line2: Vec3D = undefined;

            line1.x = triangle_to_process.vertices[1].x - triangle_to_process.vertices[0].x;
            line1.y = triangle_to_process.vertices[1].y - triangle_to_process.vertices[0].y;
            line1.z = triangle_to_process.vertices[1].z - triangle_to_process.vertices[0].z;

            line2.x = triangle_to_process.vertices[2].x - triangle_to_process.vertices[0].x;
            line2.y = triangle_to_process.vertices[2].y - triangle_to_process.vertices[0].y;
            line2.z = triangle_to_process.vertices[2].z - triangle_to_process.vertices[0].z;

            normal.x = line1.y * line2.z - line1.z * line2.y;
            normal.y = line1.z * line2.x - line1.x * line2.z;
            normal.z = line1.x * line2.y - line1.y * line2.x;

            const l: f32 = std.math.sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
            normal.x /= l;
            normal.y /= l;
            normal.z /= l;

            const camera_ray: Vec3D = Vec3D{ .x = triangle_to_process.vertices[0].x - camera.x, .y = triangle_to_process.vertices[0].y - camera.y, .z = triangle_to_process.vertices[0].z - camera.z };

            if (dotProduct(camera_ray, normal) > 0.0) {
                continue;
            }

            var light_direction: Vec3D = Vec3D{ .x = 0.0, .y = 0.0, .z = -1.0 };
            const light_direction_vector_length: f32 = std.math.sqrt(light_direction.x * light_direction.x + light_direction.y * light_direction.y + light_direction.z * light_direction.z);
            light_direction.x /= light_direction_vector_length;
            light_direction.y /= light_direction_vector_length;
            light_direction.z /= light_direction_vector_length;

            var light_intensity: f32 = dotProduct(light_direction, normal);
            light_intensity = std.math.clamp(light_intensity, -1.0, 1.0);

            const normalized_intensity: f32 = (light_intensity + 1) / 2;

            const color: rl.Color = rl.Color{
                .a = 255,
                .r = @intFromFloat(255 * normalized_intensity),
                .g = @intFromFloat(255 * normalized_intensity),
                .b = @intFromFloat(255 * normalized_intensity),
            };

            var triangle_to_project: Triangle = triangle_to_process;

            try multiplyVec3D(triangle_to_process.vertices[0], &triangle_to_project.vertices[0], projection_matrix);
            try multiplyVec3D(triangle_to_process.vertices[1], &triangle_to_project.vertices[1], projection_matrix);
            try multiplyVec3D(triangle_to_process.vertices[2], &triangle_to_project.vertices[2], projection_matrix);

            triangle_to_project.vertices[0].x = (triangle_to_project.vertices[0].x + 1.0) * 0.5 * window_width;
            triangle_to_project.vertices[0].y = (triangle_to_project.vertices[0].y + 1.0) * 0.5 * window_height;
            triangle_to_project.vertices[1].x = (triangle_to_project.vertices[1].x + 1.0) * 0.5 * window_width;
            triangle_to_project.vertices[1].y = (triangle_to_project.vertices[1].y + 1.0) * 0.5 * window_height;
            triangle_to_project.vertices[2].x = (triangle_to_project.vertices[2].x + 1.0) * 0.5 * window_width;
            triangle_to_project.vertices[2].y = (triangle_to_project.vertices[2].y + 1.0) * 0.5 * window_height;

            triangle_to_project.color = color;

            try triangles_to_rasterize.append(triangle_to_project);
        }
        std.sort.pdq(Triangle, triangles_to_rasterize.items, {}, compareTrianglesZ);

        rl.BeginDrawing();
        try renderTriangles(triangles_to_rasterize.items);
        rl.ClearBackground(rl.BLACK);
        rl.DrawFPS(10, 10);
        rl.EndDrawing();

        triangles_to_rasterize.clearRetainingCapacity();
    }
}
