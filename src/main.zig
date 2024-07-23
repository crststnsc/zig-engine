const std = @import("std");
const rl: type = @cImport(@cInclude("raylib.h"));
const obj: type = @import("obj");
const assert = std.debug.assert;

const Vec3D = struct { x: f32, y: f32, z: f32 };

const Triangle = struct { vertices: [3]Vec3D };

const Mesh = struct { triangles: []Triangle };

const Mat4x4 = struct { m: [4][4]f32 = std.mem.zeroes([4][4]f32) };

fn multiplyVec3D(output_vector: *Vec3D, input_vector: Vec3D, matrix: Mat4x4) !void {
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

fn makeMeshFromObj(comptime file_path: []const u8) !Mesh {
    const allocator = std.heap.page_allocator;

    const model = try obj.parseObj(allocator, @embedFile(file_path));

    const vertices = model.vertices;
    std.debug.print("Number of vertices: {}\n", .{vertices.len});

    var triangles = std.ArrayList(Triangle).init(allocator);

    for (model.meshes) |mesh| {
        const indices = mesh.indices;
        const indices_len = indices.len / 3;
        for (0..indices_len) |i| {
            const base_index = i * 3;
  
            const first_vertex_index: usize = @intCast(indices[base_index].vertex.?);
            const second_vertex_index: usize = @intCast(indices[base_index + 1].vertex.?);
            const third_vertex_index: usize = @intCast(indices[base_index + 2].vertex.?);

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
            try triangles.append(triangle);
        }
    }

    const mesh = Mesh{ .triangles = triangles.items };
    return mesh;

    // var vertices = std.ArrayList(Vec3D).init(allocator);
    // defer vertices.deinit();

    // var triangles = std.ArrayList(Triangle).init(allocator);

    // var buf_reader = std.io.bufferedReader(file.reader());
    // var in_stream = buf_reader.reader();
    // var buf: [64]u8 = undefined;
    // while (try in_stream.readUntilDelimiterOrEof(&buf, '\n')) |line| {
    //     var line_parts = std.mem.splitSequence(u8, line, " ");

    //     const first_token = line_parts.next().?;

    //     if (std.mem.eql(u8, first_token, "v")) {
    //         var vertex = Vec3D{ .x = 0.0, .y = 0.0, .z = 0.0 };
    //         vertex.x = try std.fmt.parseFloat(f32, line_parts.next().?);
    //         vertex.y = try std.fmt.parseFloat(f32, line_parts.next().?);
    //         vertex.z = try std.fmt.parseFloat(f32, line_parts.next().?);
    //         try vertices.append(vertex);
    //     } else if (std.mem.eql(u8, first_token, "f")) {
    //         while (line_parts.next()) |faces| {
    //             var face_parts = std.mem.splitSequence(u8, faces, "/");
    //             const v1 = try std.fmt.parseInt(u32, face_parts.next().?, 10);
    //             const v2 = try std.fmt.parseInt(u32, face_parts.next().?, 10);
    //             const v3 = try std.fmt.parseInt(u32, face_parts.next().?, 10);

    //             const triangle = Triangle{ .vertices = [3]Vec3D{ vertices.items[v1 - 1], vertices.items[v2 - 1], vertices.items[v3 - 1] } };
    //             try triangles.append(triangle);
    //         }
    //     }

    // }
    // const mesh = Mesh{ .triangles = triangles.items };
    // return mesh;
}

pub fn main() !void {
    rl.InitWindow(800, 600, "Zig Engine");
    defer rl.CloseWindow();

    // const vertices = [_]Vec3D{
    //     // Define vertices for a cube
    //     // South face
    //     Vec3D{ .x = 0, .y = 0, .z = 0 },
    //     Vec3D{ .x = 0, .y = 1, .z = 0 },
    //     Vec3D{ .x = 1, .y = 1, .z = 0 },
    //     Vec3D{ .x = 0, .y = 0, .z = 0 },
    //     Vec3D{ .x = 1, .y = 1, .z = 0 },
    //     Vec3D{ .x = 1, .y = 0, .z = 0 },

    //     // East face
    //     Vec3D{ .x = 1, .y = 0, .z = 0 },
    //     Vec3D{ .x = 1, .y = 1, .z = 0 },
    //     Vec3D{ .x = 1, .y = 1, .z = 1 },
    //     Vec3D{ .x = 1, .y = 0, .z = 0 },
    //     Vec3D{ .x = 1, .y = 1, .z = 1 },
    //     Vec3D{ .x = 1, .y = 0, .z = 1 },

    //     // North face
    //     Vec3D{ .x = 1, .y = 0, .z = 1 },
    //     Vec3D{ .x = 1, .y = 1, .z = 1 },
    //     Vec3D{ .x = 0, .y = 1, .z = 1 },
    //     Vec3D{ .x = 1, .y = 0, .z = 1 },
    //     Vec3D{ .x = 0, .y = 1, .z = 1 },
    //     Vec3D{ .x = 0, .y = 0, .z = 1 },

    //     // West face
    //     Vec3D{ .x = 0, .y = 0, .z = 1 },
    //     Vec3D{ .x = 0, .y = 1, .z = 1 },
    //     Vec3D{ .x = 0, .y = 1, .z = 0 },
    //     Vec3D{ .x = 0, .y = 0, .z = 1 },
    //     Vec3D{ .x = 0, .y = 1, .z = 0 },
    //     Vec3D{ .x = 0, .y = 0, .z = 0 },

    //     // Top face
    //     Vec3D{ .x = 0, .y = 1, .z = 0 },
    //     Vec3D{ .x = 0, .y = 1, .z = 1 },
    //     Vec3D{ .x = 1, .y = 1, .z = 1 },
    //     Vec3D{ .x = 0, .y = 1, .z = 0 },
    //     Vec3D{ .x = 1, .y = 1, .z = 1 },
    //     Vec3D{ .x = 1, .y = 1, .z = 0 },

    //     // Bottom face
    //     Vec3D{ .x = 1, .y = 0, .z = 0 },
    //     Vec3D{ .x = 1, .y = 0, .z = 1 },
    //     Vec3D{ .x = 0, .y = 0, .z = 1 },
    //     Vec3D{ .x = 1, .y = 0, .z = 0 },
    //     Vec3D{ .x = 0, .y = 0, .z = 1 },
    //     Vec3D{ .x = 0, .y = 0, .z = 0 },
    // };

    // var triangles = [_]Triangle{
    //     Triangle{ .vertices = [3]Vec3D{ vertices[0], vertices[1], vertices[2] } },
    //     Triangle{ .vertices = [3]Vec3D{ vertices[3], vertices[4], vertices[5] } },
    //     Triangle{ .vertices = [3]Vec3D{ vertices[6], vertices[7], vertices[8] } },
    //     Triangle{ .vertices = [3]Vec3D{ vertices[9], vertices[10], vertices[11] } },
    //     Triangle{ .vertices = [3]Vec3D{ vertices[12], vertices[13], vertices[14] } },
    //     Triangle{ .vertices = [3]Vec3D{ vertices[15], vertices[16], vertices[17] } },
    //     Triangle{ .vertices = [3]Vec3D{ vertices[18], vertices[19], vertices[20] } },
    //     Triangle{ .vertices = [3]Vec3D{ vertices[21], vertices[22], vertices[23] } },
    //     Triangle{ .vertices = [3]Vec3D{ vertices[24], vertices[25], vertices[26] } },
    //     Triangle{ .vertices = [3]Vec3D{ vertices[27], vertices[28], vertices[29] } },
    //     Triangle{ .vertices = [3]Vec3D{ vertices[30], vertices[31], vertices[32] } },
    //     Triangle{ .vertices = [3]Vec3D{ vertices[33], vertices[34], vertices[35] } },
    // };

    // rendering a cube
    const mesh_to_render: Mesh = try makeMeshFromObj("skull.obj");

    // Projection matrix
    const near: f16 = 0.001;
    const far: f16 = 1000.0;
    const fov: f16 = 90.0;
    const aspect_ratio: f16 = 800.0 / 600.0;

    const fov_rad: f16 = 1.0 / std.math.tan(std.math.degreesToRadians(fov / 2.0));
    const projection_matrix: Mat4x4 = Mat4x4{ .m = [4][4]f32{
        [4]f32{ fov_rad / aspect_ratio, 0.0, 0.0, 0.0 },
        [4]f32{ 0.0, fov_rad, 0.0, 0.0 },
        [4]f32{ 0.0, 0.0, far / (far - near), 1.0 },
        [4]f32{ 0.0, 0.0, (-far * near) / (far - near), 0.0 },
    } };

    const cube_center: Vec3D = Vec3D{ .x = 0.5, .y = 0.5, .z = 0.5 };
    const camera: Vec3D = Vec3D{ .x = 0.0, .y = 0.0, .z = 0.0 };

    while (!rl.WindowShouldClose()) {
        //calculate roatation matrices
        const time: f32 = @floatCast(rl.GetTime());
        const rotation_x: Mat4x4 = Mat4x4{ .m = [4][4]f32{
            [4]f32{ 1.0, 0.0, 0.0, 0.0 },
            [4]f32{ 0.0, std.math.cos(time / 2), std.math.sin(time / 2), 0.0 },
            [4]f32{ 0.0, -std.math.sin(time / 2), std.math.cos(time / 2), 0.0 },
            [4]f32{ 0.0, 0.0, 0.0, 1.0 },
        } };
        const rotation_z: Mat4x4 = Mat4x4{ .m = [4][4]f32{
            [4]f32{ std.math.cos(time), std.math.sin(time), 0.0, 0.0 },
            [4]f32{ -std.math.sin(time), std.math.cos(time), 0.0, 0.0 },
            [4]f32{ 0.0, 0.0, 1.0, 0.0 },
            [4]f32{ 0.0, 0.0, 0.0, 1.0 },
        } };

        rl.BeginDrawing();

        for (mesh_to_render.triangles) |mesh_triangle| {
            var tri_translated_to_origin: Triangle = mesh_triangle;
            var tri_roatated_x: Triangle = mesh_triangle;
            var tri_roatated_z: Triangle = mesh_triangle;

            tri_translated_to_origin.vertices[0].x -= cube_center.x;
            tri_translated_to_origin.vertices[0].y -= cube_center.y;
            tri_translated_to_origin.vertices[0].z -= cube_center.z;
            tri_translated_to_origin.vertices[1].x -= cube_center.x;
            tri_translated_to_origin.vertices[1].y -= cube_center.y;
            tri_translated_to_origin.vertices[1].z -= cube_center.z;
            tri_translated_to_origin.vertices[2].x -= cube_center.x;
            tri_translated_to_origin.vertices[2].y -= cube_center.y;
            tri_translated_to_origin.vertices[2].z -= cube_center.z;

            try multiplyVec3D(&tri_roatated_x.vertices[0], tri_translated_to_origin.vertices[0], rotation_x);
            try multiplyVec3D(&tri_roatated_x.vertices[1], tri_translated_to_origin.vertices[1], rotation_x);
            try multiplyVec3D(&tri_roatated_x.vertices[2], tri_translated_to_origin.vertices[2], rotation_x);

            try multiplyVec3D(&tri_roatated_z.vertices[0], tri_roatated_x.vertices[0], rotation_z);
            try multiplyVec3D(&tri_roatated_z.vertices[1], tri_roatated_x.vertices[1], rotation_z);
            try multiplyVec3D(&tri_roatated_z.vertices[2], tri_roatated_x.vertices[2], rotation_z);

            var transformed_triangle = Triangle{ .vertices = [3]Vec3D{
                Vec3D{ .x = 0, .y = 0, .z = 0 },
                Vec3D{ .x = 0, .y = 0, .z = 0 },
                Vec3D{ .x = 0, .y = 0, .z = 0 },
            } };
            var translated_triangle = tri_roatated_z;

            translated_triangle.vertices[0].z += 50.0;
            translated_triangle.vertices[1].z += 50.0;
            translated_triangle.vertices[2].z += 50.0;

            var normal: Vec3D = undefined;
            var line1: Vec3D = undefined;
            var line2: Vec3D = undefined;

            line1.x = translated_triangle.vertices[1].x - translated_triangle.vertices[0].x;
            line1.y = translated_triangle.vertices[1].y - translated_triangle.vertices[0].y;
            line1.z = translated_triangle.vertices[1].z - translated_triangle.vertices[0].z;

            line2.x = translated_triangle.vertices[2].x - translated_triangle.vertices[0].x;
            line2.y = translated_triangle.vertices[2].y - translated_triangle.vertices[0].y;
            line2.z = translated_triangle.vertices[2].z - translated_triangle.vertices[0].z;

            normal.x = line1.y * line2.z - line1.z * line2.y;
            normal.y = line1.z * line2.x - line1.x * line2.z;
            normal.z = line1.x * line2.y - line1.y * line2.x;

            const l: f32 = std.math.sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
            normal.x /= l;
            normal.y /= l;
            normal.z /= l;

            const camera_ray: Vec3D = Vec3D{ .x = translated_triangle.vertices[0].x - camera.x, .y = translated_triangle.vertices[0].y - camera.y, .z = translated_triangle.vertices[0].z - camera.z };

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

            try multiplyVec3D(&transformed_triangle.vertices[0], translated_triangle.vertices[0], projection_matrix);
            try multiplyVec3D(&transformed_triangle.vertices[1], translated_triangle.vertices[1], projection_matrix);
            try multiplyVec3D(&transformed_triangle.vertices[2], translated_triangle.vertices[2], projection_matrix);

            transformed_triangle.vertices[0].x = (transformed_triangle.vertices[0].x + 1.0) * 0.5 * 800.0;
            transformed_triangle.vertices[0].y = (transformed_triangle.vertices[0].y + 1.0) * 0.5 * 600.0;
            transformed_triangle.vertices[1].x = (transformed_triangle.vertices[1].x + 1.0) * 0.5 * 800.0;
            transformed_triangle.vertices[1].y = (transformed_triangle.vertices[1].y + 1.0) * 0.5 * 600.0;
            transformed_triangle.vertices[2].x = (transformed_triangle.vertices[2].x + 1.0) * 0.5 * 800.0;
            transformed_triangle.vertices[2].y = (transformed_triangle.vertices[2].y + 1.0) * 0.5 * 600.0;

            try drawTriangle(transformed_triangle, color);
        }

        rl.ClearBackground(rl.BLACK);
        rl.DrawFPS(10, 10);
        rl.EndDrawing();
    }
}
