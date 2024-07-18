const std = @import("std");
const rl: type = @cImport(@cInclude("raylib.h"));

const Vec3D = struct { x: f32, y: f32, z: f32 };

const triangle = struct { vertices: [3]Vec3D };

const mesh = struct { triangles: []triangle };

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

fn drawTriangle(tri: triangle, color: rl.Color) !void {
    rl.DrawTriangleLines(
        rl.Vector2{ .x = tri.vertices[0].x, .y = tri.vertices[0].y },
        rl.Vector2{ .x = tri.vertices[1].x, .y = tri.vertices[1].y },
        rl.Vector2{ .x = tri.vertices[2].x, .y = tri.vertices[2].y },
        color
    );
}

pub fn main() !void {
    rl.InitWindow(800, 600, "Zig Engine");
    defer rl.CloseWindow();

    const vertices = [_]Vec3D{
        // Define vertices for a cube
        // South face
        Vec3D{ .x = 0, .y = 0, .z = 0 },
        Vec3D{ .x = 0, .y = 1, .z = 0 },
        Vec3D{ .x = 1, .y = 1, .z = 0 },
        Vec3D{ .x = 0, .y = 0, .z = 0 },
        Vec3D{ .x = 1, .y = 1, .z = 0 },
        Vec3D{ .x = 1, .y = 0, .z = 0 },

        // East face
        Vec3D{ .x = 1, .y = 0, .z = 0 },
        Vec3D{ .x = 1, .y = 1, .z = 0 },
        Vec3D{ .x = 1, .y = 1, .z = 1 },
        Vec3D{ .x = 1, .y = 0, .z = 0 },
        Vec3D{ .x = 1, .y = 1, .z = 1 },
        Vec3D{ .x = 1, .y = 0, .z = 1 },

        // North face
        Vec3D{ .x = 1, .y = 0, .z = 1 },
        Vec3D{ .x = 1, .y = 1, .z = 1 },
        Vec3D{ .x = 0, .y = 1, .z = 1 },
        Vec3D{ .x = 1, .y = 0, .z = 1 },
        Vec3D{ .x = 0, .y = 1, .z = 1 },
        Vec3D{ .x = 0, .y = 0, .z = 1 },

        // West face
        Vec3D{ .x = 0, .y = 0, .z = 1 },
        Vec3D{ .x = 0, .y = 1, .z = 1 },
        Vec3D{ .x = 0, .y = 1, .z = 0 },
        Vec3D{ .x = 0, .y = 0, .z = 1 },
        Vec3D{ .x = 0, .y = 1, .z = 0 },
        Vec3D{ .x = 0, .y = 0, .z = 0 },

        // Top face
        Vec3D{ .x = 0, .y = 1, .z = 0 },
        Vec3D{ .x = 0, .y = 1, .z = 1 },
        Vec3D{ .x = 1, .y = 1, .z = 1 },
        Vec3D{ .x = 0, .y = 1, .z = 0 },
        Vec3D{ .x = 1, .y = 1, .z = 1 },
        Vec3D{ .x = 1, .y = 1, .z = 0 },

        // Bottom face
        Vec3D{ .x = 1, .y = 0, .z = 0 },
        Vec3D{ .x = 1, .y = 0, .z = 1 },
        Vec3D{ .x = 0, .y = 0, .z = 1 },
        Vec3D{ .x = 1, .y = 0, .z = 0 },
        Vec3D{ .x = 0, .y = 0, .z = 1 },
        Vec3D{ .x = 0, .y = 0, .z = 0 },
    };

    var triangles = [_]triangle{
        triangle{ .vertices = [3]Vec3D{ vertices[0], vertices[1], vertices[2] } },
        triangle{ .vertices = [3]Vec3D{ vertices[3], vertices[4], vertices[5] } },
        triangle{ .vertices = [3]Vec3D{ vertices[6], vertices[7], vertices[8] } },
        triangle{ .vertices = [3]Vec3D{ vertices[9], vertices[10], vertices[11] } },
        triangle{ .vertices = [3]Vec3D{ vertices[12], vertices[13], vertices[14] } },
        triangle{ .vertices = [3]Vec3D{ vertices[15], vertices[16], vertices[17] } },
        triangle{ .vertices = [3]Vec3D{ vertices[18], vertices[19], vertices[20] } },
        triangle{ .vertices = [3]Vec3D{ vertices[21], vertices[22], vertices[23] } },
        triangle{ .vertices = [3]Vec3D{ vertices[24], vertices[25], vertices[26] } },
        triangle{ .vertices = [3]Vec3D{ vertices[27], vertices[28], vertices[29] } },
        triangle{ .vertices = [3]Vec3D{ vertices[30], vertices[31], vertices[32] } },
        triangle{ .vertices = [3]Vec3D{ vertices[33], vertices[34], vertices[35] } },
    };

    // rendering a cube
    const mesh_to_render: mesh = mesh{ .triangles = &triangles };

    // Projection matrix
    const near: f16 = 0.1;
    const far: f16 = 1000.0;
    const fov: f16 = 90.0;
    const aspect_ratio: f16 = 800.0 / 600.0;

    const fov_rad: f16 = 1.0 / std.math.tan(std.math.degreesToRadians(fov / 2.0));
    const projection_matrix: Mat4x4 = Mat4x4{ 
        .m = [4][4]f32{
            [4]f32{ fov_rad / aspect_ratio, 0.0, 0.0, 0.0 },
            [4]f32{ 0.0, fov_rad, 0.0, 0.0 },
            [4]f32{ 0.0, 0.0, far / (far - near), 1.0 },
            [4]f32{ 0.0, 0.0, (-far * near) / (far - near), 0.0 },
        } 
    };


    while (!rl.WindowShouldClose()) {
        //calculate roatation matrices
        const time: f32 = @floatCast(rl.GetTime());
        const rotation_x: Mat4x4 = Mat4x4{ 
            .m = [4][4]f32{
                [4]f32{ 1.0, 0.0, 0.0, 0.0 },
                [4]f32{ 0.0, std.math.cos(time), -std.math.sin(time), 0.0 },
                [4]f32{ 0.0, std.math.sin(time), std.math.cos(time), 0.0 },
                [4]f32{ 0.0, 0.0, 0.0, 1.0 },
            } 
        };
        const rotation_z: Mat4x4 = Mat4x4{ 
            .m = [4][4]f32{
                [4]f32{ std.math.cos(time), -std.math.sin(time), 0.0, 0.0 },
                [4]f32{ std.math.sin(time), std.math.cos(time), 0.0, 0.0 },
                [4]f32{ 0.0, 0.0, 1.0, 0.0 },
                [4]f32{ 0.0, 0.0, 0.0, 1.0 },
            } 
        };

        rl.BeginDrawing();
        for (mesh_to_render.triangles) |mesh_triangle| {
            var tri_roatated_x: triangle = mesh_triangle;
            var tri_roatated_z: triangle = mesh_triangle;

            try multiplyVec3D(&tri_roatated_x.vertices[0], mesh_triangle.vertices[0], rotation_x);
            try multiplyVec3D(&tri_roatated_x.vertices[1], mesh_triangle.vertices[1], rotation_x);
            try multiplyVec3D(&tri_roatated_x.vertices[2], mesh_triangle.vertices[2], rotation_x);

            try multiplyVec3D(&tri_roatated_z.vertices[0], tri_roatated_x.vertices[0], rotation_z);
            try multiplyVec3D(&tri_roatated_z.vertices[1], tri_roatated_x.vertices[1], rotation_z);
            try multiplyVec3D(&tri_roatated_z.vertices[2], tri_roatated_x.vertices[2], rotation_z);

            var transformed_triangle = triangle{ 
                .vertices = [3]Vec3D{
                    Vec3D{ .x = 0, .y = 0, .z = 0 },
                    Vec3D{ .x = 0, .y = 0, .z = 0 },
                    Vec3D{ .x = 0, .y = 0, .z = 0 },
                }   
            };
            var translated_triangle = tri_roatated_z;
            translated_triangle.vertices[0].z += 3.0;
            translated_triangle.vertices[1].z += 3.0;
            translated_triangle.vertices[2].z += 3.0;

            try multiplyVec3D(&transformed_triangle.vertices[0], translated_triangle.vertices[0], projection_matrix);
            try multiplyVec3D(&transformed_triangle.vertices[1], translated_triangle.vertices[1], projection_matrix);
            try multiplyVec3D(&transformed_triangle.vertices[2], translated_triangle.vertices[2], projection_matrix);

            transformed_triangle.vertices[0].x = (transformed_triangle.vertices[0].x + 1.0) * 0.5 * 800.0;
            transformed_triangle.vertices[0].y = (transformed_triangle.vertices[0].y + 1.0) * 0.5 * 600.0;
            transformed_triangle.vertices[1].x = (transformed_triangle.vertices[1].x + 1.0) * 0.5 * 800.0;
            transformed_triangle.vertices[1].y = (transformed_triangle.vertices[1].y + 1.0) * 0.5 * 600.0;
            transformed_triangle.vertices[2].x = (transformed_triangle.vertices[2].x + 1.0) * 0.5 * 800.0;
            transformed_triangle.vertices[2].y = (transformed_triangle.vertices[2].y + 1.0) * 0.5 * 600.0;

            try drawTriangle(transformed_triangle, rl.RAYWHITE);
        }
        rl.ClearBackground(rl.BLACK);
        
        rl.DrawFPS(10, 10);
        rl.EndDrawing();
    }
}
