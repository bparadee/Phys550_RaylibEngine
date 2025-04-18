/*******************************************************************************************
*
*   raylib [models] example - Detect basic 3d collisions (box vs sphere vs box)
*
*   This example has been created using raylib 1.3 (www.raylib.com)
*   raylib is licensed under an unmodified zlib/libpng license (View raylib.h for details)
*
*   Copyright (c) 2015 Ramon Santamaria (@raysan5)
*
********************************************************************************************/

using Physics550Engine_Raylib;
using Physics550Engine_Raylib.Physics;
using Physics550Engine_Raylib.Physics.Interfaces;
using Raylib_cs;
using System.Diagnostics;
using System.Numerics;
using static Raylib_cs.Raylib;

namespace Examples.Models;

public class BoxCollisions
{
    public static int Main()
    {
        // Initialization
        //--------------------------------------------------------------------------------------
        const int screenWidth = 1600;
        const int screenHeight = 900;

        InitWindow(screenWidth, screenHeight, "raylib [models] example - box collisions");

        // Define the camera to look into our 3d world
        Camera3D camera = new();
        camera.Position = new Vector3(20.0f, 10.0f, 20.0f);
        camera.Target = new Vector3(5.0f, -2.0f, 5.0f);
        camera.Up = new Vector3(0.0f, 1.0f, 0.0f);
        camera.FovY = 50.0f;
        camera.Projection = CameraProjection.Perspective;


        SetTargetFPS(240);   // Set our game to run at 120 frames-per-second

        Vector3 curr_position_big = new Vector3(0, -7, 0);
        Vector3 curr_position_big2 = new Vector3(3, 3, 3);
        Vector3 curr_position = new Vector3(-10, 0, -10);
        Vector3 curr_position2 = new Vector3(5, 0, -5);
        Vector3 velocity_00 = new Vector3(0, 0, 0);
        Vector3 velocity_0 = new Vector3(0, -1f, 0);
        Vector3 velocity_1 = new Vector3(-1, 0, 0);
        Vector3 acceleration_0 = new Vector3(0, 0, 0);
        Vector3 curr_offset1 = new Vector3(0, 0, 0);
        Vector3 side_lengths = new Vector3(0.7f, 0.7f, 0.7f);
        Vector3 side_lengths_big = new Vector3(200f, 1f, 200f);
        Quaternion rotated = new Quaternion(.462f, .191f, .462f, .733f);
        Quaternion ninety = Quaternion.CreateFromAxisAngle(new Vector3(0, 0, 1), float.Pi / 2);
        Quaternion not_rotated = new Quaternion(0, 0, 0, 1);

        BroadcastNode stepper = new BroadcastNode();
        BroadcastNode stepper1 = new BroadcastNode();


        var shapeBig = new OrientedBoundingBox(curr_offset1, side_lengths_big, not_rotated);
        var bodyBig = new Box(shapeBig, velocity_00, curr_position_big, 1000000);
        bodyBig.IsDebugDrawn = true;
        bodyBig.CollideAll = true;
        stepper1.AddNode(bodyBig);

        //var shape = new OrientedBoundingBox(curr_offset1, side_lengths, ninety);
        //var body = new Box(shape, velocity_0, curr_position_big2, 1);
        //body.IsDebugDrawn = true;
        //stepper.AddNode(body);

        //Vector3 curr_position_copy = new Vector3(0, 0, 0);
        //for (int i = 0; i < 100; ++i)
        //{
        //    curr_position_copy.X = curr_position.X + (i / 10) % 10;
        //    curr_position_copy.Z = curr_position.Z + i % 10;
        //    var shape = new OrientedBoundingBox(curr_offset1, side_lengths, rotated);
        //    var body = new Box(shape, velocity_00, curr_position_copy, 1);
        //    body.IsGravitational = true;
        //    body.IsDebugDrawn = true;
        //    stepper.AddNode(body);
        //}

        //Vector3 curr_position_copy = new Vector3(-5, 0, -5);
        //for (int i = 0; i < 200; ++i)
        //{
        //    curr_position_copy.X = curr_position.X + (i / 10) % 10;
        //    curr_position_copy.Z = curr_position.Z + i % 10;
        //    curr_position_copy.Y = curr_position.Y + 2 * (i / 100);
        //    var shape = new BoundingSphere(curr_offset1, .5f);
        //    var body = new Sphere(shape, velocity_0, curr_position_copy, 1);
        //    //body.IsGravitational = true;
        //    body.IsDebugDrawn = true;
        //    stepper.AddNode(body);
        //}

        //var shapeBig = new BoundingSphere(curr_offset1, 5);
        //var bodyBig = new Sphere(shapeBig, velocity_00, curr_position_big, 1000);
        //bodyBig.IsDebugDrawn = true;
        //stepper.AddNode(bodyBig);

        // Main game loop
        Random rand = new Random();
        Collider.Instance.BCDType = 1;
        Stopwatch stopwatch = new Stopwatch();
        stopwatch.Start();

        int clearCounter = 0; 
        while (!WindowShouldClose())
        {
            // Draw
            //----------------------------------------------------------------------------------
            BeginDrawing();
            ClearBackground(Color.RayWhite);

            BeginMode3D(camera);
            if (clearCounter > 500)
            {
                stepper.Clear();
                clearCounter = 0;
            }

            if (stopwatch.ElapsedMilliseconds > 100)
            {
                var shape = new OrientedBoundingBox(curr_offset1, side_lengths, rotated);
                shape.MeshColor = Color.Magenta;
                var body = new Box(shape, velocity_00, new Vector3(rand.Next(11), 0, rand.Next(11)), 1);
                body.IsGravitational = true;
                //body.IsDebugDrawn = true;
                stepper.AddNode(body);
                clearCounter++;
                stopwatch.Restart();
            }

            if (IsKeyReleased(KeyboardKey.O))
            {
                Collider.Instance.BCDType = 1;
            }

            if (IsKeyReleased(KeyboardKey.I))
            {
                Collider.Instance.BCDType = 0;
            }
            if (IsKeyReleased(KeyboardKey.S))
            {
                Collider.Instance.BCDType = 2;
            }
            if (IsKeyReleased(KeyboardKey.W))
            {
                Collider.Instance.BCDType = 3;
            }


            //if (stopwatch.ElapsedMilliseconds > 10000)
            //{
            //    clearCounter++;
            //    if (clearCounter % 3 == 0)
            //    {
            //        stepper.Clear();
            //    }
            //    for (int i = 0; i < 100; ++i)
            //    {
            //        curr_position_copy.X = curr_position.X + (i / 10) % 10;
            //        curr_position_copy.Z = curr_position.Z + i % 10;
            //        var shape = new OrientedBoundingBox(curr_offset1, side_lengths, rotated);
            //        var body = new Box(shape, velocity_00, curr_position_copy, 1);
            //        body.IsGravitational = true;
            //        body.IsDebugDrawn = true;
            //        stepper.AddNode(body);
            //    }
            //    stopwatch.Restart();
            //}

            Collider.Instance.Step();

            stepper.Step();
            stepper.Draw();
            stepper1.Step();
            stepper1.Draw();
            //DrawGrid(10, 1.0f);

            EndMode3D();


            DrawFPS(10, 10);
            //DrawText("Position: " + nodes[0].Position.ToString(), 220, 40, 20, Color.Black);
            DrawText("Type = " + Collider.Instance.BCDType, 220, 40, 20, Color.Black);

            EndDrawing();
            //----------------------------------------------------------------------------------
        }

        // De-Initialization
        //--------------------------------------------------------------------------------------
        CloseWindow();
        //--------------------------------------------------------------------------------------

        return 0;
    }
}