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
using System.Numerics;
using static Raylib_cs.Raylib;

namespace Examples.Models;

public class BoxCollisions
{
    public static int Main()
    {
        // Initialization
        //--------------------------------------------------------------------------------------
        const int screenWidth = 800;
        const int screenHeight = 450;

        InitWindow(screenWidth, screenHeight, "raylib [models] example - box collisions");

        // Define the camera to look into our 3d world
        Camera3D camera = new();
        camera.Position = new Vector3(0.0f, 10.0f, 10.0f);
        camera.Target = new Vector3(0.0f, 0.0f, 0.0f);
        camera.Up = new Vector3(0.0f, 1.0f, 0.0f);
        camera.FovY = 45.0f;
        camera.Projection = CameraProjection.Perspective;

        SetTargetFPS(200);   // Set our game to run at 60 frames-per-second
        Vector3 curr_position = new Vector3(-5.0f, 0, 0);
        Vector3 velocity = new Vector3(0.5f, 0, 0);
        Vector3 curr_offset1 = new Vector3(0, 0, 0);
        Vector3 curr_offset2 = new Vector3(0, 0, 0);
        // Main game loop
        while (!WindowShouldClose())
        {
            // Draw
            //----------------------------------------------------------------------------------
            BeginDrawing();
            ClearBackground(Color.RayWhite);

            BeginMode3D(camera);

            INode bnode = new StaticNode(curr_position);
            BoudingSphere sphere = new BoudingSphere(curr_offset1, 2.0f, bnode);
            BoudingSphere sphere2 = new BoudingSphere(curr_offset2, 2.0f, bnode);

            sphere._is_debug_drawn = true;
            sphere2._is_debug_drawn = true;

            bnode.Draw();


            curr_position += velocity * GetFrameTime();
            bnode._position = curr_position;

            curr_offset1.Y = (float) Math.Sin(GetTime());
            sphere._offset = curr_offset1;
            curr_offset2.Z = (float)Math.Sin(GetTime());
            sphere2._offset = curr_offset2;

            DrawGrid(10, 1.0f);

            EndMode3D();

            DrawFPS(10, 10);

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