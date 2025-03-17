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
        camera.FovY = 50.0f;
        camera.Projection = CameraProjection.Perspective;

        SetTargetFPS(120);   // Set our game to run at 30 frames-per-second

        Vector3 curr_position = new Vector3(-5, 0, -5);
        Vector3 curr_position2 = new Vector3(5, 0, -5);
        Vector3 velocity_0 = new Vector3(0, 0, 1);
        Vector3 velocity_1 = new Vector3(-1, 0, 0);
        Vector3 acceleration_0 = new Vector3(0, 0, 0);
        Vector3 curr_offset1 = new Vector3(0, 0, 0);
        Vector3 side_lengths = new Vector3(4, 4, 4);

        List<INode> nodes = new List<INode>();

        for (int i = 0 ; i < 10; ++i)
        {
            nodes.Add(new StaticNode(curr_position));
            _ = new TranslationalPhysicsComponent(velocity_0, acceleration_0, nodes[i]);
            _ = new BoundingSphere(curr_offset1, 0.4f, nodes[i]);
            nodes[i].IsDebugDrawn = true;
            curr_position.X += 1.0f;
        }

        for (int i = 10; i < 20; ++i)
        {
            nodes.Add(new StaticNode(curr_position2));
            _ = new TranslationalPhysicsComponent(velocity_1, acceleration_0, nodes[i]);
            _ = new BoundingSphere(curr_offset1, 0.4f, nodes[i]);
            nodes[i].IsDebugDrawn = true;
            curr_position2.Z += 1.0f;
        }

        // Main game loop
        while (!WindowShouldClose())
        {
            // Draw
            //----------------------------------------------------------------------------------
            BeginDrawing();
            ClearBackground(Color.RayWhite);

            BeginMode3D(camera);

            Collider.Instance.Step();
            foreach (INode node in nodes)
            {
                node.Draw();
            }

            DrawGrid(10, 1.0f);

            EndMode3D();

            foreach (INode node in nodes)
            {
                node.Step();
            }


            DrawFPS(10, 10);
            DrawText("Position: " + nodes[0].Position.ToString(), 220, 40, 20, Color.Black);

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