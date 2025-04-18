using Physics550Engine_Raylib.Physics.Interfaces;
using Raylib_cs;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace Physics550Engine_Raylib.Physics
{
    public class OrientedBoundingBox : IBoundingShape
    {
        public bool IsColliding { get; set; }
        public Vector3 Offset { get; set; }
        public Vector3 SideLengths { get; set; }
        public Quaternion Orientation { get; set; }
        public Vector3 Position { get; set; }
        public Vector3[] Axes { get; set; }
        public Color MeshColor { get; set; }
        public OrientedBoundingBox(Vector3 offset, Vector3 side_lengths, Quaternion orientation)
        {
            Offset = offset;
            SideLengths = side_lengths;
            Orientation = orientation;
            Axes = GetAxes();
        }

        public Vector3[] GetAxes()
        {
            Matrix4x4 rotMatrix = Matrix4x4.CreateFromQuaternion(Orientation);
            return
            [
                new Vector3(rotMatrix.M11, rotMatrix.M12, rotMatrix.M13),
                new Vector3(rotMatrix.M21, rotMatrix.M22, rotMatrix.M23),
                new Vector3(rotMatrix.M31, rotMatrix.M32, rotMatrix.M33)
            ];
            
        }
        public Vector3[] GetVertices()
        {
            Vector3[] corners = new Vector3[8];

            for (int i = 0; i < 8; i++)
            {
                float signX = ((i & 1) == 0) ? -1.0f : 1.0f;
                float signY = ((i & 2) == 0) ? -1.0f : 1.0f;
                float signZ = ((i & 4) == 0) ? -1.0f : 1.0f;

                Vector3 corner = new Vector3(
                    signX * SideLengths.X / 2,
                    signY * SideLengths.Y / 2,
                    signZ * SideLengths.Z / 2
                );

                corners[i] = Vector3.Transform(corner, Orientation) + Position;
            }

            return corners;
        }
        public void Draw(bool isDebugDrawn)
        {
            //sucks but apparently raylib doesn't draw oriented boxes, 
            //so we gotta do it manually here for OBBs

            Color color = Color.Blue;
            if (IsColliding)
            {
                color = Color.Red;
            }

            Vector3[] verticesLines = GetVertices();

            Raylib.DrawLine3D(verticesLines[0], verticesLines[1], color);
            Raylib.DrawLine3D(verticesLines[1], verticesLines[3], color);
            Raylib.DrawLine3D(verticesLines[3], verticesLines[2], color);
            Raylib.DrawLine3D(verticesLines[2], verticesLines[0], color);

            Raylib.DrawLine3D(verticesLines[4], verticesLines[5], color);
            Raylib.DrawLine3D(verticesLines[5], verticesLines[7], color);
            Raylib.DrawLine3D(verticesLines[7], verticesLines[6], color);
            Raylib.DrawLine3D(verticesLines[6], verticesLines[4], color);

            Raylib.DrawLine3D(verticesLines[0], verticesLines[4], color);
            Raylib.DrawLine3D(verticesLines[1], verticesLines[5], color);
            Raylib.DrawLine3D(verticesLines[2], verticesLines[6], color);
            Raylib.DrawLine3D(verticesLines[3], verticesLines[7], color);
        

            //Vector3[] vertices = GetVertices();
            ////2 triangles per side, so 12 total calls
            //Raylib.DrawTriangle3D(vertices[0], vertices[1], vertices[2], MeshColor);// front
            //Raylib.DrawTriangle3D(vertices[2], vertices[3], vertices[0], MeshColor);

            //Raylib.DrawTriangle3D(vertices[4], vertices[5], vertices[1], MeshColor);// right
            //Raylib.DrawTriangle3D(vertices[1], vertices[0], vertices[4], MeshColor);

            //Raylib.DrawTriangle3D(vertices[7], vertices[6], vertices[5], MeshColor);// back
            //Raylib.DrawTriangle3D(vertices[5], vertices[4], vertices[7], MeshColor);    

            //Raylib.DrawTriangle3D(vertices[3], vertices[2], vertices[6], MeshColor);// left
            //Raylib.DrawTriangle3D(vertices[6], vertices[7], vertices[3], MeshColor);

            //Raylib.DrawTriangle3D(vertices[0], vertices[4], vertices[7], MeshColor);// bottom
            //Raylib.DrawTriangle3D(vertices[7], vertices[3], vertices[0], MeshColor);

            //Raylib.DrawTriangle3D(vertices[1], vertices[5], vertices[6], MeshColor);// top
            //Raylib.DrawTriangle3D(vertices[6], vertices[2], vertices[1], MeshColor);
        }
    }
}
