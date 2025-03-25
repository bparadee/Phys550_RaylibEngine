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
    internal class OrientedBoundingBox : IBoundingShape
    {
        public bool IsColliding { get; set; }
        public Vector3 Offset { get; set; }
        public Vector3 SideLengths { get; set; }
        public Quaternion Orientation { get; private set; }
        public Vector3 Position { get; set; }
        public Vector3[] Axes { get; set; }
        private bool axesStale { get; set; }


        public OrientedBoundingBox(Vector3 offset, Vector3 side_lengths, Quaternion orientation)
        {
            Offset = offset;
            SideLengths = side_lengths;
            Orientation = orientation;
            axesStale = true;
            Axes = GetAxes();
        }

        public Vector3[] GetAxes()
        {
            if (!axesStale) { 
                return Axes; 
            }

            axesStale = false;
            Matrix4x4 rotMatrix = Matrix4x4.CreateFromQuaternion(Orientation);
            return
            [
                Vector3.Normalize(new Vector3(rotMatrix.M11, rotMatrix.M12, rotMatrix.M13)),
                Vector3.Normalize(new Vector3(rotMatrix.M21, rotMatrix.M22, rotMatrix.M23)),
                Vector3.Normalize(new Vector3(rotMatrix.M31, rotMatrix.M32, rotMatrix.M33))
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

        public void Draw()
        {

            //sucks but apparently raylib doesn't draw oriented boxes, 
            //so we gotta do it manually here for OBBs
            Color color = Color.Blue;
            if (IsColliding)
            { 
                color = Color.Red; 
            }

            Vector3[] vertices = GetVertices();

            Raylib.DrawLine3D(vertices[0], vertices[1], color);
            Raylib.DrawLine3D(vertices[1], vertices[3], color);
            Raylib.DrawLine3D(vertices[3], vertices[2], color);
            Raylib.DrawLine3D(vertices[2], vertices[0], color);

            Raylib.DrawLine3D(vertices[4], vertices[5], color);
            Raylib.DrawLine3D(vertices[5], vertices[7], color);
            Raylib.DrawLine3D(vertices[7], vertices[6], color);
            Raylib.DrawLine3D(vertices[6], vertices[4], color);

            Raylib.DrawLine3D(vertices[0], vertices[4], color);
            Raylib.DrawLine3D(vertices[1], vertices[5], color);
            Raylib.DrawLine3D(vertices[2], vertices[6], color);
            Raylib.DrawLine3D(vertices[3], vertices[7], color);
        }
    }
}
