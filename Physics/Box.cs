using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using Physics550Engine_Raylib.Physics.Interfaces;
using Raylib_cs;

namespace Physics550Engine_Raylib.Physics
{
    public class Box : RigidBody
    {
        public Box(OrientedBoundingBox boundingBox, Vector3 initial_velocity, Vector3 initial_position, float mass) : base(boundingBox, initial_velocity, initial_position, mass, CreateBoxInertiaTensor(mass, boundingBox.SideLengths))
        {
        }
        private static Matrix4x4 CreateBoxInertiaTensor(float mass, Vector3 dimensions)
        {
            float width = dimensions.X;
            float height = dimensions.Y;
            float depth = dimensions.Z;

            float x2 = width * width;
            float y2 = height * height;
            float z2 = depth * depth;

            float Ixx = (1.0f / 12.0f) * mass * (y2 + z2);
            float Iyy = (1.0f / 12.0f) * mass * (x2 + z2);
            float Izz = (1.0f / 12.0f) * mass * (x2 + y2);

            return new Matrix4x4(
                Ixx, 0, 0, 0,
                0, Iyy, 0, 0,
                0, 0, Izz, 0,
                0, 0, 0, 1
            );
        }
    }
}
