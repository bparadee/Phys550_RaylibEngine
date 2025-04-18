using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using Physics550Engine_Raylib.Physics.Interfaces;

namespace Physics550Engine_Raylib.Physics
{
    internal class Sphere : RigidBody
    {
        public Sphere(BoundingSphere boundingSphere, Vector3 initial_velocity, Vector3 initial_position, float mass) : base(boundingSphere, initial_velocity, initial_position, mass, CreateSphereInertiaTensor(mass, boundingSphere.Radius))
        {
        }
        private static Matrix4x4 CreateSphereInertiaTensor(float mass, float radius)
        {
            float inertia = (2.0f / 5.0f) * mass * radius * radius;

            return new Matrix4x4(
                inertia, 0.0f, 0.0f, 0.0f,
                0.0f, inertia, 0.0f, 0.0f,
                0.0f, 0.0f, inertia, 0.0f,
                0.0f,  0.0f,  0.0f,  1.0f
            );
        }
    }
}
