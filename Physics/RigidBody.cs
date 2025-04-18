using Physics550Engine_Raylib.Physics.Interfaces;
using Raylib_cs;
using System.Numerics;

namespace Physics550Engine_Raylib.Physics
{
    public class RigidBody : INode
    {
        float GRAV_CONST = -9.8f;
        public bool IsDebugDrawn { get; set; }
        public Vector3 Position { get; private set; }
        public Vector3 Velocity { get; set; }
        public bool IsGravitational { get ; set; }
        public bool CollideAll { get; set; }
        public Vector3 RotationalVelocity { get; set; }
        public IBoundingShape BoundingShape { get; set; }
        public float Mass { get; set; }
        public Matrix4x4 InertiaTensor { get; set; }
        public float Restitution = 1.0f;
        private int ColliderId;


        public RigidBody(IBoundingShape boudingShape, Vector3 initial_velocity, Vector3 initial_position, float mass, Matrix4x4 inertiaTensor)
        {
            BoundingShape = boudingShape;
            Velocity = initial_velocity;
            Mass = mass;
            Position = initial_position;
            BoundingShape.Position = initial_position;
            ColliderId = Collider.Instance.AddRigidBody(this);
            InertiaTensor = inertiaTensor;
        }

        public Matrix4x4 GetWorldInverseInertiaTensor()
        {
            Matrix4x4 rotMatrix = Matrix4x4.CreateFromQuaternion(BoundingShape.Orientation);
            if ( Matrix4x4.Invert(InertiaTensor, out Matrix4x4 InverseInertiaTensor))
                    return Matrix4x4.Transpose(rotMatrix) * InverseInertiaTensor * rotMatrix;
            throw new Exception();
        }
        // Get the velocity at a specific point on the rigid body
        public Vector3 GetVelocityAtPoint(Vector3 point)
        {
            Vector3 relativePoint = point - Position;
            return Velocity + Vector3.Cross(RotationalVelocity, relativePoint);
        }

        private static Vector3 NewtonsMethod(Vector3 x_0, Vector3 x_delta, float timechange)
        {
            return x_0 + x_delta * timechange;
        }

        public void Clear()
        {
            Collider.Instance.RemoveRigidBody(ColliderId);
        }

        public void Draw()
        {
            if (IsDebugDrawn)
            {
                Raylib.DrawCylinderEx(Position, Position + Velocity, .05f, .03f, 16, Color.Green);
            }
                BoundingShape.Draw(IsDebugDrawn);
        }

        public void Step()
        {
            var timechange = Raylib.GetFrameTime();
            if ( IsGravitational )
                    Velocity = NewtonsMethod(Velocity, new Vector3(0, GRAV_CONST, 0), timechange);
            Vector3 new_position = NewtonsMethod(Position, Velocity, timechange);

            BoundingShape.Position = new_position;
            Position = new_position;

            float angSpeed = RotationalVelocity.Length();

            if (angSpeed < 0.00001f) return;

            Vector3 axis = Vector3.Normalize(RotationalVelocity);
            float angle = angSpeed * timechange;

            Quaternion rotationThisStep = Quaternion.CreateFromAxisAngle(axis, angle);
            BoundingShape.Orientation = Quaternion.Normalize(rotationThisStep * BoundingShape.Orientation);
        }
    }
}
