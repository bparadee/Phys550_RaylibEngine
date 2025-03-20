using Physics550Engine_Raylib.Physics.Interfaces;
using Raylib_cs;
using System.Numerics;

namespace Physics550Engine_Raylib.Physics
{
    public class RigidBody : INode
    {
        public bool IsDebugDrawn { get; set; }
        public Vector3 Position { get; private set; }
        public event EventHandler<PositionUpdateEventArgs>? PositionUpdateEvent;
        public Vector3 Velocity { get; private set; }
        public Vector3 Acceleration { get; private set; }
        public IBoundingShape BoundingShape { get; set; }

        public RigidBody(IBoundingShape boudingShape, Vector3 initial_velocity, Vector3 initial_acceleration, Vector3 initial_position)
        {
            BoundingShape = boudingShape;
            Velocity = initial_velocity;
            Acceleration = initial_acceleration;
            SetPosition(initial_position);
            Collider.Instance.AddRigidBody(this);
        }
        public void SetPosition(Vector3 new_position)
        {
            PositionUpdateEventArgs args = new()
            {
                _new_position = new_position,
            };
            BoundingShape.Position = new_position;
            this.Position = new_position;
        }

        public void OnStep(object? _, EventArgs __)
        {
            Step();
        }

        private static Vector3 NewtonsMethod(Vector3 x_0, Vector3 x_delta, float timechange)
        {
            return x_0 + x_delta * timechange;
        }
        void INode.OnDraw(object? _, EventArgs __)
        {
            Draw();
        }

        public void Draw()
        {
            if (IsDebugDrawn)
            {
                Raylib.DrawCylinderEx(Position, Position + Velocity, .1f, .1f, 16, Color.Green);
                Raylib.DrawCylinderEx(Position, Position + Acceleration, .1f, .1f, 16, Color.Blue);
                BoundingShape.Draw();
            }
            EventArgs args = new();
        }

        public void Step()
        {
            var timechange = Raylib.GetFrameTime();
            Velocity = NewtonsMethod(Velocity, Acceleration, timechange);
            SetPosition(NewtonsMethod(Position, Velocity, timechange));
        }

    }
}
