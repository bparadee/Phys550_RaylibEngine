using Physics550Engine_Raylib.Physics.Interfaces;
using Raylib_cs;
using System.Numerics;

//this file is getting removed

namespace Physics550Engine_Raylib.Physics
{
    public class VelocityUpdateEventArgs : EventArgs
    {
        public Vector3 _new_velocity { get; set; }
    }

    public class AccerlationUpdateEventArgs : EventArgs
    {
        public Vector3 _new_acceleration { get; set; }
    }

    public class TranslationalPhysicsComponent
    {

        private INode _parent_node {  get; set; }
        private Vector3 _velocity { get; set; }
        private Vector3 _acceleration { get; set; }

        public Vector3 Velocity
        {
            get { return _velocity; }
            set
            {
                _velocity = value;
            }

        }
        public Vector3 Acceleration
        {
            get { return _acceleration; }
            set
            {
                _acceleration = value;
            }
        }

        public TranslationalPhysicsComponent(Vector3 initial_velocity, Vector3 initial_acceleration, INode parent_node)
        {
            _velocity = initial_velocity;
            _acceleration = initial_acceleration;
            _parent_node = parent_node;
            parent_node.StepEvent += OnStep;
            parent_node.DrawEvent += DebugDraw;
        }

        public void OnStep(object? __sender, EventArgs __args)
        {
            var timechange = Raylib.GetFrameTime();
            _velocity = NewtonsMethod(_velocity, _acceleration, timechange);
            Vector3 current_position = _parent_node.Position;
            _parent_node.Position = NewtonsMethod(current_position, _velocity, timechange);
        }

        private static Vector3 NewtonsMethod(Vector3 x_0, Vector3 x_delta, float timechange)
        {
            return x_0 + x_delta * timechange;
        }

        private void DebugDraw(Object? sender, EventArgs _)
        {
            if (_parent_node.IsDebugDrawn)
            {
                Raylib.DrawCylinderEx(_parent_node.Position, _parent_node.Position + Velocity, .1f, .1f, 16, Color.Green);
                Raylib.DrawCylinderEx(_parent_node.Position, _parent_node.Position + Acceleration, .1f, .1f, 16, Color.Blue);
            }
        }


    }
}
