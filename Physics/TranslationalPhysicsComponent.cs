using Physics550Engine_Raylib.Physics.Interfaces;
using Raylib_cs;
using System.Numerics;

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
            //set
            //{
            //    _acceleration = value;
            //}
        }

        public TranslationalPhysicsComponent(Vector3 initial_velocity, Vector3 initial_acceleration, INode parent_node)
        {
            _velocity = initial_velocity;
            _acceleration = initial_acceleration;
            _parent_node = parent_node;
            parent_node.StepEvent += OnStep;
        }

        public void OnStep(object? __sender, EventArgs __args)
        { 
            _velocity = NewtonsMethod(_velocity, _acceleration);
            Vector3 current_position = _parent_node.Position;
            _parent_node.Position = NewtonsMethod(current_position, _velocity);
        }

        private static Vector3 NewtonsMethod(Vector3 x_0, Vector3 x_delta)
        {
            return x_0 + x_delta * Raylib.GetFrameTime();
        }



    }
}
