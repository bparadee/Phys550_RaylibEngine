using Raylib_cs;
using Physics550Engine_Raylib.Physics.Interfaces;
using System.Numerics;

namespace Physics550Engine_Raylib.Physics
{

    public class StaticNode : INode
    {
        public event EventHandler<PositionUpdateEventArgs>? PositionUpdateEvent;
        public event EventHandler<EventArgs>? DrawEvent;
        public event EventHandler<EventArgs>? StepEvent;

        public bool IsDebugDrawn { get; set; }

        private Vector3 _position;
        public Vector3 Position {
            get { return _position; }
            set
            {
                PositionUpdateEventArgs args = new()
                {
                    _new_position = value,
                };
                OnPositionUpdate(args);
                _position = value;
            }
        }
        public StaticNode(Vector3 position)
        {
            _position = position;
            IsDebugDrawn = false;
        }

        public void OnPositionUpdate(PositionUpdateEventArgs args)
        {
            PositionUpdateEvent?.Invoke(this, args);
        }

        public void Step()
        { 
            EventArgs args = new();
            StepEvent?.Invoke(this, args);
        }
        public void Draw()
        {
            Raylib.DrawSphere(_position, .1f, Color.Black);
            EventArgs args = new();
            DrawEvent?.Invoke(this, args);
        }
    }
}
