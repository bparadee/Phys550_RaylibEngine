using Raylib_cs;
using Physics550Engine_Raylib.Physics.Interfaces;
using System.Numerics;

namespace Physics550Engine_Raylib.Physics
{

    public class StaticNode : INode
    {
        public event EventHandler<PositionUpdateEventArgs>? PositionUpdateEvent;
        public event EventHandler<EventArgs>? DrawEvent;

        private Vector3 _real_position;
        public Vector3 _position {
            get { return _real_position; }
            set
            {
                PositionUpdateEventArgs args = new()
                {
                    _new_position = value,
                };
                OnPositionUpdate(args);
                _real_position = value;
            }
        }
        public StaticNode(Vector3 position)
        {
            _position = position;
        }

        public void OnPositionUpdate(PositionUpdateEventArgs args)
        {
            PositionUpdateEvent?.Invoke(this, args);
        }
        public void Draw()
        {
            Raylib.DrawSphere(_position, .1f, Color.Black);
            EventArgs args = new();
            DrawEvent?.Invoke(this, args);
        }
    }
}
