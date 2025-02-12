using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace Physics550Engine_Raylib.Physics.Interfaces
{
    public class PositionUpdateEventArgs : EventArgs
    {
        public Vector3 _new_position { get; set; }
    }
    public interface INode
    {
        public event EventHandler<PositionUpdateEventArgs>? PositionUpdateEvent;
        public event EventHandler<EventArgs>? DrawEvent;
        public event EventHandler<EventArgs>? StepEvent;

        public Vector3 Position {  get; set; }
        public void Step();
        public void Draw();
    }
}
