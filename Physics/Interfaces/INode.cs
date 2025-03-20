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
        public bool IsDebugDrawn { get; set; }
        public void Step();
        public void OnStep(Object? _, EventArgs __);
        public void Draw();
        public void OnDraw(Object? _, EventArgs __);
    }
}
