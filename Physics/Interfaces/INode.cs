using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace Physics550Engine_Raylib.Physics.Interfaces
{
    public interface INode
    {
        public bool IsDebugDrawn { get; set; }
        public void Step();
        public void Draw();
        public void Clear();
    }
}
