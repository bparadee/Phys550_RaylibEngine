using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace Physics550Engine_Raylib.Physics.Interfaces
{
    public interface IBoundingShape
    {
        public bool IsColliding { get; set; }
        public Vector3 Offset { get; set; }
        public Vector3 Position { get; set; }
        public void Draw();
    }
}
