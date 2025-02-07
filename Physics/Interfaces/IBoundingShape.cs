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
        public bool _is_colliding { get; }
        public INode _parent_node { get; set; }
        public Vector3 _offset { get; set; }
        public bool _is_debug_drawn { get; set; }
    }
}
