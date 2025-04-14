using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace Physics550Engine_Raylib.Physics
{
    internal class CubeBoundingBox : OrientedBoundingBox
    {
        public CubeBoundingBox(Vector3 offset, float side_length, Quaternion orientation) : base(offset, new Vector3(side_length), orientation)
        {
        }
    }
}
