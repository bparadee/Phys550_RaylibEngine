using Physics550Engine_Raylib.Physics.Interfaces;
using Raylib_cs;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace Physics550Engine_Raylib.Physics
{
    internal class OrientedBoundingBox : IBoundingShape
    {
        public bool IsColliding { get; set; }
        public Vector3 Offset { get; set; }
        public Vector3 SideLengths { get; set; }
        public Quaternion Orientation { get; private set; }
        public Vector3 Position { get => throw new NotImplementedException(); set => throw new NotImplementedException(); }

        public OrientedBoundingBox(Vector3 offset, Vector3 side_lengths, INode parent_node, Quaternion orientation)
        {
            Offset = offset;
            SideLengths = side_lengths;
            Orientation = orientation;
        }

        public void Draw()
        {
            Raylib.DrawCubeWiresV(Position, Vector3.Transform(SideLengths, Orientation), IsColliding ? Color.Red : Color.Blue);
        }
    }
}
