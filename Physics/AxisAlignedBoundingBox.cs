﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Raylib_cs;
using System.Text;
using System.Threading.Tasks;
using Physics550Engine_Raylib.Physics.Interfaces;

namespace Physics550Engine_Raylib.Physics
{
    public class AxisAlignedBoundingBox : IBoundingShape
    {
        public bool IsColliding { get; set; }
        public Vector3 Offset { get; set; }
        public Vector3 SideLengths { get; set; }
        public Vector3 Position { get; set; }
        public Quaternion Orientation { get; set; }

        public AxisAlignedBoundingBox(Vector3 offset, Vector3 side_lengths)
        {
            Offset = offset;
            SideLengths = side_lengths;
            Orientation = Quaternion.Identity; //no orientation, this should always be true
        }

        public void Draw(bool _isDebugDrawn)
        {
            if (_isDebugDrawn)
            {
                Raylib.DrawCubeWiresV(Position, SideLengths, IsColliding ? Color.Red : Color.Blue);
            }
        }
    }
}
