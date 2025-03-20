using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Raylib_cs;
using System.Text;
using System.Threading.Tasks;
using Physics550Engine_Raylib.Physics.Interfaces;

namespace Physics550Engine_Raylib.Physics
{
    public class BoundingSphere : IBoundingShape
    {
        public bool IsColliding {  get; set; }
        //need to add callback to update values when parent changes (if parents should be able to change).
        public Vector3 Offset { get; set; }
        public Vector3 Position { get; set; }
        public float Radius { get; set; }
        public BoundingSphere(Vector3 offset, float radius)
        {
            Offset = offset;
            Radius = radius;
        }

        public void Draw()
        {
            Raylib.DrawSphereWires(Position, Radius, 16, 16, IsColliding ? Color.Red : Color.Blue);
        }
    }
}
