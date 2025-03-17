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
        public INode ParentNode { get; private set; }
        public Vector3 Offset { get; set; }
        public Vector3 GlobalPosition { get; private set; }
        public float Radius { get; set; }

        public BoundingSphere(Vector3 offset, float radius, INode parent_node)
        {
            Offset = offset;
            Radius = radius;
            ParentNode = parent_node;

            // These calls are adding delegates that will run upon the given event invocation.
            parent_node.DrawEvent += DebugDraw;
            parent_node.PositionUpdateEvent += UpdateGlobalPosition;

            GlobalPosition = Vector3.Transform(ParentNode.Position, Matrix4x4.CreateTranslation(Offset));

            //add this shape to our collider
            Collider.Instance.AddBoundingShape(this);
        }

        private void UpdateGlobalPosition(Object? sender, PositionUpdateEventArgs args)
        {
            GlobalPosition = Vector3.Transform(args._new_position, Matrix4x4.CreateTranslation(Offset));
        }

        private void DebugDraw(Object? sender, EventArgs _)
        {
            if (ParentNode.IsDebugDrawn)
            {
                Raylib.DrawSphereWires(GlobalPosition, Radius, 16, 16, IsColliding ? Color.Red : Color.Blue);
            }
        }
    }
}
