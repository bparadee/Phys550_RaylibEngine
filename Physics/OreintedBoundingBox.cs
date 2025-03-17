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
        //need to add callback to update values when parent changes (if parents should be able to change).
        public INode ParentNode { get; private set; }
        public Vector3 Offset { get; set; }
        public Vector3 SideLengths { get; set; }
        public Vector3 GlobalPostion { get; private set; }
        public Quaternion Orientation { get; private set; }
        public OrientedBoundingBox(Vector3 offset, Vector3 side_lengths, INode parent_node, Quaternion orientation)
        {
            Offset = offset;
            ParentNode = parent_node;
            SideLengths = side_lengths;
            parent_node.DrawEvent += DebugDraw;
            parent_node.PositionUpdateEvent += UpdateGlobalPosition;
            Orientation = orientation;

            GlobalPostion = Vector3.Transform(ParentNode.Position, Matrix4x4.CreateTranslation(Offset));
        }

        private void UpdateGlobalPosition(Object? sender, PositionUpdateEventArgs args)
        {
            GlobalPostion = Vector3.Transform(args._new_position, Matrix4x4.CreateTranslation(Offset));
        }

        private void DebugDraw(Object? sender, EventArgs _)
        {
            if (ParentNode.IsDebugDrawn)
            {
                Raylib.DrawCubeWiresV(ParentNode.Position, Vector3.Transform(SideLengths, Orientation) , IsColliding ? Color.Red : Color.Blue);
            }
        }
    }
}
