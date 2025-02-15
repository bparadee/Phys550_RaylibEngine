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
    public class AxisAlignedBoundingBox : IBoundingShape
    {
        public bool IsColliding { get; set; }
        //need to add callback to update values when parent changes (if parents should be able to change).
        private INode _parent_node { get; set; }
        public INode ParentNode
        {
            get { return _parent_node; }
        }
        public Vector3 Offset { get; set; }
        public Vector3 SideLengths { get; set; }
        private Vector3 _global_position { get; set; }

        public Vector3 GlobalPostion
        {
            get { return _global_position; }
        }

        public AxisAlignedBoundingBox(Vector3 offset, Vector3 side_lengths, INode parent_node)
        {
            Offset = offset;
            _parent_node = parent_node;
            SideLengths = side_lengths;
            parent_node.DrawEvent += DebugDraw;
            parent_node.PositionUpdateEvent += UpdateGlobalPosition;

            _global_position = Vector3.Transform(_parent_node.Position, Matrix4x4.CreateTranslation(Offset));
        }

        private void UpdateGlobalPosition(Object? sender, PositionUpdateEventArgs args)
        {
            _global_position = Vector3.Transform(args._new_position, Matrix4x4.CreateTranslation(Offset));
        }

        private void DebugDraw(Object? sender, EventArgs _)
        {
            if (_parent_node.IsDebugDrawn)
            {
                Raylib.DrawCubeWiresV(_parent_node.Position, SideLengths, Color.Red);
            }
        }
    }
}
