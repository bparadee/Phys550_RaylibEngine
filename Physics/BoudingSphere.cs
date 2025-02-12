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
    public class BoudingSphere : IBoundingShape
    {
        public bool _is_colliding {  get; set; }
        //need to add callback to update values when parent changes (if parents should be able to change).
        public INode _parent_node { get; set; }
        public Vector3 _offset { get; set; }
        private Vector3 _global_position { get; set; }
        public bool _is_debug_drawn { get ; set; }
        public float _radius { get; set; }

        public BoudingSphere(Vector3 offset, float radius, INode parent_node)
        {
            _offset = offset;
            _radius = radius;
            _is_debug_drawn = false;
            _parent_node = parent_node;
            parent_node.DrawEvent += DebugDraw;
            parent_node.PositionUpdateEvent += UpdateGlobalPosition;

            _global_position = Vector3.Transform(_parent_node.Position, Matrix4x4.CreateTranslation(_offset));
        }

        private void UpdateGlobalPosition(Object? sender, PositionUpdateEventArgs args)
        {
            _global_position = Vector3.Transform(args._new_position, Matrix4x4.CreateTranslation(_offset));
        }

        private void DebugDraw(Object? sender, EventArgs _)
        {
            if (_is_debug_drawn)
            {
                Raylib.DrawSphereWires(_global_position, _radius, 16, 16, Color.Red);
            }
        }
    }
}
