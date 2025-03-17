using Physics550Engine_Raylib.Physics.Interfaces;
using System;
using System.Collections.Generic;
using System.IO.Pipes;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Physics550Engine_Raylib.Physics
{
    public class Collider
    {
        //only want one collider instance we already only have one PhysicsEgnine, so this might not be necessary
        private static readonly Lazy<Collider> instance = new Lazy<Collider>(() => new Collider());
        public static Collider Instance { get { return instance.Value; } }
        private int UniqueId = 0;
        //map of index to shape, index will be a unique identifier to that shape to be used with the broadphase collision detection.
        public Dictionary<int, IBoundingShape> ShapeMap { get; private set; }
        public Collider()
        {
            ShapeMap = new Dictionary<int, IBoundingShape>();
        }
        public bool AddBoundingShape(IBoundingShape shape)
        {
            if (ShapeMap.ContainsValue(shape))
            {
                return false;
            }
            ShapeMap.Add(++UniqueId, shape);
            return true;
        }

        public void Step()
        {
            HashSet<Tuple<int, int>> Collisions = new HashSet<Tuple<int, int>>();
            //Broad Phase Collision Detection will replace this
            for (int i = 1; i <= ShapeMap.Count; i++)
            {
                ShapeMap[i].IsColliding = false;
                for (int j = 1; j <= ShapeMap.Count; j++)
                {
                    if ( i == j || Collisions.Contains(Tuple.Create(j, i)))
                    {
                        continue;
                    }
                    Collisions.Add(Tuple.Create(i, j));
                }
            }

            foreach (var pair in Collisions)
            {
                if (DetectCollision(pair)) { ResolveCollision(pair); }
            }
        }

        void ResolveCollision(Tuple<int, int> pair)
        {
            //update here to have better collision resolution in the future!
            ShapeMap[pair.Item1].IsColliding = true;
            ShapeMap[pair.Item2].IsColliding = true;
        }

        private bool DetectCollision(Tuple<int, int> collision)
        {
            IBoundingShape shape1 = ShapeMap[collision.Item1];
            IBoundingShape shape2 = ShapeMap[collision.Item2];

            if (shape1.GetType() == typeof(BoundingSphere))
            {
                if (shape2.GetType() == typeof(BoundingSphere))
                {
                    return DetectSphereSphereCollision((BoundingSphere)shape1, (BoundingSphere)shape2);
                }
            }
            return false;
        }

        private bool DetectSphereSphereCollision(BoundingSphere sphere1, BoundingSphere sphere2)
        {
            return (sphere1.GlobalPosition - sphere2.GlobalPosition).Length() < sphere1.Radius + sphere2.Radius;
        }
    }
}
