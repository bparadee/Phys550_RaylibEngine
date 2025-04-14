using Physics550Engine_Raylib.Physics.Interfaces;
using Raylib_cs;
using System;
using System.Collections.Generic;
using System.IO.Pipes;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace Physics550Engine_Raylib.Physics
{
    public class CollisionResult
    {
        public bool HasCollision { get; set; }
        public Vector3 ContactPoint { get; set; }
        public float PenetrationDepth { get; set; }
        public Vector3 CollisionNormal { get; set; }
    }
    public class Collider
    {

        //only want one collider instance we already only have one PhysicsEgnine, so this might not be necessary
        private static readonly Lazy<Collider> instance = new Lazy<Collider>(() => new Collider());
        public static Collider Instance { get { return instance.Value; } }
        private int UniqueId = 0;
        //map of index to shape, index will be a unique identifier to that shape to be used with the broadphase collision detection.
        public Dictionary<int, RigidBody> ShapeMap { get; private set; }
        private const float EPSILON = 1e-6f;
        public Collider()
        {
            ShapeMap = new Dictionary<int, RigidBody>();
        }
        public bool AddRigidBody(RigidBody rigidBody)
        {
            if (ShapeMap.ContainsValue(rigidBody))
            {
                return false;
            }
            ShapeMap.Add(++UniqueId, rigidBody);
            return true;
        }

        public void Step()
        {
            HashSet<Tuple<int, int>> Collisions = new HashSet<Tuple<int, int>>();
            //Broad Phase Collision Detection will replace this
            for (int i = 1; i <= ShapeMap.Count; i++)
            {
                ShapeMap[i].BoundingShape.IsColliding = false;
                for (int j = 1; j <= ShapeMap.Count; j++)
                {
                    if ( i == j || Collisions.Contains(Tuple.Create(j, i)))
                    {
                        continue;
                    }
                    Collisions.Add(Tuple.Create(i, j));
                }
            }

            //BroadPhaseCollisionDetection(Collisions);
            foreach (var pair in Collisions)
            {
                CollisionResult collisionResult;
                if (DetectCollision(pair, out collisionResult)) {
                    //Raylib.DrawSphere(collisionResult.ContactPoint, 0.5f, Color.Black);
                    ResolveCollision(pair, collisionResult); 
                }

            }
        }

        void ResolveCollision(Tuple<int, int> pair, CollisionResult collisionResult)
        {
            //update here to have better collision resolution in the future!

            RigidBody shape1 = ShapeMap[pair.Item1];
            RigidBody shape2 = ShapeMap[pair.Item2];

            shape1.BoundingShape.IsColliding = true;
            shape2.BoundingShape.IsColliding = true;

            Vector3 momentum1 = shape1.Mass * shape1.Velocity;
            Vector3 momentum2 = shape2.Mass * shape2.Velocity;



            //calculate translation of shape1:
            //collisionResult.CollisionNormal 


            //calculate translation of shape2:

            //calculates rotation of shape1:

            //calcualte rotation of shape2:
            




        }

        private bool DetectCollision(Tuple<int, int> collision, out CollisionResult collisionResult)
        {
            //TODO fix collision result default here.
            IBoundingShape shape1 = ShapeMap[collision.Item1].BoundingShape;
            IBoundingShape shape2 = ShapeMap[collision.Item2].BoundingShape;

            if (shape1.GetType() == typeof(BoundingSphere))
            {
                if (shape2.GetType() == typeof(BoundingSphere))
                {
                    return DetectSphereSphereCollision((BoundingSphere)shape1, (BoundingSphere)shape2, out collisionResult);
                }
                else if (shape2.GetType() == typeof(AxisAlignedBoundingBox))
                {
                    collisionResult = new CollisionResult();
                    return false;
                }
                else if (shape2.GetType() == typeof(OrientedBoundingBox))
                {
                    collisionResult = new CollisionResult();
                    return false;
                }
            }
            else if (shape1.GetType() == typeof(AxisAlignedBoundingBox))
            {
                if (shape2.GetType() == typeof(BoundingSphere))
                {
                    collisionResult = new CollisionResult();
                    return false;
                }
                else if (shape2.GetType() == typeof(AxisAlignedBoundingBox))
                {
                    collisionResult = new CollisionResult();
                    return false;
                }
                else if (shape2.GetType() == typeof(OrientedBoundingBox))
                {
                    collisionResult = new CollisionResult();
                    return false;
                }
            }
            else if (shape1.GetType() == typeof(OrientedBoundingBox))
            {
                if (shape2.GetType() == typeof(BoundingSphere))
                {
                    collisionResult = new CollisionResult();
                    return false;
                }
                else if (shape2.GetType() == typeof(AxisAlignedBoundingBox))
                {
                    collisionResult = new CollisionResult();
                    return false;
                }
                else if (shape2.GetType() == typeof(OrientedBoundingBox))
                {
                    return DetectOBBOBBCollision((OrientedBoundingBox)shape1, (OrientedBoundingBox)shape2, out collisionResult);
                }
            }
            collisionResult = new CollisionResult();
            return false;
        }

        private bool DetectSphereSphereCollision(BoundingSphere sphere1, BoundingSphere sphere2, out CollisionResult collisionResult)
        {
            bool result = (sphere1.Position - sphere2.Position).Length() < sphere1.Radius + sphere2.Radius;
            collisionResult = new CollisionResult();
            if (!result) { return false; }

            collisionResult = new CollisionResult
            {
                HasCollision = result,
                ContactPoint = sphere1.Position + (sphere1.Position - sphere2.Position) * sphere1.Radius / (sphere2.Radius + sphere1.Radius),
                PenetrationDepth = sphere1.Radius + sphere2.Radius - (sphere1.Position - sphere2.Position).Length(),
                CollisionNormal = Vector3.Normalize(sphere2.Position - sphere1.Position)
            };
            return true;
        }

        private bool DetectOBBOBBCollision(OrientedBoundingBox box1, OrientedBoundingBox box2, out CollisionResult collisionResult)
        {
            collisionResult = new CollisionResult
            {
                HasCollision = false,
                PenetrationDepth = float.MaxValue
            };

            // Get box axes
            Vector3[] box1Axes = box1.GetAxes();
            Vector3[] box2Axes = box2.GetAxes();

            Vector3 t = box2.Position - box1.Position;

            Vector3 tInA = new Vector3(
                Vector3.Dot(t, box1Axes[0]),
                Vector3.Dot(t, box1Axes[1]),
                Vector3.Dot(t, box1Axes[2])
            );

            float[,] r = new float[3, 3];
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    r[i, j] = Vector3.Dot(box1Axes[i], box2Axes[j]);
                }
            }

            float[,] absR = new float[3, 3];
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    absR[i, j] = Math.Abs(r[i, j]) + EPSILON;
                }
            }

            Vector3 bestAxis = Vector3.Zero;
            int bestCase = -1;

            // Test axes of box1
            for (int i = 0; i < 3; i++)
            {
                float ra = box1.SideLengths[i] / 2;
                float rb = box2.SideLengths.X / 2 * absR[i, 0] + box2.SideLengths.Y / 2 * absR[i, 1] + box2.SideLengths.Z / 2 * absR[i, 2];
                float t_i = Math.Abs(tInA[i]);

                if (t_i > ra + rb)
                    return false; // No collision

                float overlap = ra + rb - t_i;
                if (overlap < collisionResult.PenetrationDepth)
                {
                    collisionResult.PenetrationDepth = overlap;
                    bestAxis = box1Axes[i] * Math.Sign(tInA[i]);
                    bestCase = i;
                }
            }

            // Test axes of box2
            for (int i = 0; i < 3; i++)
            {
                float ra = box1.SideLengths.X / 2 * absR[0, i] + box1.SideLengths.Y / 2 * absR[1, i] + box1.SideLengths.Z / 2 * absR[2, i];
                float rb = box2.SideLengths[i] / 2;

                float t_b = Vector3.Dot(t, box2Axes[i]);
                float t_i = Math.Abs(t_b);

                if (t_i > ra + rb)
                    return false; // No collision

                float overlap = ra + rb - t_i;
                if (overlap < collisionResult.PenetrationDepth)
                {
                    collisionResult.PenetrationDepth = overlap;
                    bestAxis = box2Axes[i] * Math.Sign(t_b);
                    bestCase = i + 3;
                }
            }

            // Test 9 cross product axes
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    if (Math.Abs(absR[i, j]) > (1 - EPSILON)) continue;

                    Vector3 axis = Vector3.Cross(box1Axes[i], box2Axes[j]);
                    axis = Vector3.Normalize(axis);
                    // Project both boxes onto the axis
                    float ra = 0;
                    float rb = 0;

                    for (int k = 0; k < 3; k++)
                    {
                        ra += box1.SideLengths[k] / 2 * Math.Abs(Vector3.Dot(box1Axes[k], axis));
                        rb += box2.SideLengths[k] / 2 * Math.Abs(Vector3.Dot(box2Axes[k], axis));
                    }

                    float t_i = Math.Abs(Vector3.Dot(t, axis));

                    if (t_i > ra + rb)
                        return false; // No collision

                    float overlap = ra + rb - t_i;
                    if (overlap < collisionResult.PenetrationDepth)
                    {
                        collisionResult.PenetrationDepth = overlap;
                        bestAxis = axis * Math.Sign(Vector3.Dot(t, axis));
                        bestCase = i + j + 6;
                    }
                }
            }

            // We have a collision, compute contact point
            collisionResult.HasCollision = true;
            collisionResult.CollisionNormal = bestAxis;

            Vector3 pointOnA, pointOnB;

            if (bestCase < 3) 
            {
                pointOnA = box1.Position + bestAxis * -box1.SideLengths[bestCase] / 2;

                pointOnB = box2.Position;
                for (int i = 0; i < 3; i++)
                {
                    pointOnB += box2Axes[i] * box2.SideLengths[i] / 2 * (Vector3.Dot(bestAxis, box2Axes[i]) < 0 ? -1 : 1);
                }
            }
            else if (bestCase < 6) 
            {
                int faceIdx = bestCase - 3;
                pointOnB = box2.Position + bestAxis * box2.SideLengths[faceIdx] / 2;

                pointOnA = box1.Position;
                for (int i = 0; i < 3; i++)
                {
                    pointOnA += box1Axes[i] * box2.SideLengths[i] / 2* (Vector3.Dot(bestAxis, box1Axes[i]) > 0 ? -1 : 1);
                }
            }
            else // Edge-edge collision
            {

                Vector3[] verticesA = box1.GetVertices();
                Vector3[] verticesB = box2.GetVertices();

                float minDistance = float.MaxValue;
                pointOnA = Vector3.Zero;
                pointOnB = Vector3.Zero;

                for (int i = 0; i < 8; i++)
                {
                    for (int j = 0; j < 8; j++)
                    {
                        float distance = Vector3.Distance(verticesA[i], verticesB[j]);
                        if (distance < minDistance)
                        {
                            minDistance = distance;
                            pointOnA = verticesA[i];
                            pointOnB = verticesB[j];
                        }
                    }
                }
            }

            collisionResult.ContactPoint = (pointOnA + pointOnB) * 0.5f;
            return true;
        }
    }
}
