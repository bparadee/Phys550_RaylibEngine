using Physics550Engine_Raylib.Physics.Interfaces;
using System.Diagnostics;
using System.Numerics;

namespace Physics550Engine_Raylib.Physics
{
    public class CollisionResult
    {
        public bool HasCollision { get; set; }
        public Vector3 ContactPoint { get; set; }
        public float PenetrationDepth { get; set; }
        public Vector3 CollisionNormal { get; set; }
    }
    public class Collider : IDisposable
    {
        //only want one collider instance we already only have one PhysicsEgnine, so this might not be necessary
        private static readonly Lazy<Collider> instance = new Lazy<Collider>(() => new Collider());
        public static Collider Instance { get { return instance.Value; } }
        private int UniqueId = 0;
        //map of index to shape, index will be a unique identifier to that shape to be used with the broadphase collision detection.
        public Dictionary<int, RigidBody> ShapeMap = [];
        public HashSet<Tuple<int, int>> PotentialCollisions = [];

        public int BCDType = 0;
        private const float EPSILON = 1e-6f;
        private static int MAX_DEPTH = 8;
        private static int MAX_OBJ = 8;

        private StreamWriter bpcd_times_file;
        private StreamWriter npcd_times_file;
        //private Stopwatch stopwatch;
        //private bool step_measured;// = false;

        public Collider()
        {
            string docPath = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);
            bpcd_times_file = new StreamWriter(Path.Combine(docPath, "bpcd_file.txt"), true);
            npcd_times_file = new StreamWriter(Path.Combine(docPath, "npcd_file.txt"), true);
            //stopwatch = new Stopwatch();
        }

        public void Dispose()
        {
            //if (bpcd_times_file != null) { bpcd_times_file.Close(); }
            //if (npcd_times_file != null) { npcd_times_file.Close(); }
        }
        public int AddRigidBody(RigidBody rigidBody)
        {
            if (ShapeMap.ContainsValue(rigidBody))
            {
                return -1;
            }
            ShapeMap.Add(++UniqueId, rigidBody);
            return UniqueId;
        }

        public bool RemoveRigidBody(int UniqueId)
        {
            return ShapeMap.Remove(UniqueId);
        }

        public void Clear()
        {
            ShapeMap.Clear();
        }

        public void Step()
        {
            //stopwatch.Restart();
            switch(BCDType)
            {
                case 1:
                    PotentialCollisions = OctTreeBPCD();
                break;
                case 2:
                    PotentialCollisions = SpatialSubdivision(1.0f);
                break;
                case 3:
                    PotentialCollisions = SortAndSweep();
                break;
                case 0:
                default:
                    PotentialCollisions = [];
                    //Broad Phase Collision Detection will replace 
                    foreach (var shapePair1 in ShapeMap)
                    {
                        foreach (var shapePair2 in ShapeMap)
                        {
                            if (shapePair1.Key == shapePair2.Key || PotentialCollisions.Contains(Tuple.Create(shapePair2.Key, shapePair1.Key)))
                            {
                                continue;
                            }
                            PotentialCollisions.Add(Tuple.Create(shapePair1.Key, shapePair2.Key));
                        }
                    }
                break;
            }
            //if (ShapeMap.Count % 10 == 0 && !step_measured)
            //{
            //        bpcd_times_file.WriteLine(ShapeMap.Count + ",  " + stopwatch.ElapsedMilliseconds);
            //        bpcd_times_file.Flush();
            //}
            //stopwatch.Restart();
            foreach (var pair in PotentialCollisions)
            {
                CollisionResult collisionResult;

                ShapeMap[pair.Item1].BoundingShape.IsColliding = false;
                ShapeMap[pair.Item2].BoundingShape.IsColliding = false;

                if (DetectCollision(pair, out collisionResult))
                {
                    ResolveCollision(pair, collisionResult);
                }
            }

            //if (ShapeMap.Count % 10 == 0 && !step_measured)
            //{
            //    npcd_times_file.WriteLine(stopwatch.ElapsedMilliseconds);
            //    npcd_times_file.Flush();
            //    step_measured = true;
            //}
            //if (ShapeMap.Count % 10 == 1)
            //{
            //    step_measured = false;
            //}
            //stopwatch.Stop();
        }

        private HashSet<Tuple<int, int>> SortAndSweep()
        {
            // basically just projecting to an axis, gonna arbitrarily pick the x-axis,
            // could do all of them in the future and have a way to change which you test for.
            Vector3 axis = Vector3.UnitX;
            // Tuple is projected_endoint, isBeginning(true = start, false=end), Id (value in shapeMa)
            List<Tuple<float, bool, int>> projected_endpoints = new List<Tuple<float, bool, int>>();
            foreach (var pair in ShapeMap)
            {
                ProjectOBB((OrientedBoundingBox)pair.Value.BoundingShape, axis, out float min, out float max);
                projected_endpoints.Add(Tuple.Create(min, true, pair.Key));
                projected_endpoints.Add(Tuple.Create(max, false, pair.Key));
            }

            //lambda here just uses the first item to sort the list, it's what we care about when sorting.
            projected_endpoints.Sort((x, y) => x.Item1.CompareTo(y.Item1));

            HashSet<Tuple<int, int>> pc = new HashSet<Tuple<int, int>>();
            HashSet<int> active_shapes = new HashSet<int>();

            // make sure this goes in sorter order?? it should...
            int curr_id = 0;
            foreach (var item in projected_endpoints)
            {
                if (item.Item2)
                {
                    curr_id = item.Item3;
                    foreach (var other_id in active_shapes)
                    {
                        if (other_id != curr_id && !pc.Contains(Tuple.Create(other_id, curr_id)))
                        {
                            pc.Add(Tuple.Create(other_id, curr_id));
                        }
                    }
                    active_shapes.Add(item.Item3);
                }
                else
                {
                    active_shapes.Remove(item.Item3);
                }
            }

            return pc;
        }
        private HashSet<Tuple<int, int>> SpatialSubdivision(float side)
        {
            // side param needs to 
            Dictionary<int, Tuple<int, int>> grid_spots = new Dictionary<int, Tuple<int, int>>();
            List<int> grid_spots_collide_all = new List<int>();

            //assign all of them a grid spot here 
            foreach (var pair in ShapeMap)
            {
                RigidBody body = pair.Value;
                if (body.CollideAll)
                {
                    grid_spots_collide_all.Add(pair.Key);
                }
                else
                {
                    var grid_spot = Tuple.Create((int)Math.Floor(body.BoundingShape.Position.X),
                                                 (int)Math.Floor(body.BoundingShape.Position.Z));
                    grid_spots.Add(pair.Key, grid_spot);
                }
            }


            HashSet<Tuple<int, int>> pc = new HashSet<Tuple<int, int>>();
            //now go through the list
            foreach (var Id in grid_spots.Keys)
            {
                Tuple<int, int> curr_grid = grid_spots[Id];
                foreach (var Id2 in grid_spots.Keys)
                {
                    Tuple<int, int> other_grid = grid_spots[Id2];
                    if ( Id != Id2 &&
                         !pc.Contains(Tuple.Create(Id2, Id)) &&
                         AreAdjacent(curr_grid, other_grid))
                    {
                        pc.Add(Tuple.Create(Id, Id2)); 
                    }
                }
                //also add collisions for ones that need to always be considered "collide_all"
                foreach (int Id2 in grid_spots_collide_all)
                {
                    pc.Add(Tuple.Create(Id, Id2));
                }
            }

            foreach (int Id in grid_spots_collide_all)
            {
                foreach (int Id2 in grid_spots_collide_all)
                {
                    if (Id != Id2 && !pc.Contains(Tuple.Create(Id2, Id)))
                    {
                        pc.Add(Tuple.Create(Id, Id2));
                    }
                }
            }

            return pc;
        }

        private bool AreAdjacent(Tuple<int, int> curr, Tuple<int, int> other)
        {
            // both need to be surrounding grid spaces
            return Math.Abs(curr.Item1 - other.Item1) <= 1 && Math.Abs(curr.Item2 - other.Item2) <= 1;
        }

        class OctTreeNode
        {
            public Vector3 Center;
            public float Boundary;
            public HashSet<Tuple<int, RigidBody>> Bodies = new HashSet<Tuple<int, RigidBody>>();
            public List<OctTreeNode> Children = new List<OctTreeNode>();

            public OctTreeNode(Vector3 center, float boundary)
            {
                Center = center;
                Boundary = boundary;
            }

            public bool Insert(int Id, RigidBody body, int depth)
            {
                // assumes .7 side cube!!!! will change later.
                if( (body.Position.X - 1 < Center.X - Boundary) ||
                    (body.Position.X + 1 > Center.X + Boundary) ||
                    (body.Position.Y - 1 < Center.Y - Boundary) ||
                    (body.Position.Y + 1 > Center.Y + Boundary) ||
                    (body.Position.Z - 1 < Center.Z - Boundary) ||
                    (body.Position.Z + 1 > Center.Z + Boundary) )
                {
                    return false;
                }


                if (Children.Count == 0)
                {
                    if (Bodies.Count < Collider.MAX_OBJ || depth >= MAX_DEPTH)
                    {
                        Bodies.Add(Tuple.Create(Id, body));
                        return true;
                    }
                    else
                    {
                        float newCenter = Boundary / 4;
                        Children.Add(new OctTreeNode(Center + new Vector3(newCenter, newCenter, newCenter), newCenter));
                        Children.Add(new OctTreeNode(Center + new Vector3(newCenter, newCenter, -newCenter), newCenter));
                        Children.Add(new OctTreeNode(Center + new Vector3(newCenter, -newCenter, -newCenter), newCenter));
                        Children.Add(new OctTreeNode(Center + new Vector3(-newCenter, newCenter, newCenter), newCenter));
                        Children.Add(new OctTreeNode(Center + new Vector3(-newCenter, -newCenter, newCenter), newCenter));
                        Children.Add(new OctTreeNode(Center + new Vector3(-newCenter, -newCenter, -newCenter), newCenter));
                        Children.Add(new OctTreeNode(Center + new Vector3(-newCenter, newCenter, -newCenter), newCenter));
                        Children.Add(new OctTreeNode(Center + new Vector3(newCenter, -newCenter, newCenter), newCenter));

                        foreach (var bodyPair in Bodies)
                        {
                            foreach (var child in Children)
                            {
                                if (child.Insert(bodyPair.Item1, bodyPair.Item2, depth + 1))
                                {
                                    Bodies.Remove(bodyPair);
                                }
                            }
                        }
                    }
                }

                foreach (var child in Children)
                {
                    if (child.Insert(Id, body, depth + 1))
                    {
                        return true;
                    }
                }

                Bodies.Add(Tuple.Create(Id, body));
                return true;
            }

            public void FindPotentialCollisions(int Id, RigidBody body, HashSet<Tuple<int, int>> potentialCollisions)
            {
                // assumes .7 side cube!!!! will change later.
                if ((body.Position.X - 1 < Center.X - Boundary) ||
                    (body.Position.X + 1 > Center.X + Boundary) ||
                    (body.Position.Y - 1 < Center.Y - Boundary) ||
                    (body.Position.Y + 1 > Center.Y + Boundary) ||
                    (body.Position.Z - 1 < Center.Z - Boundary) ||
                    (body.Position.Z + 1 > Center.Z + Boundary))
                {
                    return;
                }
                
                //contained this cube
                foreach (var bodyPair in Bodies)
                {
                    if (Id == bodyPair.Item1 || potentialCollisions.Contains(Tuple.Create(bodyPair.Item1, Id)))
                    {
                        continue;
                    }
                    potentialCollisions.Add(Tuple.Create(Id, bodyPair.Item1));
                }

                if (Children.Count == 0)
                {
                    return;
                }

                foreach (var child in Children)
                {
                    child.FindPotentialCollisions(Id, body, potentialCollisions);
                }
            }
        }
        private HashSet<Tuple<int, int>> OctTreeBPCD()
        {
            //build
            //centered on (0,0,0) assumed.
            float boundary = 100;
            OctTreeNode root = new OctTreeNode(new Vector3(0,0,0), boundary);
            root.Center = new Vector3(0, 0, 0);
            root.Boundary = boundary;
            
            //creation of the tree! do this every step, but it's still faster, source trust me bro
            foreach (var pair in ShapeMap)
            {
                root.Insert(pair.Key, pair.Value, 0);
            }

            HashSet < Tuple<int, int> > pc = new HashSet<Tuple<int, int>>();
            //now we use the tree we created!
            foreach (var pair in ShapeMap)
            {
                root.FindPotentialCollisions(pair.Key, pair.Value, pc);
            }

            return pc;
        }
        
        void ResolveCollision(Tuple<int, int> pair, CollisionResult collisionResult)
        {
            RigidBody shape1 = ShapeMap[pair.Item1];
            RigidBody shape2 = ShapeMap[pair.Item2];

            //shape1.BoundingShape.IsColliding = true;
            //shape2.BoundingShape.IsColliding = true;

            Vector3 relativePosition1 = collisionResult.ContactPoint - shape1.Position;
            Vector3 relativePosition2 = collisionResult.ContactPoint - shape2.Position;

            float invMass1 = 1.0f / shape1.Mass;
            float invMass2 = 1.0f / shape2.Mass;

            Matrix4x4 invInertia1 = shape1.GetWorldInverseInertiaTensor();
            Matrix4x4 invInertia2 = shape2.GetWorldInverseInertiaTensor();

            Vector3 velocityA = shape1.GetVelocityAtPoint(collisionResult.ContactPoint);
            Vector3 velocityB = shape2.GetVelocityAtPoint(collisionResult.ContactPoint);
            Vector3 relativeVelocity = velocityB - velocityA;

            float velocityAlongNormal = Vector3.Dot(relativeVelocity, collisionResult.CollisionNormal);
            if (velocityAlongNormal > EPSILON)
                return; // Objects are moving apart so we dont need to do anything.


            Vector3 cross1 = Vector3.Cross(relativePosition1, collisionResult.CollisionNormal);
            Vector3 cross2 = Vector3.Cross(relativePosition2, collisionResult.CollisionNormal);
            Vector3 angularComponent1 = Vector3.Transform(cross1, invInertia1);
            Vector3 angularComponent2 = Vector3.Transform(cross2, invInertia2);

            // Calculate restitution, this might be an oversimplification, idk should be close enough
            float restitution = Math.Min(shape1.Restitution, shape2.Restitution);

            float j = -(1 + restitution) * velocityAlongNormal /
                (invMass1 + invMass2 +
                Vector3.Dot(Vector3.Cross(angularComponent1, relativePosition1)
                + Vector3.Cross(angularComponent2, relativePosition2), collisionResult.CollisionNormal) + EPSILON);

            Vector3 impulse = j * collisionResult.CollisionNormal;

            shape1.Velocity -= (impulse * invMass1);
            shape2.Velocity += (impulse * invMass2);

            Vector3 angularImpulseA = Vector3.Cross(relativePosition1, impulse);
            Vector3 angularImpulseB = Vector3.Cross(relativePosition2, impulse);

            shape1.RotationalVelocity -= Vector3.Transform(angularImpulseA, invInertia1);
            shape2.RotationalVelocity += Vector3.Transform(angularImpulseB, invInertia2);

            //// TODO friction (if time or need)

            ResolvePosition(shape1, shape2, collisionResult.CollisionNormal, collisionResult.PenetrationDepth, invMass1, invMass2);
        }

        private static void ResolvePosition(RigidBody shape1, RigidBody shape2, Vector3 collisionNormal,
                                   float penetrationDepth, float invMass1, float invMass2)
        {
            const float PenetrationAllowance = EPSILON * 100;
            const float PenetrationCorrection = 1.0f;  // 0 to 1 (how much to fix)

            // Only fix position if penetration is significant
            if (penetrationDepth > PenetrationAllowance)
            {
                float correctionMagnitude = (penetrationDepth * PenetrationCorrection) / (invMass1 + invMass2);
                Vector3 correction = correctionMagnitude * collisionNormal;

                shape1.BoundingShape.Position += correction * invMass1;
                shape2.BoundingShape.Position -= correction * invMass2;
            }
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

        private bool DetectOBBOBBCollision(OrientedBoundingBox box1, OrientedBoundingBox box2, out CollisionResult result)
        {
            result = new CollisionResult();
            result.HasCollision = false;
            result.PenetrationDepth = float.MaxValue;
            int save_i = int.MaxValue;
            Vector3[] box1Axes = box1.GetAxes();
            Vector3[] box2Axes = box2.GetAxes();

            Vector3 translation = box2.Position - box1.Position;

            Vector3[] axes = new Vector3[15];

            for (int i = 0; i < 3; i++)
            {
                axes[i] = box1Axes[i];
                axes[i + 3] = box2Axes[i];
            }

            int index = 6;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    axes[index++] = Vector3.Cross(box1Axes[i], box2Axes[j]);
                }
            }

            for (int i = 0; i < 15; i++)
            {
                if (Vector3.Dot(axes[i], axes[i]) < 1e-6f)
                    continue;

                // Normalize the axis, not sure we have done this up to this point ? check this
                axes[i] = Vector3.Normalize(axes[i]);

                ProjectOBB(box1, axes[i], out float box1Min, out float box1Max);
                ProjectOBB(box2, axes[i], out float box2Min, out float box2Max);

                if (box1Max < box2Min || box2Max < box1Min)
                {
                    return false;
                }

                float overlap = Math.Min(box1Max, box2Max) - Math.Max(box1Min, box2Min);

                if (overlap < result.PenetrationDepth)
                {
                    result.PenetrationDepth = overlap;
                    result.CollisionNormal = axes[i];
                    save_i = i;

                    if (Vector3.Dot(translation, result.CollisionNormal) < 0)
                        result.CollisionNormal = -result.CollisionNormal;
                }
            }

            // If we get here, no separating axis was found - boxes are colliding
            result.HasCollision = true;

            // Calculate contact point - this is a bit of a simplification that uses the better of the two colliding shapes.
            if (save_i < 3) {
                result.ContactPoint = GetSupportPoint(box2, result.CollisionNormal);
            } else if (save_i < 6)
            {
                result.ContactPoint = GetSupportPoint(box1, result.CollisionNormal);
            } else
            {
                result.ContactPoint = CalculateContactPoint(box1, box2, result.CollisionNormal, result.PenetrationDepth);
            }

            //Raylib.DrawSphere(result.ContactPoint, .1f, Color.Blue);
            return true;
        }

        private static void ProjectOBB(OrientedBoundingBox box, Vector3 axis, out float min, out float max)
        {
            float center = Vector3.Dot(box.Position, axis);

            float radius = 0;
            Vector3[] axes = box.GetAxes();

            for (int i = 0; i < 3; i++)
            {
                radius += Math.Abs(Vector3.Dot(box.SideLengths[i] / 2 * axes[i], axis));
            }

            min = center - radius;
            max = center + radius;
        }

        private static Vector3 CalculateContactPoint(OrientedBoundingBox box1, OrientedBoundingBox box2, Vector3 normal, float penetration)
        {
            Vector3 supportPoint1 = GetSupportPoint(box1, -normal);
            Vector3 supportPoint2 = GetSupportPoint(box2, normal);

            Vector3 pointOnBox1 = supportPoint1 + normal * penetration * 0.5f;
            Vector3 pointOnBox2 = supportPoint2 - normal * penetration * 0.5f;

            return (pointOnBox1 + pointOnBox2) * 0.5f;
        }

        private static Vector3 GetSupportPoint(OrientedBoundingBox box, Vector3 direction)
        {
            Matrix4x4 invRotation = Matrix4x4.CreateFromQuaternion(Quaternion.Inverse(box.Orientation));
            Vector3 localDir = Vector3.Transform(direction, invRotation);

            // have seen some error with quat conversion here
            if (localDir.X < EPSILON) localDir.X = 0;
            if (localDir.Y < EPSILON) localDir.Y = 0;
            if (localDir.Z < EPSILON) localDir.Z = 0;

            Vector3 localSupport = new Vector3(
                Math.Sign(localDir.X) * box.SideLengths.X / 2,
                Math.Sign(localDir.Y) * box.SideLengths.Y / 2,
                Math.Sign(localDir.Z) * box.SideLengths.Z / 2
            );

            Matrix4x4 rotation = Matrix4x4.CreateFromQuaternion(box.Orientation);
            Vector3 worldSupport = Vector3.Transform(localSupport, rotation);

            // have seen some error with quat conversion here
            if (worldSupport.X < EPSILON) localDir.X = 0;
            if (worldSupport.Y < EPSILON) localDir.Y = 0;
            if (worldSupport.Z < EPSILON) localDir.Z = 0;

            return box.Position + worldSupport;
        }
    }
}
