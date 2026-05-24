using System;

namespace Box2DNG
{
    public static class ContactManager
    {
        public static void Evaluate(Shape shapeA, Transform xfA, Shape shapeB, Transform xfB, Manifold manifold)
        {
            if (shapeA == null || shapeB == null)
            {
                throw new ArgumentNullException(shapeA == null ? nameof(shapeA) : nameof(shapeB));
            }

            switch (shapeA.Type)
            {
                case ShapeType.Circle:
                    EvaluateCircle((CircleShape)shapeA, xfA, shapeB, xfB, manifold);
                    break;
                case ShapeType.Capsule:
                    EvaluateCapsule((CapsuleShape)shapeA, xfA, shapeB, xfB, manifold);
                    break;
                case ShapeType.Segment:
                    EvaluateSegment((SegmentShape)shapeA, xfA, shapeB, xfB, manifold);
                    break;
                case ShapeType.Polygon:
                    EvaluatePolygon((PolygonShape)shapeA, xfA, shapeB, xfB, manifold);
                    break;
                case ShapeType.ChainSegment:
                    EvaluateChainSegment((ChainSegmentShape)shapeA, xfA, shapeB, xfB, manifold);
                    break;
                default:
                    manifold.PointCount = 0;
                    break;
            }

            if (manifold.PointCount == 0)
            {
                CollisionManifold.CollideDistance(
                    manifold,
                    ShapeGeometry.ToProxy(shapeA),
                    xfA,
                    ShapeGeometry.ToProxy(shapeB),
                    xfB);
            }
        }

        private static void EvaluateCircle(CircleShape shapeA, Transform xfA, Shape shapeB, Transform xfB, Manifold manifold)
        {
            Circle circleA = ShapeGeometry.ToCircle(shapeA);

            switch (shapeB.Type)
            {
                case ShapeType.Circle:
                    CollisionManifold.CollideCircles(manifold, circleA, xfA, ShapeGeometry.ToCircle((CircleShape)shapeB), xfB);
                    break;
                case ShapeType.Polygon:
                    {
                        Manifold temp = new Manifold();
                        CollisionManifold.CollidePolygonAndCircle(temp, ShapeGeometry.ToPolygon((PolygonShape)shapeB), xfB, circleA, xfA);
                        if (temp.PointCount == 0)
                        {
                            manifold.PointCount = 0;
                            break;
                        }
                        CopyManifold(manifold, temp, ManifoldType.FaceB);
                    }
                    break;
                case ShapeType.Segment:
                    {
                        Manifold temp = new Manifold();
                        CollisionManifold.CollideSegmentAndCircle(temp, ShapeGeometry.ToSegment((SegmentShape)shapeB), xfB, circleA, xfA);
                        CopyManifold(manifold, temp, ManifoldType.FaceB);
                    }
                    break;
                case ShapeType.ChainSegment:
                    {
                        Manifold temp = new Manifold();
                        CollisionManifold.CollideChainSegmentAndCircle(temp, ShapeGeometry.ToChainSegment((ChainSegmentShape)shapeB), xfB, circleA, xfA);
                        CopyManifold(manifold, temp, ManifoldType.FaceB);
                    }
                    break;
                default:
                    CollisionManifold.CollideDistance(manifold,
                        ShapeProxyFactory.FromCircle(circleA), xfA,
                        ShapeGeometry.ToProxy(shapeB), xfB);
                    break;
            }
        }

        private static void EvaluateCapsule(CapsuleShape shapeA, Transform xfA, Shape shapeB, Transform xfB, Manifold manifold)
        {
            Capsule capsuleA = ShapeGeometry.ToCapsule(shapeA);

            switch (shapeB.Type)
            {
                case ShapeType.Circle:
                    CollisionManifold.CollideCapsuleAndCircle(manifold, capsuleA, xfA, ShapeGeometry.ToCircle((CircleShape)shapeB), xfB);
                    break;
                case ShapeType.Capsule:
                    CollisionManifold.CollideCapsules(manifold, capsuleA, xfA, ShapeGeometry.ToCapsule((CapsuleShape)shapeB), xfB);
                    break;
                case ShapeType.Polygon:
                    CollisionManifold.CollideCapsuleAndPolygon(manifold, capsuleA, xfA, ShapeGeometry.ToPolygon((PolygonShape)shapeB), xfB);
                    break;
                case ShapeType.Segment:
                    CollisionManifold.CollideDistance(manifold,
                        ShapeProxyFactory.FromCapsule(capsuleA), xfA,
                        ShapeProxyFactory.FromSegment(ShapeGeometry.ToSegment((SegmentShape)shapeB)), xfB);
                    break;
                case ShapeType.ChainSegment:
                    {
                        Manifold temp = new Manifold();
                        CollisionManifold.CollideChainSegmentAndCapsule(temp, ShapeGeometry.ToChainSegment((ChainSegmentShape)shapeB), xfB, capsuleA, xfA);
                        CopyManifold(manifold, temp, ManifoldType.FaceB);
                    }
                    break;
                default:
                    CollisionManifold.CollideDistance(manifold,
                        ShapeProxyFactory.FromCapsule(capsuleA), xfA,
                        ShapeGeometry.ToProxy(shapeB), xfB);
                    break;
            }
        }

        private static void EvaluateSegment(SegmentShape shapeA, Transform xfA, Shape shapeB, Transform xfB, Manifold manifold)
        {
            Segment segmentA = ShapeGeometry.ToSegment(shapeA);

            switch (shapeB.Type)
            {
                case ShapeType.Circle:
                    CollisionManifold.CollideSegmentAndCircle(manifold, segmentA, xfA, ShapeGeometry.ToCircle((CircleShape)shapeB), xfB);
                    break;
                case ShapeType.Capsule:
                    CollisionManifold.CollideSegmentAndCapsule(manifold, segmentA, xfA, ShapeGeometry.ToCapsule((CapsuleShape)shapeB), xfB);
                    break;
                case ShapeType.Polygon:
                    CollisionManifold.CollideSegmentAndPolygon(manifold, segmentA, xfA, ShapeGeometry.ToPolygon((PolygonShape)shapeB), xfB);
                    break;
                default:
                    CollisionManifold.CollideDistance(manifold,
                        ShapeProxyFactory.FromSegment(segmentA), xfA,
                        ShapeGeometry.ToProxy(shapeB), xfB);
                    break;
            }
        }

        private static void EvaluatePolygon(PolygonShape shapeA, Transform xfA, Shape shapeB, Transform xfB, Manifold manifold)
        {
            Polygon polygonA = ShapeGeometry.ToPolygon(shapeA);

            switch (shapeB.Type)
            {
                case ShapeType.Circle:
                    CollisionManifold.CollidePolygonAndCircle(manifold, polygonA, xfA, ShapeGeometry.ToCircle((CircleShape)shapeB), xfB);
                    break;
                case ShapeType.Polygon:
                    CollisionManifold.CollidePolygons(manifold, polygonA, xfA, ShapeGeometry.ToPolygon((PolygonShape)shapeB), xfB);
                    break;
                case ShapeType.Segment:
                    {
                        Manifold temp = new Manifold();
                        CollisionManifold.CollideSegmentAndPolygon(temp, ShapeGeometry.ToSegment((SegmentShape)shapeB), xfB, polygonA, xfA);
                        CopyManifold(manifold, temp, ManifoldType.FaceB);
                    }
                    break;
                case ShapeType.ChainSegment:
                    {
                        Manifold temp = new Manifold();
                        CollisionManifold.CollideChainSegmentAndPolygon(temp, ShapeGeometry.ToChainSegment((ChainSegmentShape)shapeB), xfB, polygonA, xfA);
                        CopyManifold(manifold, temp, ManifoldType.FaceB);
                    }
                    break;
                default:
                    CollisionManifold.CollideDistance(manifold,
                        ShapeProxyFactory.FromPolygon(polygonA), xfA,
                        ShapeGeometry.ToProxy(shapeB), xfB);
                    break;
            }
        }

        private static void EvaluateChainSegment(ChainSegmentShape shapeA, Transform xfA, Shape shapeB, Transform xfB, Manifold manifold)
        {
            ChainSegment chainA = ShapeGeometry.ToChainSegment(shapeA);

            switch (shapeB.Type)
            {
                case ShapeType.Circle:
                    CollisionManifold.CollideChainSegmentAndCircle(manifold, chainA, xfA, ShapeGeometry.ToCircle((CircleShape)shapeB), xfB);
                    break;
                case ShapeType.Capsule:
                    CollisionManifold.CollideChainSegmentAndCapsule(manifold, chainA, xfA, ShapeGeometry.ToCapsule((CapsuleShape)shapeB), xfB);
                    break;
                case ShapeType.Polygon:
                    CollisionManifold.CollideChainSegmentAndPolygon(manifold, chainA, xfA, ShapeGeometry.ToPolygon((PolygonShape)shapeB), xfB);
                    break;
                default:
                    CollisionManifold.CollideChainDistanceOneSided(manifold, chainA, xfA, ShapeGeometry.ToProxy(shapeB), xfB);
                    break;
            }
        }

        private static void CopyManifold(Manifold target, Manifold source, ManifoldType? overrideType = null)
        {
            target.PointCount = source.PointCount;
            target.LocalNormal = source.LocalNormal;
            target.LocalPoint = source.LocalPoint;
            target.Type = overrideType ?? source.Type;
            for (int i = 0; i < source.PointCount; ++i)
            {
                ManifoldPoint mp = source.Points[i];
                target.Points[i] = new ManifoldPoint(mp.LocalPoint, mp.NormalImpulse, mp.TangentImpulse, mp.Id);
            }
        }
    }
}
