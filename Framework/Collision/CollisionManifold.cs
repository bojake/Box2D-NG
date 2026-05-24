using System;

namespace Box2DNG
{
    public static class CollisionManifold
    {
        private const float RelativeTol = 0.98f;
        private const float AbsoluteTol = 0.001f;

        private static readonly ClipVertex[] IncidentEdge = new ClipVertex[2];
        private static readonly ClipVertex[] ClipPoints1 = new ClipVertex[2];
        private static readonly ClipVertex[] ClipPoints2 = new ClipVertex[2];

        public static void CollideCircles(Manifold manifold, Circle circleA, Transform xfA, Circle circleB, Transform xfB)
        {
            manifold.PointCount = 0;

            Vec2 pA = Transform.Mul(xfA, circleA.Center);
            Vec2 pB = Transform.Mul(xfB, circleB.Center);
            Vec2 d = pB - pA;

            float distSqr = d.LengthSquared;
            float radius = circleA.Radius + circleB.Radius;

            if (distSqr > radius * radius)
            {
                return;
            }

            manifold.Type = ManifoldType.Circles;
            manifold.LocalPoint = circleA.Center;
            manifold.LocalNormal = Vec2.Zero;
            manifold.PointCount = 1;
            manifold.Points[0] = new ManifoldPoint(circleB.Center, 0f, 0f, new ContactFeature(0, 0, 0, 0));
        }

        public static void CollidePolygonAndCircle(Manifold manifold, Polygon polygonA, Transform xfA, Circle circleB, Transform xfB)
        {
            manifold.PointCount = 0;

            Vec2 cWorld = Transform.Mul(xfB, circleB.Center);
            Vec2 cLocal = Transform.MulT(xfA, cWorld);

            int normalIndex = 0;
            float separation = -float.MaxValue;
            float radius = polygonA.Radius + circleB.Radius;
            int vertexCount = polygonA.Count;

            for (int i = 0; i < vertexCount; ++i)
            {
                Vec2 diff = cLocal - polygonA.Vertices[i];
                float s = Vec2.Dot(polygonA.Normals[i], diff);

                if (s > radius)
                {
                    return;
                }

                if (s > separation)
                {
                    separation = s;
                    normalIndex = i;
                }
            }

            int vertIndex1 = normalIndex;
            int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
            Vec2 v1 = polygonA.Vertices[vertIndex1];
            Vec2 v2 = polygonA.Vertices[vertIndex2];

            if (separation < Constants.Epsilon)
            {
                manifold.PointCount = 1;
                manifold.Type = ManifoldType.FaceA;
                manifold.LocalNormal = polygonA.Normals[normalIndex];
                manifold.LocalPoint = 0.5f * (v1 + v2);
                manifold.Points[0] = new ManifoldPoint(circleB.Center, 0f, 0f, new ContactFeature(0, 0, 0, 0));
                return;
            }

            float u1 = Vec2.Dot(cLocal - v1, v2 - v1);
            float u2 = Vec2.Dot(cLocal - v2, v1 - v2);

            if (u1 <= 0f)
            {
                if ((cLocal - v1).LengthSquared > radius * radius)
                {
                    return;
                }

                manifold.PointCount = 1;
                manifold.Type = ManifoldType.FaceA;
                manifold.LocalNormal = (cLocal - v1).Normalize();
                manifold.LocalPoint = v1;
                manifold.Points[0] = new ManifoldPoint(circleB.Center, 0f, 0f, new ContactFeature(0, 0, 0, 0));
            }
            else if (u2 <= 0f)
            {
                if ((cLocal - v2).LengthSquared > radius * radius)
                {
                    return;
                }

                manifold.PointCount = 1;
                manifold.Type = ManifoldType.FaceA;
                manifold.LocalNormal = (cLocal - v2).Normalize();
                manifold.LocalPoint = v2;
                manifold.Points[0] = new ManifoldPoint(circleB.Center, 0f, 0f, new ContactFeature(0, 0, 0, 0));
            }
            else
            {
                Vec2 faceCenter = 0.5f * (v1 + v2);
                separation = Vec2.Dot(cLocal - faceCenter, polygonA.Normals[vertIndex1]);
                if (separation > radius)
                {
                    return;
                }

                manifold.PointCount = 1;
                manifold.Type = ManifoldType.FaceA;
                manifold.LocalNormal = polygonA.Normals[vertIndex1];
                manifold.LocalPoint = faceCenter;
                manifold.Points[0] = new ManifoldPoint(circleB.Center, 0f, 0f, new ContactFeature(0, 0, 0, 0));
            }
        }

        public static void CollidePolygons(Manifold manifold, Polygon polyA, Transform xfA, Polygon polyB, Transform xfB)
        {
            manifold.PointCount = 0;
            float totalRadius = polyA.Radius + polyB.Radius;

            float separationA = FindMaxSeparation(out int edgeA, polyA, xfA, polyB, xfB);
            if (separationA > totalRadius)
            {
                return;
            }

            float separationB = FindMaxSeparation(out int edgeB, polyB, xfB, polyA, xfA);
            if (separationB > totalRadius)
            {
                return;
            }

            Polygon poly1;
            Polygon poly2;
            Transform xf1;
            Transform xf2;
            int edge1;
            bool flip;

            if (separationB > RelativeTol * separationA + AbsoluteTol)
            {
                poly1 = polyB;
                poly2 = polyA;
                xf1 = xfB;
                xf2 = xfA;
                edge1 = edgeB;
                manifold.Type = ManifoldType.FaceB;
                flip = true;
            }
            else
            {
                poly1 = polyA;
                poly2 = polyB;
                xf1 = xfA;
                xf2 = xfB;
                edge1 = edgeA;
                manifold.Type = ManifoldType.FaceA;
                flip = false;
            }

            FindIncidentEdge(IncidentEdge, poly1, xf1, edge1, poly2, xf2);

            int count1 = poly1.Count;
            Vec2 v11 = poly1.Vertices[edge1];
            Vec2 v12 = poly1.Vertices[edge1 + 1 < count1 ? edge1 + 1 : 0];

            Vec2 localTangent = (v12 - v11).Normalize();
            Vec2 localNormal = new Vec2(localTangent.Y, -localTangent.X);
            Vec2 planePoint = 0.5f * (v11 + v12);

            Vec2 tangent = Rot.Mul(xf1.Q, localTangent);
            Vec2 normal = new Vec2(tangent.Y, -tangent.X);

            Vec2 v11World = Transform.Mul(xf1, v11);
            Vec2 v12World = Transform.Mul(xf1, v12);

            float frontOffset = Vec2.Dot(normal, v11World);
            float sideOffset1 = -Vec2.Dot(tangent, v11World) + totalRadius;
            float sideOffset2 = Vec2.Dot(tangent, v12World) + totalRadius;

            Vec2 negTangent = -tangent;
            int np = ClipSegmentToLine(ClipPoints1, IncidentEdge, negTangent, sideOffset1, (byte)(edge1));
            if (np < 2)
            {
                return;
            }

            np = ClipSegmentToLine(ClipPoints2, ClipPoints1, tangent, sideOffset2, (byte)(edge1 + 1 < count1 ? edge1 + 1 : 0));
            if (np < 2)
            {
                return;
            }

            manifold.LocalNormal = localNormal;
            manifold.LocalPoint = planePoint;

            int pointCount = 0;
            for (int i = 0; i < 2; ++i)
            {
                Vec2 v = ClipPoints2[i].V;
                float separation = Vec2.Dot(normal, v) - frontOffset;

                if (separation <= totalRadius)
                {
                    Vec2 localPoint = Transform.MulT(xf2, v);
                    ContactFeature id = ClipPoints2[i].Id;
                    if (flip)
                    {
                        id = new ContactFeature(id.TypeB, id.TypeA, id.IndexB, id.IndexA);
                    }
                    manifold.Points[pointCount] = new ManifoldPoint(localPoint, 0f, 0f, id);
                    pointCount++;
                }
            }

            manifold.PointCount = pointCount;
        }

        public static void CollideCapsuleAndCircle(Manifold manifold, Capsule capsuleA, Transform xfA, Circle circleB, Transform xfB)
        {
            BuildDistanceManifold(manifold, ShapeProxyFactory.FromCapsule(capsuleA), xfA, ShapeProxyFactory.FromCircle(circleB), xfB);
        }

        public static void CollideCapsules(Manifold manifold, Capsule capsuleA, Transform xfA, Capsule capsuleB, Transform xfB)
        {
            BuildDistanceManifold(manifold, ShapeProxyFactory.FromCapsule(capsuleA), xfA, ShapeProxyFactory.FromCapsule(capsuleB), xfB);
        }

        public static void CollideCapsuleAndPolygon(Manifold manifold, Capsule capsuleA, Transform xfA, Polygon polygonB, Transform xfB)
        {
            BuildDistanceManifold(manifold, ShapeProxyFactory.FromCapsule(capsuleA), xfA, ShapeProxyFactory.FromPolygon(polygonB), xfB);
        }

        public static void CollideSegmentAndCircle(Manifold manifold, Segment segmentA, Transform xfA, Circle circleB, Transform xfB)
        {
            manifold.PointCount = 0;

            Vec2 p1 = Transform.Mul(xfA, segmentA.Point1);
            Vec2 p2 = Transform.Mul(xfA, segmentA.Point2);
            Vec2 c = Transform.Mul(xfB, circleB.Center);

            Vec2 d = p2 - p1;
            float denom = d.LengthSquared;
            float t = 0f;
            if (denom > Constants.Epsilon)
            {
                t = MathFng.Clamp(Vec2.Dot(c - p1, d) / denom, 0f, 1f);
            }
            Vec2 closest = p1 + t * d;

            Vec2 n = c - closest;
            float radius = circleB.Radius + Constants.PolygonRadius;
            float distSqr = n.LengthSquared;
            if (distSqr > radius * radius)
            {
                return;
            }

            Vec2 normal;
            if (distSqr > Constants.Epsilon * Constants.Epsilon)
            {
                normal = n / MathF.Sqrt(distSqr);
            }
            else
            {
                Vec2 edge = p2 - p1;
                normal = MathFng.RightPerp(edge).Normalize();
                if (normal.LengthSquared <= Constants.Epsilon * Constants.Epsilon)
                {
                    normal = new Vec2(0f, 1f);
                }
            }

            manifold.Type = ManifoldType.FaceA;
            manifold.LocalNormal = Rot.MulT(xfA.Q, normal);
            manifold.LocalPoint = Transform.MulT(xfA, closest);
            manifold.PointCount = 1;
            manifold.Points[0] = new ManifoldPoint(Transform.MulT(xfB, c), 0f, 0f, new ContactFeature(0, 0, 0, 0));
        }

        public static void CollideSegmentAndCapsule(Manifold manifold, Segment segmentA, Transform xfA, Capsule capsuleB, Transform xfB)
        {
            BuildDistanceManifold(manifold, ShapeProxyFactory.FromSegment(segmentA), xfA, ShapeProxyFactory.FromCapsule(capsuleB), xfB);
        }

        public static void CollideSegmentAndPolygon(Manifold manifold, Segment segmentA, Transform xfA, Polygon polygonB, Transform xfB)
        {
            // Treat the segment as a thin rectangle to get robust contact generation with polygons.
            Polygon segmentPoly = BuildSegmentPolygon(segmentA, Constants.PolygonRadius);
            CollidePolygons(manifold, segmentPoly, xfA, polygonB, xfB);
        }

        public static void CollideChainSegmentAndCircle(Manifold manifold, ChainSegment chainA, Transform xfA, Circle circleB, Transform xfB)
        {
            // Port of b2CollideChainSegmentAndCircle from box2d-cpp/src/manifold.c:1089.
            // Works in segment-local space using barycentric u/v to pick the Voronoi
            // region (edge, p1-corner, or p2-corner), then uses the ghost vertices to
            // *reject* corner-region contacts that belong to a neighbouring segment.
            manifold.PointCount = 0;

            // Bring circle B's center into A's local frame.
            Vec2 pBWorld = Transform.Mul(xfB, circleB.Center);
            Vec2 pB = Transform.MulT(xfA, pBWorld);

            Vec2 p1 = chainA.Segment.Point1;
            Vec2 p2 = chainA.Segment.Point2;
            Vec2 e = p2 - p1;

            // Chains are one-sided: normal points to the right of the edge. Reject
            // anything on the back side.
            float offset = Vec2.Dot(MathFng.RightPerp(e), pB - p1);
            if (offset < 0f)
            {
                return;
            }

            // Unnormalised barycentric coordinates.
            float u = Vec2.Dot(e, p2 - pB);
            float v = Vec2.Dot(e, pB - p1);

            Vec2 pA;

            if (v <= 0f)
            {
                // Behind p1 — possibly in the Voronoi region of the previous edge.
                // If so, the previous chain segment owns this contact and we skip.
                Vec2 prevEdge = p1 - chainA.Ghost1;
                float uPrev = Vec2.Dot(prevEdge, pB - p1);
                if (uPrev <= 0f)
                {
                    return;
                }
                pA = p1;
            }
            else if (u <= 0f)
            {
                // Ahead of p2 — possibly in the Voronoi region of the next edge.
                Vec2 nextEdge = chainA.Ghost2 - p2;
                float vNext = Vec2.Dot(nextEdge, pB - p2);
                if (vNext > 0f)
                {
                    return;
                }
                pA = p2;
            }
            else
            {
                // Edge region — project pB onto the edge.
                float ee = Vec2.Dot(e, e);
                Vec2 weighted = new Vec2(u * p1.X + v * p2.X, u * p1.Y + v * p2.Y);
                pA = ee > 0f ? weighted * (1f / ee) : p1;
            }

            Vec2 d = pB - pA;
            float distSqr = d.LengthSquared;
            float radius = circleB.Radius + Constants.PolygonRadius;
            if (distSqr > radius * radius)
            {
                return;
            }

            Vec2 normal;
            if (distSqr > Constants.Epsilon * Constants.Epsilon)
            {
                normal = d * (1f / MathF.Sqrt(distSqr));
            }
            else
            {
                // Coincident — fall back to the segment's outward normal.
                normal = MathFng.RightPerp(e).Normalize();
                if (normal.LengthSquared <= Constants.Epsilon * Constants.Epsilon)
                {
                    normal = new Vec2(0f, 1f);
                }
            }

            manifold.Type = ManifoldType.FaceA;
            manifold.LocalNormal = normal; // already in A-local frame
            manifold.LocalPoint = pA;      // already in A-local frame
            manifold.Points[0] = new ManifoldPoint(circleB.Center, 0f, 0f, new ContactFeature(0, 0, 0, 0));
            manifold.PointCount = 1;
        }

        public static void CollideChainSegmentAndCapsule(Manifold manifold, ChainSegment chainA, Transform xfA, Capsule capsuleB, Transform xfB)
        {
            // Mirror box2d-cpp/src/manifold.c:1180 — treat the capsule as a 2-vertex
            // polygon and run the chain-segment-vs-polygon path so we get the
            // Gauss-map ghost handling.
            Polygon polyB = MakeCapsulePolygon(capsuleB.Center1, capsuleB.Center2, capsuleB.Radius);
            CollideChainSegmentAndPolygon(manifold, chainA, xfA, polyB, xfB);
        }

        public static void CollideChainSegmentAndPolygon(Manifold manifold, ChainSegment chainA, Transform xfA, Polygon polygonB, Transform xfB)
        {
            // Port of b2CollideChainSegmentAndPolygon from box2d-cpp/src/manifold.c:1319.
            // Operates in segment-A-local frame throughout, then writes a FaceA-style
            // manifold whose LocalNormal and LocalPoint stay in A's local frame
            // (the C# convention; cpp rotates these to world).
            manifold.PointCount = 0;

            // xf = xfA^-1 * xfB — applied below to bring polygon B into A's local frame.
            Rot xfQ = Rot.MulT(xfA.Q, xfB.Q);
            Vec2 xfP = Rot.MulT(xfA.Q, xfB.P - xfA.P);
            Transform xf = new Transform(xfP, xfQ);

            Vec2 centroidB = Transform.Mul(xf, polygonB.Centroid);
            float radiusB = polygonB.Radius;

            Vec2 p1 = chainA.Segment.Point1;
            Vec2 p2 = chainA.Segment.Point2;

            Vec2 edge1Raw = p2 - p1;
            if (edge1Raw.LengthSquared <= Constants.Epsilon * Constants.Epsilon)
            {
                return;
            }
            Vec2 edge1 = edge1Raw.Normalize();

            ChainSegmentParams smoothParams;
            smoothParams.Edge1 = edge1;

            const float convexTol = 0.01f;
            Vec2 edge0Raw = p1 - chainA.Ghost1;
            Vec2 edge0 = edge0Raw.LengthSquared > Constants.Epsilon * Constants.Epsilon ? edge0Raw.Normalize() : edge1;
            smoothParams.Normal0 = MathFng.RightPerp(edge0);
            smoothParams.Convex1 = Vec2.Cross(edge0, edge1) >= convexTol;

            Vec2 edge2Raw = chainA.Ghost2 - p2;
            Vec2 edge2 = edge2Raw.LengthSquared > Constants.Epsilon * Constants.Epsilon ? edge2Raw.Normalize() : edge1;
            smoothParams.Normal2 = MathFng.RightPerp(edge2);
            smoothParams.Convex2 = Vec2.Cross(edge1, edge2) >= convexTol;

            Vec2 normal1 = MathFng.RightPerp(edge1);
            bool behind1 = Vec2.Dot(normal1, centroidB - p1) < 0f;
            bool behind0 = true;
            bool behind2 = true;
            if (smoothParams.Convex1)
            {
                behind0 = Vec2.Dot(smoothParams.Normal0, centroidB - p1) < 0f;
            }
            if (smoothParams.Convex2)
            {
                behind2 = Vec2.Dot(smoothParams.Normal2, centroidB - p2) < 0f;
            }
            if (behind1 && behind0 && behind2)
            {
                // One-sided rejection: polygon centroid is fully behind the chain.
                return;
            }

            // Bring polygon B into A's local frame.
            int count = polygonB.Count;
            Vec2[] vertices = new Vec2[count];
            Vec2[] normals = new Vec2[count];
            for (int i = 0; i < count; ++i)
            {
                vertices[i] = Transform.Mul(xf, polygonB.Vertices[i]);
                normals[i] = Rot.Mul(xf.Q, polygonB.Normals[i]);
            }

            // Distance call against the bare segment endpoints (partial-polygon GJK
            // doesn't work correctly).
            Vec2[] segPoints = new[] { p1, p2 };
            ShapeProxy proxyA = new ShapeProxy(segPoints, 2, 0f);
            ShapeProxy proxyB = new ShapeProxy(vertices, count, 0f);
            DistanceInput input = new DistanceInput(proxyA, proxyB, Transform.Identity, Transform.Identity, false);
            SimplexCache cache = new SimplexCache(0f, 0, 0, 0, 0, 0, 0, 0);
            DistanceOutput output = Distance.Compute(input, ref cache);

            float speculativeDistance = 0.5f * Constants.LinearSlop;
            if (output.Distance > radiusB + speculativeDistance)
            {
                return;
            }

            Vec2 n0 = smoothParams.Convex1 ? smoothParams.Normal0 : normal1;
            Vec2 n2 = smoothParams.Convex2 ? smoothParams.Normal2 : normal1;

            int incidentIndex = -1;
            int incidentNormal = -1;

            if (!behind1 && output.Distance > 0.1f * Constants.LinearSlop)
            {
                // Closest features are vertex-vertex or vertex-edge.
                if (cache.Count == 1)
                {
                    Vec2 pA = output.PointA;
                    Vec2 pB = output.PointB;
                    Vec2 diff = pB - pA;
                    Vec2 normal = diff.LengthSquared > Constants.Epsilon * Constants.Epsilon
                        ? diff.Normalize()
                        : normal1;

                    ChainNormalType type = ClassifyChainNormal(in smoothParams, normal);
                    if (type == ChainNormalType.Skip)
                    {
                        return;
                    }
                    if (type == ChainNormalType.Admit)
                    {
                        manifold.Type = ManifoldType.FaceA;
                        manifold.LocalNormal = normal;
                        manifold.LocalPoint = pA;
                        manifold.Points[0] = new ManifoldPoint(
                            Transform.MulT(xfB, Transform.Mul(xfA, pB)),
                            0f, 0f,
                            new ContactFeature((byte)cache.IndexA0, (byte)cache.IndexB0, 0, 0));
                        manifold.PointCount = 1;
                        return;
                    }
                    // Snap: fall through to segment-normal path.
                    incidentIndex = cache.IndexB0;
                }
                else
                {
                    // cache.Count == 2: vertex-edge case.
                    int ia1 = cache.IndexA0;
                    int ia2 = cache.IndexA1;
                    int ib1 = cache.IndexB0;
                    int ib2 = cache.IndexB1;

                    if (ia1 == ia2)
                    {
                        // Single vertex on A, edge on B. Pick the polygon normal
                        // that best aligns with the contact direction.
                        Vec2 normalB = output.PointA - output.PointB;
                        float dot1 = Vec2.Dot(normalB, normals[ib1]);
                        float dot2 = Vec2.Dot(normalB, normals[ib2]);
                        int ib = dot1 > dot2 ? ib1 : ib2;
                        normalB = normals[ib];

                        ChainNormalType type = ClassifyChainNormal(in smoothParams, -normalB);
                        if (type == ChainNormalType.Skip)
                        {
                            return;
                        }
                        if (type == ChainNormalType.Admit)
                        {
                            ib1 = ib;
                            ib2 = ib < count - 1 ? ib + 1 : 0;
                            Vec2 b1v = vertices[ib1];
                            Vec2 b2v = vertices[ib2];

                            // Pick incident segment vertex.
                            dot1 = Vec2.Dot(normalB, p1 - b1v);
                            dot2 = Vec2.Dot(normalB, p2 - b1v);
                            if (dot1 < dot2)
                            {
                                if (Vec2.Dot(n0, normalB) < Vec2.Dot(normal1, normalB)) return;
                            }
                            else
                            {
                                if (Vec2.Dot(n2, normalB) < Vec2.Dot(normal1, normalB)) return;
                            }

                            if (TryClipSegments(b1v, b2v, p1, p2, normalB, radiusB, 0f, out Vec2 a0, out Vec2 a1Pt, out float s0, out float s1))
                            {
                                manifold.Type = ManifoldType.FaceA;
                                manifold.LocalNormal = -normalB;
                                manifold.LocalPoint = 0.5f * (a0 + a1Pt);
                                manifold.Points[0] = new ManifoldPoint(
                                    Transform.MulT(xfB, Transform.Mul(xfA, a0)),
                                    0f, 0f,
                                    new ContactFeature((byte)ib1, 1, 0, 0));
                                manifold.Points[1] = new ManifoldPoint(
                                    Transform.MulT(xfB, Transform.Mul(xfA, a1Pt)),
                                    0f, 0f,
                                    new ContactFeature((byte)ib2, 0, 0, 0));
                                manifold.PointCount = 2;
                            }
                            return;
                        }
                        // Snap.
                        incidentNormal = ib;
                    }
                    else
                    {
                        // Edge on A, vertex on B — pick the one farther from segment plane.
                        float dot1 = Vec2.Dot(normal1, vertices[ib1] - p1);
                        float dot2 = Vec2.Dot(normal1, vertices[ib2] - p2);
                        incidentIndex = dot1 < dot2 ? ib1 : ib2;
                    }
                }
            }
            else
            {
                // SAT along the segment normal and ghost normals.
                float edgeSeparation = float.MaxValue;
                for (int i = 0; i < count; ++i)
                {
                    float s = Vec2.Dot(normal1, vertices[i] - p1);
                    if (s < edgeSeparation)
                    {
                        edgeSeparation = s;
                        incidentIndex = i;
                    }
                }

                if (smoothParams.Convex1)
                {
                    float s0Sep = float.MaxValue;
                    for (int i = 0; i < count; ++i)
                    {
                        float s = Vec2.Dot(smoothParams.Normal0, vertices[i] - p1);
                        if (s < s0Sep) s0Sep = s;
                    }
                    if (s0Sep > edgeSeparation)
                    {
                        edgeSeparation = s0Sep;
                        incidentIndex = -1;
                    }
                }
                if (smoothParams.Convex2)
                {
                    float s2Sep = float.MaxValue;
                    for (int i = 0; i < count; ++i)
                    {
                        float s = Vec2.Dot(smoothParams.Normal2, vertices[i] - p2);
                        if (s < s2Sep) s2Sep = s;
                    }
                    if (s2Sep > edgeSeparation)
                    {
                        edgeSeparation = s2Sep;
                        incidentIndex = -1;
                    }
                }

                // SAT polygon normals (admit only).
                float polygonSeparation = -float.MaxValue;
                int referenceIndex = -1;
                for (int i = 0; i < count; ++i)
                {
                    Vec2 n = normals[i];
                    if (ClassifyChainNormal(in smoothParams, -n) != ChainNormalType.Admit) continue;
                    Vec2 pv = vertices[i];
                    float s = MathF.Min(Vec2.Dot(n, p2 - pv), Vec2.Dot(n, p1 - pv));
                    if (s > polygonSeparation)
                    {
                        polygonSeparation = s;
                        referenceIndex = i;
                    }
                }

                if (polygonSeparation > edgeSeparation)
                {
                    int ia1 = referenceIndex;
                    int ia2 = ia1 < count - 1 ? ia1 + 1 : 0;
                    Vec2 a1 = vertices[ia1];
                    Vec2 a2 = vertices[ia2];
                    Vec2 n = normals[ia1];

                    float dot1 = Vec2.Dot(n, p1 - a1);
                    float dot2 = Vec2.Dot(n, p2 - a1);
                    if (dot1 < dot2)
                    {
                        if (Vec2.Dot(n0, n) < Vec2.Dot(normal1, n)) return;
                    }
                    else
                    {
                        if (Vec2.Dot(n2, n) < Vec2.Dot(normal1, n)) return;
                    }

                    if (TryClipSegments(a1, a2, p1, p2, n, radiusB, 0f, out Vec2 ap0, out Vec2 ap1, out float _, out float _))
                    {
                        manifold.Type = ManifoldType.FaceA;
                        manifold.LocalNormal = -n;
                        manifold.LocalPoint = 0.5f * (ap0 + ap1);
                        manifold.Points[0] = new ManifoldPoint(
                            Transform.MulT(xfB, Transform.Mul(xfA, ap0)),
                            0f, 0f,
                            new ContactFeature((byte)ia1, 1, 0, 0));
                        manifold.Points[1] = new ManifoldPoint(
                            Transform.MulT(xfB, Transform.Mul(xfA, ap1)),
                            0f, 0f,
                            new ContactFeature((byte)ia2, 0, 0, 0));
                        manifold.PointCount = 2;
                    }
                    return;
                }

                if (incidentIndex == -1)
                {
                    // Ghost edge owns the separating axis.
                    return;
                }
                // Fall through to segment-normal axis.
            }

            // Segment normal axis: clip the polygon edge incident to `incidentIndex`
            // against the chain segment.
            int ib1f, ib2f;
            Vec2 b1f, b2f;
            if (incidentNormal != -1)
            {
                ib1f = incidentNormal;
                ib2f = ib1f < count - 1 ? ib1f + 1 : 0;
            }
            else
            {
                int i2 = incidentIndex;
                int i1 = i2 > 0 ? i2 - 1 : count - 1;
                float d1 = Vec2.Dot(normal1, normals[i1]);
                float d2 = Vec2.Dot(normal1, normals[i2]);
                if (d1 < d2)
                {
                    ib1f = i1; ib2f = i2;
                }
                else
                {
                    ib1f = i2; ib2f = i2 < count - 1 ? i2 + 1 : 0;
                }
            }
            b1f = vertices[ib1f];
            b2f = vertices[ib2f];

            if (TryClipSegments(p1, p2, b1f, b2f, normal1, 0f, radiusB, out Vec2 cp0, out Vec2 cp1, out float _, out float _))
            {
                manifold.Type = ManifoldType.FaceA;
                manifold.LocalNormal = normal1;
                manifold.LocalPoint = 0.5f * (cp0 + cp1);
                manifold.Points[0] = new ManifoldPoint(
                    Transform.MulT(xfB, Transform.Mul(xfA, cp0)),
                    0f, 0f,
                    new ContactFeature(0, (byte)ib2f, 0, 0));
                manifold.Points[1] = new ManifoldPoint(
                    Transform.MulT(xfB, Transform.Mul(xfA, cp1)),
                    0f, 0f,
                    new ContactFeature(1, (byte)ib1f, 0, 0));
                manifold.PointCount = 2;
            }
        }

        // Clip segment [a1, a2] against segment [b1, b2] along the given normal.
        // Returns the two contact points in A-local frame and their separations.
        // Mirror of b2ClipSegments in box2d-cpp/src/manifold.c:1184.
        private static bool TryClipSegments(Vec2 a1, Vec2 a2, Vec2 b1, Vec2 b2, Vec2 normal, float ra, float rb,
            out Vec2 outLower, out Vec2 outUpper, out float sepLower, out float sepUpper)
        {
            outLower = Vec2.Zero;
            outUpper = Vec2.Zero;
            sepLower = 0f;
            sepUpper = 0f;

            Vec2 tangent = MathFng.LeftPerp(normal);

            float lower1 = 0f;
            float upper1 = Vec2.Dot(a2 - a1, tangent);
            float upper2 = Vec2.Dot(b1 - a1, tangent);
            float lower2 = Vec2.Dot(b2 - a1, tangent);

            if (upper2 < lower1 || upper1 < lower2)
            {
                return false;
            }

            Vec2 vLower;
            float denom = upper2 - lower2;
            if (lower2 < lower1 && denom > Constants.Epsilon)
            {
                float t = (lower1 - lower2) / denom;
                vLower = b2 + t * (b1 - b2);
            }
            else
            {
                vLower = b2;
            }

            Vec2 vUpper;
            if (upper2 > upper1 && denom > Constants.Epsilon)
            {
                float t = (upper1 - lower2) / denom;
                vUpper = b2 + t * (b1 - b2);
            }
            else
            {
                vUpper = b1;
            }

            sepLower = Vec2.Dot(vLower - a1, normal);
            sepUpper = Vec2.Dot(vUpper - a1, normal);

            // Place contact points at midpoint accounting for radii.
            vLower = vLower + 0.5f * (ra - rb - sepLower) * normal;
            vUpper = vUpper + 0.5f * (ra - rb - sepUpper) * normal;

            outLower = vLower;
            outUpper = vUpper;
            return true;
        }

        private struct ChainSegmentParams
        {
            public Vec2 Edge1;
            public Vec2 Normal0;
            public Vec2 Normal2;
            public bool Convex1;
            public bool Convex2;
        }

        private enum ChainNormalType
        {
            Skip,
            Admit,
            Snap
        }

        // Gauss-map classifier — mirror of b2ClassifyNormal in box2d-cpp/src/manifold.c:1279.
        private static ChainNormalType ClassifyChainNormal(in ChainSegmentParams p, Vec2 normal)
        {
            const float sinTol = 0.01f;
            if (Vec2.Dot(normal, p.Edge1) <= 0f)
            {
                // Tail-side
                if (p.Convex1)
                {
                    if (Vec2.Cross(normal, p.Normal0) > sinTol) return ChainNormalType.Skip;
                    return ChainNormalType.Admit;
                }
                return ChainNormalType.Snap;
            }
            else
            {
                // Head-side
                if (p.Convex2)
                {
                    if (Vec2.Cross(p.Normal2, normal) > sinTol) return ChainNormalType.Skip;
                    return ChainNormalType.Admit;
                }
                return ChainNormalType.Snap;
            }
        }

        // Build a 2-vertex capsule polygon — matches b2MakeCapsule in cpp.
        private static Polygon MakeCapsulePolygon(Vec2 p1, Vec2 p2, float radius)
        {
            Vec2 d = p2 - p1;
            Vec2 axis = d.LengthSquared > Constants.Epsilon * Constants.Epsilon ? d.Normalize() : new Vec2(1f, 0f);
            Vec2 normal = MathFng.RightPerp(axis);
            Vec2[] verts = new[] { p1, p2 };
            Vec2[] norms = new[] { normal, -normal };
            Vec2 centroid = 0.5f * (p1 + p2);
            return new Polygon(verts, norms, centroid, radius, 2);
        }

        public static bool CollideDistance(Manifold manifold, ShapeProxy proxyA, Transform xfA, ShapeProxy proxyB, Transform xfB)
        {
            return BuildDistanceManifold(manifold, proxyA, xfA, proxyB, xfB);
        }

        public static bool CollideChainDistanceOneSided(Manifold manifold, ChainSegment chain, Transform xfA, ShapeProxy proxyB, Transform xfB)
        {
            ShapeProxy proxyA = ShapeProxyFactory.FromSegment(chain.Segment);
            return TryBuildDistanceManifoldOneSided(manifold, chain, xfA, proxyA, proxyB, xfB);
        }

        private static float EdgeSeparation(Polygon poly1, Transform xf1, int edge1, Polygon poly2, Transform xf2)
        {
            Vec2 normal = poly1.Normals[edge1];

            int count2 = poly2.Count;
            Vec2 normalWorld = Rot.Mul(xf1.Q, normal);
            Vec2 normal2 = Rot.MulT(xf2.Q, normalWorld);

            int index = 0;
            float minDot = float.MaxValue;
            for (int i = 0; i < count2; ++i)
            {
                float dot = Vec2.Dot(poly2.Vertices[i], normal2);
                if (dot < minDot)
                {
                    minDot = dot;
                    index = i;
                }
            }

            Vec2 v1 = Transform.Mul(xf1, poly1.Vertices[edge1]);
            Vec2 v2 = Transform.Mul(xf2, poly2.Vertices[index]);
            Vec2 separationVec = v2 - v1;
            return Vec2.Dot(separationVec, normalWorld);
        }

        private static float FindMaxSeparation(out int edgeIndex, Polygon poly1, Transform xf1, Polygon poly2, Transform xf2)
        {
            int count1 = poly1.Count;
            Vec2 c1 = Transform.Mul(xf1, poly1.Centroid);
            Vec2 c2 = Transform.Mul(xf2, poly2.Centroid);
            Vec2 d = c2 - c1;
            Vec2 dLocal1 = Rot.MulT(xf1.Q, d);

            int edge = 0;
            float maxDot = -float.MaxValue;
            for (int i = 0; i < count1; ++i)
            {
                float dot = Vec2.Dot(poly1.Normals[i], dLocal1);
                if (dot > maxDot)
                {
                    maxDot = dot;
                    edge = i;
                }
            }

            float s = EdgeSeparation(poly1, xf1, edge, poly2, xf2);
            int prevEdge = edge - 1 >= 0 ? edge - 1 : count1 - 1;
            float sPrev = EdgeSeparation(poly1, xf1, prevEdge, poly2, xf2);
            int nextEdge = edge + 1 < count1 ? edge + 1 : 0;
            float sNext = EdgeSeparation(poly1, xf1, nextEdge, poly2, xf2);

            int bestEdge;
            float bestSeparation;
            int increment;
            if (sPrev > s && sPrev > sNext)
            {
                increment = -1;
                bestEdge = prevEdge;
                bestSeparation = sPrev;
            }
            else if (sNext > s)
            {
                increment = 1;
                bestEdge = nextEdge;
                bestSeparation = sNext;
            }
            else
            {
                edgeIndex = edge;
                return s;
            }

            for (;;)
            {
                edge = increment == -1
                    ? (bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1)
                    : (bestEdge + 1 < count1 ? bestEdge + 1 : 0);

                s = EdgeSeparation(poly1, xf1, edge, poly2, xf2);
                if (s > bestSeparation)
                {
                    bestEdge = edge;
                    bestSeparation = s;
                }
                else
                {
                    break;
                }
            }

            edgeIndex = bestEdge;
            return bestSeparation;
        }

        private static void FindIncidentEdge(ClipVertex[] c, Polygon poly1, Transform xf1, int edge1, Polygon poly2, Transform xf2)
        {
            Vec2 edge = poly1.Normals[edge1];

            int count2 = poly2.Count;
            Vec2[] vertices2 = poly2.Vertices;
            Vec2[] normals2 = poly2.Normals;

            Vec2 normal1World = Rot.Mul(xf1.Q, edge);
            Vec2 normal1 = Rot.MulT(xf2.Q, normal1World);

            int index = 0;
            float minDot = float.MaxValue;
            for (int i = 0; i < count2; ++i)
            {
                float dot = Vec2.Dot(normal1, normals2[i]);
                if (dot < minDot)
                {
                    minDot = dot;
                    index = i;
                }
            }

            int i1 = index;
            int i2 = i1 + 1 < count2 ? i1 + 1 : 0;

            Vec2 v1 = Transform.Mul(xf2, vertices2[i1]);
            Vec2 v2 = Transform.Mul(xf2, vertices2[i2]);

            c[0] = new ClipVertex(v1, new ContactFeature((byte)ContactFeatureType.Face, (byte)ContactFeatureType.Vertex, (byte)edge1, (byte)i1));
            c[1] = new ClipVertex(v2, new ContactFeature((byte)ContactFeatureType.Face, (byte)ContactFeatureType.Vertex, (byte)edge1, (byte)i2));
        }

        private static int ClipSegmentToLine(ClipVertex[] vOut, ClipVertex[] vIn, Vec2 normal, float offset, byte vertexIndexA)
        {
            int numOut = 0;

            Vec2 v0 = vIn[0].V;
            Vec2 v1 = vIn[1].V;

            float distance0 = Vec2.Dot(normal, v0) - offset;
            float distance1 = Vec2.Dot(normal, v1) - offset;

            if (distance0 <= 0f)
            {
                vOut[numOut++] = vIn[0];
            }
            if (distance1 <= 0f)
            {
                vOut[numOut++] = vIn[1];
            }

            if (distance0 * distance1 < 0f)
            {
                float interp = distance0 / (distance0 - distance1);
                Vec2 v = v0 + interp * (v1 - v0);
                ContactFeature id = new ContactFeature((byte)ContactFeatureType.Vertex, (byte)ContactFeatureType.Face, vertexIndexA, vIn[0].Id.IndexB);
                vOut[numOut++] = new ClipVertex(v, id);
            }

            return numOut;
        }

        private static bool BuildDistanceManifold(Manifold manifold, ShapeProxy proxyA, Transform xfA, ShapeProxy proxyB, Transform xfB)
        {
            manifold.PointCount = 0;

            DistanceInput input = new DistanceInput(proxyA, proxyB, xfA, xfB, true);
            SimplexCache cache = new SimplexCache(0f, 0, 0, 0, 0, 0, 0, 0);
            DistanceOutput output = Distance.Compute(input, ref cache);

            if (output.Distance > 0f)
            {
                return false;
            }

            Vec2 normal = output.PointB - output.PointA;
            if (normal.LengthSquared > 1e-12f)
            {
                normal = normal.Normalize();
            }
            else
            {
                normal = Rot.Mul(xfA.Q, new Vec2(1f, 0f));
            }

            manifold.Type = ManifoldType.FaceA;
            manifold.LocalNormal = Rot.MulT(xfA.Q, normal);
            manifold.LocalPoint = Transform.MulT(xfA, output.PointA);
            manifold.Points[0] = new ManifoldPoint(Transform.MulT(xfB, output.PointB), 0f, 0f, new ContactFeature(0, 0, 0, 0));
            manifold.PointCount = 1;
            return true;
        }

        private static bool TryBuildDistanceManifoldOneSided(Manifold manifold, ChainSegment chain, Transform xfA, ShapeProxy proxyA, ShapeProxy proxyB, Transform xfB)
        {
            DistanceInput input = new DistanceInput(proxyA, proxyB, xfA, xfB, true);
            SimplexCache cache = new SimplexCache(0f, 0, 0, 0, 0, 0, 0, 0);
            DistanceOutput output = Distance.Compute(input, ref cache);

            if (output.Distance > 0f)
            {
                return false;
            }

            Vec2 p1 = Transform.Mul(xfA, chain.Segment.Point1);
            Vec2 p2 = Transform.Mul(xfA, chain.Segment.Point2);
            Vec2 pointB = output.PointB;

            Vec2 edge1 = p2 - p1;
            if (edge1.LengthSquared <= Constants.Epsilon * Constants.Epsilon)
            {
                return false;
            }

            Vec2 edge1Dir = edge1.Normalize();
            Vec2 normal1 = MathFng.RightPerp(edge1Dir);

            const float convexTol = 0.01f;
            bool behind0 = true;
            bool behind2 = true;

            Vec2 ghost1 = Transform.Mul(xfA, chain.Ghost1);
            Vec2 edge0 = p1 - ghost1;
            if (edge0.LengthSquared > Constants.Epsilon * Constants.Epsilon)
            {
                Vec2 edge0Dir = edge0.Normalize();
                bool convex1 = Vec2.Cross(edge0Dir, edge1Dir) >= convexTol;
                if (convex1)
                {
                    Vec2 normal0 = MathFng.RightPerp(edge0Dir);
                    behind0 = Vec2.Dot(normal0, pointB - p1) < 0f;
                }
            }

            Vec2 ghost2 = Transform.Mul(xfA, chain.Ghost2);
            Vec2 edge2 = ghost2 - p2;
            if (edge2.LengthSquared > Constants.Epsilon * Constants.Epsilon)
            {
                Vec2 edge2Dir = edge2.Normalize();
                bool convex2 = Vec2.Cross(edge1Dir, edge2Dir) >= convexTol;
                if (convex2)
                {
                    Vec2 normal2 = MathFng.RightPerp(edge2Dir);
                    behind2 = Vec2.Dot(normal2, pointB - p2) < 0f;
                }
            }

            bool behind1 = Vec2.Dot(normal1, pointB - p1) < 0f;
            if (behind1 && behind0 && behind2)
            {
                return false;
            }

            Vec2 normal = output.PointB - output.PointA;
            if (normal.LengthSquared > 1e-12f)
            {
                normal = normal.Normalize();
            }
            else
            {
                normal = Rot.Mul(xfA.Q, new Vec2(1f, 0f));
            }

            manifold.Type = ManifoldType.FaceA;
            manifold.LocalNormal = Rot.MulT(xfA.Q, normal);
            manifold.LocalPoint = Transform.MulT(xfA, output.PointA);
            manifold.Points[0] = new ManifoldPoint(Transform.MulT(xfB, output.PointB), 0f, 0f, new ContactFeature(0, 0, 0, 0));
            manifold.PointCount = 1;
            return true;
        }

        private static Polygon BuildSegmentPolygon(Segment segment, float radius)
        {
            Vec2 d = segment.Point2 - segment.Point1;
            if (d.LengthSquared <= Constants.Epsilon * Constants.Epsilon)
            {
                Vec2 r = new Vec2(radius, radius);
                Vec2[] verts =
                {
                    segment.Point1 + new Vec2(-r.X, -r.Y),
                    segment.Point1 + new Vec2(r.X, -r.Y),
                    segment.Point1 + new Vec2(r.X, r.Y),
                    segment.Point1 + new Vec2(-r.X, r.Y)
                };
                return ShapeGeometry.ToPolygon(new PolygonShape(verts), radiusOverride: 0f);
            }

            Vec2 dir = d.Normalize();
            Vec2 normal = MathFng.RightPerp(dir);
            Vec2 rN = radius * normal;

            Vec2[] vertices =
            {
                segment.Point1 + rN,
                segment.Point2 + rN,
                segment.Point2 - rN,
                segment.Point1 - rN
            };
            return ShapeGeometry.ToPolygon(new PolygonShape(vertices), radiusOverride: 0f);
        }

        private readonly struct ClipVertex
        {
            public readonly Vec2 V;
            public readonly ContactFeature Id;

            public ClipVertex(Vec2 v, ContactFeature id)
            {
                V = v;
                Id = id;
            }
        }

        private enum ContactFeatureType : byte
        {
            Vertex = 0,
            Face = 1
        }
    }
}
