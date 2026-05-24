using System;

namespace Box2DNG
{
    public readonly struct SimplexCache
    {
        public readonly float Metric;
        public readonly int Count;
        public readonly byte IndexA0;
        public readonly byte IndexA1;
        public readonly byte IndexA2;
        public readonly byte IndexB0;
        public readonly byte IndexB1;
        public readonly byte IndexB2;

        public SimplexCache(float metric, int count, byte indexA0, byte indexA1, byte indexA2, byte indexB0, byte indexB1, byte indexB2)
        {
            Metric = metric;
            Count = count;
            IndexA0 = indexA0;
            IndexA1 = indexA1;
            IndexA2 = indexA2;
            IndexB0 = indexB0;
            IndexB1 = indexB1;
            IndexB2 = indexB2;
        }
    }

    public readonly struct DistanceInput
    {
        public readonly ShapeProxy ProxyA;
        public readonly ShapeProxy ProxyB;
        public readonly Transform TransformA;
        public readonly Transform TransformB;
        public readonly bool UseRadii;

        public DistanceInput(ShapeProxy proxyA, ShapeProxy proxyB, Transform transformA, Transform transformB, bool useRadii)
        {
            ProxyA = proxyA;
            ProxyB = proxyB;
            TransformA = transformA;
            TransformB = transformB;
            UseRadii = useRadii;
        }
    }

    public readonly struct DistanceOutput
    {
        public readonly Vec2 PointA;
        public readonly Vec2 PointB;
        public readonly float Distance;
        public readonly int Iterations;

        public DistanceOutput(Vec2 pointA, Vec2 pointB, float distance, int iterations)
        {
            PointA = pointA;
            PointB = pointB;
            Distance = distance;
            Iterations = iterations;
        }
    }

    public static class Distance
    {
        private const int MaxIters = 20;

        private sealed class DistanceProxy
        {
            public Vec2[] Points = Array.Empty<Vec2>();
            public int Count;
            public float Radius;

            public void Set(ShapeProxy proxy)
            {
                Points = proxy.Points;
                Count = proxy.Count;
                Radius = proxy.Radius;
            }

            public int GetSupport(Vec2 d)
            {
                int bestIndex = 0;
                float bestValue = Vec2.Dot(Points[0], d);
                for (int i = 1; i < Count; ++i)
                {
                    float value = Vec2.Dot(Points[i], d);
                    if (value > bestValue)
                    {
                        bestIndex = i;
                        bestValue = value;
                    }
                }
                return bestIndex;
            }

            public Vec2 GetVertex(int index) => Points[index];
        }

        private struct SimplexVertex
        {
            public Vec2 WA;
            public Vec2 WB;
            public Vec2 W;
            public float A;
            public int IndexA;
            public int IndexB;
        }

        private sealed class Simplex
        {
            public SimplexVertex[] V = { new SimplexVertex(), new SimplexVertex(), new SimplexVertex() };
            public int Count;

            public void ReadCache(SimplexCache cache, DistanceProxy proxyA, Transform transformA, DistanceProxy proxyB, Transform transformB)
            {
                Count = cache.Count;

                if (Count > 0)
                {
                    V[0].IndexA = cache.IndexA0;
                    V[0].IndexB = cache.IndexB0;
                    V[1].IndexA = cache.IndexA1;
                    V[1].IndexB = cache.IndexB1;
                    V[2].IndexA = cache.IndexA2;
                    V[2].IndexB = cache.IndexB2;

                    for (int i = 0; i < Count; ++i)
                    {
                        SimplexVertex v = V[i];
                        Vec2 wA = Transform.Mul(transformA, proxyA.GetVertex(v.IndexA));
                        Vec2 wB = Transform.Mul(transformB, proxyB.GetVertex(v.IndexB));
                        v.WA = wA;
                        v.WB = wB;
                        v.W = wB - wA;
                        v.A = 0f;
                        V[i] = v;
                    }

                    if (Count == 1)
                    {
                        V[0].A = 1f;
                    }
                }
                else
                {
                    Count = 1;
                    SimplexVertex v = V[0];
                    v.IndexA = 0;
                    v.IndexB = 0;
                    Vec2 wA = Transform.Mul(transformA, proxyA.GetVertex(0));
                    Vec2 wB = Transform.Mul(transformB, proxyB.GetVertex(0));
                    v.WA = wA;
                    v.WB = wB;
                    v.W = wB - wA;
                    v.A = 1f;
                    V[0] = v;
                }
            }

            public Vec2 GetClosestPoint()
            {
                switch (Count)
                {
                    case 1:
                        return V[0].W;
                    case 2:
                        return V[0].A * V[0].W + V[1].A * V[1].W;
                    case 3:
                        return Vec2.Zero;
                    default:
                        return Vec2.Zero;
                }
            }

            public void GetWitnessPoints(out Vec2 a, out Vec2 b)
            {
                switch (Count)
                {
                    case 1:
                        a = V[0].WA;
                        b = V[0].WB;
                        break;
                    case 2:
                        a = V[0].A * V[0].WA + V[1].A * V[1].WA;
                        b = V[0].A * V[0].WB + V[1].A * V[1].WB;
                        break;
                    case 3:
                        a = V[0].A * V[0].WA + V[1].A * V[1].WA + V[2].A * V[2].WA;
                        b = a;
                        break;
                    default:
                        a = Vec2.Zero;
                        b = Vec2.Zero;
                        break;
                }
            }

            public void Solve2()
            {
                Vec2 w1 = V[0].W;
                Vec2 w2 = V[1].W;

                Vec2 e12 = w2 - w1;

                float d12_2 = -Vec2.Dot(w1, e12);
                if (d12_2 <= 0.0f)
                {
                    V[0].A = 1.0f;
                    Count = 1;
                    return;
                }

                float d12_1 = Vec2.Dot(w2, e12);
                if (d12_1 <= 0.0f)
                {
                    V[1].A = 1.0f;
                    V[0] = V[1];
                    Count = 1;
                    return;
                }

                float inv = 1.0f / (d12_1 + d12_2);
                V[0].A = d12_1 * inv;
                V[1].A = d12_2 * inv;
                Count = 2;
            }

            public void Solve3()
            {
                Vec2 w1 = V[0].W;
                Vec2 w2 = V[1].W;
                Vec2 w3 = V[2].W;

                Vec2 e12 = w2 - w1;
                Vec2 e13 = w3 - w1;
                Vec2 e23 = w3 - w2;

                float d12_1 = Vec2.Dot(w2, e12);
                float d12_2 = -Vec2.Dot(w1, e12);

                float d13_1 = Vec2.Dot(w3, e13);
                float d13_2 = -Vec2.Dot(w1, e13);

                float d23_1 = Vec2.Dot(w3, e23);
                float d23_2 = -Vec2.Dot(w2, e23);

                float n123 = Vec2.Cross(e12, e13);

                float d123_1 = n123 * Vec2.Cross(w2, w3);
                float d123_2 = n123 * Vec2.Cross(w3, w1);
                float d123_3 = n123 * Vec2.Cross(w1, w2);

                if (d12_2 <= 0.0f && d13_2 <= 0.0f)
                {
                    V[0].A = 1.0f;
                    Count = 1;
                    return;
                }

                if (d12_1 > 0.0f && d12_2 > 0.0f && d123_3 <= 0.0f)
                {
                    float inv = 1.0f / (d12_1 + d12_2);
                    V[0].A = d12_1 * inv;
                    V[1].A = d12_2 * inv;
                    Count = 2;
                    return;
                }

                if (d13_1 > 0.0f && d13_2 > 0.0f && d123_2 <= 0.0f)
                {
                    float inv = 1.0f / (d13_1 + d13_2);
                    V[0].A = d13_1 * inv;
                    V[2].A = d13_2 * inv;
                    V[1] = V[2];
                    Count = 2;
                    return;
                }

                if (d12_1 <= 0.0f && d23_2 <= 0.0f)
                {
                    V[1].A = 1.0f;
                    V[0] = V[1];
                    Count = 1;
                    return;
                }

                if (d13_1 <= 0.0f && d23_1 <= 0.0f)
                {
                    V[2].A = 1.0f;
                    V[0] = V[2];
                    Count = 1;
                    return;
                }

                if (d23_1 > 0.0f && d23_2 > 0.0f && d123_1 <= 0.0f)
                {
                    float inv = 1.0f / (d23_1 + d23_2);
                    V[1].A = d23_1 * inv;
                    V[2].A = d23_2 * inv;
                    V[0] = V[1];
                    Count = 2;
                    return;
                }

                float inv123 = 1.0f / (d123_1 + d123_2 + d123_3);
                V[0].A = d123_1 * inv123;
                V[1].A = d123_2 * inv123;
                V[2].A = d123_3 * inv123;
                Count = 3;
            }

            public float GetMetric()
            {
                switch (Count)
                {
                    case 1:
                        return 0f;
                    case 2:
                        return (V[0].W - V[1].W).Length;
                    case 3:
                        return MathF.Abs(Vec2.Cross(V[1].W - V[0].W, V[2].W - V[0].W));
                    default:
                        return 0f;
                }
            }
        }

        public static DistanceOutput Compute(DistanceInput input, SimplexCache? cache = null)
        {
            SimplexCache localCache = cache ?? new SimplexCache(0f, 0, 0, 0, 0, 0, 0, 0);
            return Compute(input, ref localCache);
        }

        public static DistanceOutput Compute(DistanceInput input, ref SimplexCache cache)
        {
            DistanceProxy proxyA = new DistanceProxy();
            DistanceProxy proxyB = new DistanceProxy();
            proxyA.Set(input.ProxyA);
            proxyB.Set(input.ProxyB);

            Transform transformA = input.TransformA;
            Transform transformB = input.TransformB;

            Simplex simplex = new Simplex();
            simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB);

            Vec2 d = simplex.GetClosestPoint();
            float distanceSqr1 = d.LengthSquared;
            float distanceSqr2 = distanceSqr1;

            int iter = 0;
            int[] saveA = new int[3];
            int[] saveB = new int[3];

            while (iter < MaxIters)
            {
                int saveCount = simplex.Count;
                for (int i = 0; i < saveCount; ++i)
                {
                    saveA[i] = simplex.V[i].IndexA;
                    saveB[i] = simplex.V[i].IndexB;
                }

                if (simplex.Count == 2)
                {
                    simplex.Solve2();
                }
                else if (simplex.Count == 3)
                {
                    simplex.Solve3();
                }

                if (simplex.Count == 3)
                {
                    break;
                }

                Vec2 p = simplex.GetClosestPoint();
                distanceSqr2 = p.LengthSquared;

                if (distanceSqr2 >= distanceSqr1)
                {
                    // no progress
                }
                distanceSqr1 = distanceSqr2;

                d = p;
                if (d.LengthSquared < 1e-12f)
                {
                    break;
                }

                // The simplex stores W = wB - wA. To pull wB - wA toward origin
                // we need new wA in +d and new wB in -d. (box2d-cpp uses the
                // opposite convention w = wA - wB and so flips both signs.)
                int indexA = proxyA.GetSupport(Rot.MulT(transformA.Q, d));
                Vec2 wA = Transform.Mul(transformA, proxyA.GetVertex(indexA));
                int indexB = proxyB.GetSupport(Rot.MulT(transformB.Q, -d));
                Vec2 wB = Transform.Mul(transformB, proxyB.GetVertex(indexB));

                SimplexVertex v = new SimplexVertex
                {
                    IndexA = indexA,
                    IndexB = indexB,
                    WA = wA,
                    WB = wB,
                    W = wB - wA,
                    A = 0f
                };

                bool duplicate = false;
                for (int i = 0; i < saveCount; ++i)
                {
                    if (v.IndexA == saveA[i] && v.IndexB == saveB[i])
                    {
                        duplicate = true;
                        break;
                    }
                }

                if (duplicate)
                {
                    break;
                }

                simplex.V[simplex.Count] = v;
                simplex.Count++;
                iter++;
            }

            simplex.GetWitnessPoints(out Vec2 pointA, out Vec2 pointB);
            float distance = (pointB - pointA).Length;

            if (input.UseRadii)
            {
                float rA = proxyA.Radius;
                float rB = proxyB.Radius;
                if (distance > rA + rB && distance > 1e-6f)
                {
                    Vec2 normal = (pointB - pointA).Normalize();
                    pointA += rA * normal;
                    pointB -= rB * normal;
                    distance -= rA + rB;
                }
                else
                {
                    Vec2 p = 0.5f * (pointA + pointB);
                    pointA = p;
                    pointB = p;
                    distance = 0f;
                }
            }

            cache = BuildCache(simplex);

            return new DistanceOutput(pointA, pointB, distance, iter);
        }

        private static SimplexCache BuildCache(Simplex simplex)
        {
            byte indexA0 = 0;
            byte indexA1 = 0;
            byte indexA2 = 0;
            byte indexB0 = 0;
            byte indexB1 = 0;
            byte indexB2 = 0;

            if (simplex.Count > 0)
            {
                indexA0 = (byte)simplex.V[0].IndexA;
                indexB0 = (byte)simplex.V[0].IndexB;
            }
            if (simplex.Count > 1)
            {
                indexA1 = (byte)simplex.V[1].IndexA;
                indexB1 = (byte)simplex.V[1].IndexB;
            }
            if (simplex.Count > 2)
            {
                indexA2 = (byte)simplex.V[2].IndexA;
                indexB2 = (byte)simplex.V[2].IndexB;
            }

            float metric = simplex.GetMetric();
            return new SimplexCache(metric, simplex.Count, indexA0, indexA1, indexA2, indexB0, indexB1, indexB2);
        }
    }
}
