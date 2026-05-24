using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;

namespace Box2DNG.Tests
{
    [TestClass]
    public class MathPrimitiveTests
    {
        [TestMethod]
        public void Mat22_GetInverse_RecoversIdentity()
        {
            Mat22 m = new Mat22(new Vec2(2f, 1f), new Vec2(1f, 3f));
            Mat22 inv = m.GetInverse();

            Vec2 c0 = Mat22.Mul(m, inv.Ex);
            Vec2 c1 = Mat22.Mul(m, inv.Ey);

            Assert.AreEqual(1f, c0.X, 1e-5f);
            Assert.AreEqual(0f, c0.Y, 1e-5f);
            Assert.AreEqual(0f, c1.X, 1e-5f);
            Assert.AreEqual(1f, c1.Y, 1e-5f);
        }

        [TestMethod]
        public void Mat22_GetInverse_OfSingularMatrixReturnsZero()
        {
            Mat22 m = new Mat22(new Vec2(1f, 2f), new Vec2(2f, 4f));
            Mat22 inv = m.GetInverse();
            Assert.AreEqual(0f, inv.Ex.X);
            Assert.AreEqual(0f, inv.Ex.Y);
            Assert.AreEqual(0f, inv.Ey.X);
            Assert.AreEqual(0f, inv.Ey.Y);
        }

        [TestMethod]
        public void Mat22_Solve_RecoversBVector()
        {
            Mat22 m = new Mat22(new Vec2(2f, 1f), new Vec2(1f, 3f));
            Vec2 b = new Vec2(5f, 7f);
            Vec2 x = m.Solve(b);
            Vec2 reconstructed = Mat22.Mul(m, x);
            Assert.AreEqual(b.X, reconstructed.X, 1e-4f);
            Assert.AreEqual(b.Y, reconstructed.Y, 1e-4f);
        }

        [TestMethod]
        public void Mat33_Solve33_RecoversBVector()
        {
            Mat33 m = new Mat33(
                new Vec3(2f, 0f, 0f),
                new Vec3(1f, 3f, 0f),
                new Vec3(0f, 1f, 4f));
            Vec3 b = new Vec3(2f, 4f, 8f);
            Vec3 x = m.Solve33(b);
            Vec3 reconstructed = Mat33.Mul(m, x);
            Assert.AreEqual(b.X, reconstructed.X, 1e-4f);
            Assert.AreEqual(b.Y, reconstructed.Y, 1e-4f);
            Assert.AreEqual(b.Z, reconstructed.Z, 1e-4f);
        }

        [TestMethod]
        public void Mat33_Solve22_OperatesOnTopLeftBlock()
        {
            Mat33 m = new Mat33(
                new Vec3(2f, 1f, 99f),
                new Vec3(1f, 3f, 99f),
                new Vec3(99f, 99f, 99f));
            Vec2 b = new Vec2(5f, 7f);
            Vec2 x = m.Solve22(b);

            float r0 = m.Ex.X * x.X + m.Ey.X * x.Y;
            float r1 = m.Ex.Y * x.X + m.Ey.Y * x.Y;
            Assert.AreEqual(b.X, r0, 1e-4f);
            Assert.AreEqual(b.Y, r1, 1e-4f);
        }

        [TestMethod]
        public void Aabb_Union_TakesMinLowerAndMaxUpper()
        {
            Aabb a = new Aabb(new Vec2(-1f, 0f), new Vec2(2f, 3f));
            Aabb b = new Aabb(new Vec2(0f, -2f), new Vec2(4f, 1f));
            Aabb u = Aabb.Union(a, b);
            Assert.AreEqual(-1f, u.LowerBound.X);
            Assert.AreEqual(-2f, u.LowerBound.Y);
            Assert.AreEqual(4f, u.UpperBound.X);
            Assert.AreEqual(3f, u.UpperBound.Y);
        }

        [TestMethod]
        public void Aabb_Overlaps_TrueWhenIntersecting_FalseWhenSeparated()
        {
            Aabb a = new Aabb(new Vec2(0f, 0f), new Vec2(2f, 2f));
            Aabb touching = new Aabb(new Vec2(1f, 1f), new Vec2(3f, 3f));
            Aabb gap = new Aabb(new Vec2(3f, 3f), new Vec2(5f, 5f));
            Assert.IsTrue(Aabb.Overlaps(a, touching));
            Assert.IsFalse(Aabb.Overlaps(a, gap));
        }

        [TestMethod]
        public void MathFng_ComputeCosSin_AccuracyWithinBhaskaraBound()
        {
            float[] angles = { 0f, 0.5f, 1.0f, 1.5707f, -0.7f, 2.5f, -2.5f, 3.0f, -3.0f, 6.28f };
            foreach (float a in angles)
            {
                CosSin cs = MathFng.ComputeCosSin(a);
                float refCos = MathF.Cos(a);
                float refSin = MathF.Sin(a);
                Assert.AreEqual(refCos, cs.Cosine, 2.5e-3f, $"cos at {a}");
                Assert.AreEqual(refSin, cs.Sine, 2.5e-3f, $"sin at {a}");
            }
        }

        [TestMethod]
        public void MathFng_ComputeCosSin_IsDeterministic()
        {
            for (int i = 0; i < 50; ++i)
            {
                float a = i * 0.137f - 7f;
                CosSin first = MathFng.ComputeCosSin(a);
                CosSin second = MathFng.ComputeCosSin(a);
                Assert.AreEqual(first.Cosine, second.Cosine);
                Assert.AreEqual(first.Sine, second.Sine);
            }
        }

        [TestMethod]
        public void MathFng_UnwindAngle_ReducesToRange()
        {
            float[] angles = { 7f, -7f, 100f, -100f, MathF.PI * 4f, MathF.PI * -4f };
            foreach (float a in angles)
            {
                float u = MathFng.UnwindAngle(a);
                Assert.IsTrue(u <= MathF.PI + 1e-4f && u >= -MathF.PI - 1e-4f, $"unwind({a}) = {u}");
            }
        }
    }
}
