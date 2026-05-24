using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;

namespace Box2DNG.Tests
{
    [TestClass]
    public class CollisionAlgorithmTests
    {
        // ----- DistanceAlgorithms.SegmentDistance -----

        [TestMethod]
        public void SegmentDistance_ParallelSegmentsNonOverlapping()
        {
            // Two horizontal segments, vertically separated by 2.
            Vec2 p1 = new Vec2(0f, 0f), q1 = new Vec2(2f, 0f);
            Vec2 p2 = new Vec2(0f, 2f), q2 = new Vec2(2f, 2f);
            SegmentDistanceResult result = DistanceAlgorithms.SegmentDistance(p1, q1, p2, q2);
            Assert.AreEqual(4f, result.DistanceSquared, 1e-4f);
        }

        [TestMethod]
        public void SegmentDistance_CrossingSegmentsHaveZeroDistance()
        {
            Vec2 p1 = new Vec2(-1f, 0f), q1 = new Vec2(1f, 0f);
            Vec2 p2 = new Vec2(0f, -1f), q2 = new Vec2(0f, 1f);
            SegmentDistanceResult result = DistanceAlgorithms.SegmentDistance(p1, q1, p2, q2);
            Assert.AreEqual(0f, result.DistanceSquared, 1e-4f);
        }

        [TestMethod]
        public void SegmentDistance_DegenerateBothPoints()
        {
            // Both segments are points (zero length).
            Vec2 p1 = new Vec2(0f, 0f), q1 = new Vec2(0f, 0f);
            Vec2 p2 = new Vec2(3f, 4f), q2 = new Vec2(3f, 4f);
            SegmentDistanceResult result = DistanceAlgorithms.SegmentDistance(p1, q1, p2, q2);
            Assert.AreEqual(25f, result.DistanceSquared, 1e-4f); // 3² + 4²
        }

        [TestMethod]
        public void SegmentDistance_DegenerateFirstSegment()
        {
            // First is a point; second is a horizontal segment passing 1 below.
            Vec2 p1 = new Vec2(1f, 0f), q1 = new Vec2(1f, 0f);
            Vec2 p2 = new Vec2(0f, -1f), q2 = new Vec2(2f, -1f);
            SegmentDistanceResult result = DistanceAlgorithms.SegmentDistance(p1, q1, p2, q2);
            Assert.AreEqual(1f, result.DistanceSquared, 1e-4f);
            Assert.AreEqual(1f, result.Closest2.X, 1e-4f);
            Assert.AreEqual(-1f, result.Closest2.Y, 1e-4f);
        }

        [TestMethod]
        public void SegmentDistance_ClampsOutsideRange()
        {
            // Two skew lines, closest points should be at segment endpoints.
            Vec2 p1 = new Vec2(0f, 0f), q1 = new Vec2(1f, 0f);
            Vec2 p2 = new Vec2(2f, 1f), q2 = new Vec2(3f, 1f);
            SegmentDistanceResult result = DistanceAlgorithms.SegmentDistance(p1, q1, p2, q2);
            // closest on seg1 = q1 = (1,0), closest on seg2 = p2 = (2,1), dist² = 1 + 1
            Assert.AreEqual(2f, result.DistanceSquared, 1e-4f);
        }

        // ----- MathFng validity helpers -----

        [TestMethod]
        public void IsValidFloat_AcceptsNormalsRejectsNanAndInf()
        {
            Assert.IsTrue(MathFng.IsValidFloat(0f));
            Assert.IsTrue(MathFng.IsValidFloat(-3.14f));
            Assert.IsTrue(MathFng.IsValidFloat(1e20f));
            Assert.IsFalse(MathFng.IsValidFloat(float.NaN));
            Assert.IsFalse(MathFng.IsValidFloat(float.PositiveInfinity));
            Assert.IsFalse(MathFng.IsValidFloat(float.NegativeInfinity));
        }

        [TestMethod]
        public void IsValidVec2_ChecksBothComponents()
        {
            Assert.IsTrue(MathFng.IsValidVec2(new Vec2(1f, 2f)));
            Assert.IsFalse(MathFng.IsValidVec2(new Vec2(float.NaN, 0f)));
            Assert.IsFalse(MathFng.IsValidVec2(new Vec2(0f, float.PositiveInfinity)));
        }

        [TestMethod]
        public void IsValidRotation_AcceptsUnitAndRejectsNonUnit()
        {
            Assert.IsTrue(MathFng.IsValidRotation(Rot.Identity));
            Assert.IsTrue(MathFng.IsValidRotation(new Rot(0.7f)));
            // Not normalized — magnitude 2² + 0² = 4 → length 2, not 1.
            Assert.IsFalse(MathFng.IsValidRotation(new Rot(2f, 0f)));
            Assert.IsFalse(MathFng.IsValidRotation(new Rot(float.NaN, 1f)));
        }

        [TestMethod]
        public void IsValidTransform_ComposesPositionAndRotationChecks()
        {
            Assert.IsTrue(MathFng.IsValidTransform(Transform.Identity));
            Assert.IsTrue(MathFng.IsValidTransform(new Transform(new Vec2(1f, 2f), new Rot(0.5f))));
            Assert.IsFalse(MathFng.IsValidTransform(new Transform(new Vec2(float.NaN, 0f), Rot.Identity)));
            Assert.IsFalse(MathFng.IsValidTransform(new Transform(Vec2.Zero, new Rot(2f, 0f))));
        }

        [TestMethod]
        public void IsValidAabb_AcceptsOrdered_RejectsInverted()
        {
            Assert.IsTrue(MathFng.IsValidAabb(new Aabb(new Vec2(-1f, -1f), new Vec2(1f, 1f))));
            Assert.IsTrue(MathFng.IsValidAabb(new Aabb(new Vec2(0f, 0f), new Vec2(0f, 0f)))); // zero-area still valid
            Assert.IsFalse(MathFng.IsValidAabb(new Aabb(new Vec2(1f, 0f), new Vec2(0f, 1f)))); // inverted X
            Assert.IsFalse(MathFng.IsValidAabb(new Aabb(new Vec2(0f, 1f), new Vec2(1f, 0f)))); // inverted Y
            Assert.IsFalse(MathFng.IsValidAabb(new Aabb(new Vec2(float.NaN, 0f), new Vec2(1f, 1f))));
        }

        // ----- LeftPerp / RightPerp / Clamp -----

        [TestMethod]
        public void LeftPerp_And_RightPerp_AreOpposites()
        {
            Vec2 v = new Vec2(3f, 4f);
            Vec2 left = MathFng.LeftPerp(v);
            Vec2 right = MathFng.RightPerp(v);
            Assert.AreEqual(-left.X, right.X);
            Assert.AreEqual(-left.Y, right.Y);
            // Both perpendicular to v.
            Assert.AreEqual(0f, Vec2.Dot(v, left), 1e-5f);
            Assert.AreEqual(0f, Vec2.Dot(v, right), 1e-5f);
        }

        [TestMethod]
        public void Clamp_Vec2_ClampsComponentwise()
        {
            Vec2 c = MathFng.Clamp(new Vec2(5f, -3f), new Vec2(0f, 0f), new Vec2(2f, 2f));
            Assert.AreEqual(2f, c.X);
            Assert.AreEqual(0f, c.Y);
        }

        [TestMethod]
        public void MixFrictionAndRestitution()
        {
            Assert.AreEqual(MathF.Sqrt(0.5f * 0.2f), MathFng.MixFriction(0.5f, 0.2f), 1e-5f);
            Assert.AreEqual(0.7f, MathFng.MixRestitution(0.5f, 0.7f));
            Assert.AreEqual(0.7f, MathFng.MixRestitution(0.7f, 0.5f));
        }
    }
}
