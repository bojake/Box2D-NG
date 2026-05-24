using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;

namespace Box2DNG.Tests
{
    [TestClass]
    public class MathUtilTests
    {
        // ----- Rot -----

        [TestMethod]
        public void Rot_FromAngle_RoundTripsThroughAngle()
        {
            float[] angles = { 0f, 0.1f, 1.5707f, -0.7f, 2.5f, -2.5f, 3.1f };
            foreach (float a in angles)
            {
                Rot r = new Rot(a);
                Assert.AreEqual(a, r.Angle, 1e-4f);
            }
        }

        [TestMethod]
        public void Rot_Identity_HasUnitCosineAndZeroSine()
        {
            Rot id = Rot.Identity;
            Assert.AreEqual(1f, id.C);
            Assert.AreEqual(0f, id.S);
            Assert.AreEqual(0f, id.Angle);
        }

        [TestMethod]
        public void Rot_GetXAxisAndYAxis_AreOrthogonal()
        {
            Rot r = new Rot(0.7f);
            Vec2 x = r.GetXAxis();
            Vec2 y = r.GetYAxis();
            Assert.AreEqual(0f, Vec2.Dot(x, y), 1e-5f);
            Assert.AreEqual(1f, x.Length, 1e-5f);
            Assert.AreEqual(1f, y.Length, 1e-5f);
        }

        [TestMethod]
        public void Rot_Mul_ComposesRotations()
        {
            Rot a = new Rot(0.4f);
            Rot b = new Rot(0.3f);
            Rot ab = Rot.Mul(a, b);
            Assert.AreEqual(0.7f, ab.Angle, 1e-4f);
        }

        [TestMethod]
        public void Rot_MulT_IsInverse()
        {
            Rot a = new Rot(0.9f);
            Rot b = new Rot(0.3f);
            Rot ab = Rot.Mul(a, b);
            Rot recovered = Rot.MulT(a, ab);
            Assert.AreEqual(b.Angle, recovered.Angle, 1e-4f);
        }

        [TestMethod]
        public void Rot_MulVec_RotatesVector()
        {
            Rot r = new Rot(MathF.PI / 2f);
            Vec2 v = new Vec2(1f, 0f);
            Vec2 rotated = Rot.Mul(r, v);
            Assert.AreEqual(0f, rotated.X, 1e-5f);
            Assert.AreEqual(1f, rotated.Y, 1e-5f);
        }

        [TestMethod]
        public void Rot_MulTVec_IsInverseRotation()
        {
            Rot r = new Rot(0.6f);
            Vec2 v = new Vec2(2f, 3f);
            Vec2 rotated = Rot.Mul(r, v);
            Vec2 recovered = Rot.MulT(r, rotated);
            Assert.AreEqual(v.X, recovered.X, 1e-4f);
            Assert.AreEqual(v.Y, recovered.Y, 1e-4f);
        }

        [TestMethod]
        public void Rot_Equality_WorksByValue()
        {
            Rot a = new Rot(1.2f);
            Rot b = new Rot(1.2f);
            Assert.IsTrue(a.Equals(b));
            Assert.AreEqual(a.GetHashCode(), b.GetHashCode());
            Assert.IsFalse(a.Equals(new Rot(1.3f)));
        }

        // ----- Transform -----

        [TestMethod]
        public void Transform_Identity_HasZeroTranslationAndIdentityRotation()
        {
            Transform t = Transform.Identity;
            Assert.AreEqual(Vec2.Zero, t.P);
            Assert.AreEqual(Rot.Identity, t.Q);
        }

        [TestMethod]
        public void Transform_Mul_TransformsPoint()
        {
            Transform t = new Transform(new Vec2(1f, 2f), new Rot(MathF.PI / 2f));
            Vec2 result = Transform.Mul(t, new Vec2(1f, 0f));
            Assert.AreEqual(1f, result.X, 1e-5f);
            Assert.AreEqual(3f, result.Y, 1e-5f);
        }

        [TestMethod]
        public void Transform_MulT_IsInverseOfMul()
        {
            Transform t = new Transform(new Vec2(1f, 2f), new Rot(0.4f));
            Vec2 p = new Vec2(5f, 6f);
            Vec2 world = Transform.Mul(t, p);
            Vec2 back = Transform.MulT(t, world);
            Assert.AreEqual(p.X, back.X, 1e-4f);
            Assert.AreEqual(p.Y, back.Y, 1e-4f);
        }

        [TestMethod]
        public void Transform_Equality_WorksByValue()
        {
            Transform a = new Transform(new Vec2(1f, 2f), new Rot(0.4f));
            Transform b = new Transform(new Vec2(1f, 2f), new Rot(0.4f));
            Assert.IsTrue(a.Equals(b));
            Assert.AreEqual(a.GetHashCode(), b.GetHashCode());
        }

        // ----- Vec3 -----

        [TestMethod]
        public void Vec3_BasicOpsAndDotCross()
        {
            Vec3 a = new Vec3(1f, 2f, 3f);
            Vec3 b = new Vec3(4f, -1f, 2f);

            Vec3 sum = a + b;
            Assert.AreEqual(new Vec3(5f, 1f, 5f), sum);

            Vec3 diff = a - b;
            Assert.AreEqual(new Vec3(-3f, 3f, 1f), diff);

            Vec3 neg = -a;
            Assert.AreEqual(new Vec3(-1f, -2f, -3f), neg);

            Vec3 scaled = a * 2f;
            Assert.AreEqual(new Vec3(2f, 4f, 6f), scaled);

            Vec3 scaled2 = 2f * a;
            Assert.AreEqual(scaled, scaled2);

            Vec3 divided = a / 2f;
            Assert.AreEqual(new Vec3(0.5f, 1f, 1.5f), divided);

            float dot = Vec3.Dot(a, b);
            Assert.AreEqual(1f * 4f + 2f * -1f + 3f * 2f, dot, 1e-5f);

            Vec3 cross = Vec3.Cross(a, b);
            Assert.AreEqual(2f * 2f - 3f * -1f, cross.X, 1e-5f);
            Assert.AreEqual(3f * 4f - 1f * 2f, cross.Y, 1e-5f);
            Assert.AreEqual(1f * -1f - 2f * 4f, cross.Z, 1e-5f);
        }

        [TestMethod]
        public void Vec3_LengthAndNormalize()
        {
            Vec3 v = new Vec3(2f, 0f, 0f);
            Assert.AreEqual(2f, v.Length, 1e-5f);
            Assert.AreEqual(4f, v.LengthSquared);

            Vec3 unit = v.Normalize();
            Assert.AreEqual(1f, unit.Length, 1e-5f);

            Vec3 zero = Vec3.Zero.Normalize();
            Assert.AreEqual(Vec3.Zero, zero);
        }

        [TestMethod]
        public void Vec3_Equality_WorksByValue()
        {
            Vec3 a = new Vec3(1f, 2f, 3f);
            Vec3 b = new Vec3(1f, 2f, 3f);
            Assert.IsTrue(a.Equals(b));
            Assert.IsFalse(a.Equals(new Vec3(1f, 2f, 4f)));
            Assert.AreEqual(a.GetHashCode(), b.GetHashCode());
        }

        // ----- BitSet -----

        [TestMethod]
        public void BitSet_SetGetClear_AcrossBlockBoundary()
        {
            BitSet set = new BitSet(128);
            set.SetBitCountAndClear(128);
            Assert.IsFalse(set.GetBit(0));
            Assert.IsFalse(set.GetBit(63));
            Assert.IsFalse(set.GetBit(64));
            Assert.IsFalse(set.GetBit(127));

            set.SetBit(0);
            set.SetBit(63);
            set.SetBit(64);
            set.SetBit(127);

            Assert.IsTrue(set.GetBit(0));
            Assert.IsTrue(set.GetBit(63));
            Assert.IsTrue(set.GetBit(64));
            Assert.IsTrue(set.GetBit(127));

            set.ClearBit(63);
            Assert.IsFalse(set.GetBit(63));
            Assert.IsTrue(set.GetBit(64));
        }

        [TestMethod]
        public void BitSet_CountSetBits_MatchesPopCount()
        {
            BitSet set = new BitSet(256);
            set.SetBitCountAndClear(256);
            for (uint i = 0; i < 256; i += 3)
            {
                set.SetBit(i);
            }
            // i = 0, 3, 6, ... 255 → 86 bits set (256/3 rounded)
            int expected = (256 + 2) / 3;
            Assert.AreEqual(expected, set.CountSetBits());
        }

        [TestMethod]
        public void BitSet_SetBitGrow_GrowsCapacity()
        {
            BitSet set = new BitSet(64);
            set.SetBitCountAndClear(64);
            set.SetBitGrow(500);
            Assert.IsTrue(set.GetBit(500));
        }

        [TestMethod]
        public void BitSet_InPlaceUnion_OrsTwoSets()
        {
            BitSet a = new BitSet(128);
            a.SetBitCountAndClear(128);
            a.SetBit(1);
            a.SetBit(70);

            BitSet b = new BitSet(128);
            b.SetBitCountAndClear(128);
            b.SetBit(2);
            b.SetBit(70);

            a.InPlaceUnion(b);
            Assert.IsTrue(a.GetBit(1));
            Assert.IsTrue(a.GetBit(2));
            Assert.IsTrue(a.GetBit(70));
            Assert.AreEqual(3, a.CountSetBits());
        }

        [TestMethod]
        public void BitSet_ClearOutOfRange_DoesNotThrow()
        {
            BitSet set = new BitSet(64);
            set.SetBitCountAndClear(64);
            set.ClearBit(10000); // no-op for out-of-range
            Assert.IsFalse(set.GetBit(10000));
        }

        // ----- BitUtils -----

        [TestMethod]
        public void BitUtils_IsPowerOf2()
        {
            Assert.IsTrue(BitUtils.IsPowerOf2(1));
            Assert.IsTrue(BitUtils.IsPowerOf2(2));
            Assert.IsTrue(BitUtils.IsPowerOf2(4));
            Assert.IsTrue(BitUtils.IsPowerOf2(1024));
            Assert.IsFalse(BitUtils.IsPowerOf2(3));
            Assert.IsFalse(BitUtils.IsPowerOf2(7));
            Assert.IsFalse(BitUtils.IsPowerOf2(100));
        }

        [TestMethod]
        public void BitUtils_RoundUpPowerOf2()
        {
            Assert.AreEqual(1, BitUtils.RoundUpPowerOf2(0));
            Assert.AreEqual(1, BitUtils.RoundUpPowerOf2(1));
            Assert.AreEqual(2, BitUtils.RoundUpPowerOf2(2));
            Assert.AreEqual(4, BitUtils.RoundUpPowerOf2(3));
            Assert.AreEqual(8, BitUtils.RoundUpPowerOf2(5));
            Assert.AreEqual(1024, BitUtils.RoundUpPowerOf2(513));
        }

        [TestMethod]
        public void BitUtils_BoundingPowerOf2_IsExponentForRoundUp()
        {
            Assert.AreEqual(1, BitUtils.BoundingPowerOf2(0));
            Assert.AreEqual(1, BitUtils.BoundingPowerOf2(1));
            Assert.AreEqual(1, BitUtils.BoundingPowerOf2(2));
            Assert.AreEqual(2, BitUtils.BoundingPowerOf2(3));
            Assert.AreEqual(3, BitUtils.BoundingPowerOf2(5));
            Assert.AreEqual(10, BitUtils.BoundingPowerOf2(513));
        }
    }
}
