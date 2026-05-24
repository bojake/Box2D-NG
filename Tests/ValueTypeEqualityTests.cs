using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;

namespace Box2DNG.Tests
{
    [TestClass]
    public class ValueTypeEqualityTests
    {
        // ----- Filter -----

        [TestMethod]
        public void Filter_Equality_AndHash_ByValue()
        {
            Filter a = new Filter(0x0001, 0xFFFFul, 5);
            Filter b = new Filter(0x0001, 0xFFFFul, 5);
            Filter c = new Filter(0x0002, 0xFFFFul, 5);
            Filter d = new Filter(0x0001, 0xFFFFul, 6);
            Filter e = new Filter(0x0001, 0xFEFFul, 5);

            Assert.IsTrue(a.Equals(b));
            Assert.IsTrue(a.Equals((object)b));
            Assert.IsFalse(a.Equals(c));
            Assert.IsFalse(a.Equals(d));
            Assert.IsFalse(a.Equals(e));
            Assert.IsFalse(a.Equals((object?)null));
            Assert.IsFalse(a.Equals((object)42));
            Assert.AreEqual(a.GetHashCode(), b.GetHashCode());
        }

        [TestMethod]
        public void Filter_Default_IsCategory1MaskAll()
        {
            Filter def = Filter.Default;
            Assert.AreEqual(1ul, def.CategoryBits);
            Assert.AreEqual(ulong.MaxValue, def.MaskBits);
            Assert.AreEqual(0, def.GroupIndex);
        }

        // ----- QueryFilter -----

        [TestMethod]
        public void QueryFilter_Equality_AndHash_ByValue()
        {
            QueryFilter a = new QueryFilter(0x0001, 0xFFFFul);
            QueryFilter b = new QueryFilter(0x0001, 0xFFFFul);
            QueryFilter c = new QueryFilter(0x0002, 0xFFFFul);
            QueryFilter d = new QueryFilter(0x0001, 0xFEFFul);

            Assert.IsTrue(a.Equals(b));
            Assert.IsTrue(a.Equals((object)b));
            Assert.IsFalse(a.Equals(c));
            Assert.IsFalse(a.Equals(d));
            Assert.IsFalse(a.Equals((object?)null));
            Assert.AreEqual(a.GetHashCode(), b.GetHashCode());
        }

        [TestMethod]
        public void QueryFilter_Default_IsCategory1MaskAll()
        {
            QueryFilter def = QueryFilter.Default;
            Assert.AreEqual(1ul, def.CategoryBits);
            Assert.AreEqual(ulong.MaxValue, def.MaskBits);
        }

        // ----- SurfaceMaterial -----

        [TestMethod]
        public void SurfaceMaterial_Equality_AndHash_ByValue()
        {
            SurfaceMaterial a = new SurfaceMaterial(0.5f, 0.1f, 0.2f, 1f, 7, 0xAABBCC);
            SurfaceMaterial b = new SurfaceMaterial(0.5f, 0.1f, 0.2f, 1f, 7, 0xAABBCC);
            Assert.IsTrue(a.Equals(b));
            Assert.IsTrue(a.Equals((object)b));
            Assert.IsFalse(a.Equals((object?)null));
            Assert.IsFalse(a.Equals(new SurfaceMaterial(0.6f, 0.1f, 0.2f, 1f, 7, 0xAABBCC)));
            Assert.IsFalse(a.Equals(new SurfaceMaterial(0.5f, 0.9f, 0.2f, 1f, 7, 0xAABBCC)));
            Assert.IsFalse(a.Equals(new SurfaceMaterial(0.5f, 0.1f, 0.5f, 1f, 7, 0xAABBCC)));
            Assert.IsFalse(a.Equals(new SurfaceMaterial(0.5f, 0.1f, 0.2f, 5f, 7, 0xAABBCC)));
            Assert.IsFalse(a.Equals(new SurfaceMaterial(0.5f, 0.1f, 0.2f, 1f, 99, 0xAABBCC)));
            Assert.IsFalse(a.Equals(new SurfaceMaterial(0.5f, 0.1f, 0.2f, 1f, 7, 0xDEADBE)));
            Assert.AreEqual(a.GetHashCode(), b.GetHashCode());
        }

        [TestMethod]
        public void SurfaceMaterial_Default_HasFriction06()
        {
            SurfaceMaterial def = SurfaceMaterial.Default;
            Assert.AreEqual(0.6f, def.Friction);
            Assert.AreEqual(0f, def.Restitution);
            Assert.AreEqual(0f, def.RollingResistance);
            Assert.AreEqual(0f, def.TangentSpeed);
        }

        // ----- CosSin -----

        [TestMethod]
        public void CosSin_Equality_AndHash_ByValue()
        {
            CosSin a = new CosSin(0.5f, 0.8f);
            CosSin b = new CosSin(0.5f, 0.8f);
            CosSin c = new CosSin(0.6f, 0.8f);
            Assert.IsTrue(a.Equals(b));
            Assert.IsTrue(a.Equals((object)b));
            Assert.IsFalse(a.Equals(c));
            Assert.IsFalse(a.Equals((object?)null));
            Assert.AreEqual(a.GetHashCode(), b.GetHashCode());
            Assert.AreEqual("(cos:0.5, sin:0.8)", a.ToString());
        }

        // ----- Aabb -----

        [TestMethod]
        public void Aabb_Equality_AndHash_ByValue()
        {
            Aabb a = new Aabb(new Vec2(-1f, -1f), new Vec2(1f, 1f));
            Aabb b = new Aabb(new Vec2(-1f, -1f), new Vec2(1f, 1f));
            Aabb c = new Aabb(new Vec2(0f, -1f), new Vec2(1f, 1f));
            Assert.IsTrue(a.Equals(b));
            Assert.IsTrue(a.Equals((object)b));
            Assert.IsFalse(a.Equals(c));
            Assert.IsFalse(a.Equals((object?)null));
            Assert.AreEqual(a.GetHashCode(), b.GetHashCode());
            Assert.IsTrue(a.ToString().Contains("->"));
        }

        [TestMethod]
        public void Aabb_CenterExtentsPerimeter()
        {
            Aabb a = new Aabb(new Vec2(-1f, -2f), new Vec2(3f, 4f));
            Assert.AreEqual(new Vec2(1f, 1f), a.Center);
            Assert.AreEqual(new Vec2(2f, 3f), a.Extents);
            // Perimeter = 2 * (4 + 6) = 20
            Assert.AreEqual(20f, a.Perimeter);
            Assert.IsTrue(a.Contains(new Aabb(new Vec2(0f, 0f), new Vec2(1f, 1f))));
            Assert.IsFalse(a.Contains(new Aabb(new Vec2(-2f, 0f), new Vec2(0f, 1f))));
        }

        // ----- ContactFeature -----

        [TestMethod]
        public void ContactFeature_Equality_AndHash_ByValue()
        {
            ContactFeature a = new ContactFeature(1, 2, 3, 4);
            ContactFeature b = new ContactFeature(1, 2, 3, 4);
            ContactFeature c = new ContactFeature(9, 2, 3, 4);
            Assert.IsTrue(a.Equals(b));
            Assert.IsTrue(a.Equals((object)b));
            Assert.IsFalse(a.Equals(c));
            Assert.IsFalse(a.Equals((object?)null));
            Assert.AreEqual(a.GetHashCode(), b.GetHashCode());
            Assert.AreEqual(1, a.TypeA);
            Assert.AreEqual(2, a.TypeB);
            Assert.AreEqual(3, a.IndexA);
            Assert.AreEqual(4, a.IndexB);
        }

        // ----- Plane is gone, ToString round-trips on other small structs -----

        [TestMethod]
        public void SmallStructs_ToStringIsNonEmpty()
        {
            Assert.IsFalse(string.IsNullOrEmpty(new Vec2(1, 2).ToString()));
            Assert.IsFalse(string.IsNullOrEmpty(new Vec3(1, 2, 3).ToString()));
            Assert.IsFalse(string.IsNullOrEmpty(new Rot(0.5f).ToString()));
            Assert.IsFalse(string.IsNullOrEmpty(Transform.Identity.ToString()));
            Assert.IsFalse(string.IsNullOrEmpty(Mat22.Identity.ToString()));
            Assert.IsFalse(string.IsNullOrEmpty(Mat33.Identity.ToString()));
        }

        // ----- Vec2/Vec3/Rot Equals(object) and null/wrong-type rejections -----

        [TestMethod]
        public void ValueTypes_EqualsObject_NullAndWrongType_ReturnFalse()
        {
            Assert.IsFalse(new Vec2(1, 2).Equals((object?)null));
            Assert.IsFalse(new Vec2(1, 2).Equals((object)"hi"));
            Assert.IsFalse(new Vec3(1, 2, 3).Equals((object?)null));
            Assert.IsFalse(new Vec3(1, 2, 3).Equals((object)42));
            Assert.IsFalse(new Rot(0.4f).Equals((object?)null));
            Assert.IsFalse(new Rot(0.4f).Equals((object)0.4f));
            Assert.IsFalse(Mat22.Identity.Equals((object?)null));
            Assert.IsFalse(Mat33.Identity.Equals((object?)null));
            Assert.IsFalse(Transform.Identity.Equals((object?)null));
            Assert.IsFalse(new Aabb(Vec2.Zero, Vec2.Zero).Equals((object?)null));
        }
    }
}
