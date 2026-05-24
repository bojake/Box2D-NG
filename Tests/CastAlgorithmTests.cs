using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;

namespace Box2DNG.Tests
{
    [TestClass]
    public class CastAlgorithmTests
    {
        private static Polygon BuildBox(float hx, float hy)
        {
            PolygonShape shape = new PolygonShape(new[]
            {
                new Vec2(-hx, -hy),
                new Vec2(hx, -hy),
                new Vec2(hx, hy),
                new Vec2(-hx, hy)
            });
            return ShapeGeometry.ToPolygon(shape);
        }

        // ----- RayCastPolygon -----

        [TestMethod]
        public void RayCastPolygon_HitsFromOutside()
        {
            Polygon box = BuildBox(1f, 1f);
            RayCastInput input = new RayCastInput(new Vec2(-5f, 0f), new Vec2(10f, 0f), 1f);
            CastOutput output = Collision.RayCastPolygon(box, input);
            Assert.IsTrue(output.Hit);
            Assert.IsTrue(output.Fraction > 0f && output.Fraction < 1f);
        }

        [TestMethod]
        public void RayCastPolygon_MissesWhenParallel()
        {
            Polygon box = BuildBox(1f, 1f);
            RayCastInput input = new RayCastInput(new Vec2(-5f, 2f), new Vec2(10f, 0f), 1f);
            CastOutput output = Collision.RayCastPolygon(box, input);
            Assert.IsFalse(output.Hit);
        }

        [TestMethod]
        public void RayCastPolygon_RayStartsInsideReturnsLowerZero()
        {
            Polygon box = BuildBox(1f, 1f);
            RayCastInput input = new RayCastInput(Vec2.Zero, new Vec2(5f, 0f), 1f);
            CastOutput output = Collision.RayCastPolygon(box, input);
            // Ray starts inside the box — lower would be 0, so no useful entry hit.
            // Implementation returns lower=0 case as a non-hit (no entry).
            Assert.AreEqual(0f, output.Fraction);
        }

        // ----- RayCastCapsule -----

        [TestMethod]
        public void RayCastCapsule_HitsBody()
        {
            Capsule capsule = new Capsule(new Vec2(-1f, 0f), new Vec2(1f, 0f), 0.5f);
            RayCastInput input = new RayCastInput(new Vec2(0f, -3f), new Vec2(0f, 6f), 1f);
            CastOutput output = Collision.RayCastCapsule(capsule, input);
            Assert.IsTrue(output.Hit);
        }

        [TestMethod]
        public void RayCastCapsule_HitsEndCap()
        {
            Capsule capsule = new Capsule(new Vec2(-1f, 0f), new Vec2(1f, 0f), 0.5f);
            // Aim into the right end cap.
            RayCastInput input = new RayCastInput(new Vec2(3f, 0f), new Vec2(-5f, 0f), 1f);
            CastOutput output = Collision.RayCastCapsule(capsule, input);
            Assert.IsTrue(output.Hit);
        }

        [TestMethod]
        public void RayCastCapsule_DegenerateCapsuleActsLikeCircle()
        {
            Capsule degenerate = new Capsule(new Vec2(0f, 0f), new Vec2(0f, 0f), 1f);
            RayCastInput input = new RayCastInput(new Vec2(-5f, 0f), new Vec2(10f, 0f), 1f);
            CastOutput output = Collision.RayCastCapsule(degenerate, input);
            Assert.IsTrue(output.Hit);
        }

        [TestMethod]
        public void RayCastCapsule_Misses()
        {
            Capsule capsule = new Capsule(new Vec2(-1f, 0f), new Vec2(1f, 0f), 0.5f);
            RayCastInput input = new RayCastInput(new Vec2(0f, 3f), new Vec2(10f, 0f), 1f);
            CastOutput output = Collision.RayCastCapsule(capsule, input);
            Assert.IsFalse(output.Hit);
        }

        // ----- RayCastChainSegment (delegates to RayCastSegment) -----

        [TestMethod]
        public void RayCastChainSegment_DelegatesToSegment()
        {
            ChainSegment cs = new ChainSegment(
                new Vec2(-5f, 0f),
                new Segment(new Vec2(0f, 0f), new Vec2(2f, 0f)),
                new Vec2(5f, 0f),
                0);
            RayCastInput input = new RayCastInput(new Vec2(1f, -2f), new Vec2(0f, 4f), 1f);
            CastOutput output = Collision.RayCastChainSegment(cs, input);
            Assert.IsTrue(output.Hit);
        }

        // ----- ShapeCast (per shape type) -----

        [TestMethod]
        public void ShapeCastCircle_TranslatedTowardCircleHits()
        {
            // Static target circle proxy at origin, moving small probe circle from the right.
            ShapeProxy probe = ShapeProxyFactory.FromCircle(new Circle(new Vec2(5f, 0f), 0.3f));
            ShapeCastInput input = new ShapeCastInput(probe, new Vec2(-10f, 0f), 1f, canEncroach: false);
            CastOutput output = Collision.ShapeCastCircle(new Circle(Vec2.Zero, 1f), input);
            Assert.IsTrue(output.Hit);
            Assert.IsTrue(output.Fraction > 0f && output.Fraction < 1f);
        }

        [TestMethod]
        public void ShapeCastCircle_TranslatingAwayMisses()
        {
            ShapeProxy probe = ShapeProxyFactory.FromCircle(new Circle(new Vec2(5f, 0f), 0.3f));
            ShapeCastInput input = new ShapeCastInput(probe, new Vec2(10f, 0f), 1f, canEncroach: false);
            CastOutput output = Collision.ShapeCastCircle(new Circle(Vec2.Zero, 1f), input);
            Assert.IsFalse(output.Hit);
        }

        [TestMethod]
        public void ShapeCastCapsule_TranslatedTowardCapsuleHits()
        {
            ShapeProxy probe = ShapeProxyFactory.FromCircle(new Circle(new Vec2(0f, 5f), 0.2f));
            ShapeCastInput input = new ShapeCastInput(probe, new Vec2(0f, -10f), 1f, canEncroach: false);
            CastOutput output = Collision.ShapeCastCapsule(
                new Capsule(new Vec2(-1f, 0f), new Vec2(1f, 0f), 0.5f), input);
            Assert.IsTrue(output.Hit);
        }

        [TestMethod]
        public void ShapeCastSegment_TranslatedTowardSegmentHits()
        {
            ShapeProxy probe = ShapeProxyFactory.FromCircle(new Circle(new Vec2(0f, 5f), 0.2f));
            ShapeCastInput input = new ShapeCastInput(probe, new Vec2(0f, -10f), 1f, canEncroach: false);
            CastOutput output = Collision.ShapeCastSegment(
                new Segment(new Vec2(-2f, 0f), new Vec2(2f, 0f)), input);
            Assert.IsTrue(output.Hit);
        }

        [TestMethod]
        public void ShapeCastPolygon_TranslatedTowardPolygonHits()
        {
            ShapeProxy probe = ShapeProxyFactory.FromCircle(new Circle(new Vec2(0f, 5f), 0.2f));
            ShapeCastInput input = new ShapeCastInput(probe, new Vec2(0f, -10f), 1f, canEncroach: false);
            CastOutput output = Collision.ShapeCastPolygon(BuildBox(1f, 1f), input);
            Assert.IsTrue(output.Hit);
        }

        // ----- World-level RayCastAll returns multiple hits -----

        [TestMethod]
        public void RayCastAll_ReturnsAllHits()
        {
            World world = new World(new WorldDef());
            for (int i = 0; i < 4; ++i)
            {
                Body b = world.CreateBody(new BodyDef().AsStatic().At(i * 2f, 0f));
                b.CreateFixture(new FixtureDef(new CircleShape(0.5f)));
            }

            RayCastInput input = new RayCastInput(new Vec2(-2f, 0f), new Vec2(20f, 0f), 1f);
            var hits = world.RayCastAll(input);
            Assert.AreEqual(4, hits.Count, $"Expected 4 hits, got {hits.Count}");
        }

        // ----- BroadPhase tree-driven aabb query produces consistent results -----

        [TestMethod]
        public void QueryAabb_ReturnsAllOverlappingFixtures()
        {
            World world = new World(new WorldDef());
            Body a = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            a.CreateFixture(new FixtureDef(new CircleShape(0.5f)));
            Body b = world.CreateBody(new BodyDef().AsStatic().At(10f, 0f));
            b.CreateFixture(new FixtureDef(new CircleShape(0.5f)));

            Aabb area = new Aabb(new Vec2(-1f, -1f), new Vec2(1f, 1f));
            var results = world.QueryAabb(area);
            Assert.AreEqual(1, results.Count);
            Aabb wide = new Aabb(new Vec2(-1f, -1f), new Vec2(11f, 1f));
            Assert.AreEqual(2, world.QueryAabb(wide).Count);
        }
    }
}
