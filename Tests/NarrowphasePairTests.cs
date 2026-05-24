using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;

namespace Box2DNG.Tests
{
    [TestClass]
    public class NarrowphasePairTests
    {
        private static Transform XY(float x, float y) => new Transform(new Vec2(x, y), Rot.Identity);

        // ----- Direct ContactManager.Evaluate per shape-pair combination -----

        [TestMethod]
        public void Evaluate_CircleVsCircle_OverlapProducesPoint()
        {
            Manifold m = new Manifold();
            ContactManager.Evaluate(
                new CircleShape(0.5f), XY(0f, 0f),
                new CircleShape(0.5f), XY(0.5f, 0f),
                m);
            Assert.AreEqual(1, m.PointCount);
            Assert.AreEqual(ManifoldType.Circles, m.Type);
        }

        [TestMethod]
        public void Evaluate_CircleVsCircle_SeparatedNoPoints()
        {
            Manifold m = new Manifold();
            ContactManager.Evaluate(
                new CircleShape(0.5f), XY(0f, 0f),
                new CircleShape(0.5f), XY(5f, 0f),
                m);
            Assert.AreEqual(0, m.PointCount);
        }

        [TestMethod]
        public void Evaluate_PolygonVsPolygon_OverlapProducesPoints()
        {
            Vec2[] box = { new Vec2(-1, -1), new Vec2(1, -1), new Vec2(1, 1), new Vec2(-1, 1) };
            Manifold m = new Manifold();
            ContactManager.Evaluate(
                new PolygonShape(box), XY(0f, 0f),
                new PolygonShape(box), XY(1.5f, 0f),
                m);
            Assert.IsTrue(m.PointCount >= 1, $"Expected polygon-polygon contact, got {m.PointCount} points");
        }

        [TestMethod]
        public void Evaluate_CircleVsPolygon_BothDirections()
        {
            Vec2[] box = { new Vec2(-1, -1), new Vec2(1, -1), new Vec2(1, 1), new Vec2(-1, 1) };
            Manifold cp = new Manifold();
            ContactManager.Evaluate(
                new CircleShape(0.5f), XY(1.2f, 0f),
                new PolygonShape(box), XY(0f, 0f),
                cp);
            Assert.AreEqual(1, cp.PointCount);

            // Swap order — coverage of the polygon-vs-circle dispatch.
            Manifold pc = new Manifold();
            ContactManager.Evaluate(
                new PolygonShape(box), XY(0f, 0f),
                new CircleShape(0.5f), XY(1.2f, 0f),
                pc);
            Assert.AreEqual(1, pc.PointCount);
        }

        [TestMethod]
        public void Evaluate_CapsuleVsCircle()
        {
            CapsuleShape capsule = new CapsuleShape(new Vec2(-0.5f, 0f), new Vec2(0.5f, 0f), 0.5f);
            Manifold m = new Manifold();
            ContactManager.Evaluate(
                capsule, XY(0f, 0f),
                new CircleShape(0.4f), XY(0f, 0.5f),
                m);
            // Capsule + circle should produce a single point.
            Assert.IsTrue(m.PointCount >= 1, $"Expected capsule/circle contact, got {m.PointCount} points");
        }

        [TestMethod]
        public void Evaluate_CapsuleVsCapsule()
        {
            CapsuleShape ca = new CapsuleShape(new Vec2(-0.5f, 0f), new Vec2(0.5f, 0f), 0.4f);
            CapsuleShape cb = new CapsuleShape(new Vec2(-0.5f, 0f), new Vec2(0.5f, 0f), 0.4f);
            Manifold m = new Manifold();
            ContactManager.Evaluate(ca, XY(0f, 0f), cb, XY(0f, 0.5f), m);
            Assert.IsTrue(m.PointCount >= 1);
        }

        [TestMethod]
        public void Evaluate_CapsuleVsPolygon()
        {
            Vec2[] box = { new Vec2(-1, -1), new Vec2(1, -1), new Vec2(1, 1), new Vec2(-1, 1) };
            Manifold m = new Manifold();
            ContactManager.Evaluate(
                new CapsuleShape(new Vec2(-0.5f, 0f), new Vec2(0.5f, 0f), 0.5f), XY(0f, 0.8f),
                new PolygonShape(box), XY(0f, 0f),
                m);
            Assert.IsTrue(m.PointCount >= 1, $"Expected capsule/polygon contact, got {m.PointCount}");
        }

        [TestMethod]
        public void Evaluate_SegmentVsCircle()
        {
            SegmentShape seg = new SegmentShape(new Vec2(-5f, 0f), new Vec2(5f, 0f));
            Manifold m = new Manifold();
            ContactManager.Evaluate(
                seg, XY(0f, 0f),
                new CircleShape(0.5f), XY(0f, 0.4f),
                m);
            Assert.AreEqual(1, m.PointCount);
        }

        [TestMethod]
        public void Evaluate_SegmentVsCapsule()
        {
            SegmentShape seg = new SegmentShape(new Vec2(-5f, 0f), new Vec2(5f, 0f));
            CapsuleShape capsule = new CapsuleShape(new Vec2(-0.4f, 0f), new Vec2(0.4f, 0f), 0.5f);
            Manifold m = new Manifold();
            ContactManager.Evaluate(seg, XY(0f, 0f), capsule, XY(0f, 0.3f), m);
            Assert.IsTrue(m.PointCount >= 1, $"Expected segment/capsule contact, got {m.PointCount}");
        }

        [TestMethod]
        public void Evaluate_SegmentVsPolygon()
        {
            SegmentShape seg = new SegmentShape(new Vec2(-5f, 0f), new Vec2(5f, 0f));
            Vec2[] box = { new Vec2(-1, -1), new Vec2(1, -1), new Vec2(1, 1), new Vec2(-1, 1) };
            Manifold m = new Manifold();
            ContactManager.Evaluate(seg, XY(0f, 0f), new PolygonShape(box), XY(0f, 0.8f), m);
            Assert.IsTrue(m.PointCount >= 1);
        }

        [TestMethod]
        public void Evaluate_ChainSegmentVsCircle()
        {
            // Chain runs left-to-right along y=0 with ghosts continuing the line.
            // Circle is centered above and overlapping — should produce a contact.
            ChainSegmentShape chain = new ChainSegmentShape(
                point1: new Vec2(-2f, 0f), point2: new Vec2(2f, 0f),
                ghost1: new Vec2(-5f, 0f), ghost2: new Vec2(5f, 0f));
            Manifold m = new Manifold();
            ContactManager.Evaluate(chain, XY(0f, 0f), new CircleShape(0.5f), XY(0f, 0.2f), m);
            Assert.IsTrue(m.PointCount >= 1, $"Expected chain/circle contact, got {m.PointCount}");
        }

        [TestMethod]
        public void Evaluate_ChainSegmentVsPolygon()
        {
            ChainSegmentShape chain = new ChainSegmentShape(
                point1: new Vec2(-3f, 0f), point2: new Vec2(3f, 0f),
                ghost1: new Vec2(-6f, 0f), ghost2: new Vec2(6f, 0f));
            Vec2[] box = { new Vec2(-0.5f, -0.5f), new Vec2(0.5f, -0.5f), new Vec2(0.5f, 0.5f), new Vec2(-0.5f, 0.5f) };
            Manifold m = new Manifold();
            ContactManager.Evaluate(chain, XY(0f, 0f), new PolygonShape(box), XY(0f, 0.2f), m);
            Assert.IsTrue(m.PointCount >= 1, $"Expected chain/polygon contact, got {m.PointCount}");
        }

        // ----- Null-shape guard -----

        [TestMethod]
        public void Evaluate_NullShape_Throws()
        {
            Assert.ThrowsException<ArgumentNullException>(() =>
                ContactManager.Evaluate(null!, Transform.Identity, new CircleShape(1f), Transform.Identity, new Manifold()));
            Assert.ThrowsException<ArgumentNullException>(() =>
                ContactManager.Evaluate(new CircleShape(1f), Transform.Identity, null!, Transform.Identity, new Manifold()));
        }

        // ----- WorldManifold initialization across manifold types -----

        [TestMethod]
        public void WorldManifold_Initialize_Circles()
        {
            Manifold m = new Manifold();
            ContactManager.Evaluate(
                new CircleShape(0.5f), XY(0f, 0f),
                new CircleShape(0.5f), XY(0.6f, 0f),
                m);
            WorldManifold wm = new WorldManifold();
            wm.Initialize(m, XY(0f, 0f), 0.5f, XY(0.6f, 0f), 0.5f);
            // Normal points from A to B → along +X.
            Assert.IsTrue(wm.Normal.X > 0.9f, $"Normal should be ~+X, got {wm.Normal}");
        }

        [TestMethod]
        public void WorldManifold_Initialize_FaceA()
        {
            Vec2[] box = { new Vec2(-1, -1), new Vec2(1, -1), new Vec2(1, 1), new Vec2(-1, 1) };
            Manifold m = new Manifold();
            ContactManager.Evaluate(
                new PolygonShape(box), XY(0f, 0f),
                new PolygonShape(box), XY(1.5f, 0f),
                m);
            WorldManifold wm = new WorldManifold();
            wm.Initialize(m, XY(0f, 0f), 0f, XY(1.5f, 0f), 0f);
            Assert.IsTrue(wm.Normal.LengthSquared > 0f);
        }

        [TestMethod]
        public void WorldManifold_Initialize_FaceB_FlipsNormal()
        {
            // CircleVsPolygon dispatch produces a FaceB manifold (polygon is B in cpp convention).
            Vec2[] box = { new Vec2(-1, -1), new Vec2(1, -1), new Vec2(1, 1), new Vec2(-1, 1) };
            Manifold m = new Manifold();
            ContactManager.Evaluate(
                new CircleShape(0.5f), XY(1.2f, 0f),
                new PolygonShape(box), XY(0f, 0f),
                m);
            WorldManifold wm = new WorldManifold();
            wm.Initialize(m, XY(1.2f, 0f), 0.5f, XY(0f, 0f), 0f);
            // Normal should point from circle to polygon — toward -X.
            Assert.IsTrue(wm.Normal.X < -0.5f, $"Expected -X normal, got {wm.Normal}");
        }

        [TestMethod]
        public void WorldManifold_Initialize_EmptyManifold()
        {
            Manifold m = new Manifold(); // 0 points
            WorldManifold wm = new WorldManifold();
            wm.Initialize(m, XY(0f, 0f), 0f, XY(0f, 0f), 0f);
            Assert.AreEqual(Vec2.Zero, wm.Normal);
        }

        // ----- Contact-creation through World.Step settles the ball on the ground -----

        [TestMethod]
        public void World_BallSettlesOnGround()
        {
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)));
            Body ground = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            ground.CreateFixture(new FixtureDef(new SegmentShape(new Vec2(-10f, 0f), new Vec2(10f, 0f))));

            Body ball = world.CreateBody(new BodyDef().AsDynamic().At(0f, 5f));
            ball.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));

            for (int i = 0; i < 180; ++i)
            {
                world.Step(1f / 60f);
            }
            // Resting on the segment at y=0 with circle radius 0.5 → center near y=0.5.
            Assert.IsTrue(ball.Transform.P.Y > -0.1f && ball.Transform.P.Y < 1f,
                $"Ball should be resting near the ground. y={ball.Transform.P.Y}");
        }
    }
}
