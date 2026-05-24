using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Collections.Generic;

namespace Box2DNG.Tests
{
    [TestClass]
    public class ContactCollisionNegativeTests
    {
        // ----- Collision.IsValidRay -----

        [TestMethod]
        public void IsValidRay_RejectsNanOrigin()
        {
            RayCastInput nanOrigin = new RayCastInput(new Vec2(float.NaN, 0f), new Vec2(1f, 0f), 1f);
            Assert.IsFalse(Collision.IsValidRay(nanOrigin));
        }

        [TestMethod]
        public void IsValidRay_RejectsInfiniteTranslation()
        {
            RayCastInput infTranslation = new RayCastInput(Vec2.Zero, new Vec2(float.PositiveInfinity, 0f), 1f);
            Assert.IsFalse(Collision.IsValidRay(infTranslation));
        }

        [TestMethod]
        public void IsValidRay_RejectsNegativeMaxFraction()
        {
            RayCastInput input = new RayCastInput(Vec2.Zero, new Vec2(1f, 0f), -1f);
            Assert.IsFalse(Collision.IsValidRay(input));
        }

        [TestMethod]
        public void IsValidRay_RejectsNanMaxFraction()
        {
            RayCastInput input = new RayCastInput(Vec2.Zero, new Vec2(1f, 0f), float.NaN);
            Assert.IsFalse(Collision.IsValidRay(input));
        }

        [TestMethod]
        public void IsValidRay_AcceptsValid()
        {
            RayCastInput input = new RayCastInput(Vec2.Zero, new Vec2(1f, 0f), 1f);
            Assert.IsTrue(Collision.IsValidRay(input));
        }

        // ----- Collision.TestOverlap -----

        [TestMethod]
        public void TestOverlap_FalseForSeparatedAlongX()
        {
            Aabb a = new Aabb(new Vec2(0f, 0f), new Vec2(1f, 1f));
            Aabb b = new Aabb(new Vec2(2f, 0f), new Vec2(3f, 1f));
            Assert.IsFalse(Collision.TestOverlap(a, b));
        }

        [TestMethod]
        public void TestOverlap_FalseForSeparatedAlongY()
        {
            Aabb a = new Aabb(new Vec2(0f, 0f), new Vec2(1f, 1f));
            Aabb b = new Aabb(new Vec2(0f, 2f), new Vec2(1f, 3f));
            Assert.IsFalse(Collision.TestOverlap(a, b));
        }

        [TestMethod]
        public void TestOverlap_TrueForOverlap()
        {
            Aabb a = new Aabb(new Vec2(0f, 0f), new Vec2(2f, 2f));
            Aabb b = new Aabb(new Vec2(1f, 1f), new Vec2(3f, 3f));
            Assert.IsTrue(Collision.TestOverlap(a, b));
        }

        // ----- Ray cast misses (negative paths) -----

        [TestMethod]
        public void RayCastCircle_RayBehindCircleMisses()
        {
            // Ray originates past the circle going further away
            Circle c = new Circle(new Vec2(0f, 0f), 1f);
            RayCastInput input = new RayCastInput(new Vec2(5f, 0f), new Vec2(10f, 0f), 1f);
            CastOutput output = Collision.RayCastCircle(c, input);
            Assert.IsFalse(output.Hit);
        }

        [TestMethod]
        public void RayCastCircle_ParallelMisses()
        {
            Circle c = new Circle(new Vec2(0f, 0f), 1f);
            // Ray offset 3 above, going right — never gets near the circle
            RayCastInput input = new RayCastInput(new Vec2(-5f, 3f), new Vec2(10f, 0f), 1f);
            CastOutput output = Collision.RayCastCircle(c, input);
            Assert.IsFalse(output.Hit);
        }

        [TestMethod]
        public void RayCastCircle_HitsWhenAimedAtCircle()
        {
            Circle c = new Circle(new Vec2(0f, 0f), 1f);
            RayCastInput input = new RayCastInput(new Vec2(-5f, 0f), new Vec2(10f, 0f), 1f);
            CastOutput output = Collision.RayCastCircle(c, input);
            Assert.IsTrue(output.Hit);
        }

        [TestMethod]
        public void RayCastSegment_MissesParallelRay()
        {
            Segment seg = new Segment(new Vec2(0f, 0f), new Vec2(5f, 0f));
            // Ray running parallel to the segment, 1 above
            RayCastInput input = new RayCastInput(new Vec2(-1f, 1f), new Vec2(10f, 0f), 1f);
            CastOutput output = Collision.RayCastSegment(seg, input);
            Assert.IsFalse(output.Hit);
        }

        [TestMethod]
        public void RayCastSegment_ZeroLengthRayDoesNotHit()
        {
            Segment seg = new Segment(new Vec2(0f, 0f), new Vec2(5f, 0f));
            RayCastInput input = new RayCastInput(new Vec2(2f, 2f), Vec2.Zero, 1f);
            CastOutput output = Collision.RayCastSegment(seg, input);
            Assert.IsFalse(output.Hit);
        }

        // ----- Contact filtering: collisions blocked by Filter mask -----

        [TestMethod]
        public void FilterMaskMismatch_ProducesNoContact()
        {
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)));

            Body ground = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            ground.CreateFixture(new FixtureDef(new SegmentShape(new Vec2(-10f, 0f), new Vec2(10f, 0f)))
                .WithFilter(new Filter(0x0001, 0x0001, 0))); // only collides with category 0x0001

            // Ball is category 0x0004, mask 0x0004 — should pass through the ground.
            Body ball = world.CreateBody(new BodyDef().AsDynamic().At(0f, 2f));
            ball.CreateFixture(new FixtureDef(new CircleShape(0.3f))
                .WithDensity(1f)
                .WithFilter(new Filter(0x0004, 0x0004, 0)));

            for (int i = 0; i < 60; ++i)
            {
                world.Step(1f / 60f);
            }
            // Ball should have fallen well below the ground line because there's no contact.
            Assert.IsTrue(ball.Transform.P.Y < -1f,
                $"Ball should pass through filtered-out ground. y = {ball.Transform.P.Y}");
        }

        [TestMethod]
        public void FilterGroupIndex_NegativeBlocksCollision()
        {
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)));
            // Same negative group → no collision between members of the group.
            Filter groupBlock = new Filter(0x0001, ulong.MaxValue, -42);

            Body ground = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            ground.CreateFixture(new FixtureDef(new SegmentShape(new Vec2(-10f, 0f), new Vec2(10f, 0f)))
                .WithFilter(groupBlock));

            Body ball = world.CreateBody(new BodyDef().AsDynamic().At(0f, 2f));
            ball.CreateFixture(new FixtureDef(new CircleShape(0.3f)).WithDensity(1f).WithFilter(groupBlock));

            for (int i = 0; i < 60; ++i)
            {
                world.Step(1f / 60f);
            }
            Assert.IsTrue(ball.Transform.P.Y < -1f,
                $"Ball should pass through groupIndex-blocked ground. y = {ball.Transform.P.Y}");
        }

        // ----- Sensors don't generate contact response -----

        [TestMethod]
        public void Sensor_DoesNotImpedeMotion()
        {
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)));

            Body sensorBody = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            sensorBody.CreateFixture(new FixtureDef(new PolygonShape(new[]
                {
                    new Vec2(-2f, -0.5f), new Vec2(2f, -0.5f),
                    new Vec2(2f, 0.5f), new Vec2(-2f, 0.5f)
                }))
                .AsSensor());

            Body ball = world.CreateBody(new BodyDef().AsDynamic().At(0f, 5f));
            ball.CreateFixture(new FixtureDef(new CircleShape(0.3f)).WithDensity(1f));

            for (int i = 0; i < 120; ++i)
            {
                world.Step(1f / 60f);
            }
            // Sensor doesn't physically block, so ball falls past it.
            Assert.IsTrue(ball.Transform.P.Y < -1f,
                $"Sensor should not stop the ball. y = {ball.Transform.P.Y}");
        }

        // ----- Default filter accepts everything -----

        [TestMethod]
        public void DefaultFilter_AllowsCollision()
        {
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)));
            Body ground = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            ground.CreateFixture(new FixtureDef(new SegmentShape(new Vec2(-10f, 0f), new Vec2(10f, 0f))));

            Body ball = world.CreateBody(new BodyDef().AsDynamic().At(0f, 2f));
            ball.CreateFixture(new FixtureDef(new CircleShape(0.3f)).WithDensity(1f));

            for (int i = 0; i < 60; ++i)
            {
                world.Step(1f / 60f);
            }
            // Ball should rest near the ground.
            Assert.IsTrue(ball.Transform.P.Y > -0.5f && ball.Transform.P.Y < 1f,
                $"Default filter should collide. y = {ball.Transform.P.Y}");
        }
    }
}
