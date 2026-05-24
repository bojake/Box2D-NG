using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Box2DNG.Tests
{
    [TestClass]
    public class BreakableSampleTests
    {
        [TestMethod]
        public void Breakable_BreaksOnHighImpulse()
        {
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)));

            Body ground = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            ground.CreateFixture(new FixtureDef(new SegmentShape(new Vec2(-40f, 0f), new Vec2(40f, 0f))));

            Body body = world.CreateBody(new BodyDef().AsDynamic().At(0f, 40f).WithAngle(0.25f * MathFng.Pi));
            PolygonShape shape1 = new PolygonShape(BuildBoxVertices(0.5f, 0.5f, new Vec2(-0.5f, 0f), 0f));
            PolygonShape shape2 = new PolygonShape(BuildBoxVertices(0.5f, 0.5f, new Vec2(0.5f, 0f), 0f));
            Fixture piece1 = body.CreateFixture(new FixtureDef(shape1).WithDensity(1f).WithUserData("piece1"));
            Fixture? piece2 = body.CreateFixture(new FixtureDef(shape2).WithDensity(1f).WithUserData("piece2"));

            bool breakFlag = false;
            // Per-event ContactImpulseEvent.NormalImpulse for this scenario
            // (2-square body, 40m drop, 2 kg total mass) peaks at ~2.4 N·s with
            // the GJK direction fix and the Solve3 simplex-collapse fix in
            // place — the corrected per-point distribution is much smaller than
            // the pre-fix value (~15-20) which was inflated by stale impulses
            // and wrong GJK distances.
            const float kBreakThreshold = 2.0f;
            world.Events.ContactImpulseEvents += events =>
            {
                for (int i = 0; i < events.Events.Length; ++i)
                {
                    ContactImpulseEvent impulse = events.Events[i];
                    if (impulse.NormalImpulse > kBreakThreshold &&
                        (Equals(impulse.UserDataA, "piece1") || Equals(impulse.UserDataB, "piece1") ||
                         Equals(impulse.UserDataA, "piece2") || Equals(impulse.UserDataB, "piece2")))
                    {
                        breakFlag = true;
                        break;
                    }
                }
            };

            Vec2 cachedVelocity = Vec2.Zero;
            float cachedAngularVelocity = 0f;
            bool broke = false;

            for (int i = 0; i < 600; ++i)
            {
                world.Step(1f / 60f);

                if (!broke)
                {
                    cachedVelocity = body.LinearVelocity;
                    cachedAngularVelocity = body.AngularVelocity;
                }

                if (breakFlag && !broke)
                {
                    Break(world, body, ref piece2, shape2, cachedVelocity, cachedAngularVelocity);
                    broke = true;
                }
            }

            Assert.IsTrue(broke, "Breakable body should break on high impulse.");
            Assert.AreEqual(1, body.Fixtures.Count, "Original body should keep one fixture after breaking.");
            Assert.IsTrue(world.Bodies.Count >= 2, "Breaking should create a second body.");
        }

        private static void Break(World world, Body body, ref Fixture? piece2, PolygonShape shape2, Vec2 cachedVelocity, float cachedAngularVelocity)
        {
            if (piece2 == null)
            {
                return;
            }

            Vec2 center = body.GetWorldPoint(body.Sweep.LocalCenter);
            body.DestroyFixture(piece2);
            piece2 = null;

            Body body2 = world.CreateBody(new BodyDef().AsDynamic().At(body.Transform.P).WithAngle(body.Transform.Q.Angle));
            body2.CreateFixture(new FixtureDef(shape2).WithDensity(1f).WithUserData("piece2"));

            Vec2 center1 = body.GetWorldPoint(body.Sweep.LocalCenter);
            Vec2 center2 = body2.GetWorldPoint(body2.Sweep.LocalCenter);
            Vec2 diff1 = center1 - center;
            Vec2 diff2 = center2 - center;

            Vec2 velocity1 = cachedVelocity + Vec2.Cross(cachedAngularVelocity, diff1);
            Vec2 velocity2 = cachedVelocity + Vec2.Cross(cachedAngularVelocity, diff2);

            body.LinearVelocity = velocity1;
            body.AngularVelocity = cachedAngularVelocity;
            body2.LinearVelocity = velocity2;
            body2.AngularVelocity = cachedAngularVelocity;
        }

        private static Vec2[] BuildBoxVertices(float hx, float hy, Vec2 center, float angle)
        {
            Vec2[] verts =
            {
                new Vec2(-hx, -hy),
                new Vec2(hx, -hy),
                new Vec2(hx, hy),
                new Vec2(-hx, hy)
            };
            if (angle == 0f)
            {
                for (int i = 0; i < verts.Length; ++i)
                {
                    verts[i] = verts[i] + center;
                }
                return verts;
            }

            Rot rot = new Rot(angle);
            for (int i = 0; i < verts.Length; ++i)
            {
                verts[i] = Rot.Mul(rot, verts[i]) + center;
            }
            return verts;
        }
    }
}
