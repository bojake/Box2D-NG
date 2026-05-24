using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Box2DNG.Tests
{
    /// <summary>
    /// Phase 1: pin the soft revolute behaviour. Revolute only has a single
    /// soft axis (the point-to-point constraint) — angular motor/limit retain
    /// their existing velocity-drive / one-sided clamp semantics.
    /// </summary>
    [TestClass]
    public class SoftRevoluteJointTests
    {
        [TestMethod]
        public void SoftRevolute_RigidByDefault_AnchorsTrack()
        {
            // Two bodies pinned at the world origin. Without per-joint Hertz
            // tuning, the rigid path keeps the anchors locked together.
            World world = new World(new WorldDef().WithGravity(Vec2.Zero));
            Body a = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            Body b = world.CreateBody(new BodyDef().AsDynamic().At(2f, 0f));
            b.CreateFixture(new FixtureDef(new CircleShape(0.3f)).WithDensity(1f));
            world.CreateJoint(new RevoluteJointDef(a, b, new Vec2(0f, 0f)));

            b.LinearVelocity = new Vec2(0f, 5f);
            for (int i = 0; i < 30; ++i) world.Step(1f / 60f);

            // After half a second, body B should still orbit the anchor — its
            // anchor offset from world origin should stay near 2.
            float r = b.Transform.P.Length;
            Assert.IsTrue(MathF.Abs(r - 2f) < 0.2f,
                $"Rigid revolute should hold the anchor radius. r={r}");
        }

        [TestMethod]
        public void SoftRevolute_LowHertz_AllowsAnchorDrift()
        {
            // Soft point-to-point — body B can drift away from its anchor
            // before the spring pulls it back. Lower Hertz allows more drift.
            World world = new World(new WorldDef().WithGravity(Vec2.Zero));
            Body a = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            Body b = world.CreateBody(new BodyDef().AsDynamic().At(2f, 0f));
            b.CreateFixture(new FixtureDef(new CircleShape(0.3f)).WithDensity(1f));
            world.CreateJoint(new RevoluteJointDef(a, b, new Vec2(0f, 0f))
                .WithLinearSpring(1f, 0.1f));   // very soft, lightly damped

            b.LinearVelocity = new Vec2(5f, 0f);  // straight away from anchor
            for (int i = 0; i < 6; ++i) world.Step(1f / 60f);

            // After 0.1 s the spring hasn't fully restored — body B has drifted.
            float r = b.Transform.P.Length;
            Assert.IsTrue(r > 2.2f, $"Soft revolute should allow drift. r={r}");
        }

        [TestMethod]
        public void SoftRevolute_AllowsSpinInPlace_LikeRigid()
        {
            // The soft linear spring shouldn't fight free rotation. With the
            // anchor at body B's center (no offset), B can spin in place
            // without producing any anchor-point velocity for the joint to
            // resist. The spring stays inactive (C=0, Cdot=0) and angular
            // velocity is preserved.
            World world = new World(new WorldDef().WithGravity(Vec2.Zero));
            Body a = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            Body b = world.CreateBody(new BodyDef().AsDynamic().At(0f, 0f));
            b.CreateFixture(new FixtureDef(new CircleShape(0.3f)).WithDensity(1f));
            world.CreateJoint(new RevoluteJointDef(a, b, new Vec2(0f, 0f))
                .WithLinearSpring(60f, 0.7f));

            b.AngularVelocity = 5f;
            for (int i = 0; i < 60; ++i) world.Step(1f / 60f);
            Assert.IsTrue(MathF.Abs(b.AngularVelocity) > 4.9f,
                $"Soft revolute shouldn't damp pure spin. w={b.AngularVelocity}");
        }
    }
}
