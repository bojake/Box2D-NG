using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;

namespace Box2DNG.Tests
{
    [TestClass]
    public class JointReactionTests
    {
        [TestMethod]
        public void DistanceJoint_ReactionForce_NonZeroUnderTension()
        {
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)));
            Body anchor = world.CreateBody(new BodyDef().AsStatic().At(0f, 5f));
            anchor.CreateFixture(new FixtureDef(new CircleShape(0.1f)));

            Body hanging = world.CreateBody(new BodyDef().AsDynamic().At(0f, 3f));
            hanging.CreateFixture(new FixtureDef(new CircleShape(0.3f)).WithDensity(1f));

            DistanceJoint j = world.CreateJoint(
                new DistanceJointDef(anchor, hanging, Vec2.Zero, Vec2.Zero).WithLength(2f));

            for (int i = 0; i < 60; ++i)
            {
                world.Step(1f / 60f);
            }

            float invDt = 60f;
            Vec2 force = j.GetReactionForce(invDt);
            Assert.IsTrue(force.Length > 0.5f, $"Expected non-trivial tension. force={force}");

            float currentLength = j.GetCurrentLength();
            Assert.IsTrue(MathF.Abs(currentLength - 2f) < 0.2f, $"Length should be near 2, got {currentLength}");
        }

        [TestMethod]
        public void RevoluteJoint_ReactionForce_ResistsGravity()
        {
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)));
            Body anchor = world.CreateBody(new BodyDef().AsStatic().At(0f, 5f));
            anchor.CreateFixture(new FixtureDef(new CircleShape(0.1f)));

            Body pendulum = world.CreateBody(new BodyDef().AsDynamic().At(0f, 3f));
            pendulum.CreateFixture(new FixtureDef(new CircleShape(0.3f)).WithDensity(1f));

            RevoluteJoint j = world.CreateJoint(
                new RevoluteJointDef(anchor, pendulum, new Vec2(0f, 5f)));

            for (int i = 0; i < 60; ++i)
            {
                world.Step(1f / 60f);
            }

            Vec2 force = j.GetReactionForce(60f);
            Assert.IsTrue(force.Length > 0.5f, $"Revolute joint reaction force expected. force={force}");
        }

        [TestMethod]
        public void WheelJoint_GetReactionForce_IsFinite()
        {
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)));
            Body chassis = world.CreateBody(new BodyDef().AsDynamic().At(0f, 3f));
            chassis.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));

            Body wheel = world.CreateBody(new BodyDef().AsDynamic().At(0f, 2f));
            wheel.CreateFixture(new FixtureDef(new CircleShape(0.3f)).WithDensity(1f));

            WheelJoint j = world.CreateJoint(
                new WheelJointDef(chassis, wheel, new Vec2(0f, 2f), new Vec2(0f, 1f)));

            world.Step(1f / 60f);

            Vec2 f = j.GetReactionForce(60f);
            Assert.IsFalse(float.IsNaN(f.X));
            Assert.IsFalse(float.IsNaN(f.Y));
            Assert.IsFalse(float.IsInfinity(f.X));
            Assert.IsFalse(float.IsInfinity(f.Y));
            float t = j.GetReactionTorque(60f);
            Assert.IsFalse(float.IsNaN(t));
        }

        [TestMethod]
        public void WeldJoint_ReactionTorque_FiniteUnderLoad()
        {
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)));
            Body a = world.CreateBody(new BodyDef().AsStatic().At(0f, 5f));
            a.CreateFixture(new FixtureDef(new CircleShape(0.2f)));

            Body b = world.CreateBody(new BodyDef().AsDynamic().At(1f, 5f));
            b.CreateFixture(new FixtureDef(new PolygonShape(new[]
            {
                new Vec2(-0.5f, -0.1f), new Vec2(0.5f, -0.1f),
                new Vec2(0.5f, 0.1f), new Vec2(-0.5f, 0.1f)
            })).WithDensity(1f));

            WeldJoint w = world.CreateJoint(
                new WeldJointDef(a, b, new Vec2(0f, 5f)));

            for (int i = 0; i < 60; ++i)
            {
                world.Step(1f / 60f);
            }

            float torque = w.GetReactionTorque(60f);
            Assert.IsFalse(float.IsNaN(torque));
            Vec2 force = w.GetReactionForce(60f);
            Assert.IsFalse(float.IsNaN(force.X));
        }

        [TestMethod]
        public void PrismaticJoint_ReactionForce_IsFinite()
        {
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)));
            Body a = world.CreateBody(new BodyDef().AsStatic().At(0f, 5f));
            a.CreateFixture(new FixtureDef(new CircleShape(0.2f)));
            Body b = world.CreateBody(new BodyDef().AsDynamic().At(0f, 4f));
            b.CreateFixture(new FixtureDef(new CircleShape(0.2f)).WithDensity(1f));

            PrismaticJoint j = world.CreateJoint(
                new PrismaticJointDef(a, b, new Vec2(0f, 5f), new Vec2(1f, 0f)));

            world.Step(1f / 60f);
            Vec2 f = j.GetReactionForce(60f);
            Assert.IsFalse(float.IsNaN(f.X));
            float t = j.GetReactionTorque(60f);
            Assert.IsFalse(float.IsNaN(t));
        }

        [TestMethod]
        public void FrictionAndMotorJoints_ReactionFinite()
        {
            World world = new World(new WorldDef().WithGravity(Vec2.Zero));
            Body a = world.CreateBody(new BodyDef().AsDynamic().At(0f, 0f));
            a.CreateFixture(new FixtureDef(new CircleShape(0.3f)).WithDensity(1f));
            Body b = world.CreateBody(new BodyDef().AsDynamic().At(1f, 0f));
            b.CreateFixture(new FixtureDef(new CircleShape(0.3f)).WithDensity(1f));

            FrictionJoint friction = world.CreateJoint(
                new FrictionJointDef(a, b, new Vec2(0.5f, 0f))
                    .WithMaxForce(10f)
                    .WithMaxTorque(5f));
            MotorJoint motor = world.CreateJoint(
                new MotorJointDef(a, b)
                    .WithLinearOffset(new Vec2(0.5f, 0f))
                    .WithMaxForce(5f)
                    .WithMaxTorque(2f));

            for (int i = 0; i < 30; ++i)
            {
                world.Step(1f / 60f);
            }

            Vec2 ff = friction.GetReactionForce(60f);
            float ft = friction.GetReactionTorque(60f);
            Vec2 mf = motor.GetReactionForce(60f);
            float mt = motor.GetReactionTorque(60f);

            Assert.IsFalse(float.IsNaN(ff.X) || float.IsNaN(ft) || float.IsNaN(mf.X) || float.IsNaN(mt));
        }
    }
}
