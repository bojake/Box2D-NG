using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;

namespace Box2DNG.Tests
{
    [TestClass]
    public class JointEdgeCaseTests
    {
        private static (World world, Body a, Body b) BuildTwoBodies(Vec2 anchorA, Vec2 anchorB)
        {
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)));
            Body a = world.CreateBody(new BodyDef().AsStatic().At(anchorA));
            a.CreateFixture(new FixtureDef(new CircleShape(0.1f)));
            Body b = world.CreateBody(new BodyDef().AsDynamic().At(anchorB));
            b.CreateFixture(new FixtureDef(new CircleShape(0.3f)).WithDensity(1f));
            return (world, a, b);
        }

        // ----- Prismatic -----

        [TestMethod]
        public void Prismatic_LimitClampsMotion()
        {
            (World w, Body anchor, Body cart) = BuildTwoBodies(new Vec2(0f, 5f), new Vec2(0.5f, 5f));
            PrismaticJoint j = w.CreateJoint(
                new PrismaticJointDef(anchor, cart, new Vec2(0f, 5f), new Vec2(1f, 0f))
                    .WithLimit(-1f, 1f));

            // Push the cart hard outside the upper limit.
            for (int i = 0; i < 60; ++i)
            {
                cart.ApplyForce(new Vec2(1000f, 0f));
                w.Step(1f / 60f);
            }

            float translation = j.GetTranslation();
            Assert.IsTrue(translation <= 1f + 0.05f, $"Cart should be clamped at upper limit, got {translation}");
        }

        [TestMethod]
        public void Prismatic_LowerLimitHoldsAgainstReverseForce()
        {
            (World w, Body anchor, Body cart) = BuildTwoBodies(new Vec2(0f, 5f), new Vec2(0.5f, 5f));
            PrismaticJoint j = w.CreateJoint(
                new PrismaticJointDef(anchor, cart, new Vec2(0f, 5f), new Vec2(1f, 0f))
                    .WithLimit(-1f, 1f));

            for (int i = 0; i < 60; ++i)
            {
                cart.ApplyForce(new Vec2(-1000f, 0f));
                w.Step(1f / 60f);
            }

            float translation = j.GetTranslation();
            Assert.IsTrue(translation >= -1f - 0.05f, $"Cart should be clamped at lower limit, got {translation}");
        }

        [TestMethod]
        public void Prismatic_Motor_DrivesTowardSpeed()
        {
            (World w, Body anchor, Body cart) = BuildTwoBodies(new Vec2(0f, 5f), new Vec2(0f, 5f));
            PrismaticJoint j = w.CreateJoint(
                new PrismaticJointDef(anchor, cart, new Vec2(0f, 5f), new Vec2(1f, 0f)));
            j.SetMotorEnabled(true);
            j.SetMaxMotorForce(50f);
            j.SetMotorSpeed(2f);

            float startX = cart.Transform.P.X;
            for (int i = 0; i < 120; ++i)
            {
                w.Step(1f / 60f);
            }

            // The motor should have moved the cart along the axis.
            Assert.IsTrue(cart.Transform.P.X > startX + 0.1f,
                $"Prismatic motor should drive cart along axis. dx={cart.Transform.P.X - startX}");
            Assert.IsTrue(j.GetSpeed() > 0.5f, $"Prismatic motor should drive positive speed, got {j.GetSpeed()}");
        }

        [TestMethod]
        public void Prismatic_SpringPullsTowardTarget()
        {
            (World w, Body anchor, Body cart) = BuildTwoBodies(new Vec2(0f, 5f), new Vec2(2f, 5f));
            PrismaticJoint j = w.CreateJoint(
                new PrismaticJointDef(anchor, cart, new Vec2(0f, 5f), new Vec2(1f, 0f)));
            j.EnableSpring = true;
            j.FrequencyHz = 4f;
            j.DampingRatio = 1f;
            j.TargetTranslation = 0f;

            for (int i = 0; i < 180; ++i)
            {
                w.Step(1f / 60f);
            }

            float trans = j.GetTranslation();
            Assert.IsTrue(MathF.Abs(trans) < 1.5f, $"Spring should pull toward 0, got {trans}");
        }

        // ----- Wheel -----

        [TestMethod]
        public void Wheel_LimitClampsTranslation()
        {
            World w = new World(new WorldDef().WithGravity(new Vec2(0f, -20f)));
            Body chassis = w.CreateBody(new BodyDef().AsStatic().At(0f, 5f));
            chassis.CreateFixture(new FixtureDef(new CircleShape(0.1f)));
            Body wheel = w.CreateBody(new BodyDef().AsDynamic().At(0f, 4f));
            wheel.CreateFixture(new FixtureDef(new CircleShape(0.3f)).WithDensity(2f));

            WheelJoint j = w.CreateJoint(
                new WheelJointDef(chassis, wheel, new Vec2(0f, 4f), new Vec2(0f, 1f))
                    .WithLimits(-0.5f, 0.5f)
                    .WithSpring(0f, 0f));

            for (int i = 0; i < 240; ++i)
            {
                w.Step(1f / 60f);
            }

            float dy = wheel.Transform.P.Y - 4f;
            Assert.IsTrue(dy >= -0.5f - 0.1f, $"Wheel should be clamped to lower limit, dy = {dy}");
        }

        [TestMethod]
        public void Wheel_Setters_RoundTrip()
        {
            World w = new World(new WorldDef().WithGravity(Vec2.Zero));
            Body a = w.CreateBody(new BodyDef().AsDynamic().At(0f, 0f));
            a.CreateFixture(new FixtureDef(new CircleShape(0.2f)).WithDensity(1f));
            Body b = w.CreateBody(new BodyDef().AsDynamic().At(1f, 0f));
            b.CreateFixture(new FixtureDef(new CircleShape(0.2f)).WithDensity(1f));

            WheelJoint j = w.CreateJoint(new WheelJointDef(a, b, new Vec2(0.5f, 0f), new Vec2(0f, 1f)));
            j.SetMotorEnabled(true);
            j.SetMotorSpeed(3f);
            j.SetMaxMotorTorque(7f);
            j.SetSpringFrequencyHz(5f);
            j.SetSpringDampingRatio(0.5f);
            j.SetLimitsEnabled(true);
            j.SetLimits(-1f, 2f);

            Assert.IsTrue(j.EnableMotor);
            Assert.AreEqual(3f, j.MotorSpeed);
            Assert.AreEqual(7f, j.MaxMotorTorque);
            Assert.AreEqual(5f, j.FrequencyHz);
            Assert.AreEqual(0.5f, j.DampingRatio);
            Assert.IsTrue(j.EnableLimit);
            Assert.AreEqual(-1f, j.LowerTranslation);
            Assert.AreEqual(2f, j.UpperTranslation);
        }

        // ----- Revolute -----

        [TestMethod]
        public void Revolute_LimitClampsAngle()
        {
            (World w, Body anchor, Body lever) = BuildTwoBodies(new Vec2(0f, 5f), new Vec2(1f, 5f));
            RevoluteJoint j = w.CreateJoint(
                new RevoluteJointDef(anchor, lever, new Vec2(0f, 5f))
                    .WithLimit(-0.5f, 0.5f));

            // Push lever to rotate beyond the upper limit.
            for (int i = 0; i < 120; ++i)
            {
                lever.ApplyTorque(50f);
                w.Step(1f / 60f);
            }

            float angle = j.GetJointAngle();
            Assert.IsTrue(angle <= 0.5f + 0.1f, $"Revolute should be clamped at upper limit, got {angle}");
        }

        [TestMethod]
        public void Revolute_MotorPushesAgainstLimit()
        {
            (World w, Body anchor, Body lever) = BuildTwoBodies(new Vec2(0f, 5f), new Vec2(1f, 5f));
            RevoluteJoint j = w.CreateJoint(
                new RevoluteJointDef(anchor, lever, new Vec2(0f, 5f))
                    .WithLimit(-0.2f, 0.2f)
                    .WithMotor(enable: true, speed: 10f, maxTorque: 100f));

            for (int i = 0; i < 60; ++i)
            {
                w.Step(1f / 60f);
            }
            Assert.IsTrue(j.GetJointAngle() <= 0.2f + 0.05f);
            Assert.IsTrue(MathF.Abs(j.GetMotorTorque(60f)) > 0f);
            Assert.IsTrue(MathF.Abs(j.GetReactionTorque(60f)) > 0f);
        }

        [TestMethod]
        public void Revolute_SetLimits_ResetsImpulses()
        {
            (World w, Body anchor, Body lever) = BuildTwoBodies(new Vec2(0f, 5f), new Vec2(1f, 5f));
            RevoluteJoint j = w.CreateJoint(
                new RevoluteJointDef(anchor, lever, new Vec2(0f, 5f))
                    .WithLimit(-0.5f, 0.5f));

            for (int i = 0; i < 30; ++i)
            {
                lever.ApplyTorque(50f);
                w.Step(1f / 60f);
            }
            // Reset limits to a wider range — should accept the new range and clear limit impulse.
            j.SetLimits(-1f, 1f);
            Assert.AreEqual(-1f, j.LowerAngle);
            Assert.AreEqual(1f, j.UpperAngle);
        }

        // ----- Distance -----

        [TestMethod]
        public void Distance_CurrentLength_TracksBodyMotion()
        {
            World w = new World(new WorldDef().WithGravity(Vec2.Zero));
            Body a = w.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            Body b = w.CreateBody(new BodyDef().AsDynamic().At(3f, 0f));
            b.CreateFixture(new FixtureDef(new CircleShape(0.2f)).WithDensity(1f));
            // Anchor at the body centers so local anchors are zero — moving b changes the joint length.
            DistanceJoint j = w.CreateJoint(new DistanceJointDef(a, b, new Vec2(0f, 0f), new Vec2(3f, 0f)));

            Assert.AreEqual(3f, j.GetCurrentLength(), 1e-4f);
            b.Position = new Vec2(5f, 0f);
            Assert.AreEqual(5f, j.GetCurrentLength(), 1e-4f);
        }
    }
}
