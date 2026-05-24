using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;

namespace Box2DNG.Tests
{
    [TestClass]
    public class JointDefAndWrapperTests
    {
        private static World World() => new World(new WorldDef());

        private static (Body a, Body b) Pair(World w, Vec2 posA, Vec2 posB)
        {
            Body a = w.CreateBody(new BodyDef().AsDynamic().At(posA));
            a.CreateFixture(new FixtureDef(new CircleShape(0.2f)).WithDensity(1f));
            Body b = w.CreateBody(new BodyDef().AsDynamic().At(posB));
            b.CreateFixture(new FixtureDef(new CircleShape(0.2f)).WithDensity(1f));
            return (a, b);
        }

        // ----- DistanceJointDef builders -----

        [TestMethod]
        public void DistanceJointDef_BuildersChainAndClamp()
        {
            World w = World();
            var (a, b) = Pair(w, Vec2.Zero, new Vec2(2f, 0f));
            DistanceJointDef def = new DistanceJointDef(a, b, Vec2.Zero, new Vec2(2f, 0f))
                .WithLength(3f)
                .WithFrequency(5f)
                .WithDampingRatio(0.5f)
                .WithCollideConnected(true);
            Assert.AreEqual(3f, def.Length);
            Assert.AreEqual(5f, def.FrequencyHz);
            Assert.AreEqual(0.5f, def.DampingRatio);
            Assert.IsTrue(def.CollideConnected);
            Assert.AreEqual(a, def.BodyA);
            Assert.AreEqual(b, def.BodyB);
        }

        // ----- FrictionJointDef builders + wrapper round-trip -----

        [TestMethod]
        public void FrictionJointDef_BuildersChain()
        {
            World w = World();
            var (a, b) = Pair(w, Vec2.Zero, new Vec2(1f, 0f));
            FrictionJointDef def = new FrictionJointDef(a, b, new Vec2(0.5f, 0f))
                .WithMaxForce(3f)
                .WithMaxTorque(2f)
                .WithCollideConnected(true);
            Assert.AreEqual(3f, def.MaxForce);
            Assert.AreEqual(2f, def.MaxTorque);
            Assert.IsTrue(def.CollideConnected);
            // Negative values clamp to zero.
            FrictionJointDef neg = new FrictionJointDef(a, b, Vec2.Zero).WithMaxForce(-5f).WithMaxTorque(-1f);
            Assert.AreEqual(0f, neg.MaxForce);
            Assert.AreEqual(0f, neg.MaxTorque);
        }

        [TestMethod]
        public void FrictionJoint_SettersClampToZero()
        {
            World w = World();
            var (a, b) = Pair(w, Vec2.Zero, new Vec2(1f, 0f));
            FrictionJoint j = w.CreateJoint(new FrictionJointDef(a, b, new Vec2(0.5f, 0f))
                .WithMaxForce(10f).WithMaxTorque(5f));
            Assert.AreEqual(10f, j.MaxForce);
            Assert.AreEqual(5f, j.MaxTorque);
            j.SetMaxForce(-3f);
            j.SetMaxTorque(-2f);
            Assert.AreEqual(0f, j.MaxForce);
            Assert.AreEqual(0f, j.MaxTorque);
            Assert.IsFalse(j.CollideConnected);
            Assert.AreSame(a, j.BodyA);
            Assert.AreSame(b, j.BodyB);
            // Body A is at origin, world anchor (0.5,0) -> local (0.5,0).
            Assert.AreEqual(new Vec2(0.5f, 0f), j.LocalAnchorA);
        }

        // ----- MotorJointDef builders + wrapper -----

        [TestMethod]
        public void MotorJointDef_BuildersChainAndClamp()
        {
            World w = World();
            var (a, b) = Pair(w, Vec2.Zero, new Vec2(1f, 0f));
            MotorJointDef def = new MotorJointDef(a, b)
                .WithLinearOffset(new Vec2(3f, 4f))
                .WithAngularOffset(0.5f)
                .WithMaxForce(10f)
                .WithMaxTorque(5f)
                .WithCorrectionFactor(0.7f)
                .WithCollideConnected(true);
            Assert.AreEqual(new Vec2(3f, 4f), def.LinearOffset);
            Assert.AreEqual(0.5f, def.AngularOffset);
            Assert.AreEqual(10f, def.MaxForce);
            Assert.AreEqual(5f, def.MaxTorque);
            Assert.AreEqual(0.7f, def.CorrectionFactor);
            Assert.IsTrue(def.CollideConnected);
            // CorrectionFactor clamps to [0,1]
            MotorJointDef clampedHi = new MotorJointDef(a, b).WithCorrectionFactor(5f);
            Assert.AreEqual(1f, clampedHi.CorrectionFactor);
            MotorJointDef clampedLo = new MotorJointDef(a, b).WithCorrectionFactor(-1f);
            Assert.AreEqual(0f, clampedLo.CorrectionFactor);
        }

        [TestMethod]
        public void MotorJoint_GettersExposeData()
        {
            World w = World();
            var (a, b) = Pair(w, Vec2.Zero, new Vec2(1f, 0f));
            MotorJoint j = w.CreateJoint(new MotorJointDef(a, b)
                .WithLinearOffset(new Vec2(2f, 3f))
                .WithAngularOffset(0.4f)
                .WithMaxForce(8f).WithMaxTorque(4f)
                .WithCorrectionFactor(0.2f));
            Assert.AreEqual(new Vec2(2f, 3f), j.LinearOffset);
            Assert.AreEqual(0.4f, j.AngularOffset);
            Assert.AreEqual(8f, j.MaxForce);
            Assert.AreEqual(4f, j.MaxTorque);
            Assert.AreEqual(0.2f, j.CorrectionFactor);
            Assert.IsFalse(j.CollideConnected);
        }

        // ----- WeldJointDef + wrapper -----

        [TestMethod]
        public void WeldJointDef_BuildersAndCollideConnected()
        {
            World w = World();
            var (a, b) = Pair(w, Vec2.Zero, new Vec2(1f, 0f));
            WeldJointDef def = new WeldJointDef(a, b, new Vec2(0.5f, 0f)).WithCollideConnected(true);
            Assert.IsTrue(def.CollideConnected);
            WeldJoint j = w.CreateJoint(def);
            Assert.IsTrue(j.CollideConnected);
            Assert.AreEqual(0f, j.ReferenceAngle, 1e-5f);
            Assert.AreSame(a, j.BodyA);
        }

        // ----- PulleyJointDef + wrapper -----

        [TestMethod]
        public void PulleyJointDef_ClampsRatio()
        {
            World w = World();
            var (a, b) = Pair(w, new Vec2(-2f, 0f), new Vec2(2f, 0f));
            PulleyJointDef def = new PulleyJointDef(a, b,
                new Vec2(-2f, 5f), new Vec2(2f, 5f),
                new Vec2(-2f, 0f), new Vec2(2f, 0f),
                ratio: 2f);
            Assert.AreEqual(2f, def.Ratio);
            def.WithRatio(0f);
            Assert.IsTrue(def.Ratio >= Constants.Epsilon, $"Ratio not epsilon-clamped: {def.Ratio}");
            def.WithCollideConnected(true);
            Assert.IsTrue(def.CollideConnected);

            PulleyJoint j = w.CreateJoint(def);
            Assert.AreEqual(def.Ratio, j.Ratio);
            Assert.AreEqual(5f, j.LengthA, 1e-3f);
            Assert.AreEqual(5f, j.LengthB, 1e-3f);
            Assert.AreEqual(new Vec2(-2f, 5f), j.GroundAnchorA);
            Assert.AreEqual(new Vec2(2f, 5f), j.GroundAnchorB);
            Assert.IsTrue(j.CollideConnected);
        }

        // ----- RopeJointDef + wrapper -----

        [TestMethod]
        public void RopeJointDef_WithMaxLengthAndCollideConnected()
        {
            World w = World();
            var (a, b) = Pair(w, Vec2.Zero, new Vec2(2f, 0f));
            RopeJointDef def = new RopeJointDef(a, b, Vec2.Zero, new Vec2(2f, 0f))
                .WithMaxLength(5f)
                .WithCollideConnected(true);
            Assert.AreEqual(5f, def.MaxLength);
            Assert.IsTrue(def.CollideConnected);
            // WithMaxLength clamps to slop.
            def.WithMaxLength(0f);
            Assert.AreEqual(Constants.LinearSlop, def.MaxLength);

            // Restore a usable length, then test wrapper setter.
            def.WithMaxLength(5f);
            RopeJoint j = w.CreateJoint(def);
            Assert.AreEqual(5f, j.MaxLength);
            j.SetMaxLength(7f);
            Assert.AreEqual(7f, j.MaxLength);
            // Setting below slop clamps up.
            j.SetMaxLength(0f);
            Assert.AreEqual(Constants.LinearSlop, j.MaxLength);
        }

        // ----- GearJointDef -----

        [TestMethod]
        public void GearJointDef_BuildersChain()
        {
            World w = World();
            var (a1, b1) = Pair(w, Vec2.Zero, new Vec2(1f, 0f));
            var (a2, b2) = Pair(w, new Vec2(3f, 0f), new Vec2(4f, 0f));
            RevoluteJoint jA = w.CreateJoint(new RevoluteJointDef(a1, b1, new Vec2(0.5f, 0f)));
            RevoluteJoint jB = w.CreateJoint(new RevoluteJointDef(a2, b2, new Vec2(3.5f, 0f)));

            GearJointDef def = new GearJointDef(jA, jB, 2f)
                .WithRatio(0.5f)
                .WithCollideConnected(true);
            Assert.AreEqual(0.5f, def.Ratio);
            Assert.IsTrue(def.CollideConnected);
            Assert.AreSame(jA, def.JointA);
            Assert.AreSame(jB, def.JointB);
        }

        // ----- WheelJointDef -----

        [TestMethod]
        public void WheelJointDef_LimitsOrderedAndCollideConnected()
        {
            World w = World();
            var (a, b) = Pair(w, Vec2.Zero, new Vec2(0f, -1f));
            WheelJointDef def = new WheelJointDef(a, b, new Vec2(0f, -0.5f), new Vec2(0f, 1f))
                .WithMotor(true, 3f, 5f)
                .WithSpring(4f, 0.8f)
                .WithLimits(2f, -2f) // unordered — should be sorted
                .WithCollideConnected(true);
            Assert.IsTrue(def.EnableMotor);
            Assert.AreEqual(3f, def.MotorSpeed);
            Assert.AreEqual(5f, def.MaxMotorTorque);
            Assert.AreEqual(4f, def.FrequencyHz);
            Assert.AreEqual(0.8f, def.DampingRatio);
            Assert.AreEqual(-2f, def.LowerTranslation);
            Assert.AreEqual(2f, def.UpperTranslation);
            Assert.IsTrue(def.EnableLimit);
            Assert.IsTrue(def.CollideConnected);
        }

        // ----- RevoluteJointDef -----

        [TestMethod]
        public void RevoluteJointDef_LimitsOrderedAndCollideConnected()
        {
            World w = World();
            var (a, b) = Pair(w, Vec2.Zero, new Vec2(1f, 0f));
            RevoluteJointDef def = new RevoluteJointDef(a, b, new Vec2(0.5f, 0f))
                .WithMotor(true, 7f, 3f)
                .WithLimit(1f, -1f) // unordered
                .WithCollideConnected(true);
            Assert.IsTrue(def.EnableMotor);
            Assert.AreEqual(7f, def.MotorSpeed);
            Assert.AreEqual(3f, def.MaxMotorTorque);
            Assert.AreEqual(-1f, def.LowerAngle);
            Assert.AreEqual(1f, def.UpperAngle);
            Assert.IsTrue(def.EnableLimit);
            Assert.IsTrue(def.CollideConnected);
        }

        // ----- PrismaticJointDef -----

        [TestMethod]
        public void PrismaticJointDef_BuildersChainAllOptions()
        {
            World w = World();
            var (a, b) = Pair(w, Vec2.Zero, new Vec2(2f, 0f));
            PrismaticJointDef def = new PrismaticJointDef(a, b, Vec2.Zero, new Vec2(1f, 0f))
                .WithMotor(2f, 10f)
                .WithLimit(-3f, 3f)
                .WithSpring(5f, 0.6f, 0f)
                .WithTargetTranslation(0.5f)
                .WithConstraintTuning(2f, 0.9f)
                .WithCollideConnected(true);
            Assert.IsTrue(def.EnableMotor);
            Assert.AreEqual(2f, def.MotorSpeed);
            Assert.AreEqual(10f, def.MaxMotorForce);
            Assert.IsTrue(def.EnableLimit);
            Assert.AreEqual(-3f, def.LowerTranslation);
            Assert.AreEqual(3f, def.UpperTranslation);
            Assert.IsTrue(def.EnableSpring);
            Assert.AreEqual(5f, def.FrequencyHz);
            Assert.AreEqual(0.6f, def.DampingRatio);
            Assert.AreEqual(0.5f, def.TargetTranslation);
            Assert.AreEqual(2f, def.ConstraintHertz);
            Assert.AreEqual(0.9f, def.ConstraintDampingRatio);
            Assert.IsTrue(def.CollideConnected);
        }

        // ----- DistanceJoint wrapper setters -----

        [TestMethod]
        public void DistanceJoint_Setters_RoundTrip()
        {
            World w = World();
            var (a, b) = Pair(w, Vec2.Zero, new Vec2(2f, 0f));
            DistanceJoint j = w.CreateJoint(new DistanceJointDef(a, b, Vec2.Zero, new Vec2(2f, 0f)).WithLength(2f));
            j.Length = 3f;
            j.FrequencyHz = 5f;
            j.DampingRatio = 0.7f;
            Assert.AreEqual(3f, j.Length);
            Assert.AreEqual(5f, j.FrequencyHz);
            Assert.AreEqual(0.7f, j.DampingRatio);
            Assert.AreEqual(Vec2.Zero, j.LocalAnchorA);
            Assert.IsFalse(j.CollideConnected);
        }

        // ----- WheelJoint wrapper full setter coverage -----

        [TestMethod]
        public void WheelJoint_AllSetters_RoundTrip()
        {
            World w = World();
            var (a, b) = Pair(w, Vec2.Zero, new Vec2(0f, -1f));
            WheelJoint j = w.CreateJoint(new WheelJointDef(a, b, new Vec2(0f, -0.5f), new Vec2(0f, 1f)));
            j.SetMotorEnabled(true);
            j.SetMotorSpeed(4f);
            j.SetMaxMotorTorque(9f);
            j.SetSpringFrequencyHz(6f);
            j.SetSpringDampingRatio(0.3f);
            j.SetLimitsEnabled(true);
            j.SetLimits(-1.5f, 2.5f);
            Assert.IsTrue(j.EnableMotor);
            Assert.AreEqual(4f, j.MotorSpeed);
            Assert.AreEqual(9f, j.MaxMotorTorque);
            Assert.AreEqual(6f, j.FrequencyHz);
            Assert.AreEqual(0.3f, j.DampingRatio);
            Assert.IsTrue(j.EnableLimit);
            Assert.AreEqual(-1.5f, j.LowerTranslation);
            Assert.AreEqual(2.5f, j.UpperTranslation);
            Assert.IsFalse(j.CollideConnected);
            // Negative values clamp.
            j.SetMaxMotorTorque(-1f);
            j.SetSpringFrequencyHz(-2f);
            j.SetSpringDampingRatio(-3f);
            Assert.AreEqual(0f, j.MaxMotorTorque);
            Assert.AreEqual(0f, j.FrequencyHz);
            Assert.AreEqual(0f, j.DampingRatio);
        }

        // ----- RevoluteJoint wrapper full setter coverage -----

        [TestMethod]
        public void RevoluteJoint_AllSetters_RoundTrip()
        {
            World w = World();
            var (a, b) = Pair(w, Vec2.Zero, new Vec2(1f, 0f));
            RevoluteJoint j = w.CreateJoint(new RevoluteJointDef(a, b, new Vec2(0.5f, 0f)));
            j.EnableMotor = true;
            j.MotorSpeed = 7f;
            j.MaxMotorTorque = 12f;
            j.EnableLimit = true;
            j.SetLimits(-1f, 1f);
            Assert.IsTrue(j.EnableMotor);
            Assert.AreEqual(7f, j.MotorSpeed);
            Assert.AreEqual(12f, j.MaxMotorTorque);
            Assert.IsTrue(j.EnableLimit);
            Assert.AreEqual(-1f, j.LowerAngle);
            Assert.AreEqual(1f, j.UpperAngle);
            Assert.AreEqual(0f, j.ReferenceAngle, 1e-5f);

            // Manually wake — these setters call SetAwake(true).
            j.SetMotorSpeed(0f);
            j.SetMotorEnabled(false);
            Assert.IsFalse(j.EnableMotor);
        }

        // ----- PrismaticJoint wrapper -----

        [TestMethod]
        public void PrismaticJoint_AllSetters_RoundTrip()
        {
            World w = World();
            var (a, b) = Pair(w, Vec2.Zero, new Vec2(2f, 0f));
            PrismaticJoint j = w.CreateJoint(
                new PrismaticJointDef(a, b, Vec2.Zero, new Vec2(1f, 0f)));
            j.SetSpringEnabled(true);
            j.SetSpringFrequencyHz(4f);
            j.SetSpringDampingRatio(0.6f);
            j.SetTargetTranslation(0.5f);
            j.SetMotorEnabled(true);
            j.SetMotorSpeed(2f);
            j.SetMaxMotorForce(15f);
            j.SetLimitEnabled(true);
            j.SetLimits(-2f, 2f);
            j.SetConstraintTuning(3f, 0.8f);

            Assert.IsTrue(j.EnableSpring);
            Assert.AreEqual(4f, j.FrequencyHz);
            Assert.AreEqual(0.6f, j.DampingRatio);
            Assert.AreEqual(0.5f, j.TargetTranslation);
            Assert.IsTrue(j.EnableMotor);
            Assert.AreEqual(2f, j.MotorSpeed);
            Assert.AreEqual(15f, j.MaxMotorForce);
            Assert.IsTrue(j.EnableLimit);
            Assert.AreEqual(-2f, j.LowerTranslation);
            Assert.AreEqual(2f, j.UpperTranslation);
            Assert.AreEqual(3f, j.ConstraintHertz);
            Assert.AreEqual(0.8f, j.ConstraintDampingRatio);

            // Setting individual limits resets corresponding impulses.
            j.LowerTranslation = -3f;
            j.UpperTranslation = 3f;
            Assert.AreEqual(-3f, j.LowerTranslation);
            Assert.AreEqual(3f, j.UpperTranslation);
            Assert.AreEqual(0f, j.ReferenceAngle, 1e-5f);
            Assert.IsFalse(j.CollideConnected);
        }
    }
}
