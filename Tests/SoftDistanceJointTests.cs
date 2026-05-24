using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Box2DNG.Tests
{
    /// <summary>
    /// Phase 1: Distance joint already used a soft-spring formulation under
    /// FrequencyHz/DampingRatio. Unification routes the spring through the
    /// shared <see cref="Softness"/> path so the world's JointHertz default
    /// participates when the per-joint FrequencyHz is 0.
    /// </summary>
    [TestClass]
    public class SoftDistanceJointTests
    {
        [TestMethod]
        public void DistanceJoint_RigidByDefault_HoldsLengthExactly()
        {
            // No spring → legacy hard constraint. The two bodies stay at their
            // rest separation.
            World world = new World(new WorldDef().WithGravity(Vec2.Zero));
            Body a = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            Body b = world.CreateBody(new BodyDef().AsDynamic().At(3f, 0f));
            b.CreateFixture(new FixtureDef(new CircleShape(0.3f)).WithDensity(1f));
            world.CreateJoint(new DistanceJointDef(a, b, Vec2.Zero, new Vec2(3f, 0f)));

            // Pull body B with a hard impulse.
            b.LinearVelocity = new Vec2(10f, 0f);
            for (int i = 0; i < 30; ++i) world.Step(1f / 60f);

            float r = b.Transform.P.Length;
            Assert.IsTrue(MathF.Abs(r - 3f) < 0.1f,
                $"Rigid distance should hold length 3. r={r}");
        }

        [TestMethod]
        public void DistanceJoint_SoftSpring_AllowsStretchAndOscillation()
        {
            // Soft spring → body B can stretch the joint then swing back.
            World world = new World(new WorldDef().WithGravity(Vec2.Zero));
            Body a = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            Body b = world.CreateBody(new BodyDef().AsDynamic().At(3f, 0f));
            b.CreateFixture(new FixtureDef(new CircleShape(0.3f)).WithDensity(1f));
            DistanceJointDef def = new DistanceJointDef(a, b, Vec2.Zero, new Vec2(3f, 0f))
                .WithFrequency(2f, 0.1f);
            world.CreateJoint(def);

            b.LinearVelocity = new Vec2(10f, 0f);
            float maxRadius = 0f;
            for (int i = 0; i < 60; ++i)
            {
                world.Step(1f / 60f);
                float r = b.Transform.P.Length;
                if (r > maxRadius) maxRadius = r;
            }
            Assert.IsTrue(maxRadius > 3.5f,
                $"Soft distance should permit stretch. maxRadius={maxRadius}");
        }

        [TestMethod]
        public void DistanceJoint_WorldDefaultHertz_AppliedWhenJointFrequencyIsZero()
        {
            // No per-joint spring tuning; the world's JointHertz default should
            // make the constraint behave as a soft spring instead of rigid.
            World world = new World(new WorldDef()
                .WithGravity(Vec2.Zero)
                .WithJointHertz(2f)
                .WithJointDampingRatio(0.1f));
            Body a = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            Body b = world.CreateBody(new BodyDef().AsDynamic().At(3f, 0f));
            b.CreateFixture(new FixtureDef(new CircleShape(0.3f)).WithDensity(1f));
            world.CreateJoint(new DistanceJointDef(a, b, Vec2.Zero, new Vec2(3f, 0f)));

            b.LinearVelocity = new Vec2(10f, 0f);
            float maxRadius = 0f;
            for (int i = 0; i < 60; ++i)
            {
                world.Step(1f / 60f);
                float r = b.Transform.P.Length;
                if (r > maxRadius) maxRadius = r;
            }
            Assert.IsTrue(maxRadius > 3.5f,
                $"World JointHertz default should soften distance. maxRadius={maxRadius}");
        }
    }
}
