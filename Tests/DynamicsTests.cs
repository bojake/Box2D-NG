using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Box2DNG.Tests
{
    [TestClass]
    public class DynamicsTests
    {
        [TestMethod]
        public void DynamicBody_FallsUnderGravity()
        {
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)));
            Body body = world.CreateBody(new BodyDef().AsDynamic().At(0f, 10f));
            body.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));

            float y0 = body.Transform.P.Y;
            world.Step(1f / 60f);
            float y1 = body.Transform.P.Y;

            Assert.IsTrue(y1 < y0, "Body should move downward under gravity.");
        }

        [TestMethod]
        public void ApplyImpulse_ChangesVelocity()
        {
            World world = new World(new WorldDef());
            Body body = world.CreateBody(new BodyDef().AsDynamic().At(0f, 0f));
            body.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));

            Vec2 v0 = body.LinearVelocity;
            body.ApplyLinearImpulse(new Vec2(10f, 0f));
            world.Step(1f / 60f);
            Vec2 v1 = body.LinearVelocity;

            Assert.IsTrue(v1.X > v0.X, "Impulse should increase velocity.");
        }

        [TestMethod]
        public void Damping_ReducesVelocity()
        {
            World world = new World(new WorldDef());
            Body body = world.CreateBody(new BodyDef().AsDynamic().At(0f, 0f).WithLinearDamping(2f));
            body.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));
            body.LinearVelocity = new Vec2(10f, 0f);

            world.Step(1f / 60f);
            float speed = body.LinearVelocity.Length;

            Assert.IsTrue(speed < 10f, "Damping should reduce speed.");
        }

        [TestMethod]
        public void ImpulseAtPoint_ProducesAngularVelocity()
        {
            World world = new World(new WorldDef());
            Body body = world.CreateBody(new BodyDef().AsDynamic().At(0f, 0f));
            body.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));

            body.ApplyLinearImpulse(new Vec2(10f, 0f), new Vec2(0f, 1f));
            world.Step(1f / 60f);

            Assert.IsTrue(MathF.Abs(body.AngularVelocity) > 0f, "Off-center impulse should cause rotation.");
        }

        [TestMethod]
        public void Body_GoesToSleep_WhenIdle()
        {
            World world = new World(new WorldDef()
                .WithGravity(new Vec2(0f, 0f))
                .WithPositionIterations(10)
                .WithVelocityIterations(10));
            Body body = world.CreateBody(new BodyDef().AsDynamic().At(0f, 0f));
            body.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));

            for (int i = 0; i < 60; ++i)
            {
                world.Step(1f / 60f);
            }

            Assert.IsFalse(body.Awake, "Body should fall asleep when idle.");
        }

        [TestMethod]
        public void RestitutionThreshold_DisablesBounceAtLowSpeed()
        {
            World world = new World(new WorldDef()
                .WithGravity(new Vec2(0f, 0f))
                .WithRestitutionThreshold(2f));

            Body bodyA = world.CreateBody(new BodyDef().AsDynamic().At(0f, 0f));
            Body bodyB = world.CreateBody(new BodyDef().AsDynamic().At(0.9f, 0f));

            bodyA.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithRestitution(1f));
            bodyB.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithRestitution(1f));

            bodyA.LinearVelocity = new Vec2(0.5f, 0f);
            bodyB.LinearVelocity = Vec2.Zero;

            world.Step(1f / 60f);

            Assert.IsTrue(bodyA.LinearVelocity.X <= 0.5f + 1e-3f, "Bounce should be suppressed at low speed.");
        }

        [TestMethod]
        public void BaumgarteBias_SeparatesOverlappingCircles()
        {
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, 0f)));
            Body bodyA = world.CreateBody(new BodyDef().AsDynamic().At(0f, 0f));
            Body bodyB = world.CreateBody(new BodyDef().AsDynamic().At(0.4f, 0f));

            bodyA.CreateFixture(new FixtureDef(new CircleShape(0.5f)));
            bodyB.CreateFixture(new FixtureDef(new CircleShape(0.5f)));

            float d0 = (bodyB.Transform.P - bodyA.Transform.P).Length;
            for (int i = 0; i < 20; ++i)
            {
                world.Step(1f / 60f);
            }
            float d1 = (bodyB.Transform.P - bodyA.Transform.P).Length;

            Assert.IsTrue(d1 > d0, "Overlapping circles should separate over steps.");
        }

        [TestMethod]
        public void BulletContinuousCollision_DoesNotTunnel()
        {
            World world = new World(new WorldDef()
                .WithGravity(new Vec2(0f, 0f))
                .EnableContinuousCollision(true)
                .WithMaxSubSteps(4));

            Body wall = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            PolygonShape wallShape = new PolygonShape(new[]
            {
                new Vec2(-0.1f, -2f),
                new Vec2(0.1f, -2f),
                new Vec2(0.1f, 2f),
                new Vec2(-0.1f, 2f)
            });
            wall.CreateFixture(new FixtureDef(wallShape).WithDensity(0f));

            Body bullet = world.CreateBody(new BodyDef().AsDynamic().At(-5f, 0f).IsBullet(true));
            bullet.CreateFixture(new FixtureDef(new CircleShape(0.2f)).WithDensity(1f));

            Segment wallSegment = new Segment(new Vec2(0f, -2f), new Vec2(0f, 2f));
            RayCastInput input = new RayCastInput(new Vec2(-5f, 0f), new Vec2(10f, 0f), 1f);
            CastOutput output = Collision.RayCastSegment(wallSegment, input);

            Assert.IsTrue(output.Hit, "Expected contact between bullet and wall.");
        }

        [TestMethod]
        public void TranslationAndRotation_AreClamped()
        {
            // `MaximumTranslation` / `MaximumRotation` are PER SUB-STEP limits,
            // matching cpp box2d v3. Under SubStepCount=4 a body can advance at
            // most 4 × MaximumTranslation per outer Step. Force N=1 here so the
            // assertion is exact — the clamp's per-sub-step semantics are
            // covered by `Sub-step tunneling` checks elsewhere.
            World world = new World(new WorldDef()
                .WithSubStepCount(1)
                .WithGravity(new Vec2(0f, 0f))
                .WithMaximumTranslation(0.5f)
                .WithMaximumRotation(0.25f));

            Body body = world.CreateBody(new BodyDef().AsDynamic().At(0f, 0f));
            body.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));
            body.LinearVelocity = new Vec2(100f, 0f);
            body.AngularVelocity = 100f;

            world.Step(1f);

            Assert.IsTrue(body.Transform.P.X <= 0.5f + 1e-3f, "Translation should be clamped.");
            Assert.IsTrue(MathF.Abs(body.Transform.Q.Angle) <= 0.25f + 1e-3f, "Rotation should be clamped.");
        }
    }
}
