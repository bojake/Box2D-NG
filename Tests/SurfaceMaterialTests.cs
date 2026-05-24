using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;

namespace Box2DNG.Tests
{
    [TestClass]
    public class SurfaceMaterialTests
    {
        private static (World world, Body box) BuildBoxOnGround(float tangentSpeed, float rollingResistance)
        {
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)));

            Body ground = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            ground.CreateFixture(new FixtureDef(new SegmentShape(new Vec2(-40f, 0f), new Vec2(40f, 0f)))
                .WithFriction(1.0f)
                .WithTangentSpeed(tangentSpeed)
                .WithRollingResistance(rollingResistance));

            Body box = world.CreateBody(new BodyDef().AsDynamic().At(0f, 0.6f));
            box.CreateFixture(new FixtureDef(new CircleShape(0.5f))
                .WithDensity(1f)
                .WithFriction(1.0f)
                .WithRollingResistance(rollingResistance));

            return (world, box);
        }

        [TestMethod]
        public void TangentSpeed_DragsBoxAlongConveyor()
        {
            (World world, Body box) = BuildBoxOnGround(tangentSpeed: 5f, rollingResistance: 0f);
            // Settle first to ensure stable contact.
            for (int s = 0; s < 60; ++s) world.Step(1f / 60f);
            float startX = box.Transform.P.X;
            for (int i = 0; i < 240; ++i)
            {
                world.Step(1f / 60f);
            }
            System.Console.WriteLine($"conveyor +: startX={startX} endX={box.Transform.P.X} y={box.Transform.P.Y} vx={box.LinearVelocity.X}");
            float dx = MathF.Abs(box.Transform.P.X - startX);
            Assert.IsTrue(dx > 0.01f,
                $"Conveyor should drag the box. dx = {box.Transform.P.X - startX}");
        }

        [TestMethod]
        public void TangentSpeed_NegativeDragsOppositeDirection()
        {
            (World world, Body posBox) = BuildBoxOnGround(tangentSpeed: 5f, rollingResistance: 0f);
            for (int s = 0; s < 60; ++s) world.Step(1f / 60f);
            float posStart = posBox.Transform.P.X;
            for (int i = 0; i < 240; ++i) world.Step(1f / 60f);
            float posDx = posBox.Transform.P.X - posStart;

            (World worldN, Body negBox) = BuildBoxOnGround(tangentSpeed: -5f, rollingResistance: 0f);
            for (int s = 0; s < 60; ++s) worldN.Step(1f / 60f);
            float negStart = negBox.Transform.P.X;
            for (int i = 0; i < 240; ++i) worldN.Step(1f / 60f);
            float negDx = negBox.Transform.P.X - negStart;

            System.Console.WriteLine($"posDx={posDx} negDx={negDx}");
            // Whatever direction "+5" produces, "-5" should go the other way.
            Assert.IsTrue(MathF.Sign(posDx) == -MathF.Sign(negDx) && MathF.Abs(negDx) > 0.01f,
                $"Negative conveyor should drag opposite direction. posDx={posDx} negDx={negDx}");
        }

        [TestMethod]
        public void RollingResistance_DampsAngularVelocity()
        {
            // Set up a spinning circle that's resting on ground with high rolling resistance.
            (World world, Body box) = BuildBoxOnGround(tangentSpeed: 0f, rollingResistance: 1f);
            for (int settle = 0; settle < 30; ++settle)
            {
                world.Step(1f / 60f);
            }
            box.AngularVelocity = 20f;
            for (int i = 0; i < 120; ++i)
            {
                world.Step(1f / 60f);
            }
            Assert.IsTrue(MathF.Abs(box.AngularVelocity) < 18f,
                $"Rolling resistance should reduce angular velocity from 20. Now: {box.AngularVelocity}");
        }

        [TestMethod]
        public void Material_DefaultsAreCarriedThroughFixtureDef()
        {
            SurfaceMaterial m = new SurfaceMaterial(0.7f, 0.3f, 0.5f, 2f, 42, 0xFF00FFu);
            FixtureDef def = new FixtureDef(new CircleShape(1f)).WithMaterial(m);
            Assert.AreEqual(0.7f, def.Friction);
            Assert.AreEqual(0.3f, def.Restitution);
            Assert.AreEqual(0.5f, def.RollingResistance);
            Assert.AreEqual(2f, def.TangentSpeed);
            Assert.AreEqual(42UL, def.UserMaterialId);
            Assert.AreEqual(0xFF00FFu, def.CustomColor);

            SurfaceMaterial round = def.ToMaterial();
            Assert.AreEqual(m, round);
        }

        [TestMethod]
        public void Material_PropagatesToFixtureOnCreate()
        {
            World world = new World(new WorldDef().WithGravity(Vec2.Zero));
            Body body = world.CreateBody(new BodyDef().AsDynamic().At(0f, 0f));
            FixtureDef def = new FixtureDef(new CircleShape(1f))
                .WithFriction(0.9f)
                .WithRollingResistance(0.4f)
                .WithTangentSpeed(3f);
            Fixture fixture = body.CreateFixture(def);

            Assert.AreEqual(0.9f, fixture.Material.Friction);
            Assert.AreEqual(0.4f, fixture.Material.RollingResistance);
            Assert.AreEqual(3f, fixture.Material.TangentSpeed);
        }
    }
}
