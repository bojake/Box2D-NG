using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Box2DNG.Tests
{
    [TestClass]
    public class ConstraintGraphParityTests
    {
        [TestMethod]
        public void AddPair_BulletCrowd_KeepsAwakeContactColorsValid()
        {
            World world = new World(new WorldDef().WithGravity(Vec2.Zero).EnableSleeping(false));
            CircleShape small = new CircleShape(0.1f);

            float minX = -6f;
            float maxX = 0f;
            float minY = 4f;
            float maxY = 6f;

            Random rand = new Random(1234);
            for (int i = 0; i < 400; ++i)
            {
                Body body = world.CreateBody(new BodyDef()
                    .AsDynamic()
                    .At(RandomRange(rand, minX, maxX), RandomRange(rand, minY, maxY)));
                body.CreateFixture(new FixtureDef(small).WithDensity(0.01f));
            }

            Shape bigShape = new PolygonShape(new[]
            {
                new Vec2(-1.6f, -1.6f),
                new Vec2(1.6f, -1.6f),
                new Vec2(1.6f, 1.6f),
                new Vec2(-1.6f, 1.6f)
            });

            Body bullet = world.CreateBody(new BodyDef()
                .AsDynamic()
                .At(-40f, 5f)
                .IsBullet(true));
            bullet.CreateFixture(new FixtureDef(bigShape).WithDensity(1f));
            bullet.LinearVelocity = new Vec2(150f, 0f);

            for (int i = 0; i < 90; ++i)
            {
                world.Step(1f / 60f);
                AssertContactColorAssignmentsAreValid(world);
            }
        }

        [TestMethod]
        public void BodyTypes_TypeSwitch_KeepsContactColorsConsistent()
        {
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)));

            Body ground = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            ground.CreateFixture(new FixtureDef(new SegmentShape(new Vec2(-20f, 0f), new Vec2(20f, 0f))));

            Body attachment = world.CreateBody(new BodyDef().AsDynamic().At(0f, 3f));
            attachment.CreateFixture(new FixtureDef(new PolygonShape(new[]
            {
                new Vec2(-0.5f, -2f),
                new Vec2(0.5f, -2f),
                new Vec2(0.5f, 2f),
                new Vec2(-0.5f, 2f)
            })).WithDensity(2f));

            Body platform = world.CreateBody(new BodyDef().AsDynamic().At(-4f, 5f));
            platform.CreateFixture(new FixtureDef(new PolygonShape(BuildBoxVertices(0.5f, 4f, new Vec2(4f, 0f), 0.5f * MathFng.Pi)))
                .WithDensity(2f)
                .WithFriction(0.6f));

            world.CreateJoint(new RevoluteJointDef(attachment, platform, new Vec2(0f, 5f)).WithMotor(true, 0f, 50f));
            world.CreateJoint(new PrismaticJointDef(ground, platform, new Vec2(0f, 5f), new Vec2(1f, 0f))
                .WithMotor(0f, 1000f)
                .WithLimit(-10f, 10f));

            Body payload = world.CreateBody(new BodyDef().AsDynamic().At(0f, 8f));
            payload.CreateFixture(new FixtureDef(new PolygonShape(BuildBoxVertices(0.75f, 0.75f, Vec2.Zero, 0f)))
                .WithDensity(2f)
                .WithFriction(0.6f));

            for (int i = 0; i < 30; ++i)
            {
                world.Step(1f / 60f);
                AssertContactColorAssignmentsAreValid(world);
            }

            platform.SetType(BodyType.Kinematic);
            platform.LinearVelocity = new Vec2(-3f, 0f);
            platform.AngularVelocity = 0f;
            for (int i = 0; i < 90; ++i)
            {
                if (platform.Transform.P.X < -10f && platform.LinearVelocity.X < 0f)
                {
                    platform.LinearVelocity = new Vec2(3f, 0f);
                }
                else if (platform.Transform.P.X > 10f && platform.LinearVelocity.X > 0f)
                {
                    platform.LinearVelocity = new Vec2(-3f, 0f);
                }

                world.Step(1f / 60f);
                AssertContactColorAssignmentsAreValid(world);
            }

            platform.SetType(BodyType.Static);
            for (int i = 0; i < 30; ++i)
            {
                world.Step(1f / 60f);
                AssertContactColorAssignmentsAreValid(world);
            }

            platform.SetType(BodyType.Dynamic);
            for (int i = 0; i < 30; ++i)
            {
                world.Step(1f / 60f);
                AssertContactColorAssignmentsAreValid(world);
            }
        }

        private static void AssertContactColorAssignmentsAreValid(World world)
        {
            for (int i = 0; i < world.Contacts.Count; ++i)
            {
                Contact contact = world.Contacts[i];
                Fixture? fixtureA = contact.FixtureA;
                Fixture? fixtureB = contact.FixtureB;
                if (fixtureA == null || fixtureB == null)
                {
                    continue;
                }

                if (fixtureA.IsSensor || fixtureB.IsSensor)
                {
                    Assert.AreEqual(-1, contact.ColorIndex, "Expected sensor contact to stay out of graph colors.");
                    continue;
                }

                // Assigned colors must be coherent. Contacts that transition during
                // SolveTOI are intentionally left for the next step's UpdateContacts
                // reconciliation — matching cpp box2d v3 which doesn't run BCG after
                // CCD either, so we don't require every touching awake contact to
                // already have a color at end-of-step.
                if (contact.ColorIndex >= 0)
                {
                    Assert.IsTrue(contact.IsTouching, "Expected colored contact to be touching.");
                    Assert.AreEqual(SolverSetType.Awake, contact.SolverSetType, "Expected colored contact in awake solver set.");
                    Assert.IsTrue(contact.ColorIndex < Constants.GraphColorCount, "Expected color index in graph bounds.");
                }
            }
        }

        private static float RandomRange(Random rand, float min, float max)
        {
            return (float)(min + rand.NextDouble() * (max - min));
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
