using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Box2DNG.Tests
{
    [TestClass]
    public class ChainCollisionParityTests
    {
        [TestMethod]
        public void CharacterCollisionScene_RemainsStableUnderCharacterDrive()
        {
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)).EnableSleeping(false));
            BuildCharacterCollisionScene(world, out Body character);

            float maxSpeed = 0f;
            for (int i = 0; i < 300; ++i)
            {
                Vec2 v = character.LinearVelocity;
                character.LinearVelocity = new Vec2(-5f, v.Y);
                world.Step(1f / 60f);

                float speed = character.LinearVelocity.Length;
                if (speed > maxSpeed)
                {
                    maxSpeed = speed;
                }
            }

            Assert.IsTrue(character.Transform.P.X < -12f,
                $"Expected character to keep traversing scene to the left, x={character.Transform.P.X}.");
            Assert.IsFalse(float.IsNaN(character.Transform.P.X) || float.IsNaN(character.Transform.P.Y),
                "Expected finite character transform.");
            Assert.IsTrue(maxSpeed < 80f, $"Expected no instability spikes, maxSpeed={maxSpeed}.");
        }

        [TestMethod]
        public void EdgeShapesScene_DynamicBodiesStayFiniteAndContactTerrain()
        {
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)).EnableSleeping(false));
            BuildEdgeShapesScene(world, out Body[] circles);

            int touchingFrames = 0;
            for (int i = 0; i < 360; ++i)
            {
                world.Step(1f / 60f);

                bool anyTouching = false;
                for (int c = 0; c < world.Contacts.Count; ++c)
                {
                    Contact contact = world.Contacts[c];
                    if (!contact.IsTouching || contact.FixtureA == null || contact.FixtureB == null)
                    {
                        continue;
                    }

                    anyTouching = true;
                    break;
                }

                if (anyTouching)
                {
                    touchingFrames += 1;
                }
            }

            for (int i = 0; i < circles.Length; ++i)
            {
                Vec2 p = circles[i].Transform.P;
                Assert.IsFalse(float.IsNaN(p.X) || float.IsNaN(p.Y), "Expected finite body transform.");
                Assert.IsTrue(p.Y > -40f, $"Expected no catastrophic fall-through, body={i} y={p.Y}.");
            }

            Assert.IsTrue(touchingFrames > 60, $"Expected sustained terrain contacts, touchingFrames={touchingFrames}.");
        }

        private static void BuildCharacterCollisionScene(World world, out Body character)
        {
            Body ground = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            ground.CreateFixture(new FixtureDef(new SegmentShape(new Vec2(-20f, 0f), new Vec2(20f, 0f))));

            Body slopeBody = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f).WithAngle(0.25f * MathFng.Pi));
            Vec2[] slope =
            {
                new Vec2(5f, 7f),
                new Vec2(6f, 8f),
                new Vec2(7f, 8f),
                new Vec2(8f, 7f)
            };
            AddChainSegments(slopeBody, slope, loop: false);

            Body loopBody = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            Vec2[] loop =
            {
                new Vec2(-1f, 3f),
                new Vec2(1f, 3f),
                new Vec2(1f, 5f),
                new Vec2(-1f, 5f)
            };
            AddChainSegments(loopBody, loop, loop: true);

            character = world.CreateBody(new BodyDef().AsDynamic().At(-7f, 6f).AllowSleeping(false));
            character.CreateFixture(new FixtureDef(new CircleShape(0.25f)).WithDensity(20f).WithFriction(1f));
        }

        private static void BuildEdgeShapesScene(World world, out Body[] circles)
        {
            // Wide ground beyond the chain so circles that slide off either end
            // still land on solid geometry (otherwise they fall into infinity).
            Body ground = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            ground.CreateFixture(new FixtureDef(new SegmentShape(new Vec2(-40f, 0f), new Vec2(40f, 0f))));

            // Chain must be wound so that RightPerp(edge) points outward toward
            // the collision side. Traversing right-to-left here makes the outward
            // normal point upward — circles falling from above will hit it.
            Vec2[] chain =
            {
                new Vec2(20f, 0f),
                new Vec2(10f, -5f),
                new Vec2(0f, 0f),
                new Vec2(-10f, 5f),
                new Vec2(-20f, 0f)
            };

            for (int i = 0; i < chain.Length - 1; ++i)
            {
                Vec2 ghost1 = i > 0 ? chain[i - 1] : chain[i];
                Vec2 ghost2 = i + 2 < chain.Length ? chain[i + 2] : chain[i + 1];
                ChainSegmentShape seg = new ChainSegmentShape(chain[i], chain[i + 1], ghost1, ghost2);
                ground.CreateFixture(new FixtureDef(seg));
            }

            circles = new Body[5];
            CircleShape circle = new CircleShape(0.5f);
            for (int i = 0; i < circles.Length; ++i)
            {
                Body body = world.CreateBody(new BodyDef().AsDynamic().At(-15f + 6f * i, 10f));
                body.CreateFixture(new FixtureDef(circle).WithDensity(1f));
                circles[i] = body;
            }
        }

        private static void AddChainSegments(Body body, Vec2[] vertices, bool loop)
        {
            int count = vertices.Length;
            int segmentCount = loop ? count : count - 1;
            for (int i = 0; i < segmentCount; ++i)
            {
                int i1 = i;
                int i2 = (i + 1) % count;
                Vec2 p1 = vertices[i1];
                Vec2 p2 = vertices[i2];
                Vec2 ghost1 = vertices[(i1 - 1 + count) % count];
                Vec2 ghost2 = vertices[(i2 + 1) % count];
                body.CreateFixture(new FixtureDef(new ChainSegmentShape(p1, p2, ghost1, ghost2)).WithDensity(0f));
            }
        }
    }
}
