using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Box2DNG.Tests
{
    [TestClass]
    public class CantileverReproTests
    {
        // Reproduction of the Viewer's CantileverSample. The bug: with the
        // post-SolveTOI graph reconciliation enabled, the two unanchored welded
        // chains would fall through the ground segment after their first
        // ground impact — TOI applied an arresting impulse, the reconciliation
        // mutated the contact graph mid-step, and subsequent steps lost the
        // ground contact so the chain free-fell with only gravity acting.
        // Removing the post-TOI reconciliation pass fixes both this scene and
        // the constraint-graph parity tests (which now match cpp's relaxed
        // end-of-step invariants).

        private const int Count = 8;

        [TestMethod]
        public void Cantilever_TwoUnanchoredWeldedChains_NoFallThrough()
        {
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)));
            Body ground = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            ground.CreateFixture(new FixtureDef(new SegmentShape(new Vec2(-40f, 0f), new Vec2(40f, 0f))));

            // Unanchored welded chain at y=5
            BuildWeldedChain(world, halfWidth: 0.5f, halfHeight: 0.125f, x0: -4.5f, y: 5f);

            // Unanchored welded chain at y=10
            Body[] chain4 = BuildWeldedChain(world, halfWidth: 0.5f, halfHeight: 0.125f, x0: 5.5f, y: 10f);

            for (int s = 0; s < 240; ++s)
            {
                world.Step(1f / 60f);
            }

            for (int i = 0; i < Count; ++i)
            {
                Assert.IsTrue(chain4[i].Transform.P.Y > -1f, $"chain4[{i}] fell: y={chain4[i].Transform.P.Y}");
            }
        }

        [TestMethod]
        public void Cantilever_FullSceneNoFallThrough()
        {
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)));
            Body ground = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            ground.CreateFixture(new FixtureDef(new SegmentShape(new Vec2(-40f, 0f), new Vec2(40f, 0f))));

            // Two hanging chains anchored to ground
            BuildWeldedChain(world, halfWidth: 0.5f, halfHeight: 0.125f, x0: -14.5f, y: 5f, anchor: ground);
            BuildWeldedChain(world, halfWidth: 1f, halfHeight: 0.125f, x0: -14f, y: 15f, anchor: ground, count: 3, spacing: 2f);

            // Two unanchored welded chains
            BuildWeldedChain(world, halfWidth: 0.5f, halfHeight: 0.125f, x0: -4.5f, y: 5f);
            BuildWeldedChain(world, halfWidth: 0.5f, halfHeight: 0.125f, x0: 5.5f, y: 10f);

            // Loose triangles + circles
            for (int i = 0; i < 2; ++i)
            {
                Body body = world.CreateBody(new BodyDef().AsDynamic().At(-8f + 8f * i, 12f));
                body.CreateFixture(new FixtureDef(new PolygonShape(new[]
                {
                    new Vec2(-0.5f, 0f), new Vec2(0.5f, 0f), new Vec2(0f, 1.5f)
                })).WithDensity(1f));
            }
            for (int i = 0; i < 2; ++i)
            {
                Body body = world.CreateBody(new BodyDef().AsDynamic().At(-6f + 6f * i, 10f));
                body.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));
            }

            int n = world.Bodies.Count;
            for (int s = 0; s < 600; ++s)
            {
                world.Step(1f / 60f);
            }

            int totalFallen = 0;
            for (int i = 1; i < n; ++i)
            {
                if (world.Bodies[i].Transform.P.Y < -2f) totalFallen++;
            }
            Assert.AreEqual(0, totalFallen, $"Expected no fall-through, got {totalFallen} bodies below y=-2.");
        }

        private static Body[] BuildWeldedChain(World world, float halfWidth, float halfHeight, float x0, float y, Body? anchor = null, int count = Count, float spacing = 1f)
        {
            PolygonShape shape = new PolygonShape(BuildBoxVertices(halfWidth, halfHeight, Vec2.Zero, 0f));
            FixtureDef fd = new FixtureDef(shape).WithDensity(20f);
            Body? prev = anchor;
            Body[] chain = new Body[count];
            for (int i = 0; i < count; ++i)
            {
                Body body = world.CreateBody(new BodyDef().AsDynamic().At(x0 + spacing * i, y));
                body.CreateFixture(fd);
                if (prev != null)
                {
                    world.CreateJoint(new WeldJointDef(prev, body, new Vec2(x0 - 0.5f * spacing + spacing * i, y)));
                }
                prev = body;
                chain[i] = body;
            }
            return chain;
        }

        private static Vec2[] BuildBoxVertices(float hx, float hy, Vec2 center, float angle)
        {
            Vec2[] verts =
            {
                new Vec2(-hx, -hy), new Vec2(hx, -hy),
                new Vec2(hx, hy),   new Vec2(-hx, hy)
            };
            if (angle == 0f)
            {
                for (int i = 0; i < verts.Length; ++i) verts[i] = verts[i] + center;
                return verts;
            }
            Rot rot = new Rot(angle);
            for (int i = 0; i < verts.Length; ++i) verts[i] = Rot.Mul(rot, verts[i]) + center;
            return verts;
        }
    }
}
