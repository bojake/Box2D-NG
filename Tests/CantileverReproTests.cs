using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Box2DNG.Tests
{
    [TestClass]
    public class CantileverReproTests
    {
        // Regression tests for the Cantilever sample. The post-SolveTOI graph
        // reconciliation pass (removed) was making two unanchored welded chains
        // fall through the ground segment after their first ground impact. These
        // tests pin no-fall-through behaviour.
        //
        // The Cantilever sample additionally exhibits a slow energy buildup in
        // welded chains lying on the ground — a documented limitation of the
        // iterative solver, present in cpp box2d v3 as well. The sample uses
        // body-level linear/angular damping to suppress the residual oscillation
        // (mirroring the cpp Cantilever sample which applies damping ratios on
        // its soft weld joints). See [Viewer/Samples/CantileverSample.cs].

        private const int Count = 8;

        [TestMethod]
        public void Cantilever_TwoUnanchoredWeldedChains_NoFallThrough()
        {
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)));
            Body ground = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            ground.CreateFixture(new FixtureDef(new SegmentShape(new Vec2(-40f, 0f), new Vec2(40f, 0f))));

            BuildWeldedChain(world, halfWidth: 0.5f, halfHeight: 0.125f, x0: -4.5f, y: 5f);
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
            BuildWeldedChain(world, 0.5f, 0.125f, -14.5f, 5f, anchor: ground);
            BuildWeldedChain(world, 1f, 0.125f, -14f, 15f, anchor: ground, count: 3, spacing: 2f);

            // Two unanchored welded chains
            BuildWeldedChain(world, 0.5f, 0.125f, -4.5f, 5f);
            BuildWeldedChain(world, 0.5f, 0.125f, 5.5f, 10f);

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
            for (int s = 0; s < 600; ++s) world.Step(1f / 60f);

            int totalFallen = 0;
            for (int i = 1; i < n; ++i)
            {
                if (world.Bodies[i].Transform.P.Y < -2f) totalFallen++;
            }
            Assert.AreEqual(0, totalFallen, $"Expected no fall-through, got {totalFallen} bodies below y=-2.");
        }

        [TestMethod]
        public void Cantilever_DampedWeldChain_StabilizesUnderHeavyLoad()
        {
            // Two damped chains land and remain bounded (energy does not grow
            // catastrophically). This mirrors what the viewer's CantileverSample
            // does: applying body-level damping to suppress the residual
            // oscillation from per-contact TOI sub-stepping. Without damping the
            // peak velocity grows monotonically over time.
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)));
            Body ground = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            ground.CreateFixture(new FixtureDef(new SegmentShape(new Vec2(-40f, 0f), new Vec2(40f, 0f))));
            BuildWeldedChain(world, 0.5f, 0.125f, -4.5f, 5f, linearDamping: 2f, angularDamping: 2f);
            BuildWeldedChain(world, 0.5f, 0.125f, 5.5f, 10f, linearDamping: 2f, angularDamping: 2f);

            for (int s = 0; s < 600; ++s) world.Step(1f / 60f);

            // After 10 seconds, late-window peak speed must stay bounded.
            float latePeak = 0f;
            for (int s = 0; s < 600; ++s)
            {
                world.Step(1f / 60f);
                for (int i = 1; i < world.Bodies.Count; ++i)
                {
                    Body b = world.Bodies[i];
                    if (b.Type != BodyType.Dynamic) continue;
                    float v = b.LinearVelocity.Length;
                    if (v > latePeak) latePeak = v;
                }
            }
            Assert.IsTrue(latePeak < 3f, $"Chains should stay bounded with damping. latePeak={latePeak}");
        }

        private static Body[] BuildWeldedChain(World world, float halfWidth, float halfHeight, float x0, float y, Body? anchor = null, int count = Count, float spacing = 1f, float linearDamping = 0f, float angularDamping = 0f)
        {
            PolygonShape shape = new PolygonShape(BuildBoxVertices(halfWidth, halfHeight, Vec2.Zero, 0f));
            FixtureDef fd = new FixtureDef(shape).WithDensity(20f);
            Body? prev = anchor;
            Body[] chain = new Body[count];
            for (int i = 0; i < count; ++i)
            {
                Body body = world.CreateBody(new BodyDef()
                    .AsDynamic()
                    .At(x0 + spacing * i, y)
                    .WithLinearDamping(linearDamping)
                    .WithAngularDamping(angularDamping));
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
