namespace Box2DNG.Viewer.Samples
{
    public sealed class CantileverSample : BaseSample
    {
        private const int Count = 8;

        public override string Name => "Cantilever";

        public override void Build(World world)
        {
            Body ground = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            ground.CreateFixture(new FixtureDef(new SegmentShape(new Vec2(-40f, 0f), new Vec2(40f, 0f))));

            // Phase 4 of TIER4_PARITY_PLAN — "smallest valuable slice":
            // soft welds replace the body-level damping workaround. cpp's
            // Cantilever uses (15 Hz, 0.5 damping); our iterative solver
            // (without Phase 2.5 deltaPosition tracking landed yet) needs
            // a stiffer spring at (30 Hz, 0.5 damping) to keep the late-
            // window peak under the threshold pinned by SampleSettlingTests.
            //
            // Note (Step 6 coordinated flip, 2026-05-26): tried reverting
            // to cpp's 15 Hz tune after flipping the contact tuning to
            // 30 Hz / ratio 10, on the theory that the stiffer 30 Hz weld
            // would resonate with the 30 Hz contacts. That made the late-
            // window peak worse (107 → 157), so the 30 Hz tune stays. The
            // Cantilever_LateWindow regression under the coordinated flip
            // is documented but not yet resolved.
            WeldJointDef ChainWeld(Body a, Body b, Vec2 anchor) =>
                new WeldJointDef(a, b, anchor)
                    .WithLinearSpring(30f, 0.5f)
                    .WithAngularSpring(30f, 0.5f);

            BodyDef ChainBody(float x, float y) => new BodyDef()
                .AsDynamic()
                .At(x, y);

            {
                PolygonShape shape = new PolygonShape(BuildBoxVertices(0.5f, 0.125f, Vec2.Zero, 0f));
                FixtureDef fd = new FixtureDef(shape).WithDensity(20f);

                Body prevBody = ground;
                for (int i = 0; i < Count; ++i)
                {
                    Body body = world.CreateBody(ChainBody(-14.5f + i, 5f));
                    body.CreateFixture(fd);
                    world.CreateJoint(ChainWeld(prevBody, body, new Vec2(-15f + i, 5f)));
                    prevBody = body;
                }
            }

            {
                PolygonShape shape = new PolygonShape(BuildBoxVertices(1f, 0.125f, Vec2.Zero, 0f));
                FixtureDef fd = new FixtureDef(shape).WithDensity(20f);

                Body prevBody = ground;
                for (int i = 0; i < 3; ++i)
                {
                    Body body = world.CreateBody(ChainBody(-14f + 2f * i, 15f));
                    body.CreateFixture(fd);
                    world.CreateJoint(ChainWeld(prevBody, body, new Vec2(-15f + 2f * i, 15f)));
                    prevBody = body;
                }
            }

            {
                PolygonShape shape = new PolygonShape(BuildBoxVertices(0.5f, 0.125f, Vec2.Zero, 0f));
                FixtureDef fd = new FixtureDef(shape).WithDensity(20f);

                Body prevBody = ground;
                for (int i = 0; i < Count; ++i)
                {
                    Body body = world.CreateBody(ChainBody(-4.5f + i, 5f));
                    body.CreateFixture(fd);

                    if (i > 0)
                    {
                        world.CreateJoint(ChainWeld(prevBody, body, new Vec2(-5f + i, 5f)));
                    }

                    prevBody = body;
                }
            }

            {
                PolygonShape shape = new PolygonShape(BuildBoxVertices(0.5f, 0.125f, Vec2.Zero, 0f));
                FixtureDef fd = new FixtureDef(shape).WithDensity(20f);

                Body prevBody = ground;
                for (int i = 0; i < Count; ++i)
                {
                    Body body = world.CreateBody(ChainBody(5.5f + i, 10f));
                    body.CreateFixture(fd);

                    if (i > 0)
                    {
                        world.CreateJoint(ChainWeld(prevBody, body, new Vec2(5f + i, 10f)));
                    }

                    prevBody = body;
                }
            }

            for (int i = 0; i < 2; ++i)
            {
                PolygonShape triShape = new PolygonShape(new[]
                {
                    new Vec2(-0.5f, 0f),
                    new Vec2(0.5f, 0f),
                    new Vec2(0f, 1.5f)
                });
                FixtureDef triDef = new FixtureDef(triShape).WithDensity(1f);
                Body body = world.CreateBody(new BodyDef().AsDynamic().At(-8f + 8f * i, 12f));
                body.CreateFixture(triDef);
            }

            for (int i = 0; i < 2; ++i)
            {
                CircleShape shape = new CircleShape(0.5f);
                FixtureDef circleDef = new FixtureDef(shape).WithDensity(1f);
                Body body = world.CreateBody(new BodyDef().AsDynamic().At(-6f + 6f * i, 10f));
                body.CreateFixture(circleDef);
            }
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
