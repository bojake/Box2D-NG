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

            // Phase 1 of TIER4_PARITY_PLAN added the soft-weld API
            // (`WeldJointDef.WithLinearSpring` / `WithAngularSpring`) but the
            // Cantilever scene's many chains don't yet stabilize on top of
            // it — cpp's Cantilever uses subStepCount=4 (which we get in
            // Phase 2) and per-joint constraint tuning. Until that lands, the
            // chain bodies keep their body-level damping workaround. The
            // sample-level switch to soft welds happens in Phase 2 once the
            // sub-step solver is in place.
            WeldJointDef ChainWeld(Body a, Body b, Vec2 anchor) =>
                new WeldJointDef(a, b, anchor);

            BodyDef ChainBody(float x, float y) => new BodyDef()
                .AsDynamic()
                .At(x, y)
                .WithLinearDamping(2f)
                .WithAngularDamping(2f);

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
