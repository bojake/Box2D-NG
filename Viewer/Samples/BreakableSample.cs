using System;

namespace Box2DNG.Viewer.Samples
{
    public sealed class BreakableSample : BaseSample
    {
        // Per-event ContactImpulseEvent.NormalImpulse for the 2-square body
        // dropped from y=40 onto the ground peaks around 2.4 N·s with the GJK
        // direction fix and Solve3 simplex-collapse fix in place. The pre-fix
        // value (~15-20) was inflated by stale impulses and wrong GJK distances,
        // so the original 40 threshold from upstream box2d never triggered here.
        private const float BreakImpulse = 2f;
        private Body? _body;
        private Fixture? _piece1;
        private Fixture? _piece2;
        private PolygonShape _shape1 = new PolygonShape(Array.Empty<Vec2>());
        private PolygonShape _shape2 = new PolygonShape(Array.Empty<Vec2>());
        private Vec2 _cachedVelocity;
        private float _cachedAngularVelocity;
        private bool _break;
        private bool _broke;

        public override string Name => "Breakable";

        public override void Build(World world)
        {
            Body ground = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            ground.CreateFixture(new FixtureDef(new SegmentShape(new Vec2(-40f, 0f), new Vec2(40f, 0f))));

            _body = world.CreateBody(new BodyDef().AsDynamic().At(0f, 40f).WithAngle(0.25f * MathFng.Pi));
            _shape1 = new PolygonShape(BuildBoxVertices(0.5f, 0.5f, new Vec2(-0.5f, 0f), 0f));
            _shape2 = new PolygonShape(BuildBoxVertices(0.5f, 0.5f, new Vec2(0.5f, 0f), 0f));
            _piece1 = _body.CreateFixture(new FixtureDef(_shape1).WithDensity(1f).WithUserData("piece1"));
            _piece2 = _body.CreateFixture(new FixtureDef(_shape2).WithDensity(1f).WithUserData("piece2"));

            world.Events.ContactImpulseEvents += events =>
            {
                if (_broke || events.Events == null)
                {
                    return;
                }

                for (int i = 0; i < events.Events.Length; ++i)
                {
                    ContactImpulseEvent impulse = events.Events[i];
                    if (impulse.NormalImpulse > BreakImpulse && (Equals(impulse.UserDataA, "piece1") || Equals(impulse.UserDataB, "piece1") ||
                                            Equals(impulse.UserDataA, "piece2") || Equals(impulse.UserDataB, "piece2")))
                    {
                        _break = true;
                        break;
                    }
                }
            };
        }

        public override void Step(World world, float dt)
        {
            if (_body == null)
            {
                return;
            }

            if (_break)
            {
                Break(world);
                _broke = true;
                _break = false;
            }

            if (!_broke)
            {
                _cachedVelocity = _body.LinearVelocity;
                _cachedAngularVelocity = _body.AngularVelocity;
            }
        }

        private void Break(World world)
        {
            if (_body == null || _piece2 == null)
            {
                return;
            }

            Vec2 center = _body.GetWorldPoint(_body.Sweep.LocalCenter);
            _body.DestroyFixture(_piece2);
            _piece2 = null;

            Body body2 = world.CreateBody(new BodyDef().AsDynamic().At(_body.Transform.P).WithAngle(_body.Transform.Q.Angle));
            Fixture piece2 = body2.CreateFixture(new FixtureDef(_shape2).WithDensity(1f).WithUserData("piece2"));

            Vec2 center1 = _body.GetWorldPoint(_body.Sweep.LocalCenter);
            Vec2 center2 = body2.GetWorldPoint(body2.Sweep.LocalCenter);
            Vec2 diff1 = center1 - center;
            Vec2 diff2 = center2 - center;

            Vec2 velocity1 = _cachedVelocity + Vec2.Cross(_cachedAngularVelocity, diff1);
            Vec2 velocity2 = _cachedVelocity + Vec2.Cross(_cachedAngularVelocity, diff2);

            _body.LinearVelocity = velocity1;
            _body.AngularVelocity = _cachedAngularVelocity;
            body2.LinearVelocity = velocity2;
            body2.AngularVelocity = _cachedAngularVelocity;
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
