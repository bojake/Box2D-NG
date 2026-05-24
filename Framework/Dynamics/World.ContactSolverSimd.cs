using System.Numerics;

namespace Box2DNG
{
    public sealed partial class World
    {
        private sealed class ContactSolverSimd
        {
        private readonly World _world;
        private readonly System.Collections.Generic.List<ContactConstraint> _constraints = new System.Collections.Generic.List<ContactConstraint>();
            private readonly System.Collections.Generic.List<System.Collections.Generic.List<int>> _colorsSingle = new System.Collections.Generic.List<System.Collections.Generic.List<int>>();
            private readonly System.Collections.Generic.List<System.Collections.Generic.HashSet<Body>> _colorBodiesSingle = new System.Collections.Generic.List<System.Collections.Generic.HashSet<Body>>();
            private readonly System.Collections.Generic.List<System.Collections.Generic.List<int>> _colorsTwo = new System.Collections.Generic.List<System.Collections.Generic.List<int>>();
            private readonly System.Collections.Generic.List<System.Collections.Generic.HashSet<Body>> _colorBodiesTwo = new System.Collections.Generic.List<System.Collections.Generic.HashSet<Body>>();
            private readonly System.Collections.Generic.List<int> _scalarConstraints = new System.Collections.Generic.List<int>();
            private World.ContactSolverStats _stats = new World.ContactSolverStats();

            public ContactSolverSimd(World world)
            {
                _world = world;
            }

            public void Prepare(float timeStep, float dtRatio, System.Collections.Generic.IReadOnlyList<Contact> contacts)
            {
                _constraints.Clear();
                _colorsSingle.Clear();
                _colorBodiesSingle.Clear();
                _colorsTwo.Clear();
                _colorBodiesTwo.Clear();
                _scalarConstraints.Clear();
                int singlePoint = 0;
                int twoPoint = 0;
                int scalar = 0;
                if (contacts.Count == 0)
                {
                    _stats = new World.ContactSolverStats(0, 0, 0, 0, 0, 0);
                    return;
                }

                for (int i = 0; i < contacts.Count; ++i)
                {
                    Contact contact = contacts[i];
                    if (contact.FixtureA == null || contact.FixtureB == null)
                    {
                        continue;
                    }

                    if (contact.Manifold.PointCount == 0)
                    {
                        contact.ColorIndex = -1;
                        continue;
                    }

                    Fixture fixtureA = contact.FixtureA;
                    Fixture fixtureB = contact.FixtureB;
                    Body bodyA = fixtureA.Body;
                    Body bodyB = fixtureB.Body;

                    float radiusA = GetShapeRadius(fixtureA.Shape);
                    float radiusB = GetShapeRadius(fixtureB.Shape);

                    WorldManifold worldManifold = new WorldManifold();
                    worldManifold.Initialize(contact.Manifold, bodyA.Transform, radiusA, bodyB.Transform, radiusB);

                    Vec2 normal = worldManifold.Normal;
                    Vec2 tangent = new Vec2(-normal.Y, normal.X);

                    SurfaceMaterial matA = fixtureA.Material;
                    SurfaceMaterial matB = fixtureB.Material;
                    float mixedRolling = 0f;
                    if (matA.RollingResistance > 0f || matB.RollingResistance > 0f)
                    {
                        float maxRadius = System.MathF.Max(radiusA, radiusB);
                        mixedRolling = System.MathF.Max(matA.RollingResistance, matB.RollingResistance) * maxRadius;
                    }
                    float invIAval = _world._bodyInverseInertias[bodyA.Id];
                    float invIBval = _world._bodyInverseInertias[bodyB.Id];
                    float rollingK = invIAval + invIBval;

                    ContactConstraint constraint = new ContactConstraint(contact, bodyA.Id, bodyB.Id, normal, tangent, contact.Manifold.PointCount)
                    {
                        Friction = _world.MixFriction(fixtureA, fixtureB),
                        Restitution = _world.MixRestitution(fixtureA, fixtureB),
                        RollingResistance = mixedRolling,
                        RollingMass = rollingK > 0f ? 1f / rollingK : 0f,
                        TangentSpeed = matA.TangentSpeed + matB.TangentSpeed,
                        InvMassA = _world._bodyInverseMasses[bodyA.Id],
                        InvIA = invIAval,
                        InvMassB = _world._bodyInverseMasses[bodyB.Id],
                        InvIB = invIBval
                    };

                    Vec2 centerA = bodyA.GetWorldCenter();
                    Vec2 centerB = bodyB.GetWorldCenter();

                    for (int p = 0; p < constraint.PointCount; ++p)
                    {
                        ManifoldPoint mp = contact.Manifold.Points[p];
                        if (dtRatio != 1f)
                        {
                            mp = new ManifoldPoint(mp.LocalPoint, mp.NormalImpulse * dtRatio, mp.TangentImpulse * dtRatio, mp.Id);
                            contact.Manifold.Points[p] = mp;
                        }

                        Vec2 point = worldManifold.Points[p];
                        Vec2 rA = point - centerA;
                        Vec2 rB = point - centerB;

                        PositionManifold pm = ComputePositionManifold(contact, bodyA.Transform, bodyB.Transform, radiusA, radiusB, p);
                        float separation = pm.Separation;

                        float rnA = Vec2.Cross(rA, normal);
                        float rnB = Vec2.Cross(rB, normal);
                        float kNormal = bodyA.InverseMass + bodyB.InverseMass +
                                        bodyA.InverseInertia * rnA * rnA + bodyB.InverseInertia * rnB * rnB;
                        float normalMass = kNormal > 0f ? 1f / kNormal : 0f;

                        float rtA = Vec2.Cross(rA, tangent);
                        float rtB = Vec2.Cross(rB, tangent);
                        float kTangent = bodyA.InverseMass + bodyB.InverseMass +
                                         bodyA.InverseInertia * rtA * rtA + bodyB.InverseInertia * rtB * rtB;
                        float tangentMass = kTangent > 0f ? 1f / kTangent : 0f;

                        Vec2 vA = bodyA.LinearVelocity + Vec2.Cross(bodyA.AngularVelocity, rA);
                        Vec2 vB = bodyB.LinearVelocity + Vec2.Cross(bodyB.AngularVelocity, rB);
                        float relativeVelocity = Vec2.Dot(vB - vA, normal);

                        float bias = 0f;
                        float softness = 0f;
                        if (_world._def.EnableContactSoftening)
                        {
                            if (separation > 0f)
                            {
                                bias = separation / timeStep;
                            }
                            else if (_world._def.UseSoftConstraints)
                            {
                                _world.ComputeContactSoftness(kNormal, timeStep, separation, out bias, out softness);
                                if (softness > 0f)
                                {
                                    normalMass = 1f / (kNormal + softness);
                                }
                            }
                            else
                            {
                                float baumgarte = Constants.Baumgarte * (separation + Constants.LinearSlop) / timeStep;
                                float maxBias = MathF.Max(_world._def.ContactSpeed, 0f);
                                float legacyBias = MathFng.Clamp(baumgarte, -maxBias, 0f);
                                bias = MathFng.Clamp(legacyBias, -Constants.MaxLinearCorrection, 0f);
                            }
                        }

                        constraint.Points[p] = new ContactConstraintPoint
                        {
                            RA = rA,
                            RB = rB,
                            NormalImpulse = mp.NormalImpulse,
                            TangentImpulse = mp.TangentImpulse,
                            TotalNormalImpulse = 0f,
                            NormalMass = normalMass,
                            TangentMass = tangentMass,
                            BaseSeparation = separation - Vec2.Dot(rB - rA, normal),
                            RelativeVelocity = relativeVelocity,
                            Bias = bias,
                            Softness = softness
                        };
                    }

                    int index = _constraints.Count;
                    _constraints.Add(constraint);

                    if (constraint.PointCount != 1)
                    {
                        twoPoint++;
                        int colorIndex = TryAddToColors(index, bodyA, bodyB, _colorsTwo, _colorBodiesTwo);
                        if (colorIndex < 0)
                        {
                            _scalarConstraints.Add(index);
                            scalar++;
                            contact.ColorIndex = -1;
                        }
                        else
                        {
                            contact.ColorIndex = colorIndex;
                        }
                        continue;
                    }

                    singlePoint++;
                    int colorIndexSingle = TryAddToColors(index, bodyA, bodyB, _colorsSingle, _colorBodiesSingle);
                    if (colorIndexSingle < 0)
                    {
                        _scalarConstraints.Add(index);
                        scalar++;
                        contact.ColorIndex = -1;
                    }
                    else
                    {
                        contact.ColorIndex = colorIndexSingle;
                    }
                }

                _stats = new World.ContactSolverStats(
                    singlePoint,
                    twoPoint,
                    scalar,
                    _colorsSingle.Count + _colorsTwo.Count,
                    0,
                    0);
            }

            public void Prepare(float timeStep, float dtRatio, ConstraintGraph graph)
            {
                _constraints.Clear();
                _colorsSingle.Clear();
                _colorBodiesSingle.Clear();
                _colorsTwo.Clear();
                _colorBodiesTwo.Clear();
                _scalarConstraints.Clear();
                int singlePoint = 0;
                int twoPoint = 0;
                int scalar = 0;

                System.Collections.Generic.List<int>?[] colorSingles = new System.Collections.Generic.List<int>?[Constants.GraphColorCount];
                System.Collections.Generic.List<int>?[] colorTwos = new System.Collections.Generic.List<int>?[Constants.GraphColorCount];

                for (int colorIndex = 0; colorIndex < Constants.GraphColorCount; ++colorIndex)
                {
                    System.Collections.Generic.List<Contact> contacts = graph.Colors[colorIndex].Contacts;
                    for (int i = 0; i < contacts.Count; ++i)
                    {
                        Contact contact = contacts[i];
                        if (contact.FixtureA == null || contact.FixtureB == null)
                        {
                            continue;
                        }

                        if (contact.Manifold.PointCount == 0)
                        {
                            contact.ColorIndex = -1;
                            contact.LocalIndex = -1;
                            continue;
                        }

                        Fixture fixtureA = contact.FixtureA;
                        Fixture fixtureB = contact.FixtureB;
                        Body bodyA = fixtureA.Body;
                        Body bodyB = fixtureB.Body;

                        float radiusA = GetShapeRadius(fixtureA.Shape);
                        float radiusB = GetShapeRadius(fixtureB.Shape);

                        WorldManifold worldManifold = new WorldManifold();
                        worldManifold.Initialize(contact.Manifold, bodyA.Transform, radiusA, bodyB.Transform, radiusB);

                        Vec2 normal = worldManifold.Normal;
                        Vec2 tangent = new Vec2(-normal.Y, normal.X);

                        SurfaceMaterial graphMatA = fixtureA.Material;
                        SurfaceMaterial graphMatB = fixtureB.Material;
                        float graphMixedRolling = 0f;
                        if (graphMatA.RollingResistance > 0f || graphMatB.RollingResistance > 0f)
                        {
                            float maxRadius = System.MathF.Max(radiusA, radiusB);
                            graphMixedRolling = System.MathF.Max(graphMatA.RollingResistance, graphMatB.RollingResistance) * maxRadius;
                        }
                        float graphInvIA = _world._bodyInverseInertias[bodyA.Id];
                        float graphInvIB = _world._bodyInverseInertias[bodyB.Id];
                        float graphRollingK = graphInvIA + graphInvIB;

                        ContactConstraint constraint = new ContactConstraint(contact, bodyA.Id, bodyB.Id, normal, tangent, contact.Manifold.PointCount)
                        {
                            Friction = _world.MixFriction(fixtureA, fixtureB),
                            Restitution = _world.MixRestitution(fixtureA, fixtureB),
                            RollingResistance = graphMixedRolling,
                            RollingMass = graphRollingK > 0f ? 1f / graphRollingK : 0f,
                            TangentSpeed = graphMatA.TangentSpeed + graphMatB.TangentSpeed,
                            InvMassA = _world._bodyInverseMasses[bodyA.Id],
                            InvIA = graphInvIA,
                            InvMassB = _world._bodyInverseMasses[bodyB.Id],
                            InvIB = graphInvIB
                        };

                        Vec2 centerA = bodyA.GetWorldCenter();
                        Vec2 centerB = bodyB.GetWorldCenter();

                        for (int p = 0; p < constraint.PointCount; ++p)
                        {
                            ManifoldPoint mp = contact.Manifold.Points[p];
                            if (dtRatio != 1f)
                            {
                                mp = new ManifoldPoint(mp.LocalPoint, mp.NormalImpulse * dtRatio, mp.TangentImpulse * dtRatio, mp.Id);
                                contact.Manifold.Points[p] = mp;
                            }

                            Vec2 point = worldManifold.Points[p];
                            Vec2 rA = point - centerA;
                            Vec2 rB = point - centerB;

                            PositionManifold pm = ComputePositionManifold(contact, bodyA.Transform, bodyB.Transform, radiusA, radiusB, p);
                            float separation = pm.Separation;

                            float rnA = Vec2.Cross(rA, normal);
                            float rnB = Vec2.Cross(rB, normal);
                            float kNormal = bodyA.InverseMass + bodyB.InverseMass +
                                            bodyA.InverseInertia * rnA * rnA + bodyB.InverseInertia * rnB * rnB;
                            float normalMass = kNormal > 0f ? 1f / kNormal : 0f;

                            float rtA = Vec2.Cross(rA, tangent);
                            float rtB = Vec2.Cross(rB, tangent);
                            float kTangent = bodyA.InverseMass + bodyB.InverseMass +
                                             bodyA.InverseInertia * rtA * rtA + bodyB.InverseInertia * rtB * rtB;
                            float tangentMass = kTangent > 0f ? 1f / kTangent : 0f;

                            Vec2 vA = bodyA.LinearVelocity + Vec2.Cross(bodyA.AngularVelocity, rA);
                            Vec2 vB = bodyB.LinearVelocity + Vec2.Cross(bodyB.AngularVelocity, rB);
                            float relativeVelocity = Vec2.Dot(vB - vA, normal);

                            float bias = 0f;
                            float softness = 0f;
                            if (_world._def.EnableContactSoftening)
                            {
                                if (separation > 0f)
                                {
                                    bias = separation / timeStep;
                                }
                                else if (_world._def.UseSoftConstraints)
                                {
                                    _world.ComputeContactSoftness(kNormal, timeStep, separation, out bias, out softness);
                                    if (softness > 0f)
                                    {
                                        normalMass = 1f / (kNormal + softness);
                                    }
                                }
                                else
                                {
                                    float baumgarte = Constants.Baumgarte * (separation + Constants.LinearSlop) / timeStep;
                                    float maxBias = MathF.Max(_world._def.ContactSpeed, 0f);
                                    float legacyBias = MathFng.Clamp(baumgarte, -maxBias, 0f);
                                    bias = MathFng.Clamp(legacyBias, -Constants.MaxLinearCorrection, 0f);
                                }
                            }

                            constraint.Points[p] = new ContactConstraintPoint
                            {
                                RA = rA,
                                RB = rB,
                                NormalImpulse = mp.NormalImpulse,
                                TangentImpulse = mp.TangentImpulse,
                                TotalNormalImpulse = 0f,
                                NormalMass = normalMass,
                                TangentMass = tangentMass,
                                BaseSeparation = separation - Vec2.Dot(rB - rA, normal),
                                RelativeVelocity = relativeVelocity,
                                Bias = bias,
                                Softness = softness
                            };
                        }

                        int index = _constraints.Count;
                        _constraints.Add(constraint);

                        if (constraint.PointCount != 1)
                        {
                            twoPoint++;
                            System.Collections.Generic.List<int>? list = colorTwos[colorIndex];
                            if (list == null)
                            {
                                list = new System.Collections.Generic.List<int>();
                                colorTwos[colorIndex] = list;
                            }
                            list.Add(index);
                        }
                        else
                        {
                            singlePoint++;
                            System.Collections.Generic.List<int>? list = colorSingles[colorIndex];
                            if (list == null)
                            {
                                list = new System.Collections.Generic.List<int>();
                                colorSingles[colorIndex] = list;
                            }
                            list.Add(index);
                        }
                    }
                }

                int activeColors = 0;
                for (int i = 0; i < Constants.GraphColorCount; ++i)
                {
                    System.Collections.Generic.List<int>? singles = colorSingles[i];
                    if (singles != null && singles.Count > 0)
                    {
                        _colorsSingle.Add(singles);
                    }

                    System.Collections.Generic.List<int>? twos = colorTwos[i];
                    if (twos != null && twos.Count > 0)
                    {
                        _colorsTwo.Add(twos);
                    }

                    if ((singles != null && singles.Count > 0) || (twos != null && twos.Count > 0))
                    {
                        activeColors++;
                    }
                }

                _stats = new World.ContactSolverStats(
                    singlePoint,
                    twoPoint,
                    scalar,
                    activeColors,
                    0,
                    0);
            }

            public void WarmStart()
            {
                for (int i = 0; i < _constraints.Count; ++i)
                {
                    ContactConstraint constraint = _constraints[i];
                    int indexA = constraint.IndexA;
                    int indexB = constraint.IndexB;

                    Vec2 vA = _world._bodyLinearVelocities[indexA];
                    float wA = _world._bodyAngularVelocities[indexA];
                    Vec2 vB = _world._bodyLinearVelocities[indexB];
                    float wB = _world._bodyAngularVelocities[indexB];

                    float mA = constraint.InvMassA;
                    float iA = constraint.InvIA;
                    float mB = constraint.InvMassB;
                    float iB = constraint.InvIB;

                    for (int p = 0; p < constraint.PointCount; ++p)
                    {
                        ContactConstraintPoint cp = constraint.Points[p];
                        Vec2 P = cp.NormalImpulse * constraint.Normal + cp.TangentImpulse * constraint.Tangent;
                        cp.TotalNormalImpulse += cp.NormalImpulse;

                        wA -= iA * Vec2.Cross(cp.RA, P);
                        vA -= mA * P;
                        wB += iB * Vec2.Cross(cp.RB, P);
                        vB += mB * P;

                        constraint.Points[p] = cp;
                    }

                    _world._bodyLinearVelocities[indexA] = vA;
                    _world._bodyAngularVelocities[indexA] = wA;
                    _world._bodyLinearVelocities[indexB] = vB;
                    _world._bodyAngularVelocities[indexB] = wB;
                }
            }

            public void SolveVelocity(bool useBias)
            {
                int width = Vector<float>.Count;
                int simdBatches = 0;
                int simdLanes = 0;
                for (int i = 0; i < _scalarConstraints.Count; ++i)
                {
                    SolveVelocityScalar(_scalarConstraints[i], useBias);
                }

                if (width <= 1)
                {
                    for (int c = 0; c < _colorsSingle.Count; ++c)
                    {
                        System.Collections.Generic.List<int> list = _colorsSingle[c];
                        for (int i = 0; i < list.Count; ++i)
                        {
                            SolveVelocityScalar(list[i], useBias);
                        }
                    }
                    for (int c = 0; c < _colorsTwo.Count; ++c)
                    {
                        System.Collections.Generic.List<int> list = _colorsTwo[c];
                        for (int i = 0; i < list.Count; ++i)
                        {
                            SolveVelocityScalar(list[i], useBias);
                        }
                    }
                    return;
                }

                for (int c = 0; c < _colorsSingle.Count; ++c)
                {
                    System.Collections.Generic.List<int> list = _colorsSingle[c];
                    int i = 0;
                    for (; i + width <= list.Count; i += width)
                    {
                        int[] indices = new int[width];
                        for (int lane = 0; lane < width; ++lane)
                        {
                            indices[lane] = list[i + lane];
                        }
                        SolveVelocityBatch(indices, useBias);
                        simdBatches++;
                        simdLanes += width;
                    }

                    for (; i < list.Count; ++i)
                    {
                        SolveVelocityScalar(list[i], useBias);
                    }
                }

                for (int c = 0; c < _colorsTwo.Count; ++c)
                {
                    System.Collections.Generic.List<int> list = _colorsTwo[c];
                    int i = 0;
                    for (; i + width <= list.Count; i += width)
                    {
                        int[] indices = new int[width];
                        for (int lane = 0; lane < width; ++lane)
                        {
                            indices[lane] = list[i + lane];
                        }
                        SolveVelocityBatchTwoPoint(indices, useBias);
                        simdBatches++;
                        simdLanes += width;
                    }

                    for (; i < list.Count; ++i)
                    {
                        SolveVelocityScalar(list[i], useBias);
                    }
                }

                _stats = new World.ContactSolverStats(
                    _stats.SinglePointConstraints,
                    _stats.TwoPointConstraints,
                    _stats.ScalarConstraints,
                    _stats.Colors,
                    simdBatches,
                    simdLanes);
            }

            public void ApplyRestitution(float threshold)
            {
                int width = Vector<float>.Count;
                if (width <= 1)
                {
                    ApplyRestitutionScalar(threshold);
                    return;
                }

                ApplyRestitutionScalar(threshold, onlyScalar: true);

                for (int c = 0; c < _colorsSingle.Count; ++c)
                {
                    System.Collections.Generic.List<int> list = _colorsSingle[c];
                    int i = 0;
                    for (; i + width <= list.Count; i += width)
                    {
                        int[] indices = new int[width];
                        for (int lane = 0; lane < width; ++lane)
                        {
                            indices[lane] = list[i + lane];
                        }
                        ApplyRestitutionBatch(indices, threshold);
                    }

                    for (; i < list.Count; ++i)
                    {
                        ApplyRestitutionScalar(list[i], threshold);
                    }
                }

                for (int c = 0; c < _colorsTwo.Count; ++c)
                {
                    System.Collections.Generic.List<int> list = _colorsTwo[c];
                    int i = 0;
                    for (; i + width <= list.Count; i += width)
                    {
                        int[] indices = new int[width];
                        for (int lane = 0; lane < width; ++lane)
                        {
                            indices[lane] = list[i + lane];
                        }
                        ApplyRestitutionBatchTwoPoint(indices, threshold);
                    }

                    for (; i < list.Count; ++i)
                    {
                        ApplyRestitutionScalar(list[i], threshold);
                    }
                }
            }

            public void StoreImpulses()
            {
                for (int i = 0; i < _constraints.Count; ++i)
                {
                    ContactConstraint constraint = _constraints[i];
                    Contact contact = constraint.Contact;
                    for (int p = 0; p < constraint.PointCount; ++p)
                    {
                        ContactConstraintPoint cp = constraint.Points[p];
                        ManifoldPoint mp = contact.Manifold.Points[p];
                        contact.Manifold.Points[p] = new ManifoldPoint(mp.LocalPoint, cp.NormalImpulse, cp.TangentImpulse, mp.Id);
                    }
                }
            }

            public World.ContactSolverStats GetStats() => _stats;

            private void SolveVelocityScalar(int index, bool useBias)
            {
                ContactConstraint constraint = _constraints[index];
                int indexA = constraint.IndexA;
                int indexB = constraint.IndexB;

                Vec2 vA = _world._bodyLinearVelocities[indexA];
                float wA = _world._bodyAngularVelocities[indexA];
                Vec2 vB = _world._bodyLinearVelocities[indexB];
                float wB = _world._bodyAngularVelocities[indexB];

                float mA = constraint.InvMassA;
                float iA = constraint.InvIA;
                float mB = constraint.InvMassB;
                float iB = constraint.InvIB;

                float totalNormalImpulse = 0f;

                for (int p = 0; p < constraint.PointCount; ++p)
                {
                    ContactConstraintPoint cp = constraint.Points[p];

                    Vec2 vrA = vA + Vec2.Cross(wA, cp.RA);
                    Vec2 vrB = vB + Vec2.Cross(wB, cp.RB);
                    float vn = Vec2.Dot(vrB - vrA, constraint.Normal);

                    float bias = useBias ? cp.Bias : 0f;
                    float impulse = -(vn + bias + cp.Softness * cp.NormalImpulse) * cp.NormalMass;
                    float newImpulse = MathF.Max(cp.NormalImpulse + impulse, 0f);
                    impulse = newImpulse - cp.NormalImpulse;
                    cp.NormalImpulse = newImpulse;
                    cp.TotalNormalImpulse += impulse;
                    totalNormalImpulse += newImpulse;

                    Vec2 P = impulse * constraint.Normal;
                    vA -= mA * P;
                    wA -= iA * Vec2.Cross(cp.RA, P);
                    vB += mB * P;
                    wB += iB * Vec2.Cross(cp.RB, P);

                    constraint.Points[p] = cp;
                }

                for (int p = 0; p < constraint.PointCount; ++p)
                {
                    ContactConstraintPoint cp = constraint.Points[p];

                    Vec2 vrA = vA + Vec2.Cross(wA, cp.RA);
                    Vec2 vrB = vB + Vec2.Cross(wB, cp.RB);
                    float vt = Vec2.Dot(vrB - vrA, constraint.Tangent) - constraint.TangentSpeed;

                    float impulse = -vt * cp.TangentMass;
                    float maxFriction = constraint.Friction * cp.NormalImpulse;
                    float newImpulse = MathFng.Clamp(cp.TangentImpulse + impulse, -maxFriction, maxFriction);
                    impulse = newImpulse - cp.TangentImpulse;
                    cp.TangentImpulse = newImpulse;

                    Vec2 P = impulse * constraint.Tangent;
                    vA -= mA * P;
                    wA -= iA * Vec2.Cross(cp.RA, P);
                    vB += mB * P;
                    wB += iB * Vec2.Cross(cp.RB, P);

                    constraint.Points[p] = cp;
                }

                if (constraint.RollingResistance > 0f)
                {
                    float deltaLambda = -constraint.RollingMass * (wB - wA);
                    float lambda = constraint.RollingImpulse;
                    float maxLambda = constraint.RollingResistance * totalNormalImpulse;
                    constraint.RollingImpulse = MathFng.Clamp(lambda + deltaLambda, -maxLambda, maxLambda);
                    deltaLambda = constraint.RollingImpulse - lambda;
                    wA -= iA * deltaLambda;
                    wB += iB * deltaLambda;
                }

                _world._bodyLinearVelocities[indexA] = vA;
                _world._bodyAngularVelocities[indexA] = wA;
                _world._bodyLinearVelocities[indexB] = vB;
                _world._bodyAngularVelocities[indexB] = wB;

                _constraints[index] = constraint;
            }

            private void SolveVelocityBatch(int[] indices, bool useBias)
            {
                int width = Vector<float>.Count;

                float[] vAx = new float[width];
                float[] vAy = new float[width];
                float[] wA = new float[width];
                float[] vBx = new float[width];
                float[] vBy = new float[width];
                float[] wB = new float[width];
                float[] invMassA = new float[width];
                float[] invIA = new float[width];
                float[] invMassB = new float[width];
                float[] invIB = new float[width];
                float[] rAx = new float[width];
                float[] rAy = new float[width];
                float[] rBx = new float[width];
                float[] rBy = new float[width];
                float[] normalX = new float[width];
                float[] normalY = new float[width];
                float[] tangentX = new float[width];
                float[] tangentY = new float[width];
                float[] normalImpulse = new float[width];
                float[] tangentImpulse = new float[width];
                float[] normalMass = new float[width];
                float[] tangentMass = new float[width];
                float[] friction = new float[width];
                float[] bias = new float[width];
                float[] softness = new float[width];

                for (int lane = 0; lane < width; ++lane)
                {
                    ContactConstraint constraint = _constraints[indices[lane]];
                    ContactConstraintPoint cp = constraint.Points[0];

                    int indexA = constraint.IndexA;
                    int indexB = constraint.IndexB;

                    vAx[lane] = _world._bodyLinearVelocities[indexA].X;
                    vAy[lane] = _world._bodyLinearVelocities[indexA].Y;
                    wA[lane] = _world._bodyAngularVelocities[indexA];
                    vBx[lane] = _world._bodyLinearVelocities[indexB].X;
                    vBy[lane] = _world._bodyLinearVelocities[indexB].Y;
                    wB[lane] = _world._bodyAngularVelocities[indexB];

                    invMassA[lane] = constraint.InvMassA;
                    invIA[lane] = constraint.InvIA;
                    invMassB[lane] = constraint.InvMassB;
                    invIB[lane] = constraint.InvIB;

                    rAx[lane] = cp.RA.X;
                    rAy[lane] = cp.RA.Y;
                    rBx[lane] = cp.RB.X;
                    rBy[lane] = cp.RB.Y;

                    normalX[lane] = constraint.Normal.X;
                    normalY[lane] = constraint.Normal.Y;
                    tangentX[lane] = constraint.Tangent.X;
                    tangentY[lane] = constraint.Tangent.Y;

                    normalImpulse[lane] = cp.NormalImpulse;
                    tangentImpulse[lane] = cp.TangentImpulse;
                    normalMass[lane] = cp.NormalMass;
                    tangentMass[lane] = cp.TangentMass;
                    friction[lane] = constraint.Friction;
                    bias[lane] = useBias ? cp.Bias : 0f;
                    softness[lane] = cp.Softness;
                }

                Vector<float> vAxV = new Vector<float>(vAx);
                Vector<float> vAyV = new Vector<float>(vAy);
                Vector<float> wAV = new Vector<float>(wA);
                Vector<float> vBxV = new Vector<float>(vBx);
                Vector<float> vByV = new Vector<float>(vBy);
                Vector<float> wBV = new Vector<float>(wB);

                Vector<float> rAxV = new Vector<float>(rAx);
                Vector<float> rAyV = new Vector<float>(rAy);
                Vector<float> rBxV = new Vector<float>(rBx);
                Vector<float> rByV = new Vector<float>(rBy);

                Vector<float> normalXV = new Vector<float>(normalX);
                Vector<float> normalYV = new Vector<float>(normalY);
                Vector<float> tangentXV = new Vector<float>(tangentX);
                Vector<float> tangentYV = new Vector<float>(tangentY);

                Vector<float> invMassAV = new Vector<float>(invMassA);
                Vector<float> invIAV = new Vector<float>(invIA);
                Vector<float> invMassBV = new Vector<float>(invMassB);
                Vector<float> invIBV = new Vector<float>(invIB);

                Vector<float> normalImpulseV = new Vector<float>(normalImpulse);
                Vector<float> tangentImpulseV = new Vector<float>(tangentImpulse);
                Vector<float> normalMassV = new Vector<float>(normalMass);
                Vector<float> tangentMassV = new Vector<float>(tangentMass);
                Vector<float> frictionV = new Vector<float>(friction);
                Vector<float> biasV = new Vector<float>(bias);
                Vector<float> softnessV = new Vector<float>(softness);

                Vector<float> crossAx = -wAV * rAyV;
                Vector<float> crossAy = wAV * rAxV;
                Vector<float> crossBx = -wBV * rByV;
                Vector<float> crossBy = wBV * rBxV;

                Vector<float> dvx = (vBxV + crossBx) - (vAxV + crossAx);
                Vector<float> dvy = (vByV + crossBy) - (vAyV + crossAy);

                Vector<float> vn = dvx * normalXV + dvy * normalYV;
                Vector<float> impulse = -(vn + biasV + softnessV * normalImpulseV) * normalMassV;
                Vector<float> newImpulse = Vector.Max(normalImpulseV + impulse, Vector<float>.Zero);
                Vector<float> delta = newImpulse - normalImpulseV;
                normalImpulseV = newImpulse;

                Vector<float> Px = delta * normalXV;
                Vector<float> Py = delta * normalYV;
                vAxV -= invMassAV * Px;
                vAyV -= invMassAV * Py;
                wAV -= invIAV * (rAxV * Py - rAyV * Px);
                vBxV += invMassBV * Px;
                vByV += invMassBV * Py;
                wBV += invIBV * (rBxV * Py - rByV * Px);

                crossAx = -wAV * rAyV;
                crossAy = wAV * rAxV;
                crossBx = -wBV * rByV;
                crossBy = wBV * rBxV;

                dvx = (vBxV + crossBx) - (vAxV + crossAx);
                dvy = (vByV + crossBy) - (vAyV + crossAy);
                Vector<float> vt = dvx * tangentXV + dvy * tangentYV;

                Vector<float> tImpulse = -vt * tangentMassV;
                Vector<float> maxFriction = frictionV * normalImpulseV;
                Vector<float> newTangentImpulse = Vector.Min(Vector.Max(tangentImpulseV + tImpulse, -maxFriction), maxFriction);
                Vector<float> deltaT = newTangentImpulse - tangentImpulseV;
                tangentImpulseV = newTangentImpulse;

                Px = deltaT * tangentXV;
                Py = deltaT * tangentYV;
                vAxV -= invMassAV * Px;
                vAyV -= invMassAV * Py;
                wAV -= invIAV * (rAxV * Py - rAyV * Px);
                vBxV += invMassBV * Px;
                vByV += invMassBV * Py;
                wBV += invIBV * (rBxV * Py - rByV * Px);

                vAxV.CopyTo(vAx);
                vAyV.CopyTo(vAy);
                wAV.CopyTo(wA);
                vBxV.CopyTo(vBx);
                vByV.CopyTo(vBy);
                wBV.CopyTo(wB);
                normalImpulseV.CopyTo(normalImpulse);
                tangentImpulseV.CopyTo(tangentImpulse);

                for (int lane = 0; lane < width; ++lane)
                {
                    int idx = indices[lane];
                    ContactConstraint constraint = _constraints[idx];
                    ContactConstraintPoint cp = constraint.Points[0];

                    int indexA = constraint.IndexA;
                    int indexB = constraint.IndexB;
                    _world._bodyLinearVelocities[indexA] = new Vec2(vAx[lane], vAy[lane]);
                    _world._bodyAngularVelocities[indexA] = wA[lane];
                    _world._bodyLinearVelocities[indexB] = new Vec2(vBx[lane], vBy[lane]);
                    _world._bodyAngularVelocities[indexB] = wB[lane];

                    cp.NormalImpulse = normalImpulse[lane];
                    cp.TangentImpulse = tangentImpulse[lane];
                    constraint.Points[0] = cp;

                    _constraints[idx] = constraint;
                }
            }

            private void SolveVelocityBatchTwoPoint(int[] indices, bool useBias)
            {
                int width = Vector<float>.Count;

                float[] vAx = new float[width];
                float[] vAy = new float[width];
                float[] wA = new float[width];
                float[] vBx = new float[width];
                float[] vBy = new float[width];
                float[] wB = new float[width];
                float[] invMassA = new float[width];
                float[] invIA = new float[width];
                float[] invMassB = new float[width];
                float[] invIB = new float[width];
                float[] normalX = new float[width];
                float[] normalY = new float[width];
                float[] tangentX = new float[width];
                float[] tangentY = new float[width];
                float[] friction = new float[width];

                float[] rA1x = new float[width];
                float[] rA1y = new float[width];
                float[] rB1x = new float[width];
                float[] rB1y = new float[width];
                float[] nImp1 = new float[width];
                float[] tImp1 = new float[width];
                float[] nMass1 = new float[width];
                float[] tMass1 = new float[width];
                float[] bias1 = new float[width];
                float[] softness1 = new float[width];

                float[] rA2x = new float[width];
                float[] rA2y = new float[width];
                float[] rB2x = new float[width];
                float[] rB2y = new float[width];
                float[] nImp2 = new float[width];
                float[] tImp2 = new float[width];
                float[] nMass2 = new float[width];
                float[] tMass2 = new float[width];
                float[] bias2 = new float[width];
                float[] softness2 = new float[width];

                for (int lane = 0; lane < width; ++lane)
                {
                    ContactConstraint constraint = _constraints[indices[lane]];
                    ContactConstraintPoint cp1 = constraint.Points[0];
                    ContactConstraintPoint cp2 = constraint.Points[1];
                    int indexA = constraint.IndexA;
                    int indexB = constraint.IndexB;

                    vAx[lane] = _world._bodyLinearVelocities[indexA].X;
                    vAy[lane] = _world._bodyLinearVelocities[indexA].Y;
                    wA[lane] = _world._bodyAngularVelocities[indexA];
                    vBx[lane] = _world._bodyLinearVelocities[indexB].X;
                    vBy[lane] = _world._bodyLinearVelocities[indexB].Y;
                    wB[lane] = _world._bodyAngularVelocities[indexB];

                    invMassA[lane] = constraint.InvMassA;
                    invIA[lane] = constraint.InvIA;
                    invMassB[lane] = constraint.InvMassB;
                    invIB[lane] = constraint.InvIB;

                    normalX[lane] = constraint.Normal.X;
                    normalY[lane] = constraint.Normal.Y;
                    tangentX[lane] = constraint.Tangent.X;
                    tangentY[lane] = constraint.Tangent.Y;
                    friction[lane] = constraint.Friction;

                    rA1x[lane] = cp1.RA.X;
                    rA1y[lane] = cp1.RA.Y;
                    rB1x[lane] = cp1.RB.X;
                    rB1y[lane] = cp1.RB.Y;
                    nImp1[lane] = cp1.NormalImpulse;
                    tImp1[lane] = cp1.TangentImpulse;
                    nMass1[lane] = cp1.NormalMass;
                    tMass1[lane] = cp1.TangentMass;
                    bias1[lane] = useBias ? cp1.Bias : 0f;
                    softness1[lane] = cp1.Softness;

                    rA2x[lane] = cp2.RA.X;
                    rA2y[lane] = cp2.RA.Y;
                    rB2x[lane] = cp2.RB.X;
                    rB2y[lane] = cp2.RB.Y;
                    nImp2[lane] = cp2.NormalImpulse;
                    tImp2[lane] = cp2.TangentImpulse;
                    nMass2[lane] = cp2.NormalMass;
                    tMass2[lane] = cp2.TangentMass;
                    bias2[lane] = useBias ? cp2.Bias : 0f;
                    softness2[lane] = cp2.Softness;
                }

                Vector<float> vAxV = new Vector<float>(vAx);
                Vector<float> vAyV = new Vector<float>(vAy);
                Vector<float> wAV = new Vector<float>(wA);
                Vector<float> vBxV = new Vector<float>(vBx);
                Vector<float> vByV = new Vector<float>(vBy);
                Vector<float> wBV = new Vector<float>(wB);
                Vector<float> invMassAV = new Vector<float>(invMassA);
                Vector<float> invIAV = new Vector<float>(invIA);
                Vector<float> invMassBV = new Vector<float>(invMassB);
                Vector<float> invIBV = new Vector<float>(invIB);
                Vector<float> normalXV = new Vector<float>(normalX);
                Vector<float> normalYV = new Vector<float>(normalY);
                Vector<float> tangentXV = new Vector<float>(tangentX);
                Vector<float> tangentYV = new Vector<float>(tangentY);
                Vector<float> frictionV = new Vector<float>(friction);

                Vector<float> rA1xV = new Vector<float>(rA1x);
                Vector<float> rA1yV = new Vector<float>(rA1y);
                Vector<float> rB1xV = new Vector<float>(rB1x);
                Vector<float> rB1yV = new Vector<float>(rB1y);
                Vector<float> nImp1V = new Vector<float>(nImp1);
                Vector<float> tImp1V = new Vector<float>(tImp1);
                Vector<float> nMass1V = new Vector<float>(nMass1);
                Vector<float> tMass1V = new Vector<float>(tMass1);
                Vector<float> bias1V = new Vector<float>(bias1);
                Vector<float> softness1V = new Vector<float>(softness1);

                Vector<float> rA2xV = new Vector<float>(rA2x);
                Vector<float> rA2yV = new Vector<float>(rA2y);
                Vector<float> rB2xV = new Vector<float>(rB2x);
                Vector<float> rB2yV = new Vector<float>(rB2y);
                Vector<float> nImp2V = new Vector<float>(nImp2);
                Vector<float> tImp2V = new Vector<float>(tImp2);
                Vector<float> nMass2V = new Vector<float>(nMass2);
                Vector<float> tMass2V = new Vector<float>(tMass2);
                Vector<float> bias2V = new Vector<float>(bias2);
                Vector<float> softness2V = new Vector<float>(softness2);

                Vector<float> crossAx = -wAV * rA1yV;
                Vector<float> crossAy = wAV * rA1xV;
                Vector<float> crossBx = -wBV * rB1yV;
                Vector<float> crossBy = wBV * rB1xV;
                Vector<float> dvx = (vBxV + crossBx) - (vAxV + crossAx);
                Vector<float> dvy = (vByV + crossBy) - (vAyV + crossAy);
                Vector<float> vn = dvx * normalXV + dvy * normalYV;
                Vector<float> impulse = -(vn + bias1V + softness1V * nImp1V) * nMass1V;
                Vector<float> newImp = Vector.Max(nImp1V + impulse, Vector<float>.Zero);
                Vector<float> delta = newImp - nImp1V;
                nImp1V = newImp;

                Vector<float> Px = delta * normalXV;
                Vector<float> Py = delta * normalYV;
                vAxV -= invMassAV * Px;
                vAyV -= invMassAV * Py;
                wAV -= invIAV * (rA1xV * Py - rA1yV * Px);
                vBxV += invMassBV * Px;
                vByV += invMassBV * Py;
                wBV += invIBV * (rB1xV * Py - rB1yV * Px);

                crossAx = -wAV * rA2yV;
                crossAy = wAV * rA2xV;
                crossBx = -wBV * rB2yV;
                crossBy = wBV * rB2xV;
                dvx = (vBxV + crossBx) - (vAxV + crossAx);
                dvy = (vByV + crossBy) - (vAyV + crossAy);
                vn = dvx * normalXV + dvy * normalYV;
                impulse = -(vn + bias2V + softness2V * nImp2V) * nMass2V;
                newImp = Vector.Max(nImp2V + impulse, Vector<float>.Zero);
                delta = newImp - nImp2V;
                nImp2V = newImp;

                Px = delta * normalXV;
                Py = delta * normalYV;
                vAxV -= invMassAV * Px;
                vAyV -= invMassAV * Py;
                wAV -= invIAV * (rA2xV * Py - rA2yV * Px);
                vBxV += invMassBV * Px;
                vByV += invMassBV * Py;
                wBV += invIBV * (rB2xV * Py - rB2yV * Px);

                crossAx = -wAV * rA1yV;
                crossAy = wAV * rA1xV;
                crossBx = -wBV * rB1yV;
                crossBy = wBV * rB1xV;
                dvx = (vBxV + crossBx) - (vAxV + crossAx);
                dvy = (vByV + crossBy) - (vAyV + crossAy);
                Vector<float> vt = dvx * tangentXV + dvy * tangentYV;
                Vector<float> tImpulse = -vt * tMass1V;
                Vector<float> maxFriction = frictionV * nImp1V;
                Vector<float> newT = Vector.Min(Vector.Max(tImp1V + tImpulse, -maxFriction), maxFriction);
                Vector<float> deltaT = newT - tImp1V;
                tImp1V = newT;
                Px = deltaT * tangentXV;
                Py = deltaT * tangentYV;
                vAxV -= invMassAV * Px;
                vAyV -= invMassAV * Py;
                wAV -= invIAV * (rA1xV * Py - rA1yV * Px);
                vBxV += invMassBV * Px;
                vByV += invMassBV * Py;
                wBV += invIBV * (rB1xV * Py - rB1yV * Px);

                crossAx = -wAV * rA2yV;
                crossAy = wAV * rA2xV;
                crossBx = -wBV * rB2yV;
                crossBy = wBV * rB2xV;
                dvx = (vBxV + crossBx) - (vAxV + crossAx);
                dvy = (vByV + crossBy) - (vAyV + crossAy);
                vt = dvx * tangentXV + dvy * tangentYV;
                tImpulse = -vt * tMass2V;
                maxFriction = frictionV * nImp2V;
                newT = Vector.Min(Vector.Max(tImp2V + tImpulse, -maxFriction), maxFriction);
                deltaT = newT - tImp2V;
                tImp2V = newT;
                Px = deltaT * tangentXV;
                Py = deltaT * tangentYV;
                vAxV -= invMassAV * Px;
                vAyV -= invMassAV * Py;
                wAV -= invIAV * (rA2xV * Py - rA2yV * Px);
                vBxV += invMassBV * Px;
                vByV += invMassBV * Py;
                wBV += invIBV * (rB2xV * Py - rB2yV * Px);

                vAxV.CopyTo(vAx);
                vAyV.CopyTo(vAy);
                wAV.CopyTo(wA);
                vBxV.CopyTo(vBx);
                vByV.CopyTo(vBy);
                wBV.CopyTo(wB);
                nImp1V.CopyTo(nImp1);
                tImp1V.CopyTo(tImp1);
                nImp2V.CopyTo(nImp2);
                tImp2V.CopyTo(tImp2);

                for (int lane = 0; lane < width; ++lane)
                {
                    int idx = indices[lane];
                    ContactConstraint constraint = _constraints[idx];
                    ContactConstraintPoint cp1 = constraint.Points[0];
                    ContactConstraintPoint cp2 = constraint.Points[1];

                    int indexA = constraint.IndexA;
                    int indexB = constraint.IndexB;
                    _world._bodyLinearVelocities[indexA] = new Vec2(vAx[lane], vAy[lane]);
                    _world._bodyAngularVelocities[indexA] = wA[lane];
                    _world._bodyLinearVelocities[indexB] = new Vec2(vBx[lane], vBy[lane]);
                    _world._bodyAngularVelocities[indexB] = wB[lane];

                    cp1.NormalImpulse = nImp1[lane];
                    cp1.TangentImpulse = tImp1[lane];
                    cp2.NormalImpulse = nImp2[lane];
                    cp2.TangentImpulse = tImp2[lane];

                    constraint.Points[0] = cp1;
                    constraint.Points[1] = cp2;
                    _constraints[idx] = constraint;
                }
            }

            private void ApplyRestitutionScalar(float threshold, bool onlyScalar)
            {
                if (onlyScalar)
                {
                    for (int i = 0; i < _scalarConstraints.Count; ++i)
                    {
                        ApplyRestitutionScalar(_scalarConstraints[i], threshold);
                    }
                    return;
                }

                for (int i = 0; i < _constraints.Count; ++i)
                {
                    ApplyRestitutionScalar(i, threshold);
                }
            }

            private void ApplyRestitutionScalar(float threshold)
            {
                ApplyRestitutionScalar(threshold, onlyScalar: false);
            }

            private void ApplyRestitutionScalar(int index, float threshold)
            {
                ContactConstraint constraint = _constraints[index];
                if (constraint.Restitution == 0f)
                {
                    return;
                }

                int indexA = constraint.IndexA;
                int indexB = constraint.IndexB;

                Vec2 vA = _world._bodyLinearVelocities[indexA];
                float wA = _world._bodyAngularVelocities[indexA];
                Vec2 vB = _world._bodyLinearVelocities[indexB];
                float wB = _world._bodyAngularVelocities[indexB];

                float mA = constraint.InvMassA;
                float iA = constraint.InvIA;
                float mB = constraint.InvMassB;
                float iB = constraint.InvIB;

                for (int p = 0; p < constraint.PointCount; ++p)
                {
                    ContactConstraintPoint cp = constraint.Points[p];
                    if (cp.RelativeVelocity > -threshold || cp.TotalNormalImpulse == 0f)
                    {
                        continue;
                    }

                    Vec2 vrA = vA + Vec2.Cross(wA, cp.RA);
                    Vec2 vrB = vB + Vec2.Cross(wB, cp.RB);
                    float vn = Vec2.Dot(vrB - vrA, constraint.Normal);

                    float impulse = -cp.NormalMass * (vn + constraint.Restitution * cp.RelativeVelocity);
                    float newImpulse = MathF.Max(cp.NormalImpulse + impulse, 0f);
                    impulse = newImpulse - cp.NormalImpulse;
                    cp.NormalImpulse = newImpulse;
                    cp.TotalNormalImpulse += impulse;

                    Vec2 P = impulse * constraint.Normal;
                    vA -= mA * P;
                    wA -= iA * Vec2.Cross(cp.RA, P);
                    vB += mB * P;
                    wB += iB * Vec2.Cross(cp.RB, P);

                    constraint.Points[p] = cp;
                }

                _world._bodyLinearVelocities[indexA] = vA;
                _world._bodyAngularVelocities[indexA] = wA;
                _world._bodyLinearVelocities[indexB] = vB;
                _world._bodyAngularVelocities[indexB] = wB;

                _constraints[index] = constraint;
            }

            private void ApplyRestitutionBatch(int[] indices, float threshold)
            {
                int width = Vector<float>.Count;

                float[] vAx = new float[width];
                float[] vAy = new float[width];
                float[] wA = new float[width];
                float[] vBx = new float[width];
                float[] vBy = new float[width];
                float[] wB = new float[width];
                float[] invMassA = new float[width];
                float[] invIA = new float[width];
                float[] invMassB = new float[width];
                float[] invIB = new float[width];
                float[] rAx = new float[width];
                float[] rAy = new float[width];
                float[] rBx = new float[width];
                float[] rBy = new float[width];
                float[] normalX = new float[width];
                float[] normalY = new float[width];
                float[] normalImpulse = new float[width];
                float[] normalMass = new float[width];
                float[] restitution = new float[width];
                float[] relVel = new float[width];
                float[] totalImpulse = new float[width];

                for (int lane = 0; lane < width; ++lane)
                {
                    ContactConstraint constraint = _constraints[indices[lane]];
                    ContactConstraintPoint cp = constraint.Points[0];
                    int indexA = constraint.IndexA;
                    int indexB = constraint.IndexB;

                    vAx[lane] = _world._bodyLinearVelocities[indexA].X;
                    vAy[lane] = _world._bodyLinearVelocities[indexA].Y;
                    wA[lane] = _world._bodyAngularVelocities[indexA];
                    vBx[lane] = _world._bodyLinearVelocities[indexB].X;
                    vBy[lane] = _world._bodyLinearVelocities[indexB].Y;
                    wB[lane] = _world._bodyAngularVelocities[indexB];

                    invMassA[lane] = constraint.InvMassA;
                    invIA[lane] = constraint.InvIA;
                    invMassB[lane] = constraint.InvMassB;
                    invIB[lane] = constraint.InvIB;

                    rAx[lane] = cp.RA.X;
                    rAy[lane] = cp.RA.Y;
                    rBx[lane] = cp.RB.X;
                    rBy[lane] = cp.RB.Y;

                    normalX[lane] = constraint.Normal.X;
                    normalY[lane] = constraint.Normal.Y;
                    normalImpulse[lane] = cp.NormalImpulse;
                    normalMass[lane] = cp.NormalMass;
                    restitution[lane] = constraint.Restitution;
                    relVel[lane] = cp.RelativeVelocity;
                    totalImpulse[lane] = cp.TotalNormalImpulse;
                }

                Vector<float> vAxV = new Vector<float>(vAx);
                Vector<float> vAyV = new Vector<float>(vAy);
                Vector<float> wAV = new Vector<float>(wA);
                Vector<float> vBxV = new Vector<float>(vBx);
                Vector<float> vByV = new Vector<float>(vBy);
                Vector<float> wBV = new Vector<float>(wB);

                Vector<float> rAxV = new Vector<float>(rAx);
                Vector<float> rAyV = new Vector<float>(rAy);
                Vector<float> rBxV = new Vector<float>(rBx);
                Vector<float> rByV = new Vector<float>(rBy);

                Vector<float> normalXV = new Vector<float>(normalX);
                Vector<float> normalYV = new Vector<float>(normalY);
                Vector<float> nImpV = new Vector<float>(normalImpulse);
                Vector<float> nMassV = new Vector<float>(normalMass);
                Vector<float> restitutionV = new Vector<float>(restitution);
                Vector<float> relVelV = new Vector<float>(relVel);
                Vector<float> totalImpV = new Vector<float>(totalImpulse);

                Vector<float> thresholdV = new Vector<float>(-threshold);
                Vector<float> zero = Vector<float>.Zero;

                Vector<int> applyMask = Vector.GreaterThan(relVelV, thresholdV);
                applyMask = Vector.BitwiseOr(applyMask, Vector.Equals(totalImpV, zero));
                applyMask = Vector.BitwiseOr(applyMask, Vector.Equals(restitutionV, zero));
                Vector<float> applyMaskF = Vector.AsVectorSingle(applyMask);

                Vector<float> crossAx = -wAV * rAyV;
                Vector<float> crossAy = wAV * rAxV;
                Vector<float> crossBx = -wBV * rByV;
                Vector<float> crossBy = wBV * rBxV;
                Vector<float> dvx = (vBxV + crossBx) - (vAxV + crossAx);
                Vector<float> dvy = (vByV + crossBy) - (vAyV + crossAy);
                Vector<float> vn = dvx * normalXV + dvy * normalYV;

                Vector<float> impulse = -nMassV * (vn + restitutionV * relVelV);
                Vector<float> newImp = Vector.Max(nImpV + impulse, zero);
                Vector<float> delta = newImp - nImpV;

                delta = Vector.ConditionalSelect(applyMaskF, zero, delta);
                newImp = Vector.ConditionalSelect(applyMaskF, nImpV, newImp);

                nImpV = newImp;
                totalImpV += delta;

                Vector<float> Px = delta * normalXV;
                Vector<float> Py = delta * normalYV;
                Vector<float> invMassAV = new Vector<float>(invMassA);
                Vector<float> invIAV = new Vector<float>(invIA);
                Vector<float> invMassBV = new Vector<float>(invMassB);
                Vector<float> invIBV = new Vector<float>(invIB);
                vAxV -= invMassAV * Px;
                vAyV -= invMassAV * Py;
                wAV -= invIAV * (rAxV * Py - rAyV * Px);
                vBxV += invMassBV * Px;
                vByV += invMassBV * Py;
                wBV += invIBV * (rBxV * Py - rByV * Px);

                vAxV.CopyTo(vAx);
                vAyV.CopyTo(vAy);
                wAV.CopyTo(wA);
                vBxV.CopyTo(vBx);
                vByV.CopyTo(vBy);
                wBV.CopyTo(wB);
                nImpV.CopyTo(normalImpulse);
                totalImpV.CopyTo(totalImpulse);

                for (int lane = 0; lane < width; ++lane)
                {
                    int idx = indices[lane];
                    ContactConstraint constraint = _constraints[idx];
                    ContactConstraintPoint cp = constraint.Points[0];
                    int indexA = constraint.IndexA;
                    int indexB = constraint.IndexB;
                    _world._bodyLinearVelocities[indexA] = new Vec2(vAx[lane], vAy[lane]);
                    _world._bodyAngularVelocities[indexA] = wA[lane];
                    _world._bodyLinearVelocities[indexB] = new Vec2(vBx[lane], vBy[lane]);
                    _world._bodyAngularVelocities[indexB] = wB[lane];

                    cp.NormalImpulse = normalImpulse[lane];
                    cp.TotalNormalImpulse = totalImpulse[lane];
                    constraint.Points[0] = cp;
                    _constraints[idx] = constraint;
                }
            }

            private void ApplyRestitutionBatchTwoPoint(int[] indices, float threshold)
            {
                int width = Vector<float>.Count;

                float[] vAx = new float[width];
                float[] vAy = new float[width];
                float[] wA = new float[width];
                float[] vBx = new float[width];
                float[] vBy = new float[width];
                float[] wB = new float[width];
                float[] invMassA = new float[width];
                float[] invIA = new float[width];
                float[] invMassB = new float[width];
                float[] invIB = new float[width];
                float[] normalX = new float[width];
                float[] normalY = new float[width];
                float[] restitution = new float[width];

                float[] rA1x = new float[width];
                float[] rA1y = new float[width];
                float[] rB1x = new float[width];
                float[] rB1y = new float[width];
                float[] nImp1 = new float[width];
                float[] nMass1 = new float[width];
                float[] relVel1 = new float[width];
                float[] totalImp1 = new float[width];

                float[] rA2x = new float[width];
                float[] rA2y = new float[width];
                float[] rB2x = new float[width];
                float[] rB2y = new float[width];
                float[] nImp2 = new float[width];
                float[] nMass2 = new float[width];
                float[] relVel2 = new float[width];
                float[] totalImp2 = new float[width];

                for (int lane = 0; lane < width; ++lane)
                {
                    ContactConstraint constraint = _constraints[indices[lane]];
                    ContactConstraintPoint cp1 = constraint.Points[0];
                    ContactConstraintPoint cp2 = constraint.Points[1];
                    int indexA = constraint.IndexA;
                    int indexB = constraint.IndexB;

                    vAx[lane] = _world._bodyLinearVelocities[indexA].X;
                    vAy[lane] = _world._bodyLinearVelocities[indexA].Y;
                    wA[lane] = _world._bodyAngularVelocities[indexA];
                    vBx[lane] = _world._bodyLinearVelocities[indexB].X;
                    vBy[lane] = _world._bodyLinearVelocities[indexB].Y;
                    wB[lane] = _world._bodyAngularVelocities[indexB];

                    invMassA[lane] = constraint.InvMassA;
                    invIA[lane] = constraint.InvIA;
                    invMassB[lane] = constraint.InvMassB;
                    invIB[lane] = constraint.InvIB;

                    normalX[lane] = constraint.Normal.X;
                    normalY[lane] = constraint.Normal.Y;
                    restitution[lane] = constraint.Restitution;

                    rA1x[lane] = cp1.RA.X;
                    rA1y[lane] = cp1.RA.Y;
                    rB1x[lane] = cp1.RB.X;
                    rB1y[lane] = cp1.RB.Y;
                    nImp1[lane] = cp1.NormalImpulse;
                    nMass1[lane] = cp1.NormalMass;
                    relVel1[lane] = cp1.RelativeVelocity;
                    totalImp1[lane] = cp1.TotalNormalImpulse;

                    rA2x[lane] = cp2.RA.X;
                    rA2y[lane] = cp2.RA.Y;
                    rB2x[lane] = cp2.RB.X;
                    rB2y[lane] = cp2.RB.Y;
                    nImp2[lane] = cp2.NormalImpulse;
                    nMass2[lane] = cp2.NormalMass;
                    relVel2[lane] = cp2.RelativeVelocity;
                    totalImp2[lane] = cp2.TotalNormalImpulse;
                }

                Vector<float> vAxV = new Vector<float>(vAx);
                Vector<float> vAyV = new Vector<float>(vAy);
                Vector<float> wAV = new Vector<float>(wA);
                Vector<float> vBxV = new Vector<float>(vBx);
                Vector<float> vByV = new Vector<float>(vBy);
                Vector<float> wBV = new Vector<float>(wB);
                Vector<float> invMassAV = new Vector<float>(invMassA);
                Vector<float> invIAV = new Vector<float>(invIA);
                Vector<float> invMassBV = new Vector<float>(invMassB);
                Vector<float> invIBV = new Vector<float>(invIB);
                Vector<float> normalXV = new Vector<float>(normalX);
                Vector<float> normalYV = new Vector<float>(normalY);
                Vector<float> restitutionV = new Vector<float>(restitution);
                Vector<float> thresholdV = new Vector<float>(-threshold);
                Vector<float> zero = Vector<float>.Zero;

                Vector<float> rA1xV = new Vector<float>(rA1x);
                Vector<float> rA1yV = new Vector<float>(rA1y);
                Vector<float> rB1xV = new Vector<float>(rB1x);
                Vector<float> rB1yV = new Vector<float>(rB1y);
                Vector<float> nImp1V = new Vector<float>(nImp1);
                Vector<float> nMass1V = new Vector<float>(nMass1);
                Vector<float> relVel1V = new Vector<float>(relVel1);
                Vector<float> totalImp1V = new Vector<float>(totalImp1);

                Vector<int> mask1 = Vector.GreaterThan(relVel1V, thresholdV);
                mask1 = Vector.BitwiseOr(mask1, Vector.Equals(totalImp1V, zero));
                mask1 = Vector.BitwiseOr(mask1, Vector.Equals(restitutionV, zero));
                Vector<float> mask1F = Vector.AsVectorSingle(mask1);

                Vector<float> crossAx = -wAV * rA1yV;
                Vector<float> crossAy = wAV * rA1xV;
                Vector<float> crossBx = -wBV * rB1yV;
                Vector<float> crossBy = wBV * rB1xV;
                Vector<float> dvx = (vBxV + crossBx) - (vAxV + crossAx);
                Vector<float> dvy = (vByV + crossBy) - (vAyV + crossAy);
                Vector<float> vn = dvx * normalXV + dvy * normalYV;
                Vector<float> impulse = -nMass1V * (vn + restitutionV * relVel1V);
                Vector<float> newImp = Vector.Max(nImp1V + impulse, zero);
                Vector<float> delta = newImp - nImp1V;
                delta = Vector.ConditionalSelect(mask1F, zero, delta);
                newImp = Vector.ConditionalSelect(mask1F, nImp1V, newImp);
                nImp1V = newImp;
                totalImp1V += delta;

                Vector<float> Px = delta * normalXV;
                Vector<float> Py = delta * normalYV;
                vAxV -= invMassAV * Px;
                vAyV -= invMassAV * Py;
                wAV -= invIAV * (rA1xV * Py - rA1yV * Px);
                vBxV += invMassBV * Px;
                vByV += invMassBV * Py;
                wBV += invIBV * (rB1xV * Py - rB1yV * Px);

                Vector<float> rA2xV = new Vector<float>(rA2x);
                Vector<float> rA2yV = new Vector<float>(rA2y);
                Vector<float> rB2xV = new Vector<float>(rB2x);
                Vector<float> rB2yV = new Vector<float>(rB2y);
                Vector<float> nImp2V = new Vector<float>(nImp2);
                Vector<float> nMass2V = new Vector<float>(nMass2);
                Vector<float> relVel2V = new Vector<float>(relVel2);
                Vector<float> totalImp2V = new Vector<float>(totalImp2);

                Vector<int> mask2 = Vector.GreaterThan(relVel2V, thresholdV);
                mask2 = Vector.BitwiseOr(mask2, Vector.Equals(totalImp2V, zero));
                mask2 = Vector.BitwiseOr(mask2, Vector.Equals(restitutionV, zero));
                Vector<float> mask2F = Vector.AsVectorSingle(mask2);

                crossAx = -wAV * rA2yV;
                crossAy = wAV * rA2xV;
                crossBx = -wBV * rB2yV;
                crossBy = wBV * rB2xV;
                dvx = (vBxV + crossBx) - (vAxV + crossAx);
                dvy = (vByV + crossBy) - (vAyV + crossAy);
                vn = dvx * normalXV + dvy * normalYV;
                impulse = -nMass2V * (vn + restitutionV * relVel2V);
                newImp = Vector.Max(nImp2V + impulse, zero);
                delta = newImp - nImp2V;
                delta = Vector.ConditionalSelect(mask2F, zero, delta);
                newImp = Vector.ConditionalSelect(mask2F, nImp2V, newImp);
                nImp2V = newImp;
                totalImp2V += delta;

                Px = delta * normalXV;
                Py = delta * normalYV;
                vAxV -= invMassAV * Px;
                vAyV -= invMassAV * Py;
                wAV -= invIAV * (rA2xV * Py - rA2yV * Px);
                vBxV += invMassBV * Px;
                vByV += invMassBV * Py;
                wBV += invIBV * (rB2xV * Py - rB2yV * Px);

                vAxV.CopyTo(vAx);
                vAyV.CopyTo(vAy);
                wAV.CopyTo(wA);
                vBxV.CopyTo(vBx);
                vByV.CopyTo(vBy);
                wBV.CopyTo(wB);
                nImp1V.CopyTo(nImp1);
                totalImp1V.CopyTo(totalImp1);
                nImp2V.CopyTo(nImp2);
                totalImp2V.CopyTo(totalImp2);

                for (int lane = 0; lane < width; ++lane)
                {
                    int idx = indices[lane];
                    ContactConstraint constraint = _constraints[idx];
                    ContactConstraintPoint cp1 = constraint.Points[0];
                    ContactConstraintPoint cp2 = constraint.Points[1];
                    int indexA = constraint.IndexA;
                    int indexB = constraint.IndexB;
                    _world._bodyLinearVelocities[indexA] = new Vec2(vAx[lane], vAy[lane]);
                    _world._bodyAngularVelocities[indexA] = wA[lane];
                    _world._bodyLinearVelocities[indexB] = new Vec2(vBx[lane], vBy[lane]);
                    _world._bodyAngularVelocities[indexB] = wB[lane];

                    cp1.NormalImpulse = nImp1[lane];
                    cp1.TotalNormalImpulse = totalImp1[lane];
                    cp2.NormalImpulse = nImp2[lane];
                    cp2.TotalNormalImpulse = totalImp2[lane];
                    constraint.Points[0] = cp1;
                    constraint.Points[1] = cp2;
                    _constraints[idx] = constraint;
                }
            }

            private static int TryAddToColors(
                int index,
                Body bodyA,
                Body bodyB,
                System.Collections.Generic.List<System.Collections.Generic.List<int>> colors,
                System.Collections.Generic.List<System.Collections.Generic.HashSet<Body>> colorBodies)
            {
                bool addA = bodyA.Type != BodyType.Static;
                bool addB = bodyB.Type != BodyType.Static;
                for (int c = 0; c < colors.Count; ++c)
                {
                    System.Collections.Generic.HashSet<Body> bodies = colorBodies[c];
                    if ((addA && bodies.Contains(bodyA)) || (addB && bodies.Contains(bodyB)))
                    {
                        continue;
                    }

                    if (addA)
                    {
                        bodies.Add(bodyA);
                    }
                    if (addB)
                    {
                        bodies.Add(bodyB);
                    }
                    colors[c].Add(index);
                    return c;
                }

                System.Collections.Generic.List<int> list = new System.Collections.Generic.List<int> { index };
                colors.Add(list);
                System.Collections.Generic.HashSet<Body> newBodies = new System.Collections.Generic.HashSet<Body>
                {
                    // Static bodies are omitted so they don't block color grouping.
                };
                if (addA)
                {
                    newBodies.Add(bodyA);
                }
                if (addB)
                {
                    newBodies.Add(bodyB);
                }
                colorBodies.Add(newBodies);
                return colors.Count - 1;
            }

            private struct ContactConstraintPoint
            {
                public Vec2 RA;
                public Vec2 RB;
                public float NormalImpulse;
                public float TangentImpulse;
                public float TotalNormalImpulse;
                public float NormalMass;
                public float TangentMass;
                public float BaseSeparation;
                public float RelativeVelocity;
                public float Bias;
                public float Softness;
            }

            private struct ContactConstraint
            {
                public Contact Contact;
                public int IndexA;
                public int IndexB;
                public Vec2 Normal;
                public Vec2 Tangent;
                public int PointCount;
                public float Friction;
                public float Restitution;
                public float RollingResistance;
                public float RollingImpulse;
                public float RollingMass;
                public float TangentSpeed;
                public float InvMassA;
                public float InvIA;
                public float InvMassB;
                public float InvIB;
                public ContactConstraintPoint[] Points;

                public ContactConstraint(Contact contact, int indexA, int indexB, Vec2 normal, Vec2 tangent, int pointCount)
                {
                    Contact = contact;
                    IndexA = indexA;
                    IndexB = indexB;
                    Normal = normal;
                    Tangent = tangent;
                    PointCount = pointCount;
                    Friction = 0f;
                    Restitution = 0f;
                    RollingResistance = 0f;
                    RollingImpulse = 0f;
                    RollingMass = 0f;
                    TangentSpeed = 0f;
                    InvMassA = 0f;
                    InvIA = 0f;
                    InvMassB = 0f;
                    InvIB = 0f;
                    Points = new ContactConstraintPoint[2];
                }
            }
        }
    }
}
