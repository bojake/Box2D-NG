using System;
using System.Collections.Generic;

namespace Box2DNG
{
    public sealed class Body
    {
        private readonly World _world;
        private readonly List<Fixture> _fixtures = new List<Fixture>();

        public BodyDef Definition { get; }
        
        // Phase 2.5 Stage B: Body.Transform always returns the *effective*
        // pose = step-start (`_bodyPositions[Id]`) composed with the
        // within-step delta (`_bodyDeltaPositions[Id]`). With
        // `WorldDef.UseDeltaPositionTracking == false` (the default) the
        // delta arrays stay at (0, Identity) throughout the sub-step loop
        // — `IntegratePositions` calls `body.SetTransform` which writes
        // step-start AND zeros the delta — so the composition degenerates
        // to a direct read and the suite is byte-identical to today. With
        // the flag on, `IntegratePositions` skips `body.SetTransform` and
        // the delta arrays carry within-step movement; `ApplyBodyDeltas`
        // commits + clears them once per outer Step. Either way external
        // callers see "current world pose."
        public Transform Transform
        {
            get => new Transform(
                _world._bodyPositions[Id] + _world._bodyDeltaPositions[Id],
                Rot.Mul(_world._bodyDeltaRotations[Id], _world._bodyRotations[Id]));
            private set
            {
                // Setting Transform lands at step-start and zeros the delta
                // — callers (`SetTransform`, `SetTransformFromCenter`,
                // `SetTransformFromSweep`) all assume "this IS the new pose,
                // no pending corrections." Inside-the-solve writes that
                // need to land in the delta arrays must go through World-
                // level mutation paths (see the contact NGS / joint
                // position-constraint Stages C+).
                _world._bodyPositions[Id] = value.P;
                _world._bodyRotations[Id] = value.Q;
                _world._bodyDeltaPositions[Id] = Vec2.Zero;
                _world._bodyDeltaRotations[Id] = Rot.Identity;
            }
        }

        public Rot Rotation => Rot.Mul(_world._bodyDeltaRotations[Id], _world._bodyRotations[Id]);


        public BodyType Type
        {
            get => _world._bodyTypes[Id];
            private set => _world._bodyTypes[Id] = value;
        }

        public Vec2 Position
        {
            get => _world._bodyPositions[Id] + _world._bodyDeltaPositions[Id];
            set
            {
                _world._bodyPositions[Id] = value;
                _world._bodyDeltaPositions[Id] = Vec2.Zero;
            }
        }

        public Vec2 LinearVelocity
        {
            get => _world._bodyLinearVelocities[Id];
            set => _world._bodyLinearVelocities[Id] = value;
        }

        public float AngularVelocity
        {
            get => _world._bodyAngularVelocities[Id];
            set => _world._bodyAngularVelocities[Id] = value;
        }
        public float LinearDamping 
        { 
            get => _world._bodyLinearDampings[Id];
            set => _world._bodyLinearDampings[Id] = value;
        }
        public float AngularDamping 
        { 
            get => _world._bodyAngularDampings[Id];
            set => _world._bodyAngularDampings[Id] = value;
        }
        public float GravityScale 
        { 
            get => _world._bodyGravityScales[Id];
            set => _world._bodyGravityScales[Id] = value;
        }
        public bool FixedRotation 
        { 
            get => _world._bodyFixedRotations[Id];
            private set => _world._bodyFixedRotations[Id] = value;
        }
        public MotionLocks MotionLocks { get; } // Leaving this for now as it seems rare/complex?
        
        public float Mass 
        { 
            get => _world._bodyMasses[Id];
            private set => _world._bodyMasses[Id] = value;
        }
        public float Inertia 
        { 
            get => _world._bodyInertias[Id];
            private set => _world._bodyInertias[Id] = value;
        }
        public float InverseMass 
        { 
            get => _world._bodyInverseMasses[Id];
            private set => _world._bodyInverseMasses[Id] = value;
        }
        public float InverseInertia 
        { 
            get => _world._bodyInverseInertias[Id];
            private set => _world._bodyInverseInertias[Id] = value;
        }
        public int Id { get; internal set; } = -1;
        public Sweep Sweep { get; internal set; }
        public Vec2 LocalCenter 
        { 
            get => _world._bodyLocalCenters[Id];
            internal set => _world._bodyLocalCenters[Id] = value;
        }
        public Vec2 Force 
        { 
            get => _world._bodyForces[Id];
            private set => _world._bodyForces[Id] = value;
        }
        public float Torque 
        { 
            get => _world._bodyTorques[Id];
            private set => _world._bodyTorques[Id] = value;
        }
        public bool Awake 
        { 
            get => _world._bodyAwakes[Id];
            private set => _world._bodyAwakes[Id] = value;
        }
        public bool AllowSleep 
        { 
            get => _world._bodyAllowSleeps[Id];
            private set => _world._bodyAllowSleeps[Id] = value;
        }
        public float SleepTime 
        { 
            get => _world._bodySleepTimes[Id];
            internal set => _world._bodySleepTimes[Id] = value;
        }
        internal SolverSetType SolverSetType 
        { 
            get => _world._bodySolverSetTypes[Id];
            set => _world._bodySolverSetTypes[Id] = value;
        }
        internal ContactEdge? ContactList { get; set; }
        internal int ContactEdgeCount { get; set; }

        internal Body(World world, BodyDef def, int id)
        {
            _world = world;
            Id = id; // Must be set before accessing properties
            Definition = def ?? throw new ArgumentNullException(nameof(def));
            
            // Initialize arrays via properties
            Transform = new Transform(def.Position, def.Rotation);
            
            Type = def.Type;
            LinearVelocity = def.LinearVelocity;
            AngularVelocity = def.AngularVelocity;
            LinearDamping = def.LinearDamping;
            AngularDamping = def.AngularDamping;
            GravityScale = def.GravityScale;
            FixedRotation = def.FixedRotation;
            MotionLocks = def.MotionLocks;
            Awake = def.Awake;
            AllowSleep = def.AllowSleep;
            SolverSetType = Type == BodyType.Static ? SolverSetType.Static : (def.Awake ? SolverSetType.Awake : SolverSetType.Sleeping);

            ResetMassData();
            LocalCenter = Vec2.Zero;
            Vec2 worldCenter = Transform.P; // Uses array access
            Sweep = new Sweep(LocalCenter, worldCenter, worldCenter, def.Rotation.Angle, def.Rotation.Angle, 0f);
        }

        public IReadOnlyList<Fixture> Fixtures => _fixtures;

        public Body SetTransform(Vec2 position, float angleRadians)
        {
            Transform = new Transform(position, new Rot(angleRadians));
            Vec2 center = GetWorldCenter();
            Sweep = new Sweep(LocalCenter, center, center, angleRadians, angleRadians, 0f);
            return this;
        }

        public Body SetTransform(Vec2 position, Rot rotation)
        {
            Transform = new Transform(position, rotation);
            Vec2 center = GetWorldCenter();
            Sweep = new Sweep(LocalCenter, center, center, rotation.Angle, rotation.Angle, 0f);
            return this;
        }

        internal void SetTransformFromCenter(Vec2 center, float angleRadians)
        {
            Rot rotation = new Rot(angleRadians);
            Vec2 position = center - Rot.Mul(rotation, LocalCenter);
            Transform = new Transform(position, rotation);
            Sweep = new Sweep(LocalCenter, center, center, angleRadians, angleRadians, 0f);
        }

        internal void SetTransformFromSweep(Sweep sweep)
        {
            Transform = sweep.GetTransform(0f);
            Sweep = sweep;
        }

        public Vec2 GetWorldCenter() => Transform.P + Rot.Mul(Transform.Q, LocalCenter);

        public Vec2 GetLocalCenter() => LocalCenter;

        public Vec2 GetWorldPoint(Vec2 localPoint) => Transform.Mul(Transform, localPoint);

        public Vec2 GetLocalPoint(Vec2 worldPoint) => Transform.MulT(Transform, worldPoint);

        public Vec2 GetWorldVector(Vec2 localVector) => Rot.Mul(Transform.Q, localVector);

        public Vec2 GetLocalVector(Vec2 worldVector) => Rot.MulT(Transform.Q, worldVector);

        public void SetType(BodyType type)
        {
            if (Type == type)
            {
                return;
            }

            BodyType previousType = Type;
            Type = type;
            _world.NotifyBodyTypeChanged(this, previousType);

            if (Type == BodyType.Static)
            {
                LinearVelocity = Vec2.Zero;
                AngularVelocity = 0f;
                ClearForces();
            }

            RecalculateMass();
            SetAwake(true);
        }

        public void ApplyForce(Vec2 force)
        {
            if (Type != BodyType.Dynamic)
            {
                return;
            }

            SetAwake(true);
            Force += force;
        }

        public void ApplyForce(Vec2 force, Vec2 point)
        {
            if (Type != BodyType.Dynamic)
            {
                return;
            }

            SetAwake(true);
            Force += force;
            Torque += Vec2.Cross(point - GetWorldCenter(), force);
        }

        public void ApplyTorque(float torque)
        {
            if (Type != BodyType.Dynamic)
            {
                return;
            }

            SetAwake(true);
            Torque += torque;
        }

        public void ApplyLinearImpulse(Vec2 impulse)
        {
            if (Type != BodyType.Dynamic)
            {
                return;
            }

            SetAwake(true);
            LinearVelocity += InverseMass * impulse;
        }

        public void ApplyAngularImpulse(float impulse)
        {
            if (Type != BodyType.Dynamic)
            {
                return;
            }

            SetAwake(true);
            AngularVelocity += InverseInertia * impulse;
        }

        public void ApplyLinearImpulse(Vec2 impulse, Vec2 point)
        {
            if (Type != BodyType.Dynamic)
            {
                return;
            }

            SetAwake(true);
            LinearVelocity += InverseMass * impulse;
            AngularVelocity += InverseInertia * Vec2.Cross(point - GetWorldCenter(), impulse);
        }

        public void SetAwake(bool awake)
        {
            SetAwakeFromWorld(awake, notifyWorld: true);
        }

        internal void SetAwakeFromWorld(bool awake, bool notifyWorld)
        {
            if (Type == BodyType.Static)
            {
                Awake = false;
                SleepTime = 0f;
                return;
            }

            Awake = awake;
            if (awake)
            {
                SleepTime = 0f;
            }
            if (notifyWorld)
            {
                _world.NotifyAwakeChanged(this);
            }
        }

        internal void ClearForces()
        {
            Force = Vec2.Zero;
            Torque = 0f;
        }

        public Fixture CreateFixture(Shape shape)
        {
            Fixture fixture = _world.CreateFixture(this, shape);
            _fixtures.Add(fixture);
            RecalculateMass();
            return fixture;
        }

        public Fixture CreateFixture(FixtureDef def)
        {
            Fixture fixture = _world.CreateFixture(this, def);
            _fixtures.Add(fixture);
            RecalculateMass();
            return fixture;
        }

        public void DestroyFixture(Fixture fixture)
        {
            if (fixture == null || !_fixtures.Remove(fixture))
            {
                return;
            }

            _world.DestroyFixture(fixture);
            RecalculateMass();
        }

        internal void RecalculateMass()
        {
            if (Type != BodyType.Dynamic)
            {
                ResetMassData();
                return;
            }

            float mass = 0f;
            float inertia = 0f;
            Vec2 localCenter = Vec2.Zero;

            for (int i = 0; i < _fixtures.Count; ++i)
            {
                Fixture fixture = _fixtures[i];
                if (fixture.Density <= 0f)
                {
                    continue;
                }

                MassData md = MassProperties.Compute(fixture.Shape, fixture.Density);
                mass += md.Mass;
                localCenter += md.Mass * md.Center;
            }

            if (mass > 0f)
            {
                localCenter = (1f / mass) * localCenter;
            }

            for (int i = 0; i < _fixtures.Count; ++i)
            {
                Fixture fixture = _fixtures[i];
                if (fixture.Density <= 0f)
                {
                    continue;
                }

                MassData md = MassProperties.Compute(fixture.Shape, fixture.Density);
                Vec2 c = md.Center - localCenter;
                inertia += md.Inertia + md.Mass * Vec2.Dot(c, c);
            }

            Mass = mass;
            Inertia = FixedRotation ? 0f : inertia;
            InverseMass = mass > 0f ? 1f / mass : 0f;
            InverseInertia = (!FixedRotation && inertia > 0f) ? 1f / inertia : 0f;

            Vec2 oldCenter = GetWorldCenter();
            LocalCenter = localCenter;
            Vec2 newCenter = GetWorldCenter();
            if (!oldCenter.Equals(newCenter))
            {
                LinearVelocity += Vec2.Cross(AngularVelocity, newCenter - oldCenter);
            }
            Sweep = new Sweep(LocalCenter, newCenter, newCenter, Transform.Q.Angle, Transform.Q.Angle, 0f);
        }

        private void ResetMassData()
        {
            Mass = 0f;
            Inertia = 0f;
            InverseMass = 0f;
            InverseInertia = 0f;
            LocalCenter = Vec2.Zero;
        }
    }
}
