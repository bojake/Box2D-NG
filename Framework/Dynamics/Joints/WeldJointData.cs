namespace Box2DNG
{
    public struct WeldJointData
    {
        public int Id;
        public int BodyA;
        public int BodyB;
        public bool CollideConnected;

        public Vec2 LocalAnchorA;
        public Vec2 LocalAnchorB;
        public float ReferenceAngle;

        // Soft-constraint tuning. Hertz == 0 means "rigid" — the joint falls
        // through to the world's JointHertz / JointDampingRatio fallback, which
        // in turn defaults to Softness.Rigid (legacy hard-constraint behaviour).
        public float LinearHertz;
        public float LinearDampingRatio;
        public float AngularHertz;
        public float AngularDampingRatio;

        // Computed per-step in InitWeldJointVelocityConstraints. When non-zero
        // these inject biasRate*positionError into the velocity solve.
        public Softness LinearSpring;
        public Softness AngularSpring;

        // Rest-state anchor delta in world space, captured ONCE at joint
        // creation. The soft linear spring drives the current anchor delta
        // back toward this value — C = current_anchor_delta - DeltaCenter
        // grows with accumulated drift.
        public Vec2 DeltaCenter;

        public Mat22 LinearMass;
        public float AngularMass;
        public Vec2 Impulse;
        public float AngularImpulse;
        public Vec2 RA;
        public Vec2 RB;
    }
}
