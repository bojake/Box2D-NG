using System.Runtime.InteropServices;

namespace Box2DNG
{
    [StructLayout(LayoutKind.Sequential)]
    internal struct DistanceJointData
    {
        public int BodyA;
        public int BodyB;
        public Vec2 LocalAnchorA;
        public Vec2 LocalAnchorB;
        public float Length;
        public float FrequencyHz;
        public float DampingRatio;
        public bool CollideConnected;

        // Solver temp variables
        public float Impulse;
        public float Mass;       // Soft-effective mass: 1/(invMass + gamma) when spring active, else 1/invMass
        public float RigidMass;  // 1/invMass — used by Relax phase (useBias=false) so the relax solve isn't softened
        public float Gamma;
        public float Bias;
        public Vec2 U;
        // Resolved per-step spring tuning (Phase 1). Mirrors the Weld/Revolute
        // pattern so the world's JointHertz default can take effect when this
        // joint's FrequencyHz is 0.
        public Softness Softness;
    }
}
