using System;

namespace Box2DNG
{
    public struct RevoluteJointData
    {
        public int Id;
        public int BodyA;
        public int BodyB;
        public bool CollideConnected;
        
        public Vec2 LocalAnchorA;
        public Vec2 LocalAnchorB;
        public float ReferenceAngle;

        // Soft point-to-point tuning (Phase 1 of TIER4_PARITY_PLAN). Hertz==0
        // inherits the world default, which defaults to rigid behaviour.
        public float LinearHertz;
        public float LinearDampingRatio;
        public Softness LinearSpring;
        public Vec2 DeltaCenter;
        
        public bool EnableMotor;
        public float MotorSpeed;
        public float MaxMotorTorque;
        
        public bool EnableLimit;
        public float LowerAngle;
        public float UpperAngle;
        
        // State
        public Vec2 Impulse;
        public float MotorImpulse;
        public float LimitImpulse;
        
        // Solver Temp
        public Mat22 Mass;
        public float MotorMass;
    }
}
