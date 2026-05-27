namespace Box2DNG
{
    public sealed partial class World
    {
        internal void InitMotorJointVelocityConstraints(int index, float dt)
        {
            ref MotorJointData joint = ref _motorJointsData[index];
            int indexA = joint.BodyA;
            int indexB = joint.BodyB;

            float mA = _bodyInverseMasses[indexA];
            float mB = _bodyInverseMasses[indexB];
            float iA = _bodyInverseInertias[indexA];
            float iB = _bodyInverseInertias[indexB];

            joint.RA = Vec2.Zero;
            joint.RB = Vec2.Zero;

            float k11 = mA + mB;
            float k22 = mA + mB;
            joint.LinearMass = new Mat22(new Vec2(k11, 0f), new Vec2(0f, k22));

            float angularMass = iA + iB;
            joint.AngularMass = angularMass > 0f ? 1f / angularMass : 0f;

            Vec2 target = _bodyPositions[indexA] + Rot.Mul(_bodyRotations[indexA], joint.LinearOffset);
            Vec2 C = _bodyPositions[indexB] - target;
            joint.LinearBias = dt > 0f ? (joint.CorrectionFactor / dt) * C : Vec2.Zero;

            float angleError = (_bodyRotations[indexB].Angle - _bodyRotations[indexA].Angle) - joint.AngularOffset;
            joint.AngularBias = dt > 0f ? (joint.CorrectionFactor / dt) * angleError : 0f;
        }

        internal void SolveMotorJointVelocityConstraints(int index, float dt, bool useBias)
        {
            ref MotorJointData joint = ref _motorJointsData[index];
            int indexA = joint.BodyA;
            int indexB = joint.BodyB;

            float mA = _bodyInverseMasses[indexA];
            float mB = _bodyInverseMasses[indexB];
            float iA = _bodyInverseInertias[indexA];
            float iB = _bodyInverseInertias[indexB];

            ref Vec2 vA = ref _bodyLinearVelocities[indexA];
            ref Vec2 vB = ref _bodyLinearVelocities[indexB];
            ref float wA = ref _bodyAngularVelocities[indexA];
            ref float wB = ref _bodyAngularVelocities[indexB];

            // LinearBias / AngularBias are position-correction terms (`C/dt`-
            // scaled by CorrectionFactor at Init time) and so are gated on
            // useBias — Relax solves Cdot-only without re-pulling toward the
            // motor's target pose.
            Vec2 linearBias = useBias ? joint.LinearBias : Vec2.Zero;
            float angularBias = useBias ? joint.AngularBias : 0f;

            Vec2 Cdot = vB - vA;
            Vec2 impulse = Solve22(joint.LinearMass, -Cdot - linearBias);

            Vec2 oldImpulse = joint.LinearImpulse;
            joint.LinearImpulse += impulse;
            float maxImpulse = joint.MaxForce * dt;
            if (joint.LinearImpulse.Length > maxImpulse && maxImpulse > 0f)
            {
                joint.LinearImpulse = joint.LinearImpulse.Normalize() * maxImpulse;
            }
            impulse = joint.LinearImpulse - oldImpulse;

            vA -= mA * impulse;
            vB += mB * impulse;

            float CdotAngle = wB - wA;
            float angularImpulse = -joint.AngularMass * (CdotAngle + angularBias);
            float oldAngular = joint.AngularImpulse;
            joint.AngularImpulse += angularImpulse;
            float maxAngularImpulse = joint.MaxTorque * dt;
            joint.AngularImpulse = MathFng.Clamp(joint.AngularImpulse, -maxAngularImpulse, maxAngularImpulse);
            angularImpulse = joint.AngularImpulse - oldAngular;

            wA -= iA * angularImpulse;
            wB += iB * angularImpulse;
        }

        internal void SolveMotorJointPositionConstraints(int index)
        {
            // Motor joint is solved in velocity phase only.
        }
    }
}
