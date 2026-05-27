namespace Box2DNG
{
    public sealed partial class World
    {
        internal void InitFrictionJointVelocityConstraints(int index, float dt)
        {
            ref FrictionJointData joint = ref _frictionJointsData[index];
            int indexA = joint.BodyA;
            int indexB = joint.BodyB;

            joint.RA = Rot.Mul(_bodyRotations[indexA], joint.LocalAnchorA - _bodyLocalCenters[indexA]);
            joint.RB = Rot.Mul(_bodyRotations[indexB], joint.LocalAnchorB - _bodyLocalCenters[indexB]);

            float mA = _bodyInverseMasses[indexA];
            float mB = _bodyInverseMasses[indexB];
            float iA = _bodyInverseInertias[indexA];
            float iB = _bodyInverseInertias[indexB];

            float k11 = mA + mB + iA * joint.RA.Y * joint.RA.Y + iB * joint.RB.Y * joint.RB.Y;
            float k12 = -iA * joint.RA.X * joint.RA.Y - iB * joint.RB.X * joint.RB.Y;
            float k22 = mA + mB + iA * joint.RA.X * joint.RA.X + iB * joint.RB.X * joint.RB.X;
            joint.LinearMass = new Mat22(new Vec2(k11, k12), new Vec2(k12, k22));

            float angularMass = iA + iB;
            joint.AngularMass = angularMass > 0f ? 1f / angularMass : 0f;

            joint.LinearImpulse = Vec2.Zero;
            joint.AngularImpulse = 0f;
        }

        // FrictionJoint is pure friction — no position-correction bias to gate.
        // useBias parameter is accepted for dispatch uniformity (it lets the
        // Relax pass call this method without a branch in SolverPipeline) but
        // the solve is identical in both phases, matching cpp's friction-style
        // accumulation that runs unconditionally each iteration.
        internal void SolveFrictionJointVelocityConstraints(int index, float dt, bool useBias)
        {
            _ = useBias;

            ref FrictionJointData joint = ref _frictionJointsData[index];
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

            float CdotAngle = wB - wA;
            float angularImpulse = -joint.AngularMass * CdotAngle;
            float oldAngular = joint.AngularImpulse;
            float maxAngular = joint.MaxTorque * dt;
            joint.AngularImpulse = MathFng.Clamp(joint.AngularImpulse + angularImpulse, -maxAngular, maxAngular);
            angularImpulse = joint.AngularImpulse - oldAngular;
            wA -= iA * angularImpulse;
            wB += iB * angularImpulse;

            Vec2 Cdot = vB + Vec2.Cross(wB, joint.RB) - vA - Vec2.Cross(wA, joint.RA);
            Vec2 impulse = Solve22(joint.LinearMass, -Cdot);
            Vec2 oldImpulse = joint.LinearImpulse;
            joint.LinearImpulse += impulse;
            float maxImpulse = joint.MaxForce * dt;
            if (joint.LinearImpulse.Length > maxImpulse && maxImpulse > 0f)
            {
                joint.LinearImpulse = joint.LinearImpulse.Normalize() * maxImpulse;
            }
            impulse = joint.LinearImpulse - oldImpulse;

            vA -= mA * impulse;
            wA -= iA * Vec2.Cross(joint.RA, impulse);
            vB += mB * impulse;
            wB += iB * Vec2.Cross(joint.RB, impulse);
        }

        internal void SolveFrictionJointPositionConstraints(int index)
        {
            // Friction joint is solved in velocity phase only.
        }
    }
}
