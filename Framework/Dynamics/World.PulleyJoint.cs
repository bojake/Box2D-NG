namespace Box2DNG
{
    public sealed partial class World
    {
        internal void InitPulleyJointVelocityConstraints(int index, float dt)
        {
            ref PulleyJointData joint = ref _pulleyJointsData[index];
            int indexA = joint.BodyA;
            int indexB = joint.BodyB;

            Vec2 rA = Rot.Mul(_bodyRotations[indexA], joint.LocalAnchorA - _bodyLocalCenters[indexA]);
            Vec2 rB = Rot.Mul(_bodyRotations[indexB], joint.LocalAnchorB - _bodyLocalCenters[indexB]);

            Vec2 pA = _bodyPositions[indexA] + rA;
            Vec2 pB = _bodyPositions[indexB] + rB;

            joint.UA = pA - joint.GroundAnchorA;
            joint.UB = pB - joint.GroundAnchorB;

            float lengthA = joint.UA.Length;
            float lengthB = joint.UB.Length;

            joint.UA = lengthA > Constants.LinearSlop ? joint.UA / lengthA : Vec2.Zero;
            joint.UB = lengthB > Constants.LinearSlop ? joint.UB / lengthB : Vec2.Zero;

            float mA = _bodyInverseMasses[indexA];
            float mB = _bodyInverseMasses[indexB];
            float iA = _bodyInverseInertias[indexA];
            float iB = _bodyInverseInertias[indexB];

            float crA = Vec2.Cross(rA, joint.UA);
            float crB = Vec2.Cross(rB, joint.UB);
            float invMass = mA + iA * crA * crA + joint.Ratio * joint.Ratio * (mB + iB * crB * crB);
            joint.Mass = invMass > 0f ? 1f / invMass : 0f;
        }

        internal void SolvePulleyJointVelocityConstraints(int index)
        {
            ref PulleyJointData joint = ref _pulleyJointsData[index];
            int indexA = joint.BodyA;
            int indexB = joint.BodyB;

            // Phase 2.5 Stage H — effective rotations.
            Vec2 rA = Rot.Mul(EffectiveRotation(indexA), joint.LocalAnchorA - _bodyLocalCenters[indexA]);
            Vec2 rB = Rot.Mul(EffectiveRotation(indexB), joint.LocalAnchorB - _bodyLocalCenters[indexB]);

            Vec2 vA = _bodyLinearVelocities[indexA] + Vec2.Cross(_bodyAngularVelocities[indexA], rA);
            Vec2 vB = _bodyLinearVelocities[indexB] + Vec2.Cross(_bodyAngularVelocities[indexB], rB);

            float Cdot = -Vec2.Dot(joint.UA, vA) - joint.Ratio * Vec2.Dot(joint.UB, vB);
            float impulse = -joint.Mass * Cdot;
            joint.Impulse += impulse;

            Vec2 PA = -impulse * joint.UA;
            Vec2 PB = -joint.Ratio * impulse * joint.UB;

            _bodyLinearVelocities[indexA] += _bodyInverseMasses[indexA] * PA;
            _bodyAngularVelocities[indexA] += _bodyInverseInertias[indexA] * Vec2.Cross(rA, PA);
            _bodyLinearVelocities[indexB] += _bodyInverseMasses[indexB] * PB;
            _bodyAngularVelocities[indexB] += _bodyInverseInertias[indexB] * Vec2.Cross(rB, PB);
        }

        internal void SolvePulleyJointPositionConstraints(int index)
        {
            ref PulleyJointData joint = ref _pulleyJointsData[index];
            int indexA = joint.BodyA;
            int indexB = joint.BodyB;
            bool useDelta = _def.UseDeltaPositionTracking;

            // Phase 2.5 Stage H — effective reads; writes branch on the flag.
            Vec2 cA = EffectivePosition(indexA);
            Vec2 cB = EffectivePosition(indexB);
            float aA = EffectiveRotation(indexA).Angle;
            float aB = EffectiveRotation(indexB).Angle;

            Vec2 rA = Rot.Mul(new Rot(aA), joint.LocalAnchorA - _bodyLocalCenters[indexA]);
            Vec2 rB = Rot.Mul(new Rot(aB), joint.LocalAnchorB - _bodyLocalCenters[indexB]);
            Vec2 pA = cA + rA;
            Vec2 pB = cB + rB;

            Vec2 uA = pA - joint.GroundAnchorA;
            Vec2 uB = pB - joint.GroundAnchorB;
            float lengthA = uA.Length;
            float lengthB = uB.Length;

            uA = lengthA > Constants.LinearSlop ? uA / lengthA : Vec2.Zero;
            uB = lengthB > Constants.LinearSlop ? uB / lengthB : Vec2.Zero;

            float target = joint.LengthA + joint.Ratio * joint.LengthB;
            float C = target - (lengthA + joint.Ratio * lengthB);
            float impulse = -joint.Mass * C;

            Vec2 PA = -impulse * uA;
            Vec2 PB = -joint.Ratio * impulse * uB;

            if (useDelta)
            {
                _bodyDeltaPositions[indexA] += _bodyInverseMasses[indexA] * PA;
                _bodyDeltaPositions[indexB] += _bodyInverseMasses[indexB] * PB;
            }
            else
            {
                _bodyPositions[indexA] += _bodyInverseMasses[indexA] * PA;
                _bodyPositions[indexB] += _bodyInverseMasses[indexB] * PB;
            }
            aA += _bodyInverseInertias[indexA] * Vec2.Cross(rA, PA);
            aB += _bodyInverseInertias[indexB] * Vec2.Cross(rB, PB);

            if (useDelta)
            {
                _bodyDeltaRotations[indexA] = Rot.MulT(_bodyStepStartRotations[indexA], new Rot(aA));
                _bodyDeltaRotations[indexB] = Rot.MulT(_bodyStepStartRotations[indexB], new Rot(aB));
            }
            else
            {
                _bodyRotations[indexA] = new Rot(aA);
                _bodyRotations[indexB] = new Rot(aB);
            }
        }
    }
}
