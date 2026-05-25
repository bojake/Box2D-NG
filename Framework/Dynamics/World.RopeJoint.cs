namespace Box2DNG
{
    public sealed partial class World
    {
        internal void InitRopeJointVelocityConstraints(int index, float dt)
        {
            ref RopeJointData joint = ref _ropeJointsData[index];
            int indexA = joint.BodyA;
            int indexB = joint.BodyB;

            joint.RA = Rot.Mul(_bodyRotations[indexA], joint.LocalAnchorA - _bodyLocalCenters[indexA]);
            joint.RB = Rot.Mul(_bodyRotations[indexB], joint.LocalAnchorB - _bodyLocalCenters[indexB]);

            Vec2 pA = _bodyPositions[indexA] + joint.RA;
            Vec2 pB = _bodyPositions[indexB] + joint.RB;
            joint.U = pB - pA;
            joint.Length = joint.U.Length;

            if (joint.Length > Constants.LinearSlop)
            {
                joint.U = joint.U / joint.Length;
            }
            else
            {
                joint.U = Vec2.Zero;
                joint.Mass = 0f;
                joint.Impulse = 0f;
                return;
            }

            float mA = _bodyInverseMasses[indexA];
            float mB = _bodyInverseMasses[indexB];
            float iA = _bodyInverseInertias[indexA];
            float iB = _bodyInverseInertias[indexB];

            float crA = Vec2.Cross(joint.RA, joint.U);
            float crB = Vec2.Cross(joint.RB, joint.U);
            float invMass = mA + iA * crA * crA + mB + iB * crB * crB;
            joint.Mass = invMass > 0f ? 1f / invMass : 0f;
        }

        internal void SolveRopeJointVelocityConstraints(int index, float dt)
        {
            ref RopeJointData joint = ref _ropeJointsData[index];
            int indexA = joint.BodyA;
            int indexB = joint.BodyB;

            ref Vec2 vA = ref _bodyLinearVelocities[indexA];
            ref Vec2 vB = ref _bodyLinearVelocities[indexB];
            ref float wA = ref _bodyAngularVelocities[indexA];
            ref float wB = ref _bodyAngularVelocities[indexB];

            Vec2 vpA = vA + Vec2.Cross(wA, joint.RA);
            Vec2 vpB = vB + Vec2.Cross(wB, joint.RB);

            float C = joint.Length - joint.MaxLength;
            float Cdot = Vec2.Dot(joint.U, vpB - vpA);

            if (C < 0f && dt > 0f)
            {
                Cdot += C / dt;
            }

            float impulse = -joint.Mass * Cdot;
            float oldImpulse = joint.Impulse;
            joint.Impulse = MathF.Min(0f, joint.Impulse + impulse);
            impulse = joint.Impulse - oldImpulse;

            Vec2 P = impulse * joint.U;
            vA -= _bodyInverseMasses[indexA] * P;
            wA -= _bodyInverseInertias[indexA] * Vec2.Cross(joint.RA, P);
            vB += _bodyInverseMasses[indexB] * P;
            wB += _bodyInverseInertias[indexB] * Vec2.Cross(joint.RB, P);
        }

        internal void SolveRopeJointPositionConstraints(int index)
        {
            ref RopeJointData joint = ref _ropeJointsData[index];
            int indexA = joint.BodyA;
            int indexB = joint.BodyB;
            bool useDelta = _def.UseDeltaPositionTracking;

            // Phase 2.5 Stage I — effective reads; writes branch on the flag.
            Vec2 cA = EffectivePosition(indexA);
            Vec2 cB = EffectivePosition(indexB);
            float aA = EffectiveRotation(indexA).Angle;
            float aB = EffectiveRotation(indexB).Angle;

            Vec2 rA = Rot.Mul(new Rot(aA), joint.LocalAnchorA - _bodyLocalCenters[indexA]);
            Vec2 rB = Rot.Mul(new Rot(aB), joint.LocalAnchorB - _bodyLocalCenters[indexB]);
            Vec2 pA = cA + rA;
            Vec2 pB = cB + rB;
            Vec2 u = pB - pA;
            float length = u.Length;

            if (length <= Constants.LinearSlop)
            {
                return;
            }

            u /= length;
            float C = MathFng.Clamp(length - joint.MaxLength, 0f, Constants.MaxLinearCorrection);
            float impulse = -joint.Mass * C;
            Vec2 P = impulse * u;

            if (useDelta)
            {
                _bodyDeltaPositions[indexA] -= _bodyInverseMasses[indexA] * P;
                _bodyDeltaPositions[indexB] += _bodyInverseMasses[indexB] * P;
            }
            else
            {
                _bodyPositions[indexA] -= _bodyInverseMasses[indexA] * P;
                _bodyPositions[indexB] += _bodyInverseMasses[indexB] * P;
            }
            aA -= _bodyInverseInertias[indexA] * Vec2.Cross(rA, P);
            aB += _bodyInverseInertias[indexB] * Vec2.Cross(rB, P);

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
