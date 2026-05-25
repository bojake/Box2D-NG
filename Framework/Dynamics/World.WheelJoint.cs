namespace Box2DNG
{
    public sealed partial class World
    {
        internal void InitWheelJointVelocityConstraints(int index, float dt)
        {
            ref WheelJointData joint = ref _wheelJointsData[index];
            int indexA = joint.BodyA;
            int indexB = joint.BodyB;

            Vec2 rA = Rot.Mul(_bodyRotations[indexA], joint.LocalAnchorA - _bodyLocalCenters[indexA]);
            Vec2 rB = Rot.Mul(_bodyRotations[indexB], joint.LocalAnchorB - _bodyLocalCenters[indexB]);
            Vec2 d = (_bodyPositions[indexB] + rB) - (_bodyPositions[indexA] + rA);

            Vec2 axis = Rot.Mul(_bodyRotations[indexA], joint.LocalAxisA);
            Vec2 perp = new Vec2(-axis.Y, axis.X);

            float s1 = Vec2.Cross(d + rA, perp);
            float s2 = Vec2.Cross(rB, perp);
            float a1 = Vec2.Cross(d + rA, axis);
            float a2 = Vec2.Cross(rB, axis);

            float mA = _bodyInverseMasses[indexA];
            float mB = _bodyInverseMasses[indexB];
            float iA = _bodyInverseInertias[indexA];
            float iB = _bodyInverseInertias[indexB];

            float kPerp = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            joint.PerpMass = kPerp > 0f ? 1f / kPerp : 0f;

            float kAxis = mA + mB + iA * a1 * a1 + iB * a2 * a2;
            joint.SpringMass = kAxis > 0f ? 1f / kAxis : 0f;
            // Per-joint FrequencyHz wins; if zero, fall through to the world's
            // JointHertz default (matches the Phase 1 pattern from Weld and
            // Revolute). When neither is set, the axial spring stays inactive.
            joint.SpringSoftness = ResolveJointSpring(joint.FrequencyHz, joint.DampingRatio, dt);
            if (joint.SpringSoftness.IsZero)
            {
                joint.SpringImpulse = 0f;
            }

            if (joint.EnableMotor)
            {
                float motorMass = iA + iB;
                joint.MotorMass = motorMass > 0f ? 1f / motorMass : 0f;
            }
            else
            {
                joint.MotorMass = 0f;
                joint.MotorImpulse = 0f;
            }

            if (!joint.EnableLimit)
            {
                joint.LowerImpulse = 0f;
                joint.UpperImpulse = 0f;
            }
        }

        internal void SolveWheelJointVelocityConstraints(int index, float dt)
        {
            ref WheelJointData joint = ref _wheelJointsData[index];
            int indexA = joint.BodyA;
            int indexB = joint.BodyB;

            float mA = _bodyInverseMasses[indexA];
            float mB = _bodyInverseMasses[indexB];
            float iA = _bodyInverseInertias[indexA];
            float iB = _bodyInverseInertias[indexB];

            // Phase 2.5 Stage F — effective reads.
            Rot rotA = EffectiveRotation(indexA);
            Rot rotB = EffectiveRotation(indexB);
            Vec2 rA = Rot.Mul(rotA, joint.LocalAnchorA - _bodyLocalCenters[indexA]);
            Vec2 rB = Rot.Mul(rotB, joint.LocalAnchorB - _bodyLocalCenters[indexB]);

            ref Vec2 vA = ref _bodyLinearVelocities[indexA];
            ref Vec2 vB = ref _bodyLinearVelocities[indexB];
            ref float wA = ref _bodyAngularVelocities[indexA];
            ref float wB = ref _bodyAngularVelocities[indexB];

            Vec2 d = (EffectivePosition(indexB) + rB) - (EffectivePosition(indexA) + rA);
            Vec2 axis = Rot.Mul(rotA, joint.LocalAxisA);
            Vec2 perp = new Vec2(-axis.Y, axis.X);
            float s1 = Vec2.Cross(d + rA, perp);
            float s2 = Vec2.Cross(rB, perp);
            float a1 = Vec2.Cross(d + rA, axis);
            float a2 = Vec2.Cross(rB, axis);

            if (joint.EnableMotor)
            {
                float CdotMotor = wB - wA - joint.MotorSpeed;
                float impulseMotor = -joint.MotorMass * CdotMotor;
                float maxImpulse = joint.MaxMotorTorque * dt;
                float newImpulse = MathFng.Clamp(joint.MotorImpulse + impulseMotor, -maxImpulse, maxImpulse);
                float applied = newImpulse - joint.MotorImpulse;
                joint.MotorImpulse = newImpulse;

                wA -= iA * applied;
                wB += iB * applied;
            }

            // Gate on the resolved softness rather than the raw FrequencyHz so
            // the world's JointHertz default participates.
            if (!joint.SpringSoftness.IsZero)
            {
                float translation = Vec2.Dot(axis, d);
                float CdotAxis = Vec2.Dot(axis, vB + Vec2.Cross(wB, rB) - vA - Vec2.Cross(wA, rA));
                float impulseAxis = -joint.SpringSoftness.MassScale * joint.SpringMass * (CdotAxis + joint.SpringSoftness.BiasRate * translation)
                                    - joint.SpringSoftness.ImpulseScale * joint.SpringImpulse;
                joint.SpringImpulse += impulseAxis;

                Vec2 Paxis = impulseAxis * axis;
                float LAaxis = impulseAxis * a1;
                float LBaxis = impulseAxis * a2;
                vA -= mA * Paxis;
                wA -= iA * LAaxis;
                vB += mB * Paxis;
                wB += iB * LBaxis;
            }

            if (joint.EnableLimit)
            {
                float translation = Vec2.Dot(axis, d);
                if (translation < joint.LowerTranslation)
                {
                    float C = translation - joint.LowerTranslation;
                    float Cdot = Vec2.Dot(axis, vB + Vec2.Cross(wB, rB) - vA - Vec2.Cross(wA, rA));
                    float impulse = -joint.SpringMass * (Cdot + MathF.Min(0f, C) * 0.2f);
                    float prevImpulse = joint.LowerImpulse;
                    joint.LowerImpulse = MathF.Min(prevImpulse + impulse, 0f);
                    impulse = joint.LowerImpulse - prevImpulse;

                    Vec2 limitP = impulse * axis;
                    float limitLA = impulse * a1;
                    float limitLB = impulse * a2;
                    vA -= mA * limitP;
                    wA -= iA * limitLA;
                    vB += mB * limitP;
                    wB += iB * limitLB;
                }
                else
                {
                    joint.LowerImpulse = 0f;
                }

                if (translation > joint.UpperTranslation)
                {
                    float C = joint.UpperTranslation - translation;
                    float Cdot = Vec2.Dot(axis, vA + Vec2.Cross(wA, rA) - vB - Vec2.Cross(wB, rB));
                    float impulse = -joint.SpringMass * (Cdot + MathF.Min(0f, C) * 0.2f);
                    float prevImpulse = joint.UpperImpulse;
                    joint.UpperImpulse = MathF.Max(prevImpulse + impulse, 0f);
                    impulse = joint.UpperImpulse - prevImpulse;

                    Vec2 limitP = impulse * axis;
                    float limitLA = impulse * a1;
                    float limitLB = impulse * a2;
                    vA += mA * limitP;
                    wA += iA * limitLA;
                    vB -= mB * limitP;
                    wB -= iB * limitLB;
                }
                else
                {
                    joint.UpperImpulse = 0f;
                }
            }

            float CdotPerp = Vec2.Dot(perp, vB + Vec2.Cross(wB, rB) - vA - Vec2.Cross(wA, rA));
            float impulsePerp = -joint.PerpMass * CdotPerp;
            joint.Impulse += impulsePerp;

            Vec2 P = impulsePerp * perp;
            float LA = impulsePerp * s1;
            float LB = impulsePerp * s2;
            vA -= mA * P;
            wA -= iA * LA;
            vB += mB * P;
            wB += iB * LB;
        }

        internal void SolveWheelJointPositionConstraints(int index)
        {
            ref WheelJointData joint = ref _wheelJointsData[index];
            int indexA = joint.BodyA;
            int indexB = joint.BodyB;

            bool useDelta = _def.UseDeltaPositionTracking;

            // Phase 2.5 Stage F — effective reads, local mutation; writes at
            // end branch on the flag. We track cA / cB as local Vec2 values
            // (not refs) so the perp-then-limit two-phase accumulation flows
            // through one variable, and only commit the final delta or
            // step-start update once.
            Vec2 cA = EffectivePosition(indexA);
            Vec2 cB = EffectivePosition(indexB);
            float aA = EffectiveRotation(indexA).Angle;
            float aB = EffectiveRotation(indexB).Angle;
            // Track the position correction we've accumulated so the final
            // write applies the net change.
            Vec2 cAOriginal = cA;
            Vec2 cBOriginal = cB;

            float mA = _bodyInverseMasses[indexA];
            float mB = _bodyInverseMasses[indexB];
            float iA = _bodyInverseInertias[indexA];
            float iB = _bodyInverseInertias[indexB];

            Rot qA = new Rot(aA);
            Rot qB = new Rot(aB);
            Vec2 rA = Rot.Mul(qA, joint.LocalAnchorA - _bodyLocalCenters[indexA]);
            Vec2 rB = Rot.Mul(qB, joint.LocalAnchorB - _bodyLocalCenters[indexB]);
            Vec2 d = (cB + rB) - (cA + rA);

            Vec2 axis = Rot.Mul(qA, joint.LocalAxisA);
            Vec2 perp = new Vec2(-axis.Y, axis.X);

            float s1 = Vec2.Cross(d + rA, perp);
            float s2 = Vec2.Cross(rB, perp);
            float C = Vec2.Dot(perp, d);

            float k = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            float impulse = k > 0f ? -C / k : 0f;
            Vec2 P = impulse * perp;
            float LA = impulse * s1;
            float LB = impulse * s2;

            cA -= mA * P;
            cB += mB * P;
            aA -= iA * LA;
            aB += iB * LB;

            if (joint.EnableLimit)
            {
                Rot qA2 = new Rot(aA);
                Rot qB2 = new Rot(aB);
                Vec2 rA2 = Rot.Mul(qA2, joint.LocalAnchorA - _bodyLocalCenters[indexA]);
                Vec2 rB2 = Rot.Mul(qB2, joint.LocalAnchorB - _bodyLocalCenters[indexB]);
                Vec2 d2 = (cB + rB2) - (cA + rA2);
                Vec2 axis2 = Rot.Mul(qA2, joint.LocalAxisA);
                float a1 = Vec2.Cross(d2 + rA2, axis2);
                float a2 = Vec2.Cross(rB2, axis2);
                float kAxis = mA + mB + iA * a1 * a1 + iB * a2 * a2;

                if (kAxis > 0f)
                {
                    float translation = Vec2.Dot(axis2, d2);
                    float limitC;
                    if (translation < joint.LowerTranslation)
                    {
                        limitC = translation - joint.LowerTranslation;
                    }
                    else if (translation > joint.UpperTranslation)
                    {
                        limitC = translation - joint.UpperTranslation;
                    }
                    else
                    {
                        // Within bounds — commit only the perp correction.
                        CommitWheelDelta(useDelta, indexA, indexB, cA, cB, cAOriginal, cBOriginal, aA, aB);
                        return;
                    }

                    float limitImpulse = -limitC / kAxis;
                    Vec2 limitP = limitImpulse * axis2;
                    float limitLA = limitImpulse * a1;
                    float limitLB = limitImpulse * a2;

                    cA -= mA * limitP;
                    cB += mB * limitP;
                    aA -= iA * limitLA;
                    aB += iB * limitLB;
                }
            }

            CommitWheelDelta(useDelta, indexA, indexB, cA, cB, cAOriginal, cBOriginal, aA, aB);
        }

        private void CommitWheelDelta(bool useDelta, int indexA, int indexB,
            Vec2 cA, Vec2 cB, Vec2 cAOriginal, Vec2 cBOriginal, float aA, float aB)
        {
            // Apply the net position delta. With flag off, _bodyPositions
            // moves by (cA - cAOriginal) — algebraically identical to the
            // pre-Stage-F `cA -= mA * P` write through the `ref Vec2 cA`.
            // With flag on, _bodyPositions stays at step-start and the
            // accumulated correction lands in _bodyDeltaPositions.
            Vec2 deltaA = cA - cAOriginal;
            Vec2 deltaB = cB - cBOriginal;
            if (useDelta)
            {
                _bodyDeltaPositions[indexA] += deltaA;
                _bodyDeltaPositions[indexB] += deltaB;
                _bodyDeltaRotations[indexA] = Rot.MulT(_bodyStepStartRotations[indexA], new Rot(aA));
                _bodyDeltaRotations[indexB] = Rot.MulT(_bodyStepStartRotations[indexB], new Rot(aB));
            }
            else
            {
                _bodyPositions[indexA] += deltaA;
                _bodyPositions[indexB] += deltaB;
                _bodyRotations[indexA] = new Rot(aA);
                _bodyRotations[indexB] = new Rot(aB);
            }
        }
    }
}
