namespace Box2DNG
{
    public sealed partial class World
    {
        private bool TryGetGearCoordinateA(ref GearJointData joint, out float coordinate)
        {
            coordinate = 0f;
            if (joint.TypeA == GearJointKind.Revolute)
            {
                if (!_revoluteJointIndexById.TryGetValue(joint.JointAId, out int jIndex))
                {
                    return false;
                }
                // Phase 2.5 Stage J — TryGetGearCoordinate* is called from
                // SolveGearJointPositionConstraints (inside the sub-step
                // loop) — use effective rotations / positions so flag-on
                // sees within-step delta.
                ref RevoluteJointData rev = ref _revoluteJointsData[jIndex];
                coordinate = (EffectiveRotation(joint.BodyB).Angle - EffectiveRotation(joint.BodyA).Angle) - rev.ReferenceAngle;
                return true;
            }

            if (!_prismaticJointIndexById.TryGetValue(joint.JointAId, out int pIndex))
            {
                return false;
            }
            ref PrismaticJointData pri = ref _prismaticJointsData[pIndex];
            Rot rotA = EffectiveRotation(joint.BodyA);
            Rot rotB = EffectiveRotation(joint.BodyB);
            Vec2 axis = Rot.Mul(rotA, pri.LocalAxisA);
            Vec2 rA = Rot.Mul(rotA, pri.LocalAnchorA - _bodyLocalCenters[joint.BodyA]);
            Vec2 rB = Rot.Mul(rotB, pri.LocalAnchorB - _bodyLocalCenters[joint.BodyB]);
            Vec2 d = (EffectivePosition(joint.BodyB) + rB) - (EffectivePosition(joint.BodyA) + rA);
            coordinate = Vec2.Dot(axis, d);
            return true;
        }

        private bool TryGetGearCoordinateB(ref GearJointData joint, out float coordinate)
        {
            coordinate = 0f;
            if (joint.TypeB == GearJointKind.Revolute)
            {
                if (!_revoluteJointIndexById.TryGetValue(joint.JointBId, out int jIndex))
                {
                    return false;
                }
                // Phase 2.5 Stage J — effective rotations.
                ref RevoluteJointData rev = ref _revoluteJointsData[jIndex];
                coordinate = (EffectiveRotation(joint.BodyD).Angle - EffectiveRotation(joint.BodyC).Angle) - rev.ReferenceAngle;
                return true;
            }

            if (!_prismaticJointIndexById.TryGetValue(joint.JointBId, out int pIndex))
            {
                return false;
            }
            ref PrismaticJointData pri = ref _prismaticJointsData[pIndex];
            Rot rotC = EffectiveRotation(joint.BodyC);
            Rot rotD = EffectiveRotation(joint.BodyD);
            Vec2 axis = Rot.Mul(rotC, pri.LocalAxisA);
            Vec2 rC = Rot.Mul(rotC, pri.LocalAnchorA - _bodyLocalCenters[joint.BodyC]);
            Vec2 rD = Rot.Mul(rotD, pri.LocalAnchorB - _bodyLocalCenters[joint.BodyD]);
            Vec2 d = (EffectivePosition(joint.BodyD) + rD) - (EffectivePosition(joint.BodyC) + rC);
            coordinate = Vec2.Dot(axis, d);
            return true;
        }

        internal void InitGearJointVelocityConstraints(int index, float dt)
        {
            ref GearJointData joint = ref _gearJointsData[index];
            joint.JV_A = Vec2.Zero;
            joint.JV_B = Vec2.Zero;
            joint.JV_C = Vec2.Zero;
            joint.JV_D = Vec2.Zero;
            joint.JW_A = 0f;
            joint.JW_B = 0f;
            joint.JW_C = 0f;
            joint.JW_D = 0f;

            float mass = 0f;

            if (joint.TypeA == GearJointKind.Revolute)
            {
                joint.JW_A = -1f;
                joint.JW_B = 1f;
                mass += _bodyInverseInertias[joint.BodyA] + _bodyInverseInertias[joint.BodyB];
            }
            else
            {
                if (!_prismaticJointIndexById.TryGetValue(joint.JointAId, out int pIndex))
                {
                    joint.Mass = 0f;
                    return;
                }
                ref PrismaticJointData pri = ref _prismaticJointsData[pIndex];
                Vec2 axis = Rot.Mul(_bodyRotations[joint.BodyA], pri.LocalAxisA);
                Vec2 rA = Rot.Mul(_bodyRotations[joint.BodyA], pri.LocalAnchorA - _bodyLocalCenters[joint.BodyA]);
                Vec2 rB = Rot.Mul(_bodyRotations[joint.BodyB], pri.LocalAnchorB - _bodyLocalCenters[joint.BodyB]);
                Vec2 d = (_bodyPositions[joint.BodyB] + rB) - (_bodyPositions[joint.BodyA] + rA);
                float a1 = Vec2.Cross(d + rA, axis);
                float a2 = Vec2.Cross(rB, axis);

                joint.JV_A = -axis;
                joint.JW_A = -a1;
                joint.JV_B = axis;
                joint.JW_B = a2;
                mass += _bodyInverseMasses[joint.BodyA] + _bodyInverseMasses[joint.BodyB] +
                        _bodyInverseInertias[joint.BodyA] * a1 * a1 + _bodyInverseInertias[joint.BodyB] * a2 * a2;
            }

            if (joint.TypeB == GearJointKind.Revolute)
            {
                joint.JW_C = -joint.Ratio;
                joint.JW_D = joint.Ratio;
                mass += joint.Ratio * joint.Ratio * (_bodyInverseInertias[joint.BodyC] + _bodyInverseInertias[joint.BodyD]);
            }
            else
            {
                if (!_prismaticJointIndexById.TryGetValue(joint.JointBId, out int pIndex))
                {
                    joint.Mass = 0f;
                    return;
                }
                ref PrismaticJointData pri = ref _prismaticJointsData[pIndex];
                Vec2 axis = Rot.Mul(_bodyRotations[joint.BodyC], pri.LocalAxisA);
                Vec2 rC = Rot.Mul(_bodyRotations[joint.BodyC], pri.LocalAnchorA - _bodyLocalCenters[joint.BodyC]);
                Vec2 rD = Rot.Mul(_bodyRotations[joint.BodyD], pri.LocalAnchorB - _bodyLocalCenters[joint.BodyD]);
                Vec2 d = (_bodyPositions[joint.BodyD] + rD) - (_bodyPositions[joint.BodyC] + rC);
                float a1 = Vec2.Cross(d + rC, axis);
                float a2 = Vec2.Cross(rD, axis);

                joint.JV_C = -joint.Ratio * axis;
                joint.JW_C = -joint.Ratio * a1;
                joint.JV_D = joint.Ratio * axis;
                joint.JW_D = joint.Ratio * a2;
                mass += joint.Ratio * joint.Ratio * (_bodyInverseMasses[joint.BodyC] + _bodyInverseMasses[joint.BodyD] +
                        _bodyInverseInertias[joint.BodyC] * a1 * a1 + _bodyInverseInertias[joint.BodyD] * a2 * a2);
            }

            joint.Mass = mass > 0f ? 1f / mass : 0f;
        }

        // GearJoint is a pure Cdot-only velocity solve (no position-correction
        // bias) — position drift is handled by SolveGearJointPositionConstraints.
        // useBias is accepted for dispatch uniformity; behaviour is identical
        // in Solve and Relax.
        internal void SolveGearJointVelocityConstraints(int index, float dt, bool useBias)
        {
            _ = useBias;
            ref GearJointData joint = ref _gearJointsData[index];
            if (joint.Mass == 0f)
            {
                return;
            }

            float Cdot = Vec2.Dot(joint.JV_A, _bodyLinearVelocities[joint.BodyA]) + joint.JW_A * _bodyAngularVelocities[joint.BodyA] +
                         Vec2.Dot(joint.JV_B, _bodyLinearVelocities[joint.BodyB]) + joint.JW_B * _bodyAngularVelocities[joint.BodyB] +
                         Vec2.Dot(joint.JV_C, _bodyLinearVelocities[joint.BodyC]) + joint.JW_C * _bodyAngularVelocities[joint.BodyC] +
                         Vec2.Dot(joint.JV_D, _bodyLinearVelocities[joint.BodyD]) + joint.JW_D * _bodyAngularVelocities[joint.BodyD];

            float impulse = -joint.Mass * Cdot;

            _bodyLinearVelocities[joint.BodyA] += _bodyInverseMasses[joint.BodyA] * impulse * joint.JV_A;
            _bodyAngularVelocities[joint.BodyA] += _bodyInverseInertias[joint.BodyA] * impulse * joint.JW_A;
            _bodyLinearVelocities[joint.BodyB] += _bodyInverseMasses[joint.BodyB] * impulse * joint.JV_B;
            _bodyAngularVelocities[joint.BodyB] += _bodyInverseInertias[joint.BodyB] * impulse * joint.JW_B;
            _bodyLinearVelocities[joint.BodyC] += _bodyInverseMasses[joint.BodyC] * impulse * joint.JV_C;
            _bodyAngularVelocities[joint.BodyC] += _bodyInverseInertias[joint.BodyC] * impulse * joint.JW_C;
            _bodyLinearVelocities[joint.BodyD] += _bodyInverseMasses[joint.BodyD] * impulse * joint.JV_D;
            _bodyAngularVelocities[joint.BodyD] += _bodyInverseInertias[joint.BodyD] * impulse * joint.JW_D;
        }

        internal void SolveGearJointPositionConstraints(int index)
        {
            ref GearJointData joint = ref _gearJointsData[index];
            if (!TryGetGearCoordinateA(ref joint, out float coordA) ||
                !TryGetGearCoordinateB(ref joint, out float coordB))
            {
                return;
            }

            Vec2 jvA = Vec2.Zero;
            Vec2 jvB = Vec2.Zero;
            Vec2 jvC = Vec2.Zero;
            Vec2 jvD = Vec2.Zero;
            float jwA = 0f;
            float jwB = 0f;
            float jwC = 0f;
            float jwD = 0f;
            float mass = 0f;

            if (joint.TypeA == GearJointKind.Revolute)
            {
                jwA = -1f;
                jwB = 1f;
                mass += _bodyInverseInertias[joint.BodyA] + _bodyInverseInertias[joint.BodyB];
            }
            else
            {
                if (!_prismaticJointIndexById.TryGetValue(joint.JointAId, out int pIndex))
                {
                    return;
                }

                // Phase 2.5 Stage J — effective reads (called from position-constraint pass).
                ref PrismaticJointData pri = ref _prismaticJointsData[pIndex];
                Rot rotA = EffectiveRotation(joint.BodyA);
                Rot rotB = EffectiveRotation(joint.BodyB);
                Vec2 axis = Rot.Mul(rotA, pri.LocalAxisA);
                Vec2 rA = Rot.Mul(rotA, pri.LocalAnchorA - _bodyLocalCenters[joint.BodyA]);
                Vec2 rB = Rot.Mul(rotB, pri.LocalAnchorB - _bodyLocalCenters[joint.BodyB]);
                Vec2 d = (EffectivePosition(joint.BodyB) + rB) - (EffectivePosition(joint.BodyA) + rA);
                float a1 = Vec2.Cross(d + rA, axis);
                float a2 = Vec2.Cross(rB, axis);

                jvA = -axis;
                jwA = -a1;
                jvB = axis;
                jwB = a2;

                mass += _bodyInverseMasses[joint.BodyA] + _bodyInverseMasses[joint.BodyB] +
                        _bodyInverseInertias[joint.BodyA] * a1 * a1 + _bodyInverseInertias[joint.BodyB] * a2 * a2;
            }

            if (joint.TypeB == GearJointKind.Revolute)
            {
                jwC = -joint.Ratio;
                jwD = joint.Ratio;
                mass += joint.Ratio * joint.Ratio * (_bodyInverseInertias[joint.BodyC] + _bodyInverseInertias[joint.BodyD]);
            }
            else
            {
                if (!_prismaticJointIndexById.TryGetValue(joint.JointBId, out int pIndex))
                {
                    return;
                }

                ref PrismaticJointData pri = ref _prismaticJointsData[pIndex];
                Rot rotC = EffectiveRotation(joint.BodyC);
                Rot rotD = EffectiveRotation(joint.BodyD);
                Vec2 axis = Rot.Mul(rotC, pri.LocalAxisA);
                Vec2 rC = Rot.Mul(rotC, pri.LocalAnchorA - _bodyLocalCenters[joint.BodyC]);
                Vec2 rD = Rot.Mul(rotD, pri.LocalAnchorB - _bodyLocalCenters[joint.BodyD]);
                Vec2 d = (EffectivePosition(joint.BodyD) + rD) - (EffectivePosition(joint.BodyC) + rC);
                float a1 = Vec2.Cross(d + rC, axis);
                float a2 = Vec2.Cross(rD, axis);

                jvC = -joint.Ratio * axis;
                jwC = -joint.Ratio * a1;
                jvD = joint.Ratio * axis;
                jwD = joint.Ratio * a2;

                mass += joint.Ratio * joint.Ratio * (_bodyInverseMasses[joint.BodyC] + _bodyInverseMasses[joint.BodyD] +
                        _bodyInverseInertias[joint.BodyC] * a1 * a1 + _bodyInverseInertias[joint.BodyD] * a2 * a2);
            }

            if (mass <= 0f)
            {
                return;
            }

            float C = (coordA + joint.Ratio * coordB) - joint.Constant;
            float impulse = -C / mass;

            // Phase 2.5 Stage J — writes branch on the flag. Position delta
            // accumulates; rotation delta is set from the new *effective*
            // angle (read once via EffectiveRotation, then incremented by
            // the impulse-derived angle correction).
            bool useDelta = _def.UseDeltaPositionTracking;
            Vec2 dPosA = _bodyInverseMasses[joint.BodyA] * impulse * jvA;
            Vec2 dPosB = _bodyInverseMasses[joint.BodyB] * impulse * jvB;
            Vec2 dPosC = _bodyInverseMasses[joint.BodyC] * impulse * jvC;
            Vec2 dPosD = _bodyInverseMasses[joint.BodyD] * impulse * jvD;
            float newAngleA = EffectiveRotation(joint.BodyA).Angle + _bodyInverseInertias[joint.BodyA] * impulse * jwA;
            float newAngleB = EffectiveRotation(joint.BodyB).Angle + _bodyInverseInertias[joint.BodyB] * impulse * jwB;
            float newAngleC = EffectiveRotation(joint.BodyC).Angle + _bodyInverseInertias[joint.BodyC] * impulse * jwC;
            float newAngleD = EffectiveRotation(joint.BodyD).Angle + _bodyInverseInertias[joint.BodyD] * impulse * jwD;

            if (useDelta)
            {
                _bodyDeltaPositions[joint.BodyA] += dPosA;
                _bodyDeltaPositions[joint.BodyB] += dPosB;
                _bodyDeltaPositions[joint.BodyC] += dPosC;
                _bodyDeltaPositions[joint.BodyD] += dPosD;
                _bodyDeltaRotations[joint.BodyA] = Rot.MulT(_bodyStepStartRotations[joint.BodyA], new Rot(newAngleA));
                _bodyDeltaRotations[joint.BodyB] = Rot.MulT(_bodyStepStartRotations[joint.BodyB], new Rot(newAngleB));
                _bodyDeltaRotations[joint.BodyC] = Rot.MulT(_bodyStepStartRotations[joint.BodyC], new Rot(newAngleC));
                _bodyDeltaRotations[joint.BodyD] = Rot.MulT(_bodyStepStartRotations[joint.BodyD], new Rot(newAngleD));
            }
            else
            {
                _bodyPositions[joint.BodyA] += dPosA;
                _bodyPositions[joint.BodyB] += dPosB;
                _bodyPositions[joint.BodyC] += dPosC;
                _bodyPositions[joint.BodyD] += dPosD;
                _bodyRotations[joint.BodyA] = new Rot(newAngleA);
                _bodyRotations[joint.BodyB] = new Rot(newAngleB);
                _bodyRotations[joint.BodyC] = new Rot(newAngleC);
                _bodyRotations[joint.BodyD] = new Rot(newAngleD);
            }
        }
    }
}
