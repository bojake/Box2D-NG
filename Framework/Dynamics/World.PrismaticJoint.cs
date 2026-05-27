using System;

namespace Box2DNG
{
    public sealed partial class World
    {
        public PrismaticJoint CreateJoint(PrismaticJointDef def)
        {
            if (_prismaticJointCount >= _prismaticJointCapacity)
            {
                int newCapacity = Math.Max(_prismaticJointCapacity * 2, 256);
                Array.Resize(ref _prismaticJoints, newCapacity);
                Array.Resize(ref _prismaticJointsData, newCapacity);
                _prismaticJointCapacity = newCapacity;
            }

            int index = _prismaticJointCount;
            int id = _jointIdPool.Alloc();
            
            PrismaticJoint joint = new PrismaticJoint(this, id, index);
            _prismaticJoints[index] = joint;
            
            ref PrismaticJointData data = ref _prismaticJointsData[index];
            data.Id = id;
            data.BodyA = def.BodyA.Id;
            data.BodyB = def.BodyB.Id;
            data.CollideConnected = def.CollideConnected;
            data.LocalAnchorA = def.LocalAnchorA;
            data.LocalAnchorB = def.LocalAnchorB;
            data.LocalAxisA = def.LocalAxisA;
            data.ReferenceAngle = def.ReferenceAngle;
            
            data.EnableSpring = def.EnableSpring;
            data.FrequencyHz = def.FrequencyHz;
            data.DampingRatio = def.DampingRatio;
            data.TargetTranslation = def.TargetTranslation;
            
            data.EnableLimit = def.EnableLimit;
            data.LowerTranslation = def.LowerTranslation;
            data.UpperTranslation = def.UpperTranslation;
            
            data.EnableMotor = def.EnableMotor;
            data.MotorSpeed = def.MotorSpeed;
            data.MaxMotorForce = def.MaxMotorForce;
            
            data.ConstraintHertz = def.ConstraintHertz;
            data.ConstraintDampingRatio = def.ConstraintDampingRatio;
            
            // Reset state
            data.Impulse = Vec2.Zero;
            data.MotorImpulse = 0f;
            data.SpringImpulse = 0f;
            data.LowerImpulse = 0f;
            data.UpperImpulse = 0f;

            _prismaticJointCount++;
            _prismaticJointIndexById[id] = index;
            
            JointHandle handle = new JointHandle(JointType.Prismatic, id);
            LinkJoint(handle, def.BodyA, def.BodyB);
            SetJointSolverSet(handle, GetJointSolverSetType(def.BodyA, def.BodyB), GetJointSolverSetId(def.BodyA, def.BodyB));
            TryAddJointToSleepingSet(handle, def.BodyA, def.BodyB);

            if (TryGetJointSolverSetType(handle, out SolverSetType solverSetType) && solverSetType == SolverSetType.Awake)
            {
                _awakeSet.Joints.Add(handle);
            }

            MarkIslandDirty(def.BodyA);
            MarkIslandDirty(def.BodyB);
            HandleJointCreationAwakeState(def.BodyA, def.BodyB);

            return joint;
        }

        public bool DestroyJoint(PrismaticJoint joint)
        {
            int index = joint.Index;
            if (index == -1) return false;

            Body bodyA = joint.BodyA;
            Body bodyB = joint.BodyB;
            int jointId = joint.Id;

            DestroyDependentGearJoints(jointId);

            int lastIndex = _prismaticJointCount - 1;
            if (index != lastIndex)
            {
                // Swap with last
                PrismaticJoint lastJoint = _prismaticJoints[lastIndex];
                lastJoint.Index = index;
                _prismaticJoints[index] = lastJoint;
                _prismaticJointsData[index] = _prismaticJointsData[lastIndex];
                
                _prismaticJointIndexById[lastJoint.Id] = index;
            }

            _prismaticJoints[lastIndex] = null!;
            _prismaticJointsData[lastIndex] = default;
            _prismaticJointCount--;
            
            _jointIdPool.Free(jointId);
            _prismaticJointIndexById.Remove(jointId);
            
            joint.Index = -1;
            joint.Id = -1;
            
            JointHandle handle = new JointHandle(JointType.Prismatic, jointId);
            UnlinkJoint(handle, bodyA, bodyB);
            ClearJointSolverSet(handle);
            ClearJointConstraintGraphIndex(handle);
            MarkIslandDirty(bodyA);
            MarkIslandDirty(bodyB);
            
            RemoveJointFromSolverSets(JointType.Prismatic, jointId);
            _islandsDirty = true;
            return true;
        }

        internal void InitPrismaticJointVelocityConstraints(int index, float dt)
        {
            ref PrismaticJointData joint = ref _prismaticJointsData[index];
            int indexA = joint.BodyA;
            int indexB = joint.BodyB;

            float mA = _bodyInverseMasses[indexA];
            float mB = _bodyInverseMasses[indexB];
            float iA = _bodyInverseInertias[indexA];
            float iB = _bodyInverseInertias[indexB];

            Rot qA = _bodyRotations[indexA];
            Rot qB = _bodyRotations[indexB];
            
            Vec2 rA = Rot.Mul(qA, joint.LocalAnchorA - _bodyLocalCenters[indexA]);
            Vec2 rB = Rot.Mul(qB, joint.LocalAnchorB - _bodyLocalCenters[indexB]);
            
            Vec2 d = (_bodyPositions[indexB] + rB) - (_bodyPositions[indexA] + rA);
            joint.Axis = Rot.Mul(qA, joint.LocalAxisA);
            joint.Perp = new Vec2(-joint.Axis.Y, joint.Axis.X);
            
            joint.S1 = Vec2.Cross(d + rA, joint.Perp);
            joint.S2 = Vec2.Cross(rB, joint.Perp);
            joint.A1 = Vec2.Cross(d + rA, joint.Axis);
            joint.A2 = Vec2.Cross(rB, joint.Axis);

            float k11 = mA + mB + iA * joint.S1 * joint.S1 + iB * joint.S2 * joint.S2;
            float k12 = iA * joint.S1 + iB * joint.S2;
            float k22 = iA + iB;
            if (k22 == 0f) k22 = 1f;
            
            joint.K = new Mat22(new Vec2(k11, k12), new Vec2(k12, k22));
            
            float motorMass = mA + mB + iA * joint.A1 * joint.A1 + iB * joint.A2 * joint.A2;
            joint.AxialMass = motorMass > 0f ? 1f / motorMass : 0f;

            if (!joint.EnableMotor) joint.MotorImpulse = 0f;
            if (!joint.EnableLimit)
            {
                joint.LowerImpulse = 0f;
                joint.UpperImpulse = 0f;
            }
            
            // Per-joint spring tuning. FrequencyHz drives the axial spring;
            // EnableSpring toggles it (per-joint behaviour preserved). Falls
            // through to the world default for unified Phase 1 behaviour.
            joint.SpringSoftness = ResolveJointSpring(joint.FrequencyHz, joint.DampingRatio, dt);
            if (!joint.EnableSpring) joint.SpringImpulse = 0f;

            // Orthogonal point + angular constraint softness. cpp's b2_constraintSoftness
            // pattern: per-joint wins, else world default; else legacy rigid behaviour
            // (Softness.Rigid keeps massScale=1, impulseScale=0).
            joint.ConstraintSoftness = ResolveJointSpring(joint.ConstraintHertz, joint.ConstraintDampingRatio, dt);
            if (joint.ConstraintSoftness.IsZero)
            {
                joint.ConstraintSoftness = Softness.Rigid;
            }

            float axialImpulse = joint.SpringImpulse + joint.MotorImpulse + joint.LowerImpulse - joint.UpperImpulse;
            if (joint.Impulse.X != 0f || joint.Impulse.Y != 0f || axialImpulse != 0f)
            {
                Vec2 P = axialImpulse * joint.Axis + joint.Impulse.X * joint.Perp;
                float LA = axialImpulse * joint.A1 + joint.Impulse.X * joint.S1 + joint.Impulse.Y;
                float LB = axialImpulse * joint.A2 + joint.Impulse.X * joint.S2 + joint.Impulse.Y;
                
                _bodyLinearVelocities[indexA] -= mA * P;
                _bodyAngularVelocities[indexA] -= iA * LA;
                _bodyLinearVelocities[indexB] += mB * P;
                _bodyAngularVelocities[indexB] += iB * LB;
            }
        }

        internal void SolvePrismaticJointVelocityConstraints(int index, float dt, bool useBias)
        {
            ref PrismaticJointData joint = ref _prismaticJointsData[index];
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
            
            // Re-calculate d using positions for current step? 
            // The original code uses d computed in Init.
            // But we need the current relative velocity.
            
            // To properly handle the spring/limit length, we usually need the current position separation. 
            // In init we computed d. Let's reuse d from init?
            // Actually, for position dependent terms, we should probably recompute if we want high fidelity,
            // but standard Box2D often computes geometric constants in Init (Warm Start) and uses them.
            // Let's stick to the ported logic: it uses the cached _axis, _a1, _a2 etc.
            
            // Wait, we need 'd' for the spring translation?
            // "float translation = Vec2.Dot(_axis, d);"
            // So we need d. We can reconstruct it or cache it.
            // Let's recompute minimal necessary or cache it in Init.
            // Currently PrismaticJoint.cs computes d in Init.
            // But Solve uses d. Is it the SAME d?
            // "Vec2 d = (BodyB.GetWorldCenter() + rB) - (BodyA.GetWorldCenter() + rA);" in Init
            // And also in Solve?
            // Yes, "Vec2 d = (BodyB.GetWorldCenter() + rB) - (BodyA.GetWorldCenter() + rA);" is inside SolveVelocityConstraints too!
            // So I should recompute d here.
            
            // Phase 2.5 Stage E — effective rotations + positions.
            Rot qA = EffectiveRotation(indexA);
            Rot qB = EffectiveRotation(indexB);
            Vec2 rA = Rot.Mul(qA, joint.LocalAnchorA - _bodyLocalCenters[indexA]);
            Vec2 rB = Rot.Mul(qB, joint.LocalAnchorB - _bodyLocalCenters[indexB]);
            Vec2 d = (EffectivePosition(indexB) + rB) - (EffectivePosition(indexA) + rA);
            
            // Original code recomputes axis, perp, a1, a2, s1, s2 in SolveVelocityConstraints too.
            // Since bodies move, these change.
            // So we should recompute them here as well.
            
            joint.Axis = Rot.Mul(qA, joint.LocalAxisA);
            joint.Perp = new Vec2(-joint.Axis.Y, joint.Axis.X);
            joint.S1 = Vec2.Cross(d + rA, joint.Perp);
            joint.S2 = Vec2.Cross(rB, joint.Perp);
            joint.A1 = Vec2.Cross(d + rA, joint.Axis);
            joint.A2 = Vec2.Cross(rB, joint.Axis);
            
            float axialMass = mA + mB + iA * joint.A1 * joint.A1 + iB * joint.A2 * joint.A2;
            joint.AxialMass = axialMass > 0f ? 1f / axialMass : 0f;

            if (joint.EnableSpring)
            {
                float translation = Vec2.Dot(joint.Axis, d);
                float C = translation - joint.TargetTranslation;
                float Cdot = Vec2.Dot(joint.Axis, vB - vA) + joint.A2 * wB - joint.A1 * wA;
                float impulse = -joint.SpringSoftness.MassScale * joint.AxialMass * (Cdot + joint.SpringSoftness.BiasRate * C)
                                - joint.SpringSoftness.ImpulseScale * joint.SpringImpulse;
                joint.SpringImpulse += impulse;
                
                Vec2 P = impulse * joint.Axis;
                float LA = impulse * joint.A1;
                float LB = impulse * joint.A2;
                vA -= mA * P;
                wA -= iA * LA;
                vB += mB * P;
                wB += iB * LB;
            }

            if (joint.EnableMotor)
            {
                float Cdot = Vec2.Dot(joint.Axis, vB - vA) + joint.A2 * wB - joint.A1 * wA;
                float impulse = joint.AxialMass * (joint.MotorSpeed - Cdot);
                float oldImpulse = joint.MotorImpulse;
                float maxImpulse = joint.MaxMotorForce * dt;
                joint.MotorImpulse = MathFng.Clamp(joint.MotorImpulse + impulse, -maxImpulse, maxImpulse);
                impulse = joint.MotorImpulse - oldImpulse;
                
                Vec2 P = impulse * joint.Axis;
                float LA = impulse * joint.A1;
                float LB = impulse * joint.A2;
                vA -= mA * P;
                wA -= iA * LA;
                vB += mB * P;
                wB += iB * LB;
            }
            
            if (joint.EnableLimit)
            {
                float translation = Vec2.Dot(joint.Axis, d);
                float speculativeDistance = 0.25f * (joint.UpperTranslation - joint.LowerTranslation);
                if (speculativeDistance < 0f) speculativeDistance = 0f;
                float invDt = dt > 0f ? 1f / dt : 0f;

                // Lower limit. cpp prismatic_joint.c:501 — the penetrating-
                // limit branch gates on useBias; Relax uses bias=0,
                // massScale=1, impulseScale=0 so the impulse is purely
                // Cdot-driven (no re-correcting the limit position).
                {
                    float C = translation - joint.LowerTranslation;
                    if (C < speculativeDistance)
                    {
                        float bias = 0f;
                        float massScale = 1f;
                        float impulseScale = 0f;
                        if (C > 0f)
                        {
                            bias = Math.Min(C, 1f) * invDt;
                        }
                        else if (useBias)
                        {
                            bias = joint.ConstraintSoftness.BiasRate * C;
                            massScale = joint.ConstraintSoftness.MassScale;
                            impulseScale = joint.ConstraintSoftness.ImpulseScale;
                        }
                        
                        float Cdot = Vec2.Dot(joint.Axis, vB - vA) + joint.A2 * wB - joint.A1 * wA;
                        float oldImpulse = joint.LowerImpulse;
                        float deltaImpulse = -joint.AxialMass * massScale * (Cdot + bias) - impulseScale * oldImpulse;
                        joint.LowerImpulse = Math.Max(oldImpulse + deltaImpulse, 0f);
                        deltaImpulse = joint.LowerImpulse - oldImpulse;
                        
                        Vec2 P = deltaImpulse * joint.Axis;
                        float LA = deltaImpulse * joint.A1;
                        float LB = deltaImpulse * joint.A2;
                        vA -= mA * P;
                        wA -= iA * LA;
                        vB += mB * P;
                        wB += iB * LB;
                    }
                    else
                    {
                        joint.LowerImpulse = 0f;
                    }
                }
                
                // Upper limit — same useBias gating as the lower limit.
                {
                    float C = joint.UpperTranslation - translation;
                    if (C < speculativeDistance)
                    {
                        float bias = 0f;
                        float massScale = 1f;
                        float impulseScale = 0f;
                        if (C > 0f)
                        {
                             bias = Math.Min(C, 1f) * invDt;
                        }
                        else if (useBias)
                        {
                            bias = joint.ConstraintSoftness.BiasRate * C;
                            massScale = joint.ConstraintSoftness.MassScale;
                            impulseScale = joint.ConstraintSoftness.ImpulseScale;
                        }
                        
                        float Cdot = Vec2.Dot(joint.Axis, vA - vB) + joint.A1 * wA - joint.A2 * wB;
                        float oldImpulse = joint.UpperImpulse;
                        float deltaImpulse = -joint.AxialMass * massScale * (Cdot + bias) - impulseScale * oldImpulse;
                        joint.UpperImpulse = Math.Max(oldImpulse + deltaImpulse, 0f);
                        deltaImpulse = joint.UpperImpulse - oldImpulse;
                        
                        Vec2 P = deltaImpulse * joint.Axis;
                        float LA = deltaImpulse * joint.A1;
                        float LB = deltaImpulse * joint.A2;
                        vA += mA * P;
                        wA += iA * LA;
                        vB -= mB * P;
                        wB -= iB * LB;
                    }
                    else
                    {
                        joint.UpperImpulse = 0f;
                    }
                }
            }

            // Linear/Angular constraint (perpendicular) — cpp's p2p +
            // angular pair, gated on useBias (prismatic_joint.c:595).
            {
                Vec2 Cdot1 = new Vec2(
                    Vec2.Dot(joint.Perp, vB - vA) + joint.S2 * wB - joint.S1 * wA,
                    wB - wA);

                Vec2 bias = Vec2.Zero;
                float massScale = 1f;
                float impulseScale = 0f;
                if (useBias)
                {
                    Vec2 C1 = new Vec2(
                        Vec2.Dot(joint.Perp, d),
                        (qB.Angle - qA.Angle) - joint.ReferenceAngle);
                    bias = new Vec2(joint.ConstraintSoftness.BiasRate * C1.X, joint.ConstraintSoftness.BiasRate * C1.Y);
                    massScale = joint.ConstraintSoftness.MassScale;
                    impulseScale = joint.ConstraintSoftness.ImpulseScale;
                }
                Vec2 rhs = Cdot1 + bias;
                Vec2 impulseDelta = Solve22(joint.K, rhs);
                Vec2 df = new Vec2(
                    -massScale * impulseDelta.X - impulseScale * joint.Impulse.X,
                    -massScale * impulseDelta.Y - impulseScale * joint.Impulse.Y);
                joint.Impulse += df;
                
                Vec2 Pperp = df.X * joint.Perp;
                float LAperp = df.X * joint.S1 + df.Y;
                float LBperp = df.X * joint.S2 + df.Y;
                vA -= mA * Pperp;
                wA -= iA * LAperp;
                vB += mB * Pperp;
                wB += iB * LBperp;
            }
        }
        
        internal void SolvePrismaticJointPositionConstraints(int index)
        {
            ref PrismaticJointData joint = ref _prismaticJointsData[index];
            int indexA = joint.BodyA;
            int indexB = joint.BodyB;
            
            bool useDelta = _def.UseDeltaPositionTracking;

            // Phase 2.5 Stage E — effective reads; writes branch on the flag.
            Vec2 cA = EffectivePosition(indexA);
            Vec2 cB = EffectivePosition(indexB);
            float aA = EffectiveRotation(indexA).Angle;
            float aB = EffectiveRotation(indexB).Angle;

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
            float a1 = Vec2.Cross(d + rA, axis);
            float a2 = Vec2.Cross(rB, axis);
            
            Vec2 C1 = new Vec2(
                Vec2.Dot(perp, d),
                (aB - aA) - joint.ReferenceAngle);
            
            float linearError = Math.Abs(C1.X);
            float angularError = Math.Abs(C1.Y);
            
            bool active = false;
            float C2 = 0f;
            if (joint.EnableLimit)
            {
                float translation = Vec2.Dot(axis, d);
                if (Math.Abs(joint.UpperTranslation - joint.LowerTranslation) < 2f * Constants.LinearSlop)
                {
                    C2 = MathFng.Clamp(translation, -Constants.MaxLinearCorrection, Constants.MaxLinearCorrection);
                    linearError = Math.Max(linearError, Math.Abs(translation));
                    active = true;
                }
                else if (translation <= joint.LowerTranslation)
                {
                    C2 = MathFng.Clamp(translation - joint.LowerTranslation + Constants.LinearSlop, -Constants.MaxLinearCorrection, 0f);
                    linearError = Math.Max(linearError, joint.LowerTranslation - translation);
                    active = true;
                }
                else if (translation >= joint.UpperTranslation)
                {
                    C2 = MathFng.Clamp(translation - joint.UpperTranslation - Constants.LinearSlop, 0f, Constants.MaxLinearCorrection);
                    linearError = Math.Max(linearError, translation - joint.UpperTranslation);
                    active = true;
                }
            }
            
            Vec3 impulse;
            if (active)
            {
                float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
                float k12 = iA * s1 + iB * s2;
                float k13 = iA * s1 * a1 + iB * s2 * a2;
                float k22 = iA + iB;
                if (k22 == 0f) k22 = 1f;
                float k23 = iA * a1 + iB * a2;
                float k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;
                
                Mat33 K = new Mat33(
                    new Vec3(k11, k12, k13),
                    new Vec3(k12, k22, k23),
                    new Vec3(k13, k23, k33));
                
                Vec3 C = new Vec3(C1.X, C1.Y, C2);
                impulse = Solve33(K, -C);
            }
            else
            {
                float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
                float k12 = iA * s1 + iB * s2;
                float k22 = iA + iB;
                if (k22 == 0f) k22 = 1f;

                Mat22 K = new Mat22(new Vec2(k11, k12), new Vec2(k12, k22));
                Vec2 impulse1 = Solve22(K, -C1);
                impulse = new Vec3(impulse1.X, impulse1.Y, 0f);
            }
            
            Vec2 P = impulse.X * perp + impulse.Z * axis;
            float LA = impulse.X * s1 + impulse.Y + impulse.Z * a1;
            float LB = impulse.X * s2 + impulse.Y + impulse.Z * a2;
            
            if (useDelta)
            {
                _bodyDeltaPositions[indexA] -= mA * P;
                _bodyDeltaPositions[indexB] += mB * P;
            }
            else
            {
                _bodyPositions[indexA] -= mA * P;
                _bodyPositions[indexB] += mB * P;
            }
            aA -= iA * LA;
            aB += iB * LB;

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

