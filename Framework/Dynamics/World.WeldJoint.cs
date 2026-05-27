namespace Box2DNG
{
    public sealed partial class World
    {
        internal void InitWeldJointVelocityConstraints(int index, float dt)
        {
            ref WeldJointData joint = ref _weldJointsData[index];
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

            // Soft springs (Phase 1 of TIER4_PARITY_PLAN). Per-joint Hertz wins;
            // if zero, fall through to the world default; if that's also zero,
            // the joint is rigid and behaves like the legacy split-impulse path
            // (no bias in velocity solve, full-strength position correction).
            joint.LinearSpring = ResolveJointSpring(joint.LinearHertz, joint.LinearDampingRatio, dt);
            joint.AngularSpring = ResolveJointSpring(joint.AngularHertz, joint.AngularDampingRatio, dt);

            // joint.DeltaCenter stays at the creation-time anchor offset
            // in BOTH flag modes (Phase 2.5 Stage L *reverted* after
            // empirical evidence — see BASELINE.md "cause #1").
            //
            // The original Phase 2.5 plan called for re-capturing
            // DeltaCenter at each Init when `UseDeltaPositionTracking` is
            // on, mirroring cpp box2d v3's "deltaCenter captured fresh at
            // Prepare each step" semantic. The intent: under the delta-
            // position model the bias signal becomes `C = (effective
            // anchor delta) - (step-start anchor delta) = within-step
            // anchor drift`, which is the within-step bias cpp v3 builds
            // on.
            //
            // In practice that *broke* Cantilever (flag-on lateV
            // 1.49 → 16.52, fellThrough 0 → 2) without delivering any
            // win elsewhere. The flag-on plumbing alone (`_bodyPositions`
            // stays at step-start in the sub-step loop, delta arrays
            // accumulate, `ApplyBodyDeltas` commits) is what unlocks the
            // Phase 2.5 improvements on Pyramid / Dominos /
            // CompoundShapes — re-anchoring DeltaCenter to step-start
            // is orthogonal and only useful if the entire position-
            // correction architecture also moves to cpp v3's bias-only
            // model (we still keep the v2-style position-constraint NGS
            // pass for rigid axes, so creation-anchored DeltaCenter is
            // the right reference for our hybrid model).
        }

        internal void SolveWeldJointVelocityConstraints(int index, float dt, bool useBias)
        {
            ref WeldJointData joint = ref _weldJointsData[index];
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

            // Order preserved from the legacy rigid path: linear first, then
            // angular. cpp does angular-first; matching cpp's order changes
            // Gauss-Seidel iteration enough to perturb the rigid-weld scenes
            // (Cantilever body-damping workaround moves the threshold), so we
            // stick with linear-first until Phase 1 propagates soft welds to
            // those samples.

            // -- Linear constraint --------------------------------------------------
            Vec2 linearCdot = (vB + Vec2.Cross(wB, joint.RB)) - (vA + Vec2.Cross(wA, joint.RA));
            Vec2 linearBias = Vec2.Zero;
            float linearMassScale = 1f;
            float linearImpulseScale = 0f;
            // useBias gates the soft-spring branch — matches cpp's
            // b2SolveWeldJoint (weld_joint.c:293 `if (useBias || hertz>0)`).
            // In the Relax phase (useBias=false) and with a rigid spring
            // (IsZero), this branch is skipped and the solve produces the
            // pure -Solve22(K, Cdot) form — exactly what cpp does.
            if (useBias && !joint.LinearSpring.IsZero)
            {
                // Phase 2.5 Stage C — read effective anchor positions (step-
                // start + within-step delta) so the bias signal includes the
                // post-IntegratePositions drift when the flag is on. With
                // the flag off the delta arrays stay at zero through the
                // sub-step loop and EffectivePosition degenerates to a
                // direct read of _bodyPositions — byte-identical.
                Vec2 currentDelta = (EffectivePosition(indexB) + joint.RB) - (EffectivePosition(indexA) + joint.RA);
                Vec2 C = currentDelta - joint.DeltaCenter;
                linearBias = joint.LinearSpring.BiasRate * C;
                linearMassScale = joint.LinearSpring.MassScale;
                linearImpulseScale = joint.LinearSpring.ImpulseScale;
            }
            Vec2 b = Solve22(joint.LinearMass, linearCdot + linearBias);
            Vec2 linearImpulse = new Vec2(
                -linearMassScale * b.X - linearImpulseScale * joint.Impulse.X,
                -linearMassScale * b.Y - linearImpulseScale * joint.Impulse.Y);
            joint.Impulse += linearImpulse;

            vA -= mA * linearImpulse;
            wA -= iA * Vec2.Cross(joint.RA, linearImpulse);
            vB += mB * linearImpulse;
            wB += iB * Vec2.Cross(joint.RB, linearImpulse);

            // -- Angular constraint -------------------------------------------------
            float angularCdot = wB - wA;
            float angularBias = 0f;
            float angularMassScale = 1f;
            float angularImpulseScale = 0f;
            if (useBias && !joint.AngularSpring.IsZero)
            {
                // Phase 2.5 Stage C — effective rotations include within-step delta.
                float angleError = (EffectiveRotation(indexB).Angle - EffectiveRotation(indexA).Angle) - joint.ReferenceAngle;
                angularBias = joint.AngularSpring.BiasRate * angleError;
                angularMassScale = joint.AngularSpring.MassScale;
                angularImpulseScale = joint.AngularSpring.ImpulseScale;
            }
            float angularImpulse =
                -angularMassScale * joint.AngularMass * (angularCdot + angularBias)
                - angularImpulseScale * joint.AngularImpulse;
            joint.AngularImpulse += angularImpulse;
            wA -= iA * angularImpulse;
            wB += iB * angularImpulse;
        }

        internal void SolveWeldJointPositionConstraints(int index)
        {
            ref WeldJointData joint = ref _weldJointsData[index];

            // When either axis is configured as a soft spring, the velocity
            // solve already folded position correction in via bias — there's
            // nothing for the split-impulse position pass to do for that axis.
            // For rigid axes, fall through to the legacy NGS correction.
            bool linearSoft = !joint.LinearSpring.IsZero;
            bool angularSoft = !joint.AngularSpring.IsZero;
            if (linearSoft && angularSoft)
            {
                return;
            }

            int indexA = joint.BodyA;
            int indexB = joint.BodyB;
            bool useDelta = _def.UseDeltaPositionTracking;

            // Phase 2.5 Stage C — reads are effective; writes branch on the flag.
            // With flag off, EffectivePosition / EffectiveRotation degenerate to
            // direct array reads (delta is zero), and the writes go to
            // _bodyPositions / _bodyRotations exactly like the pre-Stage-C
            // path. With flag on, the step-start arrays stay frozen and we
            // accumulate corrections into the delta arrays.
            Vec2 cA = EffectivePosition(indexA);
            Vec2 cB = EffectivePosition(indexB);
            float aA = EffectiveRotation(indexA).Angle;
            float aB = EffectiveRotation(indexB).Angle;

            float mA = _bodyInverseMasses[indexA];
            float mB = _bodyInverseMasses[indexB];
            float iA = _bodyInverseInertias[indexA];
            float iB = _bodyInverseInertias[indexB];

            Vec2 rA = Rot.Mul(new Rot(aA), joint.LocalAnchorA - _bodyLocalCenters[indexA]);
            Vec2 rB = Rot.Mul(new Rot(aB), joint.LocalAnchorB - _bodyLocalCenters[indexB]);

            if (!linearSoft)
            {
                Vec2 C = (cB + rB) - (cA + rA);
                float k11 = mA + mB + iA * rA.Y * rA.Y + iB * rB.Y * rB.Y;
                float k12 = -iA * rA.X * rA.Y - iB * rB.X * rB.Y;
                float k22 = mA + mB + iA * rA.X * rA.X + iB * rB.X * rB.X;
                Mat22 k = new Mat22(new Vec2(k11, k12), new Vec2(k12, k22));
                Vec2 impulse = Solve22(k, -C);

                if (useDelta)
                {
                    _bodyDeltaPositions[indexA] -= mA * impulse;
                    _bodyDeltaPositions[indexB] += mB * impulse;
                }
                else
                {
                    _bodyPositions[indexA] -= mA * impulse;
                    _bodyPositions[indexB] += mB * impulse;
                }
                aA -= iA * Vec2.Cross(rA, impulse);
                aB += iB * Vec2.Cross(rB, impulse);
            }

            if (!angularSoft)
            {
                float angleError = (aB - aA) - joint.ReferenceAngle;
                float angularImpulse = -joint.AngularMass * angleError;
                aA -= iA * angularImpulse;
                aB += iB * angularImpulse;
            }

            // Write the final effective angles back. Under flag-on, that means
            // updating the delta rotation so that `Mul(deltaRot, stepStartRot) ==
            // Rot(aA)` — i.e., `deltaRot = MulT(stepStartRot, Rot(aA))`. The
            // ApplyBodyDeltas pass at end of outer Step composes delta with
            // step-start and normalizes (see Stage B's magnitude-drift fix).
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

        /// <summary>
        /// Per-joint spring resolution: explicit Hertz wins, else world default,
        /// else <see cref="Softness.Zero"/> (which callers treat as legacy hard).
        /// </summary>
        internal Softness ResolveJointSpring(float hertz, float dampingRatio, float h)
        {
            if (hertz > 0f)
            {
                return Softness.Make(hertz, dampingRatio, h);
            }
            if (_def.JointHertz > 0f)
            {
                return Softness.Make(_def.JointHertz, _def.JointDampingRatio, h);
            }
            return Softness.Zero;
        }
    }
}
