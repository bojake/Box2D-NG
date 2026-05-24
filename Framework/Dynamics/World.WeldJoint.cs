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
            // joint.DeltaCenter is set once at CreateJoint and stays put — it
            // represents the rest-state world anchor delta. Re-capturing it
            // here would erase accumulated drift.
        }

        internal void SolveWeldJointVelocityConstraints(int index, float dt)
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
            if (!joint.LinearSpring.IsZero)
            {
                // Current world-space anchor delta minus the rest value captured
                // at Prepare time. Drives drift back to zero.
                Vec2 currentDelta = (_bodyPositions[indexB] + joint.RB) - (_bodyPositions[indexA] + joint.RA);
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
            if (!joint.AngularSpring.IsZero)
            {
                float angleError = (_bodyRotations[indexB].Angle - _bodyRotations[indexA].Angle) - joint.ReferenceAngle;
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

            ref Vec2 cA = ref _bodyPositions[indexA];
            ref Vec2 cB = ref _bodyPositions[indexB];
            float aA = _bodyRotations[indexA].Angle;
            float aB = _bodyRotations[indexB].Angle;

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

                cA -= mA * impulse;
                cB += mB * impulse;
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

            _bodyRotations[indexA] = new Rot(aA);
            _bodyRotations[indexB] = new Rot(aB);
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
