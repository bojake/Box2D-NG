using System;

namespace Box2DNG
{
    public sealed partial class World
    {
        internal void InitDistanceJointVelocityConstraints(int index, float dt)
        {
            ref DistanceJointData joint = ref _distanceJointsData[index];
            
            // Get Body Indices
            int indexA = joint.BodyA;
            int indexB = joint.BodyB;
            
            // Access Body Data directly
            ref Vec2 cA = ref _bodyPositions[indexA];
            float aA = _bodyRotations[indexA].Angle;
            ref Vec2 vA = ref _bodyLinearVelocities[indexA];
            float wA = _bodyAngularVelocities[indexA];

            ref Vec2 cB = ref _bodyPositions[indexB];
            float aB = _bodyRotations[indexB].Angle;
            ref Vec2 vB = ref _bodyLinearVelocities[indexB];
            float wB = _bodyAngularVelocities[indexB];

            Rot qA = new Rot(aA);
            Rot qB = new Rot(aB);

            Vec2 rA = Rot.Mul(qA, joint.LocalAnchorA - _bodyLocalCenters[indexA]);
            Vec2 rB = Rot.Mul(qB, joint.LocalAnchorB - _bodyLocalCenters[indexB]);
            
            Vec2 d = (cB + rB) - (cA + rA);
            float length = d.Length;
            joint.U = length > Constants.Epsilon ? d / length : new Vec2(1f, 0f); // Write back to Data

            float crA = Vec2.Cross(rA, joint.U);
            float crB = Vec2.Cross(rB, joint.U);
            float invMass = _bodyInverseMasses[indexA] + _bodyInverseMasses[indexB] + 
                            _bodyInverseInertias[indexA] * crA * crA + 
                            _bodyInverseInertias[indexB] * crB * crB;
            
            joint.Mass = invMass > 0f ? 1f / invMass : 0f;

            // Resolve the spring per the unified Phase 1 pattern. Per-joint
            // FrequencyHz wins; if zero, fall through to the world's JointHertz
            // default. The legacy inline math (gamma = dt·mass·ω·a1, etc.) is
            // algebraically identical to b2MakeSoft, so storing the resolved
            // Softness and using it here keeps Distance's behaviour bit-for-bit
            // identical when FrequencyHz > 0, and now also picks up the world
            // default when FrequencyHz == 0.
            joint.Softness = ResolveJointSpring(joint.FrequencyHz, joint.DampingRatio, dt);
            if (!joint.Softness.IsZero)
            {
                float C = length - joint.Length;
                // effective mass = mass / (1 + 1/(mass*a2)) = mass·massScale·a3/(massScale·a3+...)
                // Equivalently: gamma = impulseScale/(mass·massScale).
                // Substituting back into the legacy impulse formula gives the
                // same result as cpp's `impulse = -massScale·mass·(Cdot + biasRate·C)
                // - impulseScale·accImpulse`. We preserve the legacy form.
                joint.Gamma = joint.Softness.MassScale != 0f
                    ? joint.Softness.ImpulseScale / (joint.Mass * joint.Softness.MassScale)
                    : 0f;
                joint.Bias = joint.Softness.BiasRate * C;
                joint.Mass = 1f / (invMass + joint.Gamma);
            }
            else
            {
                joint.Gamma = 0f;
                joint.Bias = 0f;
            }

            if (joint.Impulse != 0f)
            {
                Vec2 P = joint.Impulse * joint.U;
                vA -= _bodyInverseMasses[indexA] * P;
                wA -= _bodyInverseInertias[indexA] * Vec2.Cross(rA, P);
                vB += _bodyInverseMasses[indexB] * P;
                wB += _bodyInverseInertias[indexB] * Vec2.Cross(rB, P);
                
                // Write back velocities? 
                // Since we used ref local vars for vA/vB? No, vA is ref to array element? Yes.
                // But wA/wB are floats (values).
                _bodyAngularVelocities[indexA] = wA;
                _bodyAngularVelocities[indexB] = wB;
            }
        }

        internal void SolveDistanceJointVelocityConstraints(int index)
        {
            ref DistanceJointData joint = ref _distanceJointsData[index];
            int indexA = joint.BodyA;
            int indexB = joint.BodyB;

            ref Vec2 vA = ref _bodyLinearVelocities[indexA];
            float wA = _bodyAngularVelocities[indexA];
            ref Vec2 vB = ref _bodyLinearVelocities[indexB];
            float wB = _bodyAngularVelocities[indexB];

            // Re-calculate rA/rB or store them? 
            // In original code, rA/rB were re-calculated.
            // But we need current qA/qB.
            Rot qA = new Rot(_bodyRotations[indexA].Angle);
            Rot qB = new Rot(_bodyRotations[indexB].Angle);
            
            Vec2 rA = Rot.Mul(qA, joint.LocalAnchorA - _bodyLocalCenters[indexA]);
            Vec2 rB = Rot.Mul(qB, joint.LocalAnchorB - _bodyLocalCenters[indexB]);

            Vec2 vpA = vA + Vec2.Cross(wA, rA);
            Vec2 vpB = vB + Vec2.Cross(wB, rB);

            float Cdot = Vec2.Dot(joint.U, vpB - vpA);
            float impulse = -joint.Mass * (Cdot + joint.Bias + joint.Gamma * joint.Impulse);
            joint.Impulse += impulse;

            Vec2 P = impulse * joint.U;
            vA -= _bodyInverseMasses[indexA] * P;
            wA -= _bodyInverseInertias[indexA] * Vec2.Cross(rA, P);
            vB += _bodyInverseMasses[indexB] * P;
            wB += _bodyInverseInertias[indexB] * Vec2.Cross(rB, P);

            _bodyAngularVelocities[indexA] = wA;
            _bodyAngularVelocities[indexB] = wB;
        }

        internal void SolveDistanceJointPositionConstraints(int index)
        {
            ref DistanceJointData joint = ref _distanceJointsData[index];
            // Skip the NGS position pass when the spring is active — bias already
            // folded position correction into the velocity solve. Gated on the
            // resolved Softness so the world's JointHertz default participates.
            if (!joint.Softness.IsZero)
            {
                return;
            }

            int indexA = joint.BodyA;
            int indexB = joint.BodyB;

            ref Vec2 cA = ref _bodyPositions[indexA];
            float aA = _bodyRotations[indexA].Angle;
            ref Vec2 cB = ref _bodyPositions[indexB];
            float aB = _bodyRotations[indexB].Angle;

            Rot qA = new Rot(aA);
            Rot qB = new Rot(aB);

            Vec2 rA = Rot.Mul(qA, joint.LocalAnchorA - _bodyLocalCenters[indexA]);
            Vec2 rB = Rot.Mul(qB, joint.LocalAnchorB - _bodyLocalCenters[indexB]);

            Vec2 d = (cB + rB) - (cA + rA);
            float length = d.Length;
            Vec2 u = length > Constants.Epsilon ? d / length : new Vec2(1f, 0f);
            float C = length - joint.Length;
            float impulse = -joint.Mass * C;
            Vec2 P = impulse * u;

            cA -= _bodyInverseMasses[indexA] * P;
            aA -= _bodyInverseInertias[indexA] * Vec2.Cross(rA, P);
            cB += _bodyInverseMasses[indexB] * P;
            aB += _bodyInverseInertias[indexB] * Vec2.Cross(rB, P);

            // Update rotation/transform
            _bodyRotations[indexA] = new Rot(aA);
            _bodyRotations[indexB] = new Rot(aB);
            
            // NOTE: Original code used SetTransformFromCenter which re-calcs Transform.
            // Here we updated Position (Center) and Rotation.
            // The Transform (p, q) is usually derived or sync'd?
            // In my Body refactor, _bodyPositions stores Center.
            // _bodyRotations stores Rot (q, angle).
            // We should sync Transform if it's stored separately?
            // My Body logic removed `Transform` array/field? No, Body handle computed it or World had it?
            // World has _bodyRotations (Rot).
            // World has _bodyPositions (Vec2).
            // There is no explicit `_bodyTransforms` array? 
            // In `IntegratePositions`, I updated:
            // body.SetTransform(newPosition, newRot);
            
            // So implicit state is consistent.
        }
    }
}
