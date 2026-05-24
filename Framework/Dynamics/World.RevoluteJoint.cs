using System;


namespace Box2DNG
{
    public sealed partial class World
    {
        public RevoluteJoint CreateJoint(RevoluteJointDef def)
        {
            if (_revoluteJointCount >= _revoluteJointCapacity)
            {
                int newCapacity = Math.Max(_revoluteJointCapacity * 2, 256);
                Array.Resize(ref _revoluteJoints, newCapacity);
                Array.Resize(ref _revoluteJointsData, newCapacity);
                _revoluteJointCapacity = newCapacity;
            }

            int index = _revoluteJointCount;
            int id = _jointIdPool.Alloc();
            
            RevoluteJoint joint = new RevoluteJoint(this, id, index);
            _revoluteJoints[index] = joint;
            
            ref RevoluteJointData data = ref _revoluteJointsData[index];
            data.Id = id;
            data.BodyA = def.BodyA.Id;
            data.BodyB = def.BodyB.Id;
            data.CollideConnected = def.CollideConnected;
            data.LocalAnchorA = def.LocalAnchorA;
            data.LocalAnchorB = def.LocalAnchorB;
            data.ReferenceAngle = def.ReferenceAngle;
            data.EnableMotor = def.EnableMotor;
            data.MotorSpeed = def.MotorSpeed;
            data.MaxMotorTorque = def.MaxMotorTorque;
            data.EnableLimit = def.EnableLimit;
            data.LowerAngle = def.LowerAngle;
            data.UpperAngle = def.UpperAngle;
            data.LinearHertz = def.LinearHertz;
            data.LinearDampingRatio = def.LinearDampingRatio;

            // Rest-state world anchor delta captured once at creation. The soft
            // linear spring drives the current anchor delta back toward this
            // value; recapturing each step would erase the drift we want to
            // correct.
            Vec2 worldAnchorA = Transform.Mul(def.BodyA.Transform, def.LocalAnchorA);
            Vec2 worldAnchorB = Transform.Mul(def.BodyB.Transform, def.LocalAnchorB);
            data.DeltaCenter = worldAnchorB - worldAnchorA;

            // Reset state
            data.Impulse = Vec2.Zero;
            data.MotorImpulse = 0f;
            data.LimitImpulse = 0f;

            _revoluteJointCount++;
            
            // Bookkeeping
            // _revoluteJointIndexById[id] = index; // If we keep this map?
            // DistanceJoint used _distanceJointIndexById.
            // Do we have _revoluteJointIndexById?
            // Yes, in World.cs: private readonly Dictionary<int, int> _revoluteJointIndexById
            _revoluteJointIndexById[id] = index;
            
            JointHandle handle = new JointHandle(JointType.Revolute, id);
            LinkJoint(handle, def.BodyA, def.BodyB); // Use def.BodyA which are Objects
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

        public bool DestroyJoint(RevoluteJoint joint)
        {
            int index = joint.Index;
            if (index == -1) return false;

            Body bodyA = joint.BodyA;
            Body bodyB = joint.BodyB;
            int jointId = joint.Id;

            DestroyDependentGearJoints(jointId);

            int lastIndex = _revoluteJointCount - 1;
            if (index != lastIndex)
            {
                // Swap with last
                RevoluteJoint lastJoint = _revoluteJoints[lastIndex];
                lastJoint.Index = index;
                _revoluteJoints[index] = lastJoint;
                _revoluteJointsData[index] = _revoluteJointsData[lastIndex];
                
                // Update map for moved joint
                _revoluteJointIndexById[lastJoint.Id] = index;
            }

            _revoluteJoints[lastIndex] = null!;
            _revoluteJointsData[lastIndex] = default;
            _revoluteJointCount--;
            
            _jointIdPool.Free(jointId);
            _revoluteJointIndexById.Remove(jointId);
            
            joint.Index = -1;
            joint.Id = -1;
            
            // Cleanup logic
            
            JointHandle handle = new JointHandle(JointType.Revolute, jointId);
            UnlinkJoint(handle, bodyA, bodyB);
            ClearJointSolverSet(handle);
            ClearJointConstraintGraphIndex(handle);
            MarkIslandDirty(bodyA);
            MarkIslandDirty(bodyB);
            
            RemoveJointFromSolverSets(JointType.Revolute, jointId);
            _islandsDirty = true;
            return true;
        }

        internal void InitRevoluteJointVelocityConstraints(int index, float dt)
        {
            ref RevoluteJointData joint = ref _revoluteJointsData[index];
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

            float k11 = mA + mB + iA * rA.Y * rA.Y + iB * rB.Y * rB.Y;
            float k12 = -iA * rA.X * rA.Y - iB * rB.X * rB.Y;
            float k22 = mA + mB + iA * rA.X * rA.X + iB * rB.X * rB.X;

            joint.Mass = new Mat22(new Vec2(k11, k12), new Vec2(k12, k22));

            joint.MotorMass = iA + iB;
            if (joint.MotorMass > 0.0f)
            {
                joint.MotorMass = 1.0f / joint.MotorMass;
            }

            // Soft point-to-point spring. Falls through to the world default
            // when LinearHertz==0; if that's also 0 the spring is Zero and the
            // legacy rigid path runs (no bias in velocity solve, full-strength
            // position correction).
            joint.LinearSpring = ResolveJointSpring(joint.LinearHertz, joint.LinearDampingRatio, dt);

            if (!joint.EnableLimit)
            {
                joint.LimitImpulse = 0.0f;
            }
            if (!joint.EnableMotor)
            {
                joint.MotorImpulse = 0.0f;
            }

            if (dt * (joint.Impulse.X + joint.Impulse.Y + joint.MotorImpulse + joint.LimitImpulse) != 0f)
            {
                Vec2 P = new Vec2(joint.Impulse.X, joint.Impulse.Y);
                float axialImpulse = joint.MotorImpulse + joint.LimitImpulse;
                
                _bodyLinearVelocities[indexA] -= mA * P;
                _bodyAngularVelocities[indexA] -= iA * (Vec2.Cross(rA, P) + axialImpulse);
                
                _bodyLinearVelocities[indexB] += mB * P;
                _bodyAngularVelocities[indexB] += iB * (Vec2.Cross(rB, P) + axialImpulse);
            }
            else 
            {
                joint.Impulse = Vec2.Zero;
                joint.MotorImpulse = 0f;
                joint.LimitImpulse = 0f;
            }
        }

        internal void SolveRevoluteJointVelocityConstraints(int index, float dt)
        {
            ref RevoluteJointData joint = ref _revoluteJointsData[index];
            int indexA = joint.BodyA;
            int indexB = joint.BodyB;
            
            ref Vec2 vA = ref _bodyLinearVelocities[indexA];
            float wA = _bodyAngularVelocities[indexA];
            ref Vec2 vB = ref _bodyLinearVelocities[indexB];
            float wB = _bodyAngularVelocities[indexB];
            
            float mA = _bodyInverseMasses[indexA];
            float mB = _bodyInverseMasses[indexB];
            float iA = _bodyInverseInertias[indexA];
            float iB = _bodyInverseInertias[indexB];
            
            Rot qA = _bodyRotations[indexA];
            Rot qB = _bodyRotations[indexB];
            Vec2 rA = Rot.Mul(qA, joint.LocalAnchorA - _bodyLocalCenters[indexA]);
            Vec2 rB = Rot.Mul(qB, joint.LocalAnchorB - _bodyLocalCenters[indexB]);

            // Solve Motor
            if (joint.EnableMotor)
            {
                float Cdot = wB - wA - joint.MotorSpeed;
                float impulse = -joint.MotorMass * Cdot;
                float oldImpulse = joint.MotorImpulse;
                float maxImpulse = dt * joint.MaxMotorTorque;
                joint.MotorImpulse = Math.Clamp(joint.MotorImpulse + impulse, -maxImpulse, maxImpulse);
                impulse = joint.MotorImpulse - oldImpulse;

                wA -= iA * impulse;
                wB += iB * impulse;
            }

            // Solve Limit
            if (joint.EnableLimit)
            {
                float angle = qB.Angle - qA.Angle - joint.ReferenceAngle;
                float Cdot = wB - wA;
                
                float impulse = 0f;
                if (MathF.Abs(joint.UpperAngle - joint.LowerAngle) < 2f * Constants.AngularSlop)
                {
                    float C = angle - joint.LowerAngle;
                    impulse = -joint.MotorMass * (Cdot + C); 
                }
                else if (angle <= joint.LowerAngle)
                {
                    float C = angle - joint.LowerAngle;
                    float CdotMin = Cdot + MathF.Max(C, 0f);
                    impulse = -joint.MotorMass * CdotMin;
                    float oldImpulse = joint.LimitImpulse;
                    joint.LimitImpulse = MathF.Max(joint.LimitImpulse + impulse, 0f);
                    impulse = joint.LimitImpulse - oldImpulse;
                }
                else if (angle >= joint.UpperAngle)
                {
                    float C = angle - joint.UpperAngle;
                    float CdotMax = Cdot + MathF.Min(C, 0f);
                    impulse = -joint.MotorMass * CdotMax;
                    float oldImpulse = joint.LimitImpulse;
                    joint.LimitImpulse = MathF.Min(joint.LimitImpulse + impulse, 0f);
                    impulse = joint.LimitImpulse - oldImpulse;
                }

                wA -= iA * impulse;
                wB += iB * impulse;
            }

            // Solve Point-to-Point with optional soft-spring bias. Rigid path
            // (LinearSpring.IsZero) is mathematically identical to the legacy
            // -Solve22(K, Cdot) formulation: massScale=1, impulseScale=0,
            // bias=0, so impulse = -1 * Solve22(K, Cdot + 0) - 0 = -Solve22(K, Cdot).
            {
                Vec2 vpA = vA + Vec2.Cross(wA, rA);
                Vec2 vpB = vB + Vec2.Cross(wB, rB);
                Vec2 Cdot = vpB - vpA;

                Vec2 linearBias = Vec2.Zero;
                float linearMassScale = 1f;
                float linearImpulseScale = 0f;
                if (!joint.LinearSpring.IsZero)
                {
                    Vec2 currentDelta = (_bodyPositions[indexB] + rB) - (_bodyPositions[indexA] + rA);
                    Vec2 C = currentDelta - joint.DeltaCenter;
                    linearBias = joint.LinearSpring.BiasRate * C;
                    linearMassScale = joint.LinearSpring.MassScale;
                    linearImpulseScale = joint.LinearSpring.ImpulseScale;
                }
                Vec2 b = Solve22(joint.Mass, Cdot + linearBias);
                Vec2 impulse = new Vec2(
                    -linearMassScale * b.X - linearImpulseScale * joint.Impulse.X,
                    -linearMassScale * b.Y - linearImpulseScale * joint.Impulse.Y);
                joint.Impulse += impulse;

                vA -= mA * impulse;
                wA -= iA * Vec2.Cross(rA, impulse);
                vB += mB * impulse;
                wB += iB * Vec2.Cross(rB, impulse);
            }

            _bodyAngularVelocities[indexA] = wA;
            _bodyAngularVelocities[indexB] = wB;
        }
        


        internal void SolveRevoluteJointPositionConstraints(int index)
        {
            ref RevoluteJointData joint = ref _revoluteJointsData[index];
            int indexA = joint.BodyA;
            int indexB = joint.BodyB;
            
            ref Vec2 cA = ref _bodyPositions[indexA];
            float aA = _bodyRotations[indexA].Angle;
            ref Vec2 cB = ref _bodyPositions[indexB];
            float aB = _bodyRotations[indexB].Angle;

            Rot qA = new Rot(aA);
            Rot qB = new Rot(aB);

            float mA = _bodyInverseMasses[indexA];
            float mB = _bodyInverseMasses[indexB];
            float iA = _bodyInverseInertias[indexA];
            float iB = _bodyInverseInertias[indexB];
            
            // Solve angular limit
            if (joint.EnableLimit)
            {
                float angle = aB - aA - joint.ReferenceAngle;
                float limitImpulse = 0f;

                if (MathF.Abs(joint.UpperAngle - joint.LowerAngle) < 2.0f * Constants.AngularSlop)
                {
                    float C = Math.Clamp(angle - joint.LowerAngle, -Constants.MaxAngularCorrection, Constants.MaxAngularCorrection);
                    limitImpulse = -joint.MotorMass * C;
                }
                else if (angle <= joint.LowerAngle)
                {
                    float C = Math.Clamp(angle - joint.LowerAngle + Constants.AngularSlop, -Constants.MaxAngularCorrection, 0f);
                    limitImpulse = -joint.MotorMass * C;
                }
                else if (angle >= joint.UpperAngle)
                {
                    float C = Math.Clamp(angle - joint.UpperAngle - Constants.AngularSlop, 0f, Constants.MaxAngularCorrection);
                    limitImpulse = -joint.MotorMass * C;
                }

                aA -= iA * limitImpulse;
                aB += iB * limitImpulse;
                qA = new Rot(aA);
                qB = new Rot(aB);
            }

            // Solve point constraint. Skipped when the linear spring is active —
            // bias already folded the position correction into the velocity solve.
            if (joint.LinearSpring.IsZero)
            {
                Vec2 rA = Rot.Mul(qA, joint.LocalAnchorA - _bodyLocalCenters[indexA]);
                Vec2 rB = Rot.Mul(qB, joint.LocalAnchorB - _bodyLocalCenters[indexB]);

                Vec2 C = (cB + rB) - (cA + rA);

                float k11 = mA + mB + iA * rA.Y * rA.Y + iB * rB.Y * rB.Y;
                float k12 = -iA * rA.X * rA.Y - iB * rB.X * rB.Y;
                float k22 = mA + mB + iA * rA.X * rA.X + iB * rB.X * rB.X;
                Mat22 K = new Mat22(new Vec2(k11, k12), new Vec2(k12, k22));

                Vec2 impulse = Solve22(K, -C);

                cA -= mA * impulse;
                aA -= iA * Vec2.Cross(rA, impulse);
                cB += mB * impulse;
                aB += iB * Vec2.Cross(rB, impulse);
            }
            
            _bodyRotations[indexA] = new Rot(aA);
            _bodyRotations[indexB] = new Rot(aB);
        }
    }
}

