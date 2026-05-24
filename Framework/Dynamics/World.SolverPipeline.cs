namespace Box2DNG
{
    public sealed partial class World
    {
        private sealed class SolverPipeline
        {
            private readonly World _world;
            private readonly System.Collections.Generic.List<Contact> _awakeContactsScratch = new System.Collections.Generic.List<Contact>();

            public SolverPipeline(World world)
            {
                _world = world;
            }

            public void Step(float timeStep)
            {
                if (timeStep <= 0f)
                {
                    return;
                }

                _world.ResetSweeps();

                float dtRatio = _world._prevTimeStep > 0f ? timeStep / _world._prevTimeStep : 1f;
                _world._prevTimeStep = timeStep;
                if (_world._def.EnableContactHertzClamp)
                {
                    float invH = 1f / timeStep;
                    _world._stepContactHertz = MathF.Min(_world._def.ContactHertz, 0.125f * invH);
                }
                else
                {
                    _world._stepContactHertz = 0f;
                }
                _world.UpdateContacts(includeSensors: false);
                if (_world._islandsDirty || _world._lastIslands.Count == 0)
                {
                    _world.BuildIslands(awakeOnly: _world._def.EnableSleep);
                }
                _world.BuildConstraintGraph();

                // Phase 2 of TIER4_PARITY_PLAN: split the outer timestep into N
                // mini-steps. Each sub-step uses h = timeStep / N. cpp box2d v3
                // exposes the same control via `b2World_Step(..., subStepCount)`.
                //
                // Pipeline order preserved from pre-Phase-2 (so N=1 is
                // byte-identical):
                //   integrate velocities (full dt)            — ONCE
                //   prepare contacts + joints + warm-start    — ONCE (h)
                //   for sub in 1..N:
                //     iterate velocity solve
                //     (on final sub-step only) restitution + store impulses
                //     integrate positions (h)
                //     iterate position solve (rigid axes only)
                //
                // Restitution + StoreImpulses run on the LAST sub-step before
                // its position integration so velocities reflect the bounce
                // before bodies advance — matches the legacy single-step order.
                // For multi-sub-step the intermediate sub-steps use the warm-
                // started impulses without re-applying restitution; the final
                // sub-step closes out impulse accounting.
                int subStepCount = Math.Max(1, _world._def.SubStepCount);
                float h = timeStep / subStepCount;
                IntegrateVelocities(timeStep);
                PrepareConstraints(h, dtRatio);
                for (int sub = 0; sub < subStepCount; ++sub)
                {
                    SolveVelocityConstraints(h);
                    if (sub == subStepCount - 1)
                    {
                        FinalizeContactSolver();
                    }
                    IntegratePositions(h);
                    SolvePositionConstraints(h);
                }
                _world.RaiseContactImpulseEvents();
                _world.SyncSweeps();
                FinalizeStep(timeStep);
                _world.SplitAwakeIslandsIfNeeded();
            }

            private void IntegrateVelocities(float timeStep)
            {
                for (int i = 0; i < _world._awakeSet.Islands.Count; ++i)
                {
                    Island island = _world._awakeSet.Islands[i];
                    if (!island.IsAwake)
                    {
                        continue;
                    }
                    for (int j = 0; j < island.Bodies.Count; ++j)
                    {
                        Body body = island.Bodies[j];
                        if (body.Type == BodyType.Static)
                        {
                            continue;
                        }

                        if (_world._def.EnableSleep && body.AllowSleep == false)
                        {
                            body.SetAwake(true);
                        }

                        if (_world._def.EnableSleep && body.Awake == false)
                        {
                            continue;
                        }

                        if (body.Type == BodyType.Dynamic)
                        {
                            Vec2 accel = body.GravityScale * _world.Gravity;
                            if (body.InverseMass > 0f)
                            {
                                accel += body.InverseMass * body.Force;
                            }
                            body.LinearVelocity += timeStep * accel;
                            if (body.InverseInertia > 0f)
                            {
                                body.AngularVelocity += timeStep * body.InverseInertia * body.Torque;
                            }
                            body.LinearVelocity = ApplyLinearDamping(body.LinearVelocity, body.LinearDamping, timeStep);
                            body.AngularVelocity = ApplyAngularDamping(body.AngularVelocity, body.AngularDamping, timeStep);

                            if ((body.MotionLocks & MotionLocks.LinearX) != 0)
                            {
                                body.LinearVelocity = new Vec2(0f, body.LinearVelocity.Y);
                            }
                            if ((body.MotionLocks & MotionLocks.LinearY) != 0)
                            {
                                body.LinearVelocity = new Vec2(body.LinearVelocity.X, 0f);
                            }
                            if ((body.MotionLocks & MotionLocks.AngularZ) != 0)
                            {
                                body.AngularVelocity = 0f;
                            }
                        }

                        body.LinearVelocity = ClampLinearSpeed(body.LinearVelocity, _world._def.MaximumLinearSpeed);
                        body.AngularVelocity = ClampAngularSpeed(body.AngularVelocity, _world._def.MaximumAngularSpeed);
                        body.ClearForces();
                    }
                }
            }

            /// <summary>
            /// Prepare contacts + joints + warm-start. Runs ONCE per outer step
            /// before the sub-step loop. Re-running warm-start in each sub-step
            /// would re-apply the accumulated impulse N times, blowing up the
            /// solver.
            /// </summary>
            private bool _useSimd;
            private void PrepareConstraints(float h, float dtRatio)
            {
                _useSimd = _world._def.EnableContactSolverSimd && System.Numerics.Vector.IsHardwareAccelerated;
                if (_useSimd)
                {
                    _world._contactSolverSimd.Prepare(h, dtRatio, _world._constraintGraph);
                    _world._contactSolverSimd.WarmStart();
                }
                else
                {
                    _world._contactSolver.Prepare(h, dtRatio, _world._constraintGraph);
                    _world._contactSolver.WarmStart();
                }

                for (int colorIndex = 0; colorIndex < Constants.GraphColorCount; ++colorIndex)
                {
                    System.Collections.Generic.List<JointHandle> joints = _world._constraintGraph.Colors[colorIndex].Joints;
                    for (int i = 0; i < joints.Count; ++i)
                    {
                        InitJointVelocityConstraints(joints[i], h);
                    }
                }
            }

            /// <summary>
            /// Iterate the velocity constraints for one sub-step. Each call runs
            /// the configured number of velocity iterations over the existing
            /// (pre-prepared) constraint state.
            /// </summary>
            private void SolveVelocityConstraints(float h)
            {
                for (int iter = 0; iter < _world._def.VelocityIterations; ++iter)
                {
                    if (_useSimd)
                    {
                        _world._contactSolverSimd.SolveVelocity(useBias: true);
                    }
                    else
                    {
                        _world._contactSolver.SolveVelocity(useBias: true);
                    }

                    for (int colorIndex = 0; colorIndex < Constants.GraphColorCount; ++colorIndex)
                    {
                        System.Collections.Generic.List<JointHandle> joints = _world._constraintGraph.Colors[colorIndex].Joints;
                        for (int i = 0; i < joints.Count; ++i)
                        {
                            SolveJointVelocityConstraints(joints[i], h);
                        }
                    }
                }
            }

            /// <summary>
            /// Restitution pass + impulse storage. Runs ONCE per outer step
            /// after the sub-step loop so that contact impulse events reflect
            /// the final post-sub-step state.
            /// </summary>
            private void FinalizeContactSolver()
            {
                World.ContactSolverStats aggregateStats;
                if (_useSimd)
                {
                    _world._contactSolverSimd.ApplyRestitution(_world._def.RestitutionThreshold);
                    _world._contactSolverSimd.StoreImpulses();
                    aggregateStats = _world._contactSolverSimd.GetStats();
                }
                else
                {
                    _world._contactSolver.ApplyRestitution(_world._def.RestitutionThreshold);
                    _world._contactSolver.StoreImpulses();
                    aggregateStats = _world._contactSolver.GetStats();
                }
                _world._lastContactSolverStats = aggregateStats;
            }

            private void IntegratePositions(float timeStep)
            {
                for (int i = 0; i < _world._awakeSet.Islands.Count; ++i)
                {
                    Island island = _world._awakeSet.Islands[i];
                    if (!island.IsAwake)
                    {
                        continue;
                    }
                    for (int j = 0; j < island.Bodies.Count; ++j)
                    {
                        Body body = island.Bodies[j];
                        if (body.Type == BodyType.Static)
                        {
                            continue;
                        }

                        if (_world._def.EnableSleep && body.Awake == false)
                        {
                            continue;
                        }

                        Vec2 oldCenter = body.GetWorldCenter();
                        float oldAngle = body.Transform.Q.Angle;

                        Vec2 translation = timeStep * body.LinearVelocity;
                        float rotation = timeStep * body.AngularVelocity;

                        float maxTranslation = _world._def.MaximumTranslation;
                        float maxRotation = _world._def.MaximumRotation;
                        bool translationClamped = translation.LengthSquared > maxTranslation * maxTranslation;
                        bool rotationClamped = MathF.Abs(rotation) > maxRotation;

                        translation = ClampTranslation(translation, maxTranslation);
                        rotation = ClampRotation(rotation, maxRotation);
                        if (translationClamped && timeStep > 0f)
                        {
                            body.LinearVelocity = translation / timeStep;
                        }
                        if (rotationClamped && timeStep > 0f)
                        {
                            body.AngularVelocity = rotation / timeStep;
                        }

                        Vec2 newCenter = oldCenter + translation;
                        float newAngle = oldAngle + rotation;
                        Rot newRot = new Rot(newAngle);
                        Vec2 newPosition = newCenter - Rot.Mul(newRot, body.LocalCenter);
                        body.SetTransform(newPosition, newRot);

                        body.Sweep = new Sweep(body.LocalCenter, oldCenter, newCenter, oldAngle, newAngle, 0f);
                    }
                }
            }

            private void SolvePositionConstraints(float timeStep)
            {
                for (int iter = 0; iter < _world._def.PositionIterations; ++iter)
                {
                    for (int colorIndex = 0; colorIndex < Constants.GraphColorCount; ++colorIndex)
                    {
                        System.Collections.Generic.List<Contact> contacts = _world._constraintGraph.Colors[colorIndex].Contacts;
                        if (contacts.Count > 0)
                        {
                            _world.SolvePositionConstraints(contacts);
                        }
                    }

                    for (int colorIndex = 0; colorIndex < Constants.GraphColorCount; ++colorIndex)
                    {
                        System.Collections.Generic.List<JointHandle> joints = _world._constraintGraph.Colors[colorIndex].Joints;
                        for (int i = 0; i < joints.Count; ++i)
                        {
                            JointHandle handle = joints[i];
                            SolveJointPositionConstraints(handle);
                        }
                    }
                }
            }

            private void FinalizeStep(float timeStep)
            {
                if (_world._def.EnableSleep)
                {
                    _world.UpdateSleep(timeStep);
                }
                _world.SolveTOI();
                _world.UpdateSensors();
                _world.RaiseBodyEvents();
                _world.RaiseJointEvents(timeStep > 0f ? 1f / timeStep : 0f);
            }

            private static World.ContactSolverStats SumStats(World.ContactSolverStats a, World.ContactSolverStats b)
            {
                return new World.ContactSolverStats(
                    a.SinglePointConstraints + b.SinglePointConstraints,
                    a.TwoPointConstraints + b.TwoPointConstraints,
                    a.ScalarConstraints + b.ScalarConstraints,
                    a.Colors + b.Colors,
                    a.SimdBatches + b.SimdBatches,
                    a.SimdLanes + b.SimdLanes);
            }

            private System.Collections.Generic.List<Contact> FilterAwakeContacts(System.Collections.Generic.IReadOnlyList<Contact> contacts)
            {
                _awakeContactsScratch.Clear();
                for (int i = 0; i < contacts.Count; ++i)
                {
                    Contact contact = contacts[i];
                    if (contact.SolverSetType == SolverSetType.Awake && contact.SolverSetId == 0)
                    {
                        _awakeContactsScratch.Add(contact);
                    }
                }
                return _awakeContactsScratch;
            }

            private void InitJointVelocityConstraints(JointHandle handle, float timeStep)
            {
                if (!_world.TryGetJointIndex(handle, out int index))
                {
                    return;
                }

                switch (handle.Type)
                {
                    case JointType.Distance:
                        _world.InitDistanceJointVelocityConstraints(index, timeStep);
                        break;
                    case JointType.Revolute:
                        _world.InitRevoluteJointVelocityConstraints(index, timeStep);
                        break;
                    case JointType.Prismatic:
                        _world.InitPrismaticJointVelocityConstraints(index, timeStep);
                        break;
                    case JointType.Wheel:
                        _world.InitWheelJointVelocityConstraints(index, timeStep);
                        break;
                    case JointType.Pulley:
                        _world.InitPulleyJointVelocityConstraints(index, timeStep);
                        break;
                    case JointType.Weld:
                        _world.InitWeldJointVelocityConstraints(index, timeStep);
                        break;
                    case JointType.Motor:
                        _world.InitMotorJointVelocityConstraints(index, timeStep);
                        break;
                    case JointType.Gear:
                        _world.InitGearJointVelocityConstraints(index, timeStep);
                        break;
                    case JointType.Rope:
                        _world.InitRopeJointVelocityConstraints(index, timeStep);
                        break;
                    case JointType.Friction:
                        _world.InitFrictionJointVelocityConstraints(index, timeStep);
                        break;
                }
            }

            private void SolveJointVelocityConstraints(JointHandle handle, float timeStep)
            {
                if (!_world.TryGetJointIndex(handle, out int index))
                {
                    return;
                }

                switch (handle.Type)
                {
                    case JointType.Distance:
                        _world.SolveDistanceJointVelocityConstraints(index);
                        break;
                    case JointType.Revolute:
                        _world.SolveRevoluteJointVelocityConstraints(index, timeStep);
                        break;
                    case JointType.Prismatic:
                        _world.SolvePrismaticJointVelocityConstraints(index, timeStep);
                        break;
                    case JointType.Wheel:
                        _world.SolveWheelJointVelocityConstraints(index, timeStep);
                        break;
                    case JointType.Pulley:
                        _world.SolvePulleyJointVelocityConstraints(index);
                        break;
                    case JointType.Weld:
                        _world.SolveWeldJointVelocityConstraints(index, timeStep);
                        break;
                    case JointType.Motor:
                        _world.SolveMotorJointVelocityConstraints(index, timeStep);
                        break;
                    case JointType.Gear:
                        _world.SolveGearJointVelocityConstraints(index, timeStep);
                        break;
                    case JointType.Rope:
                        _world.SolveRopeJointVelocityConstraints(index, timeStep);
                        break;
                    case JointType.Friction:
                        _world.SolveFrictionJointVelocityConstraints(index, timeStep);
                        break;
                }
            }

            private void SolveJointPositionConstraints(JointHandle handle)
            {
                if (!_world.TryGetJointIndex(handle, out int index))
                {
                    return;
                }

                switch (handle.Type)
                {
                    case JointType.Distance:
                        _world.SolveDistanceJointPositionConstraints(index);
                        break;
                    case JointType.Revolute:
                        _world.SolveRevoluteJointPositionConstraints(index);
                        break;
                    case JointType.Prismatic:
                        _world.SolvePrismaticJointPositionConstraints(index);
                        break;
                    case JointType.Wheel:
                        _world.SolveWheelJointPositionConstraints(index);
                        break;
                    case JointType.Pulley:
                        _world.SolvePulleyJointPositionConstraints(index);
                        break;
                    case JointType.Weld:
                        _world.SolveWeldJointPositionConstraints(index);
                        break;
                    case JointType.Motor:
                        _world.SolveMotorJointPositionConstraints(index);
                        break;
                    case JointType.Gear:
                        _world.SolveGearJointPositionConstraints(index);
                        break;
                    case JointType.Rope:
                        _world.SolveRopeJointPositionConstraints(index);
                        break;
                    case JointType.Friction:
                        _world.SolveFrictionJointPositionConstraints(index);
                        break;
                }
            }
        }
    }
}
