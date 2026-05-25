using System;

namespace Box2DNG
{
    public sealed partial class World : System.IDisposable
    {
        private readonly WorldDef _def;
        private readonly WorldEvents _events = new WorldEvents();
        private readonly BroadPhase<Fixture> _broadPhase = new BroadPhase<Fixture>();
        private readonly System.Collections.Generic.List<Body> _bodies = new System.Collections.Generic.List<Body>();
        private readonly System.Collections.Generic.List<Contact> _contacts = new System.Collections.Generic.List<Contact>();
        private readonly System.Collections.Generic.Dictionary<ContactKey, Contact> _contactMap = new System.Collections.Generic.Dictionary<ContactKey, Contact>();
        private readonly HashSet64 _sensorPairs = new HashSet64(32);
        private readonly System.Collections.Generic.Dictionary<ContactKey, (object? A, object? B)> _sensorUserData = new System.Collections.Generic.Dictionary<ContactKey, (object? A, object? B)>();
        private readonly System.Collections.Generic.Dictionary<int, System.Collections.Generic.HashSet<Fixture>> _sensorOverlaps =
            new System.Collections.Generic.Dictionary<int, System.Collections.Generic.HashSet<Fixture>>();
        internal DistanceJoint[] _distanceJoints = new DistanceJoint[128];
        internal DistanceJointData[] _distanceJointsData = new DistanceJointData[128];
        internal int _distanceJointCount = 0;
        
        internal RevoluteJoint[] _revoluteJoints = new RevoluteJoint[128];
        internal RevoluteJointData[] _revoluteJointsData = new RevoluteJointData[128];
        internal int _revoluteJointCount = 0;
        
        internal PrismaticJoint[] _prismaticJoints = new PrismaticJoint[128];
        internal PrismaticJointData[] _prismaticJointsData = new PrismaticJointData[128];
        internal int _prismaticJointCount = 0;
        private WheelJoint[] _wheelJoints = new WheelJoint[128];
        internal int _wheelJointCount = 0;
        internal WheelJointData[] _wheelJointsData = new WheelJointData[128];
        private PulleyJoint[] _pulleyJoints = new PulleyJoint[128];
        internal int _pulleyJointCount = 0;
        internal PulleyJointData[] _pulleyJointsData = new PulleyJointData[128];
        private WeldJoint[] _weldJoints = new WeldJoint[128];
        internal int _weldJointCount = 0;
        internal WeldJointData[] _weldJointsData = new WeldJointData[128];
        private MotorJoint[] _motorJoints = new MotorJoint[128];
        internal int _motorJointCount = 0;
        internal MotorJointData[] _motorJointsData = new MotorJointData[128];
        private GearJoint[] _gearJoints = new GearJoint[128];
        internal int _gearJointCount = 0;
        internal GearJointData[] _gearJointsData = new GearJointData[128];
        private RopeJoint[] _ropeJoints = new RopeJoint[128];
        internal int _ropeJointCount = 0;
        internal RopeJointData[] _ropeJointsData = new RopeJointData[128];
        private FrictionJoint[] _frictionJoints = new FrictionJoint[128];
        internal int _frictionJointCount = 0;
        internal FrictionJointData[] _frictionJointsData = new FrictionJointData[128];
        private readonly System.Collections.Generic.Dictionary<int, int> _distanceJointIndexById = new System.Collections.Generic.Dictionary<int, int>();
        private readonly System.Collections.Generic.Dictionary<int, int> _revoluteJointIndexById = new System.Collections.Generic.Dictionary<int, int>();
        private readonly System.Collections.Generic.Dictionary<int, int> _prismaticJointIndexById = new System.Collections.Generic.Dictionary<int, int>();
        private readonly System.Collections.Generic.Dictionary<int, int> _wheelJointIndexById = new System.Collections.Generic.Dictionary<int, int>();
        private readonly System.Collections.Generic.Dictionary<int, int> _pulleyJointIndexById = new System.Collections.Generic.Dictionary<int, int>();
        private readonly System.Collections.Generic.Dictionary<int, int> _weldJointIndexById = new System.Collections.Generic.Dictionary<int, int>();
        private readonly System.Collections.Generic.Dictionary<int, int> _motorJointIndexById = new System.Collections.Generic.Dictionary<int, int>();
        private readonly System.Collections.Generic.Dictionary<int, int> _gearJointIndexById = new System.Collections.Generic.Dictionary<int, int>();
        private readonly System.Collections.Generic.Dictionary<int, int> _ropeJointIndexById = new System.Collections.Generic.Dictionary<int, int>();
        private readonly System.Collections.Generic.Dictionary<int, int> _frictionJointIndexById = new System.Collections.Generic.Dictionary<int, int>();
        private float _prevTimeStep;
        private readonly SolverPipeline _solverPipeline;
        private readonly ContactSolver _contactSolver;
        private readonly ContactSolverSimd _contactSolverSimd;
        private ContactSolverStats _lastContactSolverStats = new ContactSolverStats();
        private bool _islandsDirty = true;
        private readonly System.Collections.Generic.HashSet<int> _dirtyIslandIds = new System.Collections.Generic.HashSet<int>();
        private readonly IdPool _islandIdPool = new IdPool();
        private readonly IdPool _bodyIdPool = new IdPool();
        private readonly IdPool _fixtureIdPool = new IdPool();
        private readonly IdPool _jointIdPool = new IdPool();
        private readonly IdPool _contactIdPool = new IdPool();
        private readonly IdPool _solverSetIdPool = new IdPool();
        private readonly IdPool _contactEdgeIdPool = new IdPool();
        private readonly ConstraintGraph _constraintGraph = new ConstraintGraph(8);
        private readonly ArenaAllocator _arena;
        private bool _disposed;
        private WorldId _worldId;
        private int[] _jointColorIndicesById = System.Array.Empty<int>();
        private int[] _jointLocalIndicesById = System.Array.Empty<int>();
        private bool[] _jointConstraintGraphUsedById = System.Array.Empty<bool>();

        // DoD Storage
        internal int _bodyCapacity;
        internal int _bodyCount; // Actual count of simulated bodies
        internal int _distanceJointCapacity;
        internal int _revoluteJointCapacity; // Added
        internal int _prismaticJointCapacity; // Added
        internal int _wheelJointCapacity;
        internal int _pulleyJointCapacity;
        internal int _weldJointCapacity;
        internal int _motorJointCapacity;
        internal int _gearJointCapacity;
        internal int _ropeJointCapacity;
        internal int _frictionJointCapacity;
        internal Vec2[] _bodyPositions;
        internal Vec2[] _bodyLocalCenters;
        internal Rot[] _bodyRotations;
        // Phase 2.5 of TIER4_PARITY_PLAN: per-body within-step movement,
        // matching cpp box2d v3's `b2BodyState.deltaPosition` /
        // `deltaRotation`. Reset to (0, identity) at the start of every
        // outer Step. Each IntegratePositions sub-step adds its
        // translation/rotation into these arrays. Joint + contact solvers
        // read `body.Position + _bodyDeltaPositions[id]` (and the composed
        // rotation) so iterations see the predicted post-integration
        // anchor offset without bodies' transforms actually advancing.
        // Stage A (this commit): arrays accumulate but no consumer reads
        // them yet — body.Transform still advances inside IntegratePositions
        // as before. Stage B+ switches consumers over and stops advancing
        // body.Transform until the end of the outer step.
        internal Vec2[] _bodyDeltaPositions;
        internal Rot[] _bodyDeltaRotations;
        // Phase 2.5 Stage A.2: snapshot of the body's position / rotation at
        // the top of each outer Step. `ResetSweeps` writes them; the rest of
        // the pipeline never mutates them within a Step. Together with the
        // delta arrays they let the consumer migration in Stages B–K compute
        //   stepStart   = _bodyStepStartPositions[id]
        //   delta       = _bodyDeltaPositions[id]   // = newPos - stepStart
        //   current     = _bodyPositions[id]        // advances each sub-step today
        // without needing every consumer to switch storage semantics at once.
        // While `IntegratePositions` still calls `body.SetTransform` (Stage A
        // dual-track), `_bodyPositions[id]` advances and equals
        // `_bodyStepStartPositions[id] + _bodyDeltaPositions[id]`. When the
        // full migration lands `body.SetTransform` is removed and the
        // identity above is the *only* way to recover the current pose.
        internal Vec2[] _bodyStepStartPositions;
        internal Rot[] _bodyStepStartRotations;
        internal Vec2[] _bodyLinearVelocities;
        internal float[] _bodyAngularVelocities;
        internal float[] _bodyLinearDampings;
        internal float[] _bodyAngularDampings;
        internal float[] _bodyGravityScales;
        internal BodyType[] _bodyTypes;
        internal float[] _bodyMasses;
        internal float[] _bodyInertias;
        internal float[] _bodyInverseMasses;
        internal float[] _bodyInverseInertias;
        internal bool[] _bodyFixedRotations;
        internal bool[] _bodyAwakes;
        internal bool[] _bodyAwakeAtStepStart;
        internal bool[] _bodyAllowSleeps;
        internal Vec2[] _bodyForces;
        internal float[] _bodyTorques;
        internal float[] _bodySleepTimes;
        internal SolverSetType[] _bodySolverSetTypes;
        internal int[] _bodyPoolIds;
        internal int[] _bodyIslandIdsByBody;

        private void EnsureBodyCapacity(int capacity)
        {
            if (capacity <= _bodyCapacity)
            {
                return;
            }

            int newCapacity = Math.Max(_bodyCapacity * 2, capacity);
            Array.Resize(ref _bodyPositions, newCapacity);
            Array.Resize(ref _bodyLocalCenters, newCapacity);
            Array.Resize(ref _bodyRotations, newCapacity);
            int oldCapacityForDeltas = _bodyDeltaRotations?.Length ?? 0;
            Array.Resize(ref _bodyDeltaPositions, newCapacity);
            Array.Resize(ref _bodyDeltaRotations, newCapacity);
            Array.Resize(ref _bodyStepStartPositions, newCapacity);
            Array.Resize(ref _bodyStepStartRotations, newCapacity);
            // New Rot slots default to (S=0, C=0) which is NOT identity;
            // fill them so a body inserted at a fresh slot has a sane
            // (0, identity) delta and step-start.
            if (newCapacity > oldCapacityForDeltas)
            {
                Array.Fill(_bodyDeltaRotations, Rot.Identity, oldCapacityForDeltas, newCapacity - oldCapacityForDeltas);
                Array.Fill(_bodyStepStartRotations, Rot.Identity, oldCapacityForDeltas, newCapacity - oldCapacityForDeltas);
            }
            Array.Resize(ref _bodyLinearVelocities, newCapacity);
            Array.Resize(ref _bodyAngularVelocities, newCapacity);
            Array.Resize(ref _bodyLinearDampings, newCapacity);
            Array.Resize(ref _bodyAngularDampings, newCapacity);
            Array.Resize(ref _bodyGravityScales, newCapacity);
            Array.Resize(ref _bodyTypes, newCapacity);
            Array.Resize(ref _bodyMasses, newCapacity);
            Array.Resize(ref _bodyInertias, newCapacity);
            Array.Resize(ref _bodyInverseMasses, newCapacity);
            Array.Resize(ref _bodyInverseInertias, newCapacity);
            Array.Resize(ref _bodyFixedRotations, newCapacity);
            Array.Resize(ref _bodyAwakes, newCapacity);
            Array.Resize(ref _bodyAwakeAtStepStart, newCapacity);
            Array.Resize(ref _bodyAllowSleeps, newCapacity);
            Array.Resize(ref _bodyForces, newCapacity);
            Array.Resize(ref _bodyTorques, newCapacity);
            Array.Resize(ref _bodySleepTimes, newCapacity);
            Array.Resize(ref _bodySolverSetTypes, newCapacity);
            Array.Resize(ref _bodyPoolIds, newCapacity);
            int previousCapacity = _bodyCapacity;
            Array.Resize(ref _bodyIslandIdsByBody, newCapacity);
            Array.Fill(_bodyIslandIdsByBody, -1, previousCapacity, newCapacity - previousCapacity);
            Array.Resize(ref _bodyJoints, newCapacity);
            _bodyCapacity = newCapacity;
        }

        public WorldDef Def => _def;

        public World(WorldDef def)
        {
            _def = def ?? throw new ArgumentNullException(nameof(def));
            _solverPipeline = new SolverPipeline(this);
            _contactSolver = new ContactSolver(this);
            _contactSolverSimd = new ContactSolverSimd(this);
            _arena = new ArenaAllocator(_def.ArenaCapacity);
            _worldId = default;

            _bodyCapacity = 1024;
            _bodyPositions = new Vec2[_bodyCapacity];
            _bodyRotations = new Rot[_bodyCapacity];
            _bodyDeltaPositions = new Vec2[_bodyCapacity];
            _bodyDeltaRotations = new Rot[_bodyCapacity];
            _bodyStepStartPositions = new Vec2[_bodyCapacity];
            _bodyStepStartRotations = new Rot[_bodyCapacity];
            Array.Fill(_bodyDeltaRotations, Rot.Identity);
            Array.Fill(_bodyStepStartRotations, Rot.Identity);
            _bodyLocalCenters = new Vec2[_bodyCapacity]; // Added
            _bodyLinearVelocities = new Vec2[_bodyCapacity];
            _bodyAngularVelocities = new float[_bodyCapacity];
            _bodyLinearDampings = new float[_bodyCapacity];
            _bodyAngularDampings = new float[_bodyCapacity];
            _bodyGravityScales = new float[_bodyCapacity];
            _bodyTypes = new BodyType[_bodyCapacity];
            _bodyMasses = new float[_bodyCapacity];
            _bodyInertias = new float[_bodyCapacity];
            _bodyInverseMasses = new float[_bodyCapacity];
            _bodyInverseInertias = new float[_bodyCapacity];
            _bodyFixedRotations = new bool[_bodyCapacity];
            _bodyAwakes = new bool[_bodyCapacity];
            _bodyAwakeAtStepStart = new bool[_bodyCapacity];
            _bodyAllowSleeps = new bool[_bodyCapacity];
            _bodyForces = new Vec2[_bodyCapacity];
            _bodyTorques = new float[_bodyCapacity];
            _bodySleepTimes = new float[_bodyCapacity];
            _bodySolverSetTypes = new SolverSetType[_bodyCapacity];
            _bodyPoolIds = new int[_bodyCapacity];
            _bodyIslandIdsByBody = new int[_bodyCapacity];
            Array.Fill(_bodyIslandIdsByBody, -1);
            _bodyJoints = new System.Collections.Generic.List<JointHandle>[_bodyCapacity];
            
            _distanceJointCapacity = 256;
            _distanceJoints = new DistanceJoint[_distanceJointCapacity];
            _distanceJointsData = new DistanceJointData[_distanceJointCapacity];
            
            _revoluteJointCapacity = 256;
            _revoluteJoints = new RevoluteJoint[_revoluteJointCapacity];
            _revoluteJointsData = new RevoluteJointData[_revoluteJointCapacity];
            
            _prismaticJointCapacity = 256;
            _prismaticJoints = new PrismaticJoint[_prismaticJointCapacity];
            _prismaticJointsData = new PrismaticJointData[_prismaticJointCapacity];

            _wheelJointCapacity = 256;
            _wheelJoints = new WheelJoint[_wheelJointCapacity];
            _wheelJointsData = new WheelJointData[_wheelJointCapacity];

            _pulleyJointCapacity = 256;
            _pulleyJoints = new PulleyJoint[_pulleyJointCapacity];
            _pulleyJointsData = new PulleyJointData[_pulleyJointCapacity];

            _weldJointCapacity = 256;
            _weldJoints = new WeldJoint[_weldJointCapacity];
            _weldJointsData = new WeldJointData[_weldJointCapacity];

            _motorJointCapacity = 256;
            _motorJoints = new MotorJoint[_motorJointCapacity];
            _motorJointsData = new MotorJointData[_motorJointCapacity];

            _gearJointCapacity = 256;
            _gearJoints = new GearJoint[_gearJointCapacity];
            _gearJointsData = new GearJointData[_gearJointCapacity];

            _ropeJointCapacity = 256;
            _ropeJoints = new RopeJoint[_ropeJointCapacity];
            _ropeJointsData = new RopeJointData[_ropeJointCapacity];

            _frictionJointCapacity = 256;
            _frictionJoints = new FrictionJoint[_frictionJointCapacity];
            _frictionJointsData = new FrictionJointData[_frictionJointCapacity];

            int jointMetaCapacity = Math.Max(256, _jointIdPool.Capacity);
            _jointSolverSetTypesById = new SolverSetType[jointMetaCapacity];
            _jointSolverSetIdsById = new int[jointMetaCapacity];
            _jointSolverSetUsedById = new bool[jointMetaCapacity];
            _jointColorIndicesById = new int[jointMetaCapacity];
            _jointLocalIndicesById = new int[jointMetaCapacity];
            _jointConstraintGraphUsedById = new bool[jointMetaCapacity];
            Array.Fill(_jointColorIndicesById, -1);
            Array.Fill(_jointLocalIndicesById, -1);
        }

        public readonly struct ContactSolverStats
        {
            public readonly int SinglePointConstraints;
            public readonly int TwoPointConstraints;
            public readonly int ScalarConstraints;
            public readonly int Colors;
            public readonly int SimdBatches;
            public readonly int SimdLanes;

            public ContactSolverStats(
                int singlePointConstraints,
                int twoPointConstraints,
                int scalarConstraints,
                int colors,
                int simdBatches,
                int simdLanes)
            {
                SinglePointConstraints = singlePointConstraints;
                TwoPointConstraints = twoPointConstraints;
                ScalarConstraints = scalarConstraints;
                Colors = colors;
                SimdBatches = simdBatches;
                SimdLanes = simdLanes;
            }
        }

        public readonly struct AllocationStats
        {
            public readonly int BodyCount;
            public readonly int FixtureCount;
            public readonly int JointCount;
            public readonly int ContactCount;
            public readonly int IslandCount;
            public readonly int SleepingSetCount;
            public readonly int BodyIdCapacity;
            public readonly int BodyIdCount;
            public readonly int FixtureIdCapacity;
            public readonly int FixtureIdCount;
            public readonly int JointIdCapacity;
            public readonly int JointIdCount;
            public readonly int ContactIdCapacity;
            public readonly int ContactIdCount;
            public readonly int IslandIdCapacity;
            public readonly int IslandIdCount;
            public readonly int SolverSetIdCapacity;
            public readonly int SolverSetIdCount;
            public readonly int ContactEdgeIdCapacity;
            public readonly int ContactEdgeIdCount;
            public readonly int SensorPairCapacity;
            public readonly int SensorPairCount;
            public readonly int BodyIdPoolBytes;
            public readonly int FixtureIdPoolBytes;
            public readonly int JointIdPoolBytes;
            public readonly int ContactIdPoolBytes;
            public readonly int IslandIdPoolBytes;
            public readonly int SolverSetIdPoolBytes;
            public readonly int ContactEdgeIdPoolBytes;
            public readonly int SensorPairBytes;
            public readonly int AllocatorBytes;

            public AllocationStats(
                int bodyCount,
                int fixtureCount,
                int jointCount,
                int contactCount,
                int islandCount,
                int sleepingSetCount,
                int bodyIdCapacity,
                int bodyIdCount,
                int fixtureIdCapacity,
                int fixtureIdCount,
                int jointIdCapacity,
                int jointIdCount,
                int contactIdCapacity,
                int contactIdCount,
                int islandIdCapacity,
                int islandIdCount,
                int solverSetIdCapacity,
                int solverSetIdCount,
                int contactEdgeIdCapacity,
                int contactEdgeIdCount,
                int sensorPairCapacity,
                int sensorPairCount,
                int bodyIdPoolBytes,
                int fixtureIdPoolBytes,
                int jointIdPoolBytes,
                int contactIdPoolBytes,
                int islandIdPoolBytes,
                int solverSetIdPoolBytes,
                int contactEdgeIdPoolBytes,
                int sensorPairBytes,
                int allocatorBytes)
            {
                BodyCount = bodyCount;
                FixtureCount = fixtureCount;
                JointCount = jointCount;
                ContactCount = contactCount;
                IslandCount = islandCount;
                SleepingSetCount = sleepingSetCount;
                BodyIdCapacity = bodyIdCapacity;
                BodyIdCount = bodyIdCount;
                FixtureIdCapacity = fixtureIdCapacity;
                FixtureIdCount = fixtureIdCount;
                JointIdCapacity = jointIdCapacity;
                JointIdCount = jointIdCount;
                ContactIdCapacity = contactIdCapacity;
                ContactIdCount = contactIdCount;
                IslandIdCapacity = islandIdCapacity;
                IslandIdCount = islandIdCount;
                SolverSetIdCapacity = solverSetIdCapacity;
                SolverSetIdCount = solverSetIdCount;
                ContactEdgeIdCapacity = contactEdgeIdCapacity;
                ContactEdgeIdCount = contactEdgeIdCount;
                SensorPairCapacity = sensorPairCapacity;
                SensorPairCount = sensorPairCount;
                BodyIdPoolBytes = bodyIdPoolBytes;
                FixtureIdPoolBytes = fixtureIdPoolBytes;
                JointIdPoolBytes = jointIdPoolBytes;
                ContactIdPoolBytes = contactIdPoolBytes;
                IslandIdPoolBytes = islandIdPoolBytes;
                SolverSetIdPoolBytes = solverSetIdPoolBytes;
                ContactEdgeIdPoolBytes = contactEdgeIdPoolBytes;
                SensorPairBytes = sensorPairBytes;
                AllocatorBytes = allocatorBytes;
            }
        }

        public Vec2 Gravity => _def.Gravity;

        public WorldEvents Events => _events;

        public System.Collections.Generic.IReadOnlyList<Body> Bodies => _bodies;

        public System.Collections.Generic.IReadOnlyList<Contact> Contacts => _contacts;
        public System.Collections.Generic.IReadOnlyList<DistanceJoint> DistanceJoints => new System.ArraySegment<DistanceJoint>(_distanceJoints, 0, _distanceJointCount);
        public System.Collections.Generic.IReadOnlyList<RevoluteJoint> RevoluteJoints => new System.ArraySegment<RevoluteJoint>(_revoluteJoints, 0, _revoluteJointCount);
        public System.Collections.Generic.IReadOnlyList<PrismaticJoint> PrismaticJoints => new System.ArraySegment<PrismaticJoint>(_prismaticJoints, 0, _prismaticJointCount);
        public System.Collections.Generic.IReadOnlyList<WheelJoint> WheelJoints => new System.ArraySegment<WheelJoint>(_wheelJoints, 0, _wheelJointCount);
        public System.Collections.Generic.IReadOnlyList<PulleyJoint> PulleyJoints => new System.ArraySegment<PulleyJoint>(_pulleyJoints, 0, _pulleyJointCount);
        public System.Collections.Generic.IReadOnlyList<WeldJoint> WeldJoints => new System.ArraySegment<WeldJoint>(_weldJoints, 0, _weldJointCount);
        public System.Collections.Generic.IReadOnlyList<MotorJoint> MotorJoints => new System.ArraySegment<MotorJoint>(_motorJoints, 0, _motorJointCount);
        public System.Collections.Generic.IReadOnlyList<GearJoint> GearJoints => new System.ArraySegment<GearJoint>(_gearJoints, 0, _gearJointCount);
        public System.Collections.Generic.IReadOnlyList<RopeJoint> RopeJoints => new System.ArraySegment<RopeJoint>(_ropeJoints, 0, _ropeJointCount);
        public System.Collections.Generic.IReadOnlyList<FrictionJoint> FrictionJoints => new System.ArraySegment<FrictionJoint>(_frictionJoints, 0, _frictionJointCount);
        public System.Collections.Generic.IReadOnlyList<Island> LastIslands => _lastIslands;
        public SolverSet AwakeSet => _awakeSet;
        public SolverSet SleepingSet => _sleepingSet;
        public SolverSet StaticSet => _staticSet;
        public SolverSet DisabledSet => _disabledSet;
        public ContactSolverStats LastContactSolverStats => _lastContactSolverStats;
        public WorldId Id => _worldId;
        public AllocationStats GetAllocationStats()
        {
            return new AllocationStats(
                _bodies.Count,
                CountFixtures(),
                CountJoints(),
                _contactMap.Count,
                _lastIslands.Count,
                _sleepingIslandSets.Count,
                _bodyIdPool.Capacity,
                _bodyIdPool.Count,
                _fixtureIdPool.Capacity,
                _fixtureIdPool.Count,
                _jointIdPool.Capacity,
                _jointIdPool.Count,
                _contactIdPool.Capacity,
                _contactIdPool.Count,
                _islandIdPool.Capacity,
                _islandIdPool.Count,
                _solverSetIdPool.Capacity,
                _solverSetIdPool.Count,
                _contactEdgeIdPool.Capacity,
                _contactEdgeIdPool.Count,
                _sensorPairs.Capacity,
                _sensorPairs.Count,
                _bodyIdPool.ByteCount,
                _fixtureIdPool.ByteCount,
                _jointIdPool.ByteCount,
                _contactIdPool.ByteCount,
                _islandIdPool.ByteCount,
                _solverSetIdPool.ByteCount,
                _contactEdgeIdPool.ByteCount,
                _sensorPairs.GetByteCount(),
                Box2D.GetByteCount());
        }

        internal void SetWorldId(WorldId id)
        {
            _worldId = id;
        }

        public void Dispose()
        {
            Dispose(disposing: true);
            GC.SuppressFinalize(this);
        }

        private void Dispose(bool disposing)
        {
            if (_disposed)
            {
                return;
            }

            _arena.Destroy();
            _disposed = true;
        }

        ~World()
        {
            Dispose(disposing: false);
        }
        public int? GetIslandId(Body body)
        {
            if (TryGetBodyIslandId(body, out int id))
            {
                return id;
            }
            return null;
        }

        private bool TryGetBodyIslandId(Body body, out int islandId)
        {
            int bodyId = body.Id;
            if ((uint)bodyId < (uint)_bodyCount)
            {
                int mappedId = _bodyIslandIdsByBody[bodyId];
                if (mappedId >= 0)
                {
                    islandId = mappedId;
                    return true;
                }
            }

            islandId = -1;
            return false;
        }

        private System.Collections.Generic.List<Island> _lastIslands = new System.Collections.Generic.List<Island>();
        private readonly SolverSet _staticSet = new SolverSet();
        private readonly SolverSet _disabledSet = new SolverSet();
        private readonly SolverSet _awakeSet = new SolverSet();
        private readonly SolverSet _sleepingSet = new SolverSet();
        private readonly System.Collections.Generic.Dictionary<Body, System.Collections.Generic.List<Contact>> _bodyContacts =
            new System.Collections.Generic.Dictionary<Body, System.Collections.Generic.List<Contact>>();
        private System.Collections.Generic.List<JointHandle>[] _bodyJoints = System.Array.Empty<System.Collections.Generic.List<JointHandle>>();
        private readonly System.Collections.Generic.Dictionary<int, SolverSet> _sleepingIslandSets =
            new System.Collections.Generic.Dictionary<int, SolverSet>();
        private readonly System.Collections.Generic.Dictionary<int, int> _sleepingSetIdByIslandId =
            new System.Collections.Generic.Dictionary<int, int>();
        private SolverSetType[] _jointSolverSetTypesById = System.Array.Empty<SolverSetType>();
        private int[] _jointSolverSetIdsById = System.Array.Empty<int>();
        private bool[] _jointSolverSetUsedById = System.Array.Empty<bool>();
        private float _stepContactHertz;

        internal Body GetBody(int index)
        {
             return _bodies[index];
        }

        public Body CreateBody(BodyDef def)
        {
            int id = _bodyCount;
            EnsureBodyCapacity(id + 1);
            int poolId = _bodyIdPool.Alloc();

            Body body = new Body(this, def, id);
            _bodyPoolIds[id] = poolId;
            // Phase 2.5: initialize within-step delta to (0, identity) so a
            // body inserted mid-frame (or into a slot vacated by a swap-
            // remove) sees a fresh, post-step-start state. Step-start
            // snapshot picks up the BodyDef's pose immediately — `_bodyPositions[id]`
            // / `_bodyRotations[id]` are set by the Body ctor below, but we
            // mirror what they'll be so a mid-step query sees a consistent
            // pair (step-start == current == BodyDef.Position).
            _bodyDeltaPositions[id] = Vec2.Zero;
            _bodyDeltaRotations[id] = Rot.Identity;
            _bodyStepStartPositions[id] = def.Position;
            _bodyStepStartRotations[id] = def.Rotation;

            _bodies.Add(body);
            _bodyCount++;

            EnsureBodyAdjacency(body);
            if (body.Type == BodyType.Static)
            {
                if (!_staticSet.Bodies.Contains(body))
                {
                    _staticSet.Bodies.Add(body);
                }
            }
            return body;
        }

        public void DestroyBody(Body body)
        {
            if (body == null) throw new ArgumentNullException(nameof(body));
            if (body.Id == -1) return; // Already destroyed
            int removedPoolId = _bodyPoolIds[body.Id];

            System.Collections.Generic.List<JointHandle>? joints = _bodyJoints[body.Id];
            if (joints != null && joints.Count > 0)
            {
                System.Collections.Generic.List<JointHandle> toDestroy = new System.Collections.Generic.List<JointHandle>(joints);
                for (int i = toDestroy.Count - 1; i >= 0; --i)
                {
                    JointHandle handle = toDestroy[i];
                    if (!TryGetJointIndex(handle, out int jointIndex))
                    {
                        continue;
                    }

                    switch (handle.Type)
                    {
                        case JointType.Distance:
                            DestroyJoint(_distanceJoints[jointIndex]);
                            break;
                        case JointType.Revolute:
                            DestroyJoint(_revoluteJoints[jointIndex]);
                            break;
                        case JointType.Prismatic:
                            DestroyJoint(_prismaticJoints[jointIndex]);
                            break;
                        case JointType.Wheel:
                            DestroyJoint(_wheelJoints[jointIndex]);
                            break;
                        case JointType.Pulley:
                            DestroyJoint(_pulleyJoints[jointIndex]);
                            break;
                        case JointType.Weld:
                            DestroyJoint(_weldJoints[jointIndex]);
                            break;
                        case JointType.Motor:
                            DestroyJoint(_motorJoints[jointIndex]);
                            break;
                        case JointType.Gear:
                            DestroyJoint(_gearJoints[jointIndex]);
                            break;
                        case JointType.Rope:
                            DestroyJoint(_ropeJoints[jointIndex]);
                            break;
                        case JointType.Friction:
                            DestroyJoint(_frictionJoints[jointIndex]);
                            break;
                    }
                }
            }

            while (body.Fixtures.Count > 0)
            {
                body.DestroyFixture(body.Fixtures[body.Fixtures.Count - 1]);
            }
            body.ContactList = null;
            body.ContactEdgeCount = 0;

            int idToRemove = body.Id;
            int lastId = _bodyCount - 1;

            if (idToRemove != lastId)
            {
                // Swap-Remove Logic (O(1))
                // Move the last element into the hole
                _bodyPositions[idToRemove] = _bodyPositions[lastId];
                _bodyLocalCenters[idToRemove] = _bodyLocalCenters[lastId];
                _bodyRotations[idToRemove] = _bodyRotations[lastId];
                _bodyDeltaPositions[idToRemove] = _bodyDeltaPositions[lastId];
                _bodyDeltaRotations[idToRemove] = _bodyDeltaRotations[lastId];
                _bodyStepStartPositions[idToRemove] = _bodyStepStartPositions[lastId];
                _bodyStepStartRotations[idToRemove] = _bodyStepStartRotations[lastId];
                _bodyLinearVelocities[idToRemove] = _bodyLinearVelocities[lastId];
                _bodyAngularVelocities[idToRemove] = _bodyAngularVelocities[lastId];
                _bodyLinearDampings[idToRemove] = _bodyLinearDampings[lastId];
                _bodyAngularDampings[idToRemove] = _bodyAngularDampings[lastId];
                _bodyGravityScales[idToRemove] = _bodyGravityScales[lastId];
                _bodyTypes[idToRemove] = _bodyTypes[lastId];
                _bodyMasses[idToRemove] = _bodyMasses[lastId];
                _bodyInertias[idToRemove] = _bodyInertias[lastId];
                _bodyInverseMasses[idToRemove] = _bodyInverseMasses[lastId];
                _bodyInverseInertias[idToRemove] = _bodyInverseInertias[lastId];
                _bodyFixedRotations[idToRemove] = _bodyFixedRotations[lastId];
                _bodyAwakes[idToRemove] = _bodyAwakes[lastId];
                _bodyAllowSleeps[idToRemove] = _bodyAllowSleeps[lastId];
                _bodyForces[idToRemove] = _bodyForces[lastId];
                _bodyTorques[idToRemove] = _bodyTorques[lastId];
                _bodySleepTimes[idToRemove] = _bodySleepTimes[lastId];
                _bodySolverSetTypes[idToRemove] = _bodySolverSetTypes[lastId];
                _bodyPoolIds[idToRemove] = _bodyPoolIds[lastId];
                _bodyIslandIdsByBody[idToRemove] = _bodyIslandIdsByBody[lastId];
                _bodyJoints[idToRemove] = _bodyJoints[lastId];

                // Move the Body wrapper in the list
                Body lastBody = _bodies[lastId];
                _bodies[idToRemove] = lastBody;
                lastBody.Id = idToRemove; // CRITICAL: Update the index in the wrapper!

                RemapBodyIndexInJointData(lastId, idToRemove);
            }

            // Remove the last element
            _bodies.RemoveAt(lastId);
            _bodyCount--;
            _bodyIslandIdsByBody[lastId] = -1;
            _bodyJoints[lastId] = null!;
            
            // Clean up other references
            _bodyContacts.Remove(body);
            _bodyJoints[idToRemove] = null!;
            if (idToRemove == lastId && (uint)idToRemove < (uint)_bodyIslandIdsByBody.Length)
            {
                _bodyIslandIdsByBody[idToRemove] = -1;
            }
            _staticSet.Bodies.Remove(body);
            _awakeSet.Bodies.Remove(body);
            _sleepingSet.Bodies.Remove(body);
            _disabledSet.Bodies.Remove(body);

            _bodyIdPool.Free(removedPoolId);
            body.Id = -1; // Mark as destroyed AFTER using it logic
        }

        private void RemapBodyIndexInJointData(int fromIndex, int toIndex)
        {
            static void Remap(ref int value, int from, int to)
            {
                if (value == from) value = to;
            }

            for (int i = 0; i < _distanceJointCount; ++i)
            {
                Remap(ref _distanceJointsData[i].BodyA, fromIndex, toIndex);
                Remap(ref _distanceJointsData[i].BodyB, fromIndex, toIndex);
            }
            for (int i = 0; i < _revoluteJointCount; ++i)
            {
                Remap(ref _revoluteJointsData[i].BodyA, fromIndex, toIndex);
                Remap(ref _revoluteJointsData[i].BodyB, fromIndex, toIndex);
            }
            for (int i = 0; i < _prismaticJointCount; ++i)
            {
                Remap(ref _prismaticJointsData[i].BodyA, fromIndex, toIndex);
                Remap(ref _prismaticJointsData[i].BodyB, fromIndex, toIndex);
            }
            for (int i = 0; i < _wheelJointCount; ++i)
            {
                Remap(ref _wheelJointsData[i].BodyA, fromIndex, toIndex);
                Remap(ref _wheelJointsData[i].BodyB, fromIndex, toIndex);
            }
            for (int i = 0; i < _pulleyJointCount; ++i)
            {
                Remap(ref _pulleyJointsData[i].BodyA, fromIndex, toIndex);
                Remap(ref _pulleyJointsData[i].BodyB, fromIndex, toIndex);
            }
            for (int i = 0; i < _weldJointCount; ++i)
            {
                Remap(ref _weldJointsData[i].BodyA, fromIndex, toIndex);
                Remap(ref _weldJointsData[i].BodyB, fromIndex, toIndex);
            }
            for (int i = 0; i < _motorJointCount; ++i)
            {
                Remap(ref _motorJointsData[i].BodyA, fromIndex, toIndex);
                Remap(ref _motorJointsData[i].BodyB, fromIndex, toIndex);
            }
            for (int i = 0; i < _gearJointCount; ++i)
            {
                Remap(ref _gearJointsData[i].BodyA, fromIndex, toIndex);
                Remap(ref _gearJointsData[i].BodyB, fromIndex, toIndex);
                Remap(ref _gearJointsData[i].BodyC, fromIndex, toIndex);
                Remap(ref _gearJointsData[i].BodyD, fromIndex, toIndex);
            }
            for (int i = 0; i < _ropeJointCount; ++i)
            {
                Remap(ref _ropeJointsData[i].BodyA, fromIndex, toIndex);
                Remap(ref _ropeJointsData[i].BodyB, fromIndex, toIndex);
            }
            for (int i = 0; i < _frictionJointCount; ++i)
            {
                Remap(ref _frictionJointsData[i].BodyA, fromIndex, toIndex);
                Remap(ref _frictionJointsData[i].BodyB, fromIndex, toIndex);
            }
        }

        private void EnsureDistanceJointCapacity(int capacity)
        {
            if (_distanceJoints.Length < capacity)
            {
                int newSize = Math.Max(capacity, _distanceJoints.Length * 2);
                Array.Resize(ref _distanceJoints, newSize);
                Array.Resize(ref _distanceJointsData, newSize);
            }
        }

        public DistanceJoint CreateJoint(DistanceJointDef def)
        {
            EnsureDistanceJointCapacity(_distanceJointCount + 1);
            DistanceJoint joint = new DistanceJoint(this, def);
            joint.Id = _jointIdPool.Alloc();
            
            // DOD Storage
            int index = _distanceJointCount;
            _distanceJoints[index] = joint;
            _distanceJointsData[index] = new DistanceJointData
            {
                BodyA = def.BodyA.Id,
                BodyB = def.BodyB.Id,
                LocalAnchorA = def.LocalAnchorA,
                LocalAnchorB = def.LocalAnchorB,
                Length = def.Length,
                FrequencyHz = def.FrequencyHz,
                DampingRatio = def.DampingRatio,
                CollideConnected = def.CollideConnected,
                Impulse = 0f,
                Mass = 0f,
                Gamma = 0f,
                Bias = 0f,
                U = Vec2.Zero
            };
            joint.Index = index;
            _distanceJointCount++;

            _distanceJointIndexById[joint.Id] = index;
            JointHandle handle = new JointHandle(JointType.Distance, joint.Id);
            LinkJoint(handle, joint.BodyA, joint.BodyB);
                        SetJointSolverSet(handle, GetJointSolverSetType(joint.BodyA, joint.BodyB), GetJointSolverSetId(joint.BodyA, joint.BodyB));
            TryAddJointToSleepingSet(handle, joint.BodyA, joint.BodyB);
            MarkIslandDirty(joint.BodyA);
            MarkIslandDirty(joint.BodyB);
            HandleJointCreationAwakeState(joint.BodyA, joint.BodyB);
            return joint;
        }

/*
        public RevoluteJoint CreateJoint(RevoluteJointDef def)
        {
            RevoluteJoint joint = new RevoluteJoint(def);
            joint.Id = _jointIdPool.Alloc();
            _revoluteJoints.Add(joint);
            _revoluteJointIndexById[joint.Id] = _revoluteJoints.Count - 1;
            JointHandle handle = new JointHandle(JointType.Revolute, joint.Id);
            LinkJoint(handle, joint.BodyA, joint.BodyB);
                        SetJointSolverSet(handle, GetJointSolverSetType(joint.BodyA, joint.BodyB), GetJointSolverSetId(joint.BodyA, joint.BodyB));
            TryAddJointToSleepingSet(handle, joint.BodyA, joint.BodyB);
            MarkIslandDirty(joint.BodyA);
            MarkIslandDirty(joint.BodyB);
            HandleJointCreationAwakeState(joint.BodyA, joint.BodyB);
            return joint;
        }
*/

/*
        public PrismaticJoint CreateJoint(PrismaticJointDef def)
        {
            PrismaticJoint joint = new PrismaticJoint(def);
            joint.Id = _jointIdPool.Alloc();
            _prismaticJoints.Add(joint);
            _prismaticJointIndexById[joint.Id] = _prismaticJoints.Count - 1;
            JointHandle handle = new JointHandle(JointType.Prismatic, joint.Id);
            LinkJoint(handle, joint.BodyA, joint.BodyB);
                        SetJointSolverSet(handle, GetJointSolverSetType(joint.BodyA, joint.BodyB), GetJointSolverSetId(joint.BodyA, joint.BodyB));
            TryAddJointToSleepingSet(handle, joint.BodyA, joint.BodyB);
            MarkIslandDirty(joint.BodyA);
            MarkIslandDirty(joint.BodyB);
            HandleJointCreationAwakeState(joint.BodyA, joint.BodyB);
            return joint;
        }
*/

        public WheelJoint CreateJoint(WheelJointDef def)
        {
            if (_wheelJointCount >= _wheelJointCapacity)
            {
                int newCapacity = Math.Max(_wheelJointCapacity * 2, 256);
                Array.Resize(ref _wheelJoints, newCapacity);
                Array.Resize(ref _wheelJointsData, newCapacity);
                _wheelJointCapacity = newCapacity;
            }

            int id = _jointIdPool.Alloc();
            int index = _wheelJointCount;
            WheelJoint joint = new WheelJoint(this, id, index);
            _wheelJoints[index] = joint;
            _wheelJointsData[index] = new WheelJointData
            {
                Id = id,
                BodyA = def.BodyA.Id,
                BodyB = def.BodyB.Id,
                CollideConnected = def.CollideConnected,
                LocalAnchorA = def.LocalAnchorA,
                LocalAnchorB = def.LocalAnchorB,
                LocalAxisA = def.LocalAxisA.Normalize(),
                EnableMotor = def.EnableMotor,
                MotorSpeed = def.MotorSpeed,
                MaxMotorTorque = def.MaxMotorTorque,
                FrequencyHz = def.FrequencyHz,
                DampingRatio = def.DampingRatio,
                EnableLimit = def.EnableLimit,
                LowerTranslation = def.LowerTranslation,
                UpperTranslation = def.UpperTranslation
            };
            _wheelJointCount++;

            _wheelJointIndexById[id] = index;
            JointHandle handle = new JointHandle(JointType.Wheel, id);
            LinkJoint(handle, def.BodyA, def.BodyB);
                        SetJointSolverSet(handle, GetJointSolverSetType(def.BodyA, def.BodyB), GetJointSolverSetId(def.BodyA, def.BodyB));
            TryAddJointToSleepingSet(handle, def.BodyA, def.BodyB);
            MarkIslandDirty(def.BodyA);
            MarkIslandDirty(def.BodyB);
            HandleJointCreationAwakeState(def.BodyA, def.BodyB);
            return joint;
        }

        public PulleyJoint CreateJoint(PulleyJointDef def)
        {
            if (_pulleyJointCount >= _pulleyJointCapacity)
            {
                int newCapacity = Math.Max(_pulleyJointCapacity * 2, 256);
                Array.Resize(ref _pulleyJoints, newCapacity);
                Array.Resize(ref _pulleyJointsData, newCapacity);
                _pulleyJointCapacity = newCapacity;
            }

            int id = _jointIdPool.Alloc();
            int index = _pulleyJointCount;
            PulleyJoint joint = new PulleyJoint(this, id, index);
            _pulleyJoints[index] = joint;
            _pulleyJointsData[index] = new PulleyJointData
            {
                Id = id,
                BodyA = def.BodyA.Id,
                BodyB = def.BodyB.Id,
                CollideConnected = def.CollideConnected,
                GroundAnchorA = def.GroundAnchorA,
                GroundAnchorB = def.GroundAnchorB,
                LocalAnchorA = def.LocalAnchorA,
                LocalAnchorB = def.LocalAnchorB,
                LengthA = def.LengthA,
                LengthB = def.LengthB,
                Ratio = def.Ratio
            };
            _pulleyJointCount++;

            _pulleyJointIndexById[id] = index;
            JointHandle handle = new JointHandle(JointType.Pulley, id);
            LinkJoint(handle, def.BodyA, def.BodyB);
                        SetJointSolverSet(handle, GetJointSolverSetType(def.BodyA, def.BodyB), GetJointSolverSetId(def.BodyA, def.BodyB));
            TryAddJointToSleepingSet(handle, def.BodyA, def.BodyB);
            MarkIslandDirty(def.BodyA);
            MarkIslandDirty(def.BodyB);
            HandleJointCreationAwakeState(def.BodyA, def.BodyB);
            return joint;
        }

        public WeldJoint CreateJoint(WeldJointDef def)
        {
            if (_weldJointCount >= _weldJointCapacity)
            {
                int newCapacity = Math.Max(_weldJointCapacity * 2, 256);
                Array.Resize(ref _weldJoints, newCapacity);
                Array.Resize(ref _weldJointsData, newCapacity);
                _weldJointCapacity = newCapacity;
            }

            int id = _jointIdPool.Alloc();
            int index = _weldJointCount;
            WeldJoint joint = new WeldJoint(this, id, index);
            _weldJoints[index] = joint;
            // Capture the rest-state world anchor delta. The soft linear spring
            // drives `(current B anchor) - (current A anchor) - DeltaCenter`
            // back toward zero — DeltaCenter is the joint's reference and
            // must be fixed at creation, not re-captured each step (which
            // would erase the very drift we want to correct).
            Vec2 worldAnchorA = Transform.Mul(def.BodyA.Transform, def.LocalAnchorA);
            Vec2 worldAnchorB = Transform.Mul(def.BodyB.Transform, def.LocalAnchorB);

            _weldJointsData[index] = new WeldJointData
            {
                Id = id,
                BodyA = def.BodyA.Id,
                BodyB = def.BodyB.Id,
                CollideConnected = def.CollideConnected,
                LocalAnchorA = def.LocalAnchorA,
                LocalAnchorB = def.LocalAnchorB,
                ReferenceAngle = def.ReferenceAngle,
                LinearHertz = def.LinearHertz,
                LinearDampingRatio = def.LinearDampingRatio,
                AngularHertz = def.AngularHertz,
                AngularDampingRatio = def.AngularDampingRatio,
                DeltaCenter = worldAnchorB - worldAnchorA,
            };
            _weldJointCount++;

            _weldJointIndexById[id] = index;
            JointHandle handle = new JointHandle(JointType.Weld, id);
            LinkJoint(handle, def.BodyA, def.BodyB);
                        SetJointSolverSet(handle, GetJointSolverSetType(def.BodyA, def.BodyB), GetJointSolverSetId(def.BodyA, def.BodyB));
            TryAddJointToSleepingSet(handle, def.BodyA, def.BodyB);
            MarkIslandDirty(def.BodyA);
            MarkIslandDirty(def.BodyB);
            HandleJointCreationAwakeState(def.BodyA, def.BodyB);
            return joint;
        }

        public MotorJoint CreateJoint(MotorJointDef def)
        {
            if (_motorJointCount >= _motorJointCapacity)
            {
                int newCapacity = Math.Max(_motorJointCapacity * 2, 256);
                Array.Resize(ref _motorJoints, newCapacity);
                Array.Resize(ref _motorJointsData, newCapacity);
                _motorJointCapacity = newCapacity;
            }

            int id = _jointIdPool.Alloc();
            int index = _motorJointCount;
            MotorJoint joint = new MotorJoint(this, id, index);
            _motorJoints[index] = joint;
            _motorJointsData[index] = new MotorJointData
            {
                Id = id,
                BodyA = def.BodyA.Id,
                BodyB = def.BodyB.Id,
                CollideConnected = def.CollideConnected,
                LinearOffset = def.LinearOffset,
                AngularOffset = def.AngularOffset,
                MaxForce = def.MaxForce,
                MaxTorque = def.MaxTorque,
                CorrectionFactor = def.CorrectionFactor
            };
            _motorJointCount++;

            _motorJointIndexById[id] = index;
            JointHandle handle = new JointHandle(JointType.Motor, id);
            LinkJoint(handle, def.BodyA, def.BodyB);
                        SetJointSolverSet(handle, GetJointSolverSetType(def.BodyA, def.BodyB), GetJointSolverSetId(def.BodyA, def.BodyB));
            TryAddJointToSleepingSet(handle, def.BodyA, def.BodyB);
            MarkIslandDirty(def.BodyA);
            MarkIslandDirty(def.BodyB);
            HandleJointCreationAwakeState(def.BodyA, def.BodyB);
            return joint;
        }

        public GearJoint CreateJoint(GearJointDef def)
        {
            if (_gearJointCount >= _gearJointCapacity)
            {
                int newCapacity = Math.Max(_gearJointCapacity * 2, 256);
                Array.Resize(ref _gearJoints, newCapacity);
                Array.Resize(ref _gearJointsData, newCapacity);
                _gearJointCapacity = newCapacity;
            }

            GearJointKind typeA;
            int jointAId;
            Body bodyAObj;
            Body bodyBObj;
            if (def.JointA is RevoluteJoint revA)
            {
                typeA = GearJointKind.Revolute;
                jointAId = revA.Id;
                bodyAObj = revA.BodyA;
                bodyBObj = revA.BodyB;
            }
            else if (def.JointA is PrismaticJoint priA)
            {
                typeA = GearJointKind.Prismatic;
                jointAId = priA.Id;
                bodyAObj = priA.BodyA;
                bodyBObj = priA.BodyB;
            }
            else
            {
                throw new ArgumentException("JointA must be RevoluteJoint or PrismaticJoint.", nameof(def));
            }

            GearJointKind typeB;
            int jointBId;
            Body bodyCObj;
            Body bodyDObj;
            if (def.JointB is RevoluteJoint revB)
            {
                typeB = GearJointKind.Revolute;
                jointBId = revB.Id;
                bodyCObj = revB.BodyA;
                bodyDObj = revB.BodyB;
            }
            else if (def.JointB is PrismaticJoint priB)
            {
                typeB = GearJointKind.Prismatic;
                jointBId = priB.Id;
                bodyCObj = priB.BodyA;
                bodyDObj = priB.BodyB;
            }
            else
            {
                throw new ArgumentException("JointB must be RevoluteJoint or PrismaticJoint.", nameof(def));
            }

            int id = _jointIdPool.Alloc();
            int index = _gearJointCount;
            GearJoint joint = new GearJoint(this, id, index);
            _gearJoints[index] = joint;

            _gearJointsData[index] = new GearJointData
            {
                Id = id,
                CollideConnected = def.CollideConnected,
                BodyA = bodyAObj.Id,
                BodyB = bodyBObj.Id,
                BodyC = bodyCObj.Id,
                BodyD = bodyDObj.Id,
                TypeA = typeA,
                TypeB = typeB,
                JointAId = jointAId,
                JointBId = jointBId,
                Ratio = def.Ratio
            };
            _gearJointCount++;

            ref GearJointData data = ref _gearJointsData[index];
            if (TryGetGearCoordinateA(ref data, out float coordA) &&
                TryGetGearCoordinateB(ref data, out float coordB))
            {
                data.Constant = coordA + data.Ratio * coordB;
            }
            else
            {
                data.Constant = 0f;
            }

            _gearJointIndexById[id] = index;
            JointHandle handle = new JointHandle(JointType.Gear, id);
            LinkJoint(handle, bodyAObj, bodyBObj);
            LinkJointToBody(handle, bodyCObj);
            LinkJointToBody(handle, bodyDObj);
                        SetJointSolverSet(handle, GetJointSolverSetType(bodyBObj, bodyDObj), GetJointSolverSetId(bodyBObj, bodyDObj));
            TryAddJointToSleepingSet(handle, bodyBObj, bodyDObj);
            MarkIslandDirty(bodyAObj);
            MarkIslandDirty(bodyBObj);
            MarkIslandDirty(bodyCObj);
            MarkIslandDirty(bodyDObj);
            HandleJointCreationAwakeState(bodyAObj, bodyBObj);
            HandleJointCreationAwakeState(bodyCObj, bodyDObj);
            return joint;
        }

        public RopeJoint CreateJoint(RopeJointDef def)
        {
            if (_ropeJointCount >= _ropeJointCapacity)
            {
                int newCapacity = Math.Max(_ropeJointCapacity * 2, 256);
                Array.Resize(ref _ropeJoints, newCapacity);
                Array.Resize(ref _ropeJointsData, newCapacity);
                _ropeJointCapacity = newCapacity;
            }

            int id = _jointIdPool.Alloc();
            int index = _ropeJointCount;
            RopeJoint joint = new RopeJoint(this, id, index);
            _ropeJoints[index] = joint;
            _ropeJointsData[index] = new RopeJointData
            {
                Id = id,
                BodyA = def.BodyA.Id,
                BodyB = def.BodyB.Id,
                CollideConnected = def.CollideConnected,
                LocalAnchorA = def.LocalAnchorA,
                LocalAnchorB = def.LocalAnchorB,
                MaxLength = def.MaxLength
            };
            _ropeJointCount++;

            _ropeJointIndexById[id] = index;
            JointHandle handle = new JointHandle(JointType.Rope, id);
            LinkJoint(handle, def.BodyA, def.BodyB);
                        SetJointSolverSet(handle, GetJointSolverSetType(def.BodyA, def.BodyB), GetJointSolverSetId(def.BodyA, def.BodyB));
            TryAddJointToSleepingSet(handle, def.BodyA, def.BodyB);
            MarkIslandDirty(def.BodyA);
            MarkIslandDirty(def.BodyB);
            HandleJointCreationAwakeState(def.BodyA, def.BodyB);
            return joint;
        }

        public FrictionJoint CreateJoint(FrictionJointDef def)
        {
            if (_frictionJointCount >= _frictionJointCapacity)
            {
                int newCapacity = Math.Max(_frictionJointCapacity * 2, 256);
                Array.Resize(ref _frictionJoints, newCapacity);
                Array.Resize(ref _frictionJointsData, newCapacity);
                _frictionJointCapacity = newCapacity;
            }

            int id = _jointIdPool.Alloc();
            int index = _frictionJointCount;
            FrictionJoint joint = new FrictionJoint(this, id, index);
            _frictionJoints[index] = joint;
            _frictionJointsData[index] = new FrictionJointData
            {
                Id = id,
                BodyA = def.BodyA.Id,
                BodyB = def.BodyB.Id,
                CollideConnected = def.CollideConnected,
                LocalAnchorA = def.LocalAnchorA,
                LocalAnchorB = def.LocalAnchorB,
                MaxForce = def.MaxForce,
                MaxTorque = def.MaxTorque
            };
            _frictionJointCount++;

            _frictionJointIndexById[id] = index;
            JointHandle handle = new JointHandle(JointType.Friction, id);
            LinkJoint(handle, def.BodyA, def.BodyB);
                        SetJointSolverSet(handle, GetJointSolverSetType(def.BodyA, def.BodyB), GetJointSolverSetId(def.BodyA, def.BodyB));
            TryAddJointToSleepingSet(handle, def.BodyA, def.BodyB);
            MarkIslandDirty(def.BodyA);
            MarkIslandDirty(def.BodyB);
            HandleJointCreationAwakeState(def.BodyA, def.BodyB);
            return joint;
        }

        public bool DestroyJoint(DistanceJoint joint)
        {
             if (joint.Index == -1) return false;
             int index = joint.Index;
             int lastIndex = _distanceJointCount - 1;
             
             if (index != lastIndex)
             {
                 _distanceJointsData[index] = _distanceJointsData[lastIndex];
                 _distanceJoints[index] = _distanceJoints[lastIndex];
                 
                 DistanceJoint movedJoint = _distanceJoints[index];
                 movedJoint.Index = index;
                 _distanceJointIndexById[movedJoint.Id] = index;
             }
             
             _distanceJointCount--;
             _distanceJoints[lastIndex] = null!;

             int jointId = joint.Id;
             _jointIdPool.Free(jointId);
             SetJointId(joint, -1);
             _distanceJointIndexById.Remove(jointId); // Explicit removal

             // Cleanup logic duplicated from RemoveJoint to avoid List dependency
             Body bodyA = joint.BodyA;
             Body bodyB = joint.BodyB;
             JointHandle handle = new JointHandle(JointType.Distance, jointId);
             UnlinkJoint(handle, bodyA, bodyB);
             ClearJointSolverSet(handle);
             
             MarkIslandDirty(bodyA);
             MarkIslandDirty(bodyB);
             
             // RemoveJointFromSolverSets logic? 
             RemoveJointFromSolverSets(JointType.Distance, jointId);
             
             _islandsDirty = true;
             
             joint.Index = -1;
             return true;
        }
// public bool DestroyJoint(RevoluteJoint joint) => RemoveJoint(_revoluteJoints, joint);
// public bool DestroyJoint(PrismaticJoint joint) => RemoveJoint(_prismaticJoints, joint);
        public bool DestroyJoint(WheelJoint joint)
        {
            int index = joint.Index;
            if (index < 0 || index >= _wheelJointCount) return false;

            int jointId = joint.Id;
            Body bodyA = joint.BodyA;
            Body bodyB = joint.BodyB;
            int lastIndex = _wheelJointCount - 1;
            if (index != lastIndex)
            {
                WheelJoint moved = _wheelJoints[lastIndex];
                moved.Index = index;
                _wheelJoints[index] = moved;
                _wheelJointsData[index] = _wheelJointsData[lastIndex];
                _wheelJointIndexById[moved.Id] = index;
            }
            _wheelJoints[lastIndex] = null!;
            _wheelJointsData[lastIndex] = default;
            _wheelJointCount--;

            _jointIdPool.Free(jointId);
            _wheelJointIndexById.Remove(jointId);
            joint.Id = -1;
            joint.Index = -1;

            JointHandle handle = new JointHandle(JointType.Wheel, jointId);
            UnlinkJoint(handle, bodyA, bodyB);
            ClearJointSolverSet(handle);
            
            ClearJointConstraintGraphIndex(handle);
            MarkIslandDirty(bodyA);
            MarkIslandDirty(bodyB);
            RemoveJointFromSolverSets(JointType.Wheel, jointId);
            _islandsDirty = true;
            return true;
        }

        public bool DestroyJoint(PulleyJoint joint)
        {
            int index = joint.Index;
            if (index < 0 || index >= _pulleyJointCount) return false;

            int jointId = joint.Id;
            Body bodyA = joint.BodyA;
            Body bodyB = joint.BodyB;
            int lastIndex = _pulleyJointCount - 1;
            if (index != lastIndex)
            {
                PulleyJoint moved = _pulleyJoints[lastIndex];
                moved.Index = index;
                _pulleyJoints[index] = moved;
                _pulleyJointsData[index] = _pulleyJointsData[lastIndex];
                _pulleyJointIndexById[moved.Id] = index;
            }
            _pulleyJoints[lastIndex] = null!;
            _pulleyJointsData[lastIndex] = default;
            _pulleyJointCount--;

            _jointIdPool.Free(jointId);
            _pulleyJointIndexById.Remove(jointId);
            joint.Id = -1;
            joint.Index = -1;

            JointHandle handle = new JointHandle(JointType.Pulley, jointId);
            UnlinkJoint(handle, bodyA, bodyB);
            ClearJointSolverSet(handle);
            
            ClearJointConstraintGraphIndex(handle);
            MarkIslandDirty(bodyA);
            MarkIslandDirty(bodyB);
            RemoveJointFromSolverSets(JointType.Pulley, jointId);
            _islandsDirty = true;
            return true;
        }

        public bool DestroyJoint(WeldJoint joint)
        {
            int index = joint.Index;
            if (index < 0 || index >= _weldJointCount) return false;

            int jointId = joint.Id;
            Body bodyA = joint.BodyA;
            Body bodyB = joint.BodyB;
            int lastIndex = _weldJointCount - 1;
            if (index != lastIndex)
            {
                WeldJoint moved = _weldJoints[lastIndex];
                moved.Index = index;
                _weldJoints[index] = moved;
                _weldJointsData[index] = _weldJointsData[lastIndex];
                _weldJointIndexById[moved.Id] = index;
            }
            _weldJoints[lastIndex] = null!;
            _weldJointsData[lastIndex] = default;
            _weldJointCount--;

            _jointIdPool.Free(jointId);
            _weldJointIndexById.Remove(jointId);
            joint.Id = -1;
            joint.Index = -1;

            JointHandle handle = new JointHandle(JointType.Weld, jointId);
            UnlinkJoint(handle, bodyA, bodyB);
            ClearJointSolverSet(handle);
            
            ClearJointConstraintGraphIndex(handle);
            MarkIslandDirty(bodyA);
            MarkIslandDirty(bodyB);
            RemoveJointFromSolverSets(JointType.Weld, jointId);
            _islandsDirty = true;
            return true;
        }

        public bool DestroyJoint(MotorJoint joint)
        {
            int index = joint.Index;
            if (index < 0 || index >= _motorJointCount) return false;

            int jointId = joint.Id;
            Body bodyA = joint.BodyA;
            Body bodyB = joint.BodyB;
            int lastIndex = _motorJointCount - 1;
            if (index != lastIndex)
            {
                MotorJoint moved = _motorJoints[lastIndex];
                moved.Index = index;
                _motorJoints[index] = moved;
                _motorJointsData[index] = _motorJointsData[lastIndex];
                _motorJointIndexById[moved.Id] = index;
            }
            _motorJoints[lastIndex] = null!;
            _motorJointsData[lastIndex] = default;
            _motorJointCount--;

            _jointIdPool.Free(jointId);
            _motorJointIndexById.Remove(jointId);
            joint.Id = -1;
            joint.Index = -1;

            JointHandle handle = new JointHandle(JointType.Motor, jointId);
            UnlinkJoint(handle, bodyA, bodyB);
            ClearJointSolverSet(handle);
            
            ClearJointConstraintGraphIndex(handle);
            MarkIslandDirty(bodyA);
            MarkIslandDirty(bodyB);
            RemoveJointFromSolverSets(JointType.Motor, jointId);
            _islandsDirty = true;
            return true;
        }

        public bool DestroyJoint(GearJoint joint)
        {
            int index = joint.Index;
            if (index < 0 || index >= _gearJointCount) return false;

            int jointId = joint.Id;
            Body bodyA = joint.BodyA;
            Body bodyB = joint.BodyB;
            Body bodyC = joint.BodyC;
            Body bodyD = joint.BodyD;
            int lastIndex = _gearJointCount - 1;
            if (index != lastIndex)
            {
                GearJoint moved = _gearJoints[lastIndex];
                moved.Index = index;
                _gearJoints[index] = moved;
                _gearJointsData[index] = _gearJointsData[lastIndex];
                _gearJointIndexById[moved.Id] = index;
            }
            _gearJoints[lastIndex] = null!;
            _gearJointsData[lastIndex] = default;
            _gearJointCount--;

            _jointIdPool.Free(jointId);
            _gearJointIndexById.Remove(jointId);
            joint.Id = -1;
            joint.Index = -1;

            JointHandle handle = new JointHandle(JointType.Gear, jointId);
            UnlinkJoint(handle, bodyA, bodyB);
            UnlinkJointFromBody(handle, bodyC);
            UnlinkJointFromBody(handle, bodyD);
            ClearJointSolverSet(handle);
            
            ClearJointConstraintGraphIndex(handle);
            MarkIslandDirty(bodyA);
            MarkIslandDirty(bodyB);
            MarkIslandDirty(bodyC);
            MarkIslandDirty(bodyD);
            RemoveJointFromSolverSets(JointType.Gear, jointId);
            _islandsDirty = true;
            return true;
        }

        public bool DestroyJoint(RopeJoint joint)
        {
            int index = joint.Index;
            if (index < 0 || index >= _ropeJointCount) return false;

            int jointId = joint.Id;
            Body bodyA = joint.BodyA;
            Body bodyB = joint.BodyB;
            int lastIndex = _ropeJointCount - 1;
            if (index != lastIndex)
            {
                RopeJoint moved = _ropeJoints[lastIndex];
                moved.Index = index;
                _ropeJoints[index] = moved;
                _ropeJointsData[index] = _ropeJointsData[lastIndex];
                _ropeJointIndexById[moved.Id] = index;
            }
            _ropeJoints[lastIndex] = null!;
            _ropeJointsData[lastIndex] = default;
            _ropeJointCount--;

            _jointIdPool.Free(jointId);
            _ropeJointIndexById.Remove(jointId);
            joint.Id = -1;
            joint.Index = -1;

            JointHandle handle = new JointHandle(JointType.Rope, jointId);
            UnlinkJoint(handle, bodyA, bodyB);
            ClearJointSolverSet(handle);
            
            ClearJointConstraintGraphIndex(handle);
            MarkIslandDirty(bodyA);
            MarkIslandDirty(bodyB);
            RemoveJointFromSolverSets(JointType.Rope, jointId);
            _islandsDirty = true;
            return true;
        }

        public bool DestroyJoint(FrictionJoint joint)
        {
            int index = joint.Index;
            if (index < 0 || index >= _frictionJointCount) return false;

            int jointId = joint.Id;
            Body bodyA = joint.BodyA;
            Body bodyB = joint.BodyB;
            int lastIndex = _frictionJointCount - 1;
            if (index != lastIndex)
            {
                FrictionJoint moved = _frictionJoints[lastIndex];
                moved.Index = index;
                _frictionJoints[index] = moved;
                _frictionJointsData[index] = _frictionJointsData[lastIndex];
                _frictionJointIndexById[moved.Id] = index;
            }
            _frictionJoints[lastIndex] = null!;
            _frictionJointsData[lastIndex] = default;
            _frictionJointCount--;

            _jointIdPool.Free(jointId);
            _frictionJointIndexById.Remove(jointId);
            joint.Id = -1;
            joint.Index = -1;

            JointHandle handle = new JointHandle(JointType.Friction, jointId);
            UnlinkJoint(handle, bodyA, bodyB);
            ClearJointSolverSet(handle);
            
            ClearJointConstraintGraphIndex(handle);
            MarkIslandDirty(bodyA);
            MarkIslandDirty(bodyB);
            RemoveJointFromSolverSets(JointType.Friction, jointId);
            _islandsDirty = true;
            return true;
        }

        private void DestroyDependentGearJoints(int referencedJointId)
        {
            for (int i = _gearJointCount - 1; i >= 0; --i)
            {
                ref GearJointData data = ref _gearJointsData[i];
                if (data.JointAId != referencedJointId && data.JointBId != referencedJointId)
                {
                    continue;
                }

                DestroyJoint(_gearJoints[i]);
            }
        }

        private bool RemoveJoint<TJoint>(System.Collections.Generic.List<TJoint> joints, TJoint joint) where TJoint : class
        {
            int index = joints.IndexOf(joint);
            bool removed = joints.Remove(joint);
            if (removed)
            {
                int jointId = GetJointId(joint);
                if (jointId >= 0)
                {
                    _jointIdPool.Free(jointId);
                    SetJointId(joint, -1);
                    RemoveJointIndexMap(GetJointType(joint), jointId);
                }
                Body bodyA = GetJointBodyA(joint);
                Body bodyB = GetJointBodyB(joint);
                JointHandle handle = new JointHandle(GetJointType(joint), jointId);
                UnlinkJoint(handle, bodyA, bodyB);
                ClearJointSolverSet(handle);
                
                MarkIslandDirty(bodyA);
                MarkIslandDirty(bodyB);
                if (index >= 0)
                {
                    RemoveJointFromSolverSets(GetJointType(joint), jointId);
                    UpdateJointIndexMap(GetJointType(joint), index);
                }
                _islandsDirty = true;
            }
            return removed;
        }

        internal void NotifyAwakeChanged(Body body)
        {
            if (body.Type == BodyType.Static)
            {
                return;
            }

            MarkIslandDirty(body);

            if (body.Awake)
            {
                body.SolverSetType = SolverSetType.Awake;
                if (_sleepingSet.Bodies.Remove(body))
                {
                    if (!_awakeSet.Bodies.Contains(body))
                    {
                        _awakeSet.Bodies.Add(body);
                    }
                }
                else if (!_awakeSet.Bodies.Contains(body))
                {
                    _awakeSet.Bodies.Add(body);
                }

                if (TryGetBodyIslandId(body, out int islandId) && islandId >= 0 && islandId < _lastIslands.Count)
                {
                    Island island = _lastIslands[islandId];
                    if (!island.IsAwake)
                    {
                        island.IsAwake = true;
                        WakeIslandFromSleepingSet(island);
                        RemoveSleepingSetForIsland(island);
                        for (int i = 0; i < island.Bodies.Count; ++i)
                        {
                            Body islandBody = island.Bodies[i];
                            if (islandBody.Type == BodyType.Static)
                            {
                                continue;
                            }
                            if (!islandBody.Awake)
                            {
                                islandBody.SetAwakeFromWorld(true, notifyWorld: false);
                            }
                            if (!_awakeSet.Bodies.Contains(islandBody))
                            {
                                _awakeSet.Bodies.Add(islandBody);
                            }
                            _sleepingSet.Bodies.Remove(islandBody);
                        }
                    }
                }
            }
            else
            {
                body.SolverSetType = SolverSetType.Sleeping;
                if (_awakeSet.Bodies.Remove(body))
                {
                    if (!_sleepingSet.Bodies.Contains(body))
                    {
                        _sleepingSet.Bodies.Add(body);
                    }
                }
                else if (!_sleepingSet.Bodies.Contains(body))
                {
                    _sleepingSet.Bodies.Add(body);
                }

                if (TryGetBodyIslandId(body, out int islandId) && islandId >= 0 && islandId < _lastIslands.Count)
                {
                    Island island = _lastIslands[islandId];
                    if (island.IsAwake)
                    {
                        island.IsAwake = false;
                        MoveIslandBetweenSets(island, _awakeSet, _sleepingSet);
                        CreateSleepingSetForIsland(island);
                    }
                }
            }

            _islandsDirty = true;
        }

        internal void NotifyBodyTypeChanged(Body body, BodyType previousType)
        {
            if (previousType == BodyType.Static)
            {
                _staticSet.Bodies.Remove(body);
            }

            if (body.Type == BodyType.Static && !_staticSet.Bodies.Contains(body))
            {
                _staticSet.Bodies.Add(body);
            }

            _islandsDirty = true;
        }

        private void MarkIslandDirty(Body body)
        {
            if (TryGetBodyIslandId(body, out int islandId))
            {
                _dirtyIslandIds.Add(islandId);
            }
            _islandsDirty = true;
        }

        public System.Collections.Generic.IReadOnlyList<Island> BuildIslands(bool awakeOnly = true)
        {
            if (!_islandsDirty && _lastIslands.Count > 0)
            {
                return _lastIslands;
            }

            _islandsDirty = false;

            IslandAdjacency adjacency = BuildIslandAdjacency();
            System.Collections.Generic.List<Island> islands;
            if (_lastIslands.Count == 0 || _dirtyIslandIds.Count == 0)
            {
                islands = BuildAllIslands(adjacency);
            }
            else
            {
                islands = BuildDirtyIslands(adjacency, awakeOnly);
            }

            AssignIslandIds(islands, adjacency.BodyIndex);
            UpdateIslandAwakeFlags(islands);
            NormalizeAwakeIslands(islands);
            RebuildSolverSets(islands);
            SortSolverSets();
            _lastIslands = islands;
            RebuildIslandIds();
            _dirtyIslandIds.Clear();

            return islands;
        }

        private sealed class IslandAdjacency
        {
            public readonly System.Collections.Generic.Dictionary<Body, int> BodyIndex;
            public readonly System.Collections.Generic.List<Contact>[] BodyContacts;
            public readonly System.Collections.Generic.List<JointHandle>[] BodyJoints;

            public IslandAdjacency(
                System.Collections.Generic.Dictionary<Body, int> bodyIndex,
                System.Collections.Generic.List<Contact>[] bodyContacts,
                System.Collections.Generic.List<JointHandle>[] bodyJoints)
            {
                BodyIndex = bodyIndex;
                BodyContacts = bodyContacts;
                BodyJoints = bodyJoints;
            }
        }

        private IslandAdjacency BuildIslandAdjacency()
        {
            System.Collections.Generic.Dictionary<Body, int> bodyIndex = new System.Collections.Generic.Dictionary<Body, int>(_bodies.Count);
            System.Collections.Generic.List<Contact>[] bodyContacts = new System.Collections.Generic.List<Contact>[_bodies.Count];
            System.Collections.Generic.List<JointHandle>[] bodyJoints = new System.Collections.Generic.List<JointHandle>[_bodies.Count];

            for (int i = 0; i < _bodies.Count; ++i)
            {
                Body body = _bodies[i];
                bodyIndex[body] = i;
                EnsureBodyAdjacency(body);
                bodyContacts[i] = new System.Collections.Generic.List<Contact>(_bodyContacts[body]);
                bodyContacts[i].Sort(CompareContacts);
                System.Collections.Generic.List<JointHandle>? joints = _bodyJoints[body.Id];
                bodyJoints[i] = joints != null
                    ? new System.Collections.Generic.List<JointHandle>(joints)
                    : new System.Collections.Generic.List<JointHandle>();
                bodyJoints[i].Sort(CompareJointHandles);
            }

            return new IslandAdjacency(bodyIndex, bodyContacts, bodyJoints);
        }

        private System.Collections.Generic.List<Island> BuildAllIslands(IslandAdjacency adjacency)
        {
            System.Collections.Generic.List<Island> islands = new System.Collections.Generic.List<Island>();
            bool[] bodyVisited = _arena.AllocateArray<bool>(_bodies.Count, "build_all_islands_body_visited");
            System.Collections.Generic.HashSet<Contact> contactAdded = new System.Collections.Generic.HashSet<Contact>();
            System.Collections.Generic.HashSet<JointHandle> jointAdded = new System.Collections.Generic.HashSet<JointHandle>();
            System.Collections.Generic.Stack<int> stack = new System.Collections.Generic.Stack<int>();
            try
            {
                for (int i = 0; i < _bodies.Count; ++i)
                {
                    Body body = _bodies[i];
                    if (body.Type == BodyType.Static)
                    {
                        continue;
                    }
                    if (bodyVisited[i])
                    {
                        continue;
                    }

                    Island island = new Island();
                    stack.Push(i);
                    bodyVisited[i] = true;
                    BuildIslandFromSeed(adjacency, island, stack, bodyVisited, contactAdded, jointAdded);
                    if (island.Bodies.Count > 0)
                    {
                        islands.Add(island);
                    }
                }
            }
            finally
            {
                _arena.FreeArray(bodyVisited);
            }

            return islands;
        }

        private System.Collections.Generic.List<Island> BuildDirtyIslands(IslandAdjacency adjacency, bool awakeOnly)
        {
            bool[] dirtyBodies = _arena.AllocateArray<bool>(_bodies.Count, "build_dirty_islands_dirty_bodies");
            System.Collections.Generic.List<int> splitSeeds = new System.Collections.Generic.List<int>();
            try
            {
                for (int i = 0; i < _lastIslands.Count; ++i)
                {
                    Island island = _lastIslands[i];
                    if (!island.IsAwake || island.ConstraintRemoveCount == 0)
                    {
                        continue;
                    }

                    for (int b = 0; b < island.Bodies.Count; ++b)
                    {
                        Body body = island.Bodies[b];
                        if (body.Type == BodyType.Static)
                        {
                            continue;
                        }
                        if (awakeOnly && !body.Awake)
                        {
                            continue;
                        }
                        if (adjacency.BodyIndex.TryGetValue(body, out int bodyIndex))
                        {
                            dirtyBodies[bodyIndex] = true;
                            splitSeeds.Add(bodyIndex);
                        }
                    }
                }

                foreach (int islandId in _dirtyIslandIds)
                {
                    if (islandId < 0 || islandId >= _lastIslands.Count)
                    {
                        continue;
                    }

                    Island island = _lastIslands[islandId];
                    if (awakeOnly && !island.IsAwake)
                    {
                        continue;
                    }
                    for (int b = 0; b < island.Bodies.Count; ++b)
                    {
                        Body body = island.Bodies[b];
                        if (body.Type == BodyType.Static)
                        {
                            continue;
                        }
                        if (awakeOnly && !body.Awake)
                        {
                            continue;
                        }
                        if (adjacency.BodyIndex.TryGetValue(body, out int bodyIndex))
                        {
                            dirtyBodies[bodyIndex] = true;
                        }
                    }
                }

                System.Collections.Generic.List<Island> islands = new System.Collections.Generic.List<Island>();
                bool[] bodyVisited = _arena.AllocateArray<bool>(_bodies.Count, "build_dirty_islands_body_visited");
                try
                {
                    System.Collections.Generic.HashSet<Contact> contactAdded = new System.Collections.Generic.HashSet<Contact>();
                    System.Collections.Generic.HashSet<JointHandle> jointAdded = new System.Collections.Generic.HashSet<JointHandle>();
                    System.Collections.Generic.Stack<int> stack = new System.Collections.Generic.Stack<int>();
                    if (splitSeeds.Count > 0 && _dirtyIslandIds.Count == 0)
                    {
                        splitSeeds.Sort();
                        for (int i = 0; i < splitSeeds.Count; ++i)
                        {
                            int seedIndex = splitSeeds[i];
                            if (bodyVisited[seedIndex])
                            {
                                continue;
                            }

                            Body body = _bodies[seedIndex];
                            if (body.Type == BodyType.Static)
                            {
                                continue;
                            }
                            if (awakeOnly && !body.Awake)
                            {
                                continue;
                            }

                            Island island = new Island();
                            stack.Push(seedIndex);
                            bodyVisited[seedIndex] = true;
                            BuildIslandFromSeed(adjacency, island, stack, bodyVisited, contactAdded, jointAdded);
                            if (island.Bodies.Count > 0)
                            {
                                island.ConstraintRemoveCount = 0;
                                islands.Add(island);
                            }
                        }

                        for (int i = 0; i < _lastIslands.Count; ++i)
                        {
                            Island island = _lastIslands[i];
                            bool rebuilt = false;
                            for (int b = 0; b < island.Bodies.Count; ++b)
                            {
                                Body body = island.Bodies[b];
                                if (adjacency.BodyIndex.TryGetValue(body, out int bodyIndex) && bodyVisited[bodyIndex])
                                {
                                    rebuilt = true;
                                    break;
                                }
                            }
                            if (!rebuilt)
                            {
                                islands.Add(island);
                            }
                        }

                        islands.Sort((a, b) => GetIslandSortKey(adjacency.BodyIndex, a).CompareTo(GetIslandSortKey(adjacency.BodyIndex, b)));
                        return islands;
                    }

                    if (splitSeeds.Count > 0)
                    {
                        splitSeeds.Sort();
                        for (int i = 0; i < splitSeeds.Count; ++i)
                        {
                            int seedIndex = splitSeeds[i];
                            if (bodyVisited[seedIndex])
                            {
                                continue;
                            }

                            Body body = _bodies[seedIndex];
                            if (body.Type == BodyType.Static)
                            {
                                continue;
                            }
                            if (awakeOnly && !body.Awake)
                            {
                                continue;
                            }

                            Island island = new Island();
                            stack.Push(seedIndex);
                            bodyVisited[seedIndex] = true;
                            BuildIslandFromSeed(adjacency, island, stack, bodyVisited, contactAdded, jointAdded);
                            if (island.Bodies.Count > 0)
                            {
                                island.ConstraintRemoveCount = 0;
                                islands.Add(island);
                            }
                        }
                    }

                    for (int i = 0; i < _bodies.Count; ++i)
                    {
                        if (!dirtyBodies[i])
                        {
                            continue;
                        }

                        Body body = _bodies[i];
                        if (body.Type == BodyType.Static)
                        {
                            continue;
                        }
                        if (awakeOnly && !body.Awake)
                        {
                            continue;
                        }
                        if (bodyVisited[i])
                        {
                            continue;
                        }

                        Island island = new Island();
                        stack.Push(i);
                        bodyVisited[i] = true;
                        BuildIslandFromSeed(adjacency, island, stack, bodyVisited, contactAdded, jointAdded);
                        if (island.Bodies.Count > 0)
                        {
                            island.ConstraintRemoveCount = 0;
                            islands.Add(island);
                        }
                    }

                    for (int i = 0; i < _lastIslands.Count; ++i)
                    {
                        Island island = _lastIslands[i];
                        bool rebuilt = false;
                        for (int b = 0; b < island.Bodies.Count; ++b)
                        {
                            Body body = island.Bodies[b];
                            if (adjacency.BodyIndex.TryGetValue(body, out int bodyIndex) && bodyVisited[bodyIndex])
                            {
                                rebuilt = true;
                                break;
                            }
                        }
                        if (!rebuilt)
                        {
                            islands.Add(island);
                        }
                    }

                    islands.Sort((a, b) => GetIslandSortKey(adjacency.BodyIndex, a).CompareTo(GetIslandSortKey(adjacency.BodyIndex, b)));
                    return islands;
                }
                finally
                {
                    _arena.FreeArray(bodyVisited);
                }
            }
            finally
            {
                _arena.FreeArray(dirtyBodies);
            }
        }

        private static int GetIslandSortKey(System.Collections.Generic.Dictionary<Body, int> bodyIndex, Island island)
        {
            int best = int.MaxValue;
            for (int i = 0; i < island.Bodies.Count; ++i)
            {
                if (bodyIndex.TryGetValue(island.Bodies[i], out int index))
                {
                    best = Math.Min(best, index);
                }
            }
            return best;
        }

        private bool HasPendingSplit(System.Collections.Generic.List<Island> islands)
        {
            for (int i = 0; i < islands.Count; ++i)
            {
                Island island = islands[i];
                if (island.IsAwake && island.ConstraintRemoveCount > 0 && island.Bodies.Count > 1)
                {
                    return true;
                }
            }
            return false;
        }

        private void SplitAwakeIslandsIfNeeded()
        {
            if (_lastIslands.Count == 0 || !HasPendingSplit(_lastIslands))
            {
                return;
            }

            IslandAdjacency adjacency = BuildIslandAdjacency();
            System.Collections.Generic.List<Island> islands = SplitAwakeIslands(adjacency, _lastIslands);

            AssignIslandIds(islands, adjacency.BodyIndex);
            UpdateIslandAwakeFlags(islands);
            NormalizeAwakeIslands(islands);
            RebuildSolverSets(islands);
            SortSolverSets();
            _lastIslands = islands;
            RebuildIslandIds();
            _dirtyIslandIds.Clear();
            _islandsDirty = false;
        }

        private System.Collections.Generic.List<Island> SplitAwakeIslands(IslandAdjacency adjacency, System.Collections.Generic.List<Island> islands)
        {
            System.Collections.Generic.List<Island> results = new System.Collections.Generic.List<Island>(islands.Count);
            bool[] inIsland = _arena.AllocateArray<bool>(_bodies.Count, "split_awake_islands_in_island");
            bool[] visited = _arena.AllocateArray<bool>(_bodies.Count, "split_awake_islands_visited");
            System.Collections.Generic.Stack<int> stack = new System.Collections.Generic.Stack<int>();
            try
            {
                for (int islandIndex = 0; islandIndex < islands.Count; ++islandIndex)
                {
                    Island island = islands[islandIndex];
                    if (!island.IsAwake || island.ConstraintRemoveCount == 0 || island.Bodies.Count <= 1)
                    {
                        results.Add(island);
                        island.ConstraintRemoveCount = 0;
                        continue;
                    }
                    Array.Clear(inIsland, 0, inIsland.Length);
                    Array.Clear(visited, 0, visited.Length);

                    System.Collections.Generic.List<int> seeds = new System.Collections.Generic.List<int>(island.Bodies.Count);
                    for (int i = 0; i < island.Bodies.Count; ++i)
                    {
                        Body body = island.Bodies[i];
                        if (adjacency.BodyIndex.TryGetValue(body, out int bodyIndex))
                        {
                            inIsland[bodyIndex] = true;
                            seeds.Add(bodyIndex);
                        }
                    }

                    seeds.Sort();
                    System.Collections.Generic.HashSet<Contact> contactAdded = new System.Collections.Generic.HashSet<Contact>();
                    System.Collections.Generic.HashSet<JointHandle> jointAdded = new System.Collections.Generic.HashSet<JointHandle>();

                    for (int i = 0; i < seeds.Count; ++i)
                    {
                        int seedIndex = seeds[i];
                        if (visited[seedIndex])
                        {
                            continue;
                        }

                        Island split = new Island();
                        stack.Push(seedIndex);
                        visited[seedIndex] = true;

                        while (stack.Count > 0)
                        {
                            int bodyIndexValue = stack.Pop();
                            if (!inIsland[bodyIndexValue])
                            {
                                continue;
                            }

                            Body node = _bodies[bodyIndexValue];
                            split.Bodies.Add(node);

                            foreach (Contact contact in adjacency.BodyContacts[bodyIndexValue])
                            {
                                if (!contact.IsTouching)
                                {
                                    continue;
                                }

                                Fixture? fixtureA = contact.FixtureA;
                                Fixture? fixtureB = contact.FixtureB;
                                if (fixtureA == null || fixtureB == null)
                                {
                                    continue;
                                }

                                int otherIndex = fixtureA.Body == node ? adjacency.BodyIndex[fixtureB.Body] : adjacency.BodyIndex[fixtureA.Body];
                                if (!inIsland[otherIndex])
                                {
                                    continue;
                                }

                                if (contactAdded.Add(contact))
                                {
                                    split.Contacts.Add(contact);
                                }

                                Body otherBody = _bodies[otherIndex];
                                if (otherBody.Type == BodyType.Static)
                                {
                                    continue;
                                }
                                if (!visited[otherIndex])
                                {
                                    visited[otherIndex] = true;
                                    stack.Push(otherIndex);
                                }
                            }

                            foreach (JointHandle handle in adjacency.BodyJoints[bodyIndexValue])
                            {
                                Body? otherBody = GetJointOtherBody(handle, node);
                                if (otherBody == null || otherBody.Type == BodyType.Static)
                                {
                                    continue;
                                }
                                if (node.Type != BodyType.Dynamic && otherBody.Type != BodyType.Dynamic)
                                {
                                    continue;
                                }

                                if (!adjacency.BodyIndex.TryGetValue(otherBody, out int otherIndex) || !inIsland[otherIndex])
                                {
                                    continue;
                                }

                                if (jointAdded.Add(handle))
                                {
                                    split.Joints.Add(handle);
                                }

                                if (!visited[otherIndex])
                                {
                                    visited[otherIndex] = true;
                                    stack.Push(otherIndex);
                                }
                            }
                        }

                        split.ConstraintRemoveCount = 0;
                        if (split.Bodies.Count > 0)
                        {
                            results.Add(split);
                        }
                    }
                }

                results.Sort((a, b) => GetIslandSortKey(adjacency.BodyIndex, a).CompareTo(GetIslandSortKey(adjacency.BodyIndex, b)));
                return results;
            }
            finally
            {
                _arena.FreeArray(visited);
                _arena.FreeArray(inIsland);
            }
        }

        private void BuildIslandFromSeed(
            IslandAdjacency adjacency,
            Island island,
            System.Collections.Generic.Stack<int> stack,
            bool[] bodyVisited,
            System.Collections.Generic.HashSet<Contact> contactAdded,
            System.Collections.Generic.HashSet<JointHandle> jointAdded)
        {
            while (stack.Count > 0)
            {
                int bodyIndexValue = stack.Pop();
                Body node = _bodies[bodyIndexValue];
                island.Bodies.Add(node);

                foreach (Contact contact in adjacency.BodyContacts[bodyIndexValue])
                {
                    if (!contact.IsTouching)
                    {
                        continue;
                    }
                    if (contactAdded.Add(contact))
                    {
                        island.Contacts.Add(contact);
                    }

                    Fixture? fixtureA = contact.FixtureA;
                    Fixture? fixtureB = contact.FixtureB;
                    if (fixtureA == null || fixtureB == null)
                    {
                        continue;
                    }

                    int otherIndex = fixtureA.Body == node ? adjacency.BodyIndex[fixtureB.Body] : adjacency.BodyIndex[fixtureA.Body];
                    Body otherBody = _bodies[otherIndex];
                    if (otherBody.Type == BodyType.Static)
                    {
                        continue;
                    }
                    if (bodyVisited[otherIndex])
                    {
                        continue;
                    }

                    bodyVisited[otherIndex] = true;
                    stack.Push(otherIndex);
                }

                foreach (JointHandle handle in adjacency.BodyJoints[bodyIndexValue])
                {
                    if (jointAdded.Add(handle))
                    {
                        island.Joints.Add(handle);
                    }

                    Body? otherBody = GetJointOtherBody(handle, node);
                    if (otherBody == null || otherBody.Type == BodyType.Static)
                    {
                        continue;
                    }
                    if (node.Type != BodyType.Dynamic && otherBody.Type != BodyType.Dynamic)
                    {
                        continue;
                    }

                    int otherIndex = adjacency.BodyIndex[otherBody];
                    if (bodyVisited[otherIndex])
                    {
                        continue;
                    }

                    bodyVisited[otherIndex] = true;
                    stack.Push(otherIndex);
                }
            }
        }

        private void UpdateIslandAwakeFlags(System.Collections.Generic.List<Island> islands)
        {
            for (int i = 0; i < islands.Count; ++i)
            {
                Island island = islands[i];
                bool islandAwake = !_def.EnableSleep;
                if (_def.EnableSleep)
                {
                    for (int j = 0; j < island.Bodies.Count; ++j)
                    {
                        if (island.Bodies[j].Awake)
                        {
                            islandAwake = true;
                            break;
                        }
                    }
                }
                island.IsAwake = islandAwake;
            }
        }

        private void NormalizeAwakeIslands(System.Collections.Generic.List<Island> islands)
        {
            if (!_def.EnableSleep)
            {
                return;
            }

            for (int i = 0; i < islands.Count; ++i)
            {
                Island island = islands[i];
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
                    if (!body.Awake)
                    {
                        body.SetAwakeFromWorld(true, notifyWorld: false);
                    }
                    body.SolverSetType = SolverSetType.Awake;
                }
            }
        }

        private void AssignIslandIds(System.Collections.Generic.List<Island> islands, System.Collections.Generic.Dictionary<Body, int> bodyIndex)
        {
            islands.Sort((a, b) => GetIslandSortKey(bodyIndex, a).CompareTo(GetIslandSortKey(bodyIndex, b)));
            _islandIdPool.Reset();
            for (int i = 0; i < islands.Count; ++i)
            {
                islands[i].Id = _islandIdPool.Alloc();
            }
        }

        private void RebuildSolverSets(System.Collections.Generic.List<Island> islands)
        {
            _staticSet.Bodies.Clear();
            _staticSet.Islands.Clear();
            _staticSet.Contacts.Clear();
            _staticSet.NonTouchingContacts.Clear();
            _staticSet.Joints.Clear();
            _disabledSet.Bodies.Clear();
            _disabledSet.Islands.Clear();
            _disabledSet.Joints.Clear();
            _awakeSet.Bodies.Clear();
            _awakeSet.Islands.Clear();
            _awakeSet.Contacts.Clear();
            _awakeSet.NonTouchingContacts.Clear();
            _awakeSet.Joints.Clear();
            _sleepingSet.Bodies.Clear();
            _sleepingSet.Islands.Clear();
            _sleepingSet.Contacts.Clear();
            _sleepingSet.NonTouchingContacts.Clear();
            _sleepingSet.Joints.Clear();
            _sleepingIslandSets.Clear();
            _sleepingSetIdByIslandId.Clear();
            ClearJointSolverSets();
            
            _solverSetIdPool.Reset();
            int sleepingSetBaseId = _solverSetIdPool.Alloc();
            System.Diagnostics.Debug.Assert(sleepingSetBaseId == 0);

            for (int i = 0; i < _bodies.Count; ++i)
            {
                Body body = _bodies[i];
                if (body.Type == BodyType.Static)
                {
                    body.SolverSetType = SolverSetType.Static;
                    _staticSet.Bodies.Add(body);
                    continue;
                }
                if (_def.EnableSleep && body.Awake == false)
                {
                    _sleepingSet.Bodies.Add(body);
                    body.SolverSetType = SolverSetType.Sleeping;
                }
                else
                {
                    _awakeSet.Bodies.Add(body);
                    body.SolverSetType = SolverSetType.Awake;
                }
            }

            for (int i = 0; i < islands.Count; ++i)
            {
                Island island = islands[i];
                if (island.IsAwake)
                {
                    AddIslandToSet(island, _awakeSet);
                }
                else
                {
                    AddIslandToSet(island, _sleepingSet);
                    CreateSleepingSetForIsland(island);
                }
            }

            for (int i = 0; i < _awakeSet.Islands.Count; ++i)
            {
                Island island = _awakeSet.Islands[i];
                for (int j = 0; j < island.Contacts.Count; ++j)
                {
                    island.Contacts[j].SolverSetType = SolverSetType.Awake;
                    island.Contacts[j].SolverSetId = 0;
                }
                for (int j = 0; j < island.Joints.Count; ++j)
                {
                                        SetJointSolverSet(island.Joints[j], SolverSetType.Awake, 0);
                }
            }

            for (int i = 0; i < _sleepingSet.Islands.Count; ++i)
            {
                Island island = _sleepingSet.Islands[i];
                int setId = GetSleepingSetId(island);
                for (int j = 0; j < island.Contacts.Count; ++j)
                {
                    island.Contacts[j].SolverSetType = SolverSetType.Sleeping;
                    island.Contacts[j].SolverSetId = setId;
                }
                for (int j = 0; j < island.Joints.Count; ++j)
                {
                                        SetJointSolverSet(island.Joints[j], SolverSetType.Sleeping, setId);
                }
            }

            for (int i = 0; i < _awakeSet.Contacts.Count; ++i)
            {
                _awakeSet.Contacts[i].SolverSetType = SolverSetType.Awake;
                _awakeSet.Contacts[i].SolverSetId = 0;
            }
            for (int i = 0; i < _sleepingSet.Contacts.Count; ++i)
            {
                _sleepingSet.Contacts[i].SolverSetType = SolverSetType.Sleeping;
            }
            for (int i = 0; i < _awakeSet.Joints.Count; ++i)
            {
                                SetJointSolverSet(_awakeSet.Joints[i], SolverSetType.Awake, 0);
            }
            for (int i = 0; i < _sleepingSet.Joints.Count; ++i)
            {
                JointHandle handle = _sleepingSet.Joints[i];
                if (!TryGetJointSolverSetType(handle, out _))
                {
                    SetJointSolverSet(handle, SolverSetType.Sleeping, FindSleepingSetIdForJoint(handle));
                }
            }

            for (int i = 0; i < _awakeSet.Islands.Count; ++i)
            {
                Island island = _awakeSet.Islands[i];
                for (int j = 0; j < island.Contacts.Count; ++j)
                {
                    Contact contact = island.Contacts[j];
                    if (!_awakeSet.Contacts.Contains(contact))
                    {
                        _awakeSet.Contacts.Add(contact);
                    }
                    if (!_sleepingSet.Contacts.Contains(contact))
                    {
                        continue;
                    }
                    if (TryGetBodyIslandId(contact.FixtureA!.Body, out int idA) &&
                        TryGetBodyIslandId(contact.FixtureB!.Body, out int idB) &&
                        idA == idB && idA >= 0 && idA < _lastIslands.Count &&
                        _lastIslands[idA].IsAwake)
                    {
                        _sleepingSet.Contacts.Remove(contact);
                    }
                }
            }
        }

        private void SortSolverSets()
        {
            System.Collections.Generic.Dictionary<Body, int> bodyIndex = new System.Collections.Generic.Dictionary<Body, int>(_bodies.Count);
            for (int i = 0; i < _bodies.Count; ++i)
            {
                bodyIndex[_bodies[i]] = i;
            }

            _staticSet.Islands.Sort(CompareIslands);
            _disabledSet.Islands.Sort(CompareIslands);
            _awakeSet.Islands.Sort(CompareIslands);
            _sleepingSet.Islands.Sort(CompareIslands);
            SortSolverSet(_staticSet);
            SortSolverSet(_disabledSet);
            SortSolverSet(_awakeSet);
            SortSolverSet(_sleepingSet);
        }

        private void SortSolverSet(SolverSet set)
        {
            set.Contacts.Sort(CompareContacts);
            set.NonTouchingContacts.Sort(CompareContacts);
            set.Joints.Sort(CompareJointHandles);
        }

        private void BuildConstraintGraph()
        {
            uint bodyCapacity = (uint)Math.Max(_bodyIdPool.Capacity, 8);
            _constraintGraph.Reset(bodyCapacity);
            ClearJointConstraintGraphIndices();

            for (int i = 0; i < _awakeSet.Contacts.Count; ++i)
            {
                Contact contact = _awakeSet.Contacts[i];
                if (contact.Manifold.PointCount == 0)
                {
                    contact.ColorIndex = -1;
                    contact.LocalIndex = -1;
                    continue;
                }
                AddContactToConstraintGraph(contact);
            }

            for (int i = 0; i < _awakeSet.Joints.Count; ++i)
            {
                AddJointToConstraintGraph(_awakeSet.Joints[i]);
            }

            ValidateConstraintGraph();
        }

        private void AddContactToConstraintGraph(Contact contact)
        {
            Fixture? fixtureA = contact.FixtureA;
            Fixture? fixtureB = contact.FixtureB;
            if (fixtureA == null || fixtureB == null)
            {
                return;
            }

            Body bodyA = fixtureA.Body;
            Body bodyB = fixtureB.Body;
            if (bodyA.Type == BodyType.Static && bodyB.Type == BodyType.Static)
            {
                contact.ColorIndex = -1;
                contact.LocalIndex = -1;
                contact.LocalIndex = -1;
                return;
            }

            int colorIndex = _constraintGraph.AssignContactColor(bodyA, bodyB);
            ConstraintGraph.GraphColor color = _constraintGraph.Colors[colorIndex];
            contact.ColorIndex = colorIndex;
            contact.LocalIndex = color.Contacts.Count;
            color.Contacts.Add(contact);
        }

        private void AddJointToConstraintGraph(JointHandle handle)
        {
            if (!TryGetJointBodies(handle, out Body bodyA, out Body bodyB))
            {
                return;
            }

            if (bodyA.Type == BodyType.Static && bodyB.Type == BodyType.Static)
            {
                SetJointConstraintGraphIndex(handle, -1, -1);
                return;
            }

            int colorIndex = _constraintGraph.AssignJointColor(bodyA, bodyB);
            ConstraintGraph.GraphColor color = _constraintGraph.Colors[colorIndex];
            SetJointConstraintGraphIndex(handle, colorIndex, color.Joints.Count);
            color.Joints.Add(handle);
        }

        private bool TryGetJointBodies(JointHandle handle, out Body bodyA, out Body bodyB)
        {
            bodyA = null!;
            bodyB = null!;
            if (!TryGetJointIndex(handle, out int index))
            {
                return false;
            }

            switch (handle.Type)
            {
                case JointType.Distance:
                    bodyA = GetBody(_distanceJointsData[index].BodyA);
                    bodyB = GetBody(_distanceJointsData[index].BodyB);
                    return true;
                case JointType.Revolute:
                    bodyA = GetBody(_revoluteJointsData[index].BodyA);
                    bodyB = GetBody(_revoluteJointsData[index].BodyB);
                    return true;
                case JointType.Prismatic:
                    bodyA = GetBody(_prismaticJointsData[index].BodyA);
                    bodyB = GetBody(_prismaticJointsData[index].BodyB);
                    return true;
                case JointType.Wheel:
                    bodyA = GetBody(_wheelJointsData[index].BodyA);
                    bodyB = GetBody(_wheelJointsData[index].BodyB);
                    return true;
                case JointType.Pulley:
                    bodyA = GetBody(_pulleyJointsData[index].BodyA);
                    bodyB = GetBody(_pulleyJointsData[index].BodyB);
                    return true;
                case JointType.Weld:
                    bodyA = GetBody(_weldJointsData[index].BodyA);
                    bodyB = GetBody(_weldJointsData[index].BodyB);
                    return true;
                case JointType.Motor:
                    bodyA = GetBody(_motorJointsData[index].BodyA);
                    bodyB = GetBody(_motorJointsData[index].BodyB);
                    return true;
                case JointType.Gear:
                    return TryGetGearRepresentativeBodies(ref _gearJointsData[index], out bodyA, out bodyB);
                case JointType.Rope:
                    bodyA = GetBody(_ropeJointsData[index].BodyA);
                    bodyB = GetBody(_ropeJointsData[index].BodyB);
                    return true;
                case JointType.Friction:
                    bodyA = GetBody(_frictionJointsData[index].BodyA);
                    bodyB = GetBody(_frictionJointsData[index].BodyB);
                    return true;
                default:
                    return false;
            }
        }

        private static int CompareIslands(Island? a, Island? b)
        {
            if (ReferenceEquals(a, b))
            {
                return 0;
            }
            if (a == null)
            {
                return -1;
            }
            if (b == null)
            {
                return 1;
            }

            return a.Id.CompareTo(b.Id);
        }

        private static int CompareContacts(Contact? a, Contact? b)
        {
            if (ReferenceEquals(a, b))
            {
                return 0;
            }
            if (a == null)
            {
                return -1;
            }
            if (b == null)
            {
                return 1;
            }

            int keyA0 = GetContactKeyPart(a, first: true);
            int keyA1 = GetContactKeyPart(a, first: false);
            int keyB0 = GetContactKeyPart(b, first: true);
            int keyB1 = GetContactKeyPart(b, first: false);

            int cmp = keyA0.CompareTo(keyB0);
            return cmp != 0 ? cmp : keyA1.CompareTo(keyB1);
        }

        private static int GetContactKeyPart(Contact contact, bool first)
        {
            if (contact.FixtureA == null || contact.FixtureB == null)
            {
                return 0;
            }
            int a = contact.FixtureA.ProxyId;
            int b = contact.FixtureB.ProxyId;
            int min = a < b ? a : b;
            int max = a < b ? b : a;
            return first ? min : max;
        }

        private static int CompareJointHandles(JointHandle a, JointHandle b)
        {
            int typeCmp = a.Type.CompareTo(b.Type);
            return typeCmp != 0 ? typeCmp : a.Id.CompareTo(b.Id);
        }

        private void EnsureBodyAdjacency(Body body)
        {
            if (!_bodyContacts.ContainsKey(body))
            {
                _bodyContacts[body] = new System.Collections.Generic.List<Contact>();
            }
            if (_bodyJoints[body.Id] == null)
            {
                _bodyJoints[body.Id] = new System.Collections.Generic.List<JointHandle>();
            }
        }

        private void LinkContact(Contact contact)
        {
            if (contact.FixtureA == null || contact.FixtureB == null)
            {
                return;
            }
            if (contact.FixtureA.IsSensor || contact.FixtureB.IsSensor)
            {
                return;
            }

            Body bodyA = contact.FixtureA.Body;
            Body bodyB = contact.FixtureB.Body;
            EnsureBodyAdjacency(bodyA);
            EnsureBodyAdjacency(bodyB);

            if (contact.EdgeIdA < 0)
            {
                ContactEdge edgeA = new ContactEdge
                {
                    Id = _contactEdgeIdPool.Alloc(),
                    Contact = contact,
                    Body = bodyA,
                    Other = bodyB,
                    Next = bodyA.ContactList
                };
                if (bodyA.ContactList != null)
                {
                    bodyA.ContactList.Prev = edgeA;
                }
                bodyA.ContactList = edgeA;
                bodyA.ContactEdgeCount++;
                contact.EdgeA = edgeA;
                contact.EdgeIdA = edgeA.Id;
            }
            if (contact.EdgeIdB < 0)
            {
                ContactEdge edgeB = new ContactEdge
                {
                    Id = _contactEdgeIdPool.Alloc(),
                    Contact = contact,
                    Body = bodyB,
                    Other = bodyA,
                    Next = bodyB.ContactList
                };
                if (bodyB.ContactList != null)
                {
                    bodyB.ContactList.Prev = edgeB;
                }
                bodyB.ContactList = edgeB;
                bodyB.ContactEdgeCount++;
                contact.EdgeB = edgeB;
                contact.EdgeIdB = edgeB.Id;
            }

            System.Collections.Generic.List<Contact> listA = _bodyContacts[bodyA];
            if (!listA.Contains(contact))
            {
                listA.Add(contact);
            }

            System.Collections.Generic.List<Contact> listB = _bodyContacts[bodyB];
            if (!listB.Contains(contact))
            {
                listB.Add(contact);
            }

            // A new contact may need to merge two previously-disjoint islands. The
            // dirty-island optimisation in BuildIslands only knows about bodies in
            // _lastIslands, so it can miss the new connectivity. Clear the per-
            // island dirty index so BuildIslands falls back to BuildAllIslands.
            _dirtyIslandIds.Clear();
            MarkIslandDirty(bodyA);
            MarkIslandDirty(bodyB);
            _dirtyIslandIds.Clear();
            _islandsDirty = true;
        }

        private void UnlinkContact(Contact contact)
        {
            if (contact.FixtureA == null || contact.FixtureB == null)
            {
                return;
            }

            Body bodyA = contact.FixtureA.Body;
            Body bodyB = contact.FixtureB.Body;
            if (_bodyContacts.TryGetValue(bodyA, out System.Collections.Generic.List<Contact>? listA))
            {
                listA.Remove(contact);
            }
            if (_bodyContacts.TryGetValue(bodyB, out System.Collections.Generic.List<Contact>? listB))
            {
                listB.Remove(contact);
            }

            if (contact.EdgeA != null)
            {
                DetachContactEdge(contact.EdgeA);
                _contactEdgeIdPool.Free(contact.EdgeA.Id);
                contact.EdgeA = null;
                contact.EdgeIdA = -1;
            }
            if (contact.EdgeB != null)
            {
                DetachContactEdge(contact.EdgeB);
                _contactEdgeIdPool.Free(contact.EdgeB.Id);
                contact.EdgeB = null;
                contact.EdgeIdB = -1;
            }

            IncrementConstraintRemoveCount(bodyA, bodyB);
            MarkIslandDirty(bodyA);
            MarkIslandDirty(bodyB);
        }

        private static void DetachContactEdge(ContactEdge edge)
        {
            Body body = edge.Body;
            if (edge.Prev != null)
            {
                edge.Prev.Next = edge.Next;
            }
            else
            {
                body.ContactList = edge.Next;
            }
            if (edge.Next != null)
            {
                edge.Next.Prev = edge.Prev;
            }
            body.ContactEdgeCount = Math.Max(0, body.ContactEdgeCount - 1);
            edge.Prev = null;
            edge.Next = null;
        }

        private void LinkJoint(JointHandle handle, Body bodyA, Body bodyB)
        {
            LinkJointToBody(handle, bodyA);
            if (!ReferenceEquals(bodyA, bodyB))
            {
                LinkJointToBody(handle, bodyB);
            }
        }

        private void UnlinkJoint(JointHandle handle, Body bodyA, Body bodyB)
        {
            UnlinkJointFromBody(handle, bodyA);
            if (!ReferenceEquals(bodyA, bodyB))
            {
                UnlinkJointFromBody(handle, bodyB);
            }

            IncrementConstraintRemoveCount(bodyA, bodyB);
            MarkIslandDirty(bodyA);
            MarkIslandDirty(bodyB);
        }

        private void LinkJointToBody(JointHandle handle, Body body)
        {
            EnsureBodyAdjacency(body);
            System.Collections.Generic.List<JointHandle> list = _bodyJoints[body.Id];
            if (!list.Contains(handle))
            {
                list.Add(handle);
            }
        }

        private void UnlinkJointFromBody(JointHandle handle, Body body)
        {
            System.Collections.Generic.List<JointHandle>? list = _bodyJoints[body.Id];
            if (list != null)
            {
                list.Remove(handle);
            }
        }

        private void UpdateJointIndexMap(JointType type, int removedIndex)
        {
            switch (type)
            {
                // case JointType.Distance:
                //    UpdateIndexMap(_distanceJoints, _distanceJointIndexById, removedIndex, joint => joint.Id);
                //    break;
                // case JointType.Revolute:
                //    UpdateIndexMap(_revoluteJoints, _revoluteJointIndexById, removedIndex, joint => joint.Id);
                //    break;
                case JointType.Prismatic:
                    for (int i = removedIndex; i < _prismaticJointCount; ++i)
                    {
                        _prismaticJointIndexById[_prismaticJoints[i].Id] = i;
                    }
                    break;
                case JointType.Wheel:
                    for (int i = removedIndex; i < _wheelJointCount; ++i)
                    {
                        _wheelJointIndexById[_wheelJoints[i].Id] = i;
                    }
                    break;
                case JointType.Pulley:
                    for (int i = removedIndex; i < _pulleyJointCount; ++i)
                    {
                        _pulleyJointIndexById[_pulleyJoints[i].Id] = i;
                    }
                    break;
                case JointType.Weld:
                    for (int i = removedIndex; i < _weldJointCount; ++i)
                    {
                        _weldJointIndexById[_weldJoints[i].Id] = i;
                    }
                    break;
                case JointType.Motor:
                    for (int i = removedIndex; i < _motorJointCount; ++i)
                    {
                        _motorJointIndexById[_motorJoints[i].Id] = i;
                    }
                    break;
                case JointType.Gear:
                    for (int i = removedIndex; i < _gearJointCount; ++i)
                    {
                        _gearJointIndexById[_gearJoints[i].Id] = i;
                    }
                    break;
                case JointType.Rope:
                    for (int i = removedIndex; i < _ropeJointCount; ++i)
                    {
                        _ropeJointIndexById[_ropeJoints[i].Id] = i;
                    }
                    break;
                case JointType.Friction:
                    for (int i = removedIndex; i < _frictionJointCount; ++i)
                    {
                        _frictionJointIndexById[_frictionJoints[i].Id] = i;
                    }
                    break;
            }
        }

        private void RemoveJointIndexMap(JointType type, int jointId)
        {
            switch (type)
            {
                case JointType.Distance:
                    _distanceJointIndexById.Remove(jointId);
                    break;
                case JointType.Revolute:
                    _revoluteJointIndexById.Remove(jointId);
                    break;
                case JointType.Prismatic:
                    _prismaticJointIndexById.Remove(jointId);
                    break;
                case JointType.Wheel:
                    _wheelJointIndexById.Remove(jointId);
                    break;
                case JointType.Pulley:
                    _pulleyJointIndexById.Remove(jointId);
                    break;
                case JointType.Weld:
                    _weldJointIndexById.Remove(jointId);
                    break;
                case JointType.Motor:
                    _motorJointIndexById.Remove(jointId);
                    break;
                case JointType.Gear:
                    _gearJointIndexById.Remove(jointId);
                    break;
                case JointType.Rope:
                    _ropeJointIndexById.Remove(jointId);
                    break;
                case JointType.Friction:
                    _frictionJointIndexById.Remove(jointId);
                    break;
            }
        }

        private static void UpdateIndexMap<TJoint>(
            System.Collections.Generic.List<TJoint> list,
            System.Collections.Generic.Dictionary<int, int> indexById,
            int removedIndex,
            System.Func<TJoint, int> idSelector)
            where TJoint : class
        {
            for (int i = removedIndex; i < list.Count; ++i)
            {
                int id = idSelector(list[i]);
                indexById[id] = i;
            }
        }

        private void IncrementConstraintRemoveCount(Body bodyA, Body bodyB)
        {
            Body? candidate = bodyA.Type != BodyType.Static ? bodyA : bodyB.Type != BodyType.Static ? bodyB : null;
            if (candidate == null)
            {
                return;
            }

            if (TryGetBodyIslandId(candidate, out int islandIndex) &&
                islandIndex >= 0 &&
                islandIndex < _lastIslands.Count)
            {
                Island island = _lastIslands[islandIndex];
                if (island.IsAwake)
                {
                    island.ConstraintRemoveCount += 1;
                }
            }
        }

        private SolverSetType GetJointSolverSetType(Body bodyA, Body bodyB)
        {
            if (!_def.EnableSleep)
            {
                return SolverSetType.Awake;
            }

            if (bodyA.SolverSetType == SolverSetType.Awake || bodyB.SolverSetType == SolverSetType.Awake)
            {
                return SolverSetType.Awake;
            }

            return SolverSetType.Sleeping;
        }

        private int GetJointSolverSetId(Body bodyA, Body bodyB)
        {
            if (!_def.EnableSleep)
            {
                return 0;
            }

            if (!TryGetBodyIslandId(bodyA, out int islandId) &&
                !TryGetBodyIslandId(bodyB, out islandId))
            {
                return 0;
            }

            return islandId >= 0 && islandId < _lastIslands.Count ? _lastIslands[islandId].Id : 0;
        }

        private void EnsureJointSolverSetCapacity(int capacity)
        {
            if (capacity <= _jointSolverSetUsedById.Length)
            {
                return;
            }

            int previousCapacity = _jointSolverSetUsedById.Length;
            int newCapacity = Math.Max(capacity, Math.Max(256, previousCapacity * 2));
            Array.Resize(ref _jointSolverSetTypesById, newCapacity);
            Array.Resize(ref _jointSolverSetIdsById, newCapacity);
            Array.Resize(ref _jointSolverSetUsedById, newCapacity);
            Array.Clear(_jointSolverSetTypesById, previousCapacity, newCapacity - previousCapacity);
            Array.Clear(_jointSolverSetIdsById, previousCapacity, newCapacity - previousCapacity);
            Array.Clear(_jointSolverSetUsedById, previousCapacity, newCapacity - previousCapacity);
        }

        private void SetJointSolverSet(JointHandle handle, SolverSetType type, int setId)
        {
            int jointId = handle.Id;
            if (jointId < 0)
            {
                return;
            }

            EnsureJointSolverSetCapacity(jointId + 1);
            _jointSolverSetTypesById[jointId] = type;
            _jointSolverSetIdsById[jointId] = setId;
            _jointSolverSetUsedById[jointId] = true;
        }

        private void ClearJointSolverSet(JointHandle handle)
        {
            int jointId = handle.Id;
            if ((uint)jointId >= (uint)_jointSolverSetUsedById.Length)
            {
                return;
            }

            _jointSolverSetUsedById[jointId] = false;
            _jointSolverSetIdsById[jointId] = 0;
            _jointSolverSetTypesById[jointId] = default;
        }

        private void ClearJointSolverSets()
        {
            Array.Clear(_jointSolverSetUsedById, 0, _jointSolverSetUsedById.Length);
            Array.Clear(_jointSolverSetIdsById, 0, _jointSolverSetIdsById.Length);
            Array.Clear(_jointSolverSetTypesById, 0, _jointSolverSetTypesById.Length);
        }

        private void EnsureJointConstraintGraphCapacity(int capacity)
        {
            if (capacity <= _jointConstraintGraphUsedById.Length)
            {
                return;
            }

            int previousCapacity = _jointConstraintGraphUsedById.Length;
            int newCapacity = Math.Max(capacity, Math.Max(256, previousCapacity * 2));
            Array.Resize(ref _jointColorIndicesById, newCapacity);
            Array.Resize(ref _jointLocalIndicesById, newCapacity);
            Array.Resize(ref _jointConstraintGraphUsedById, newCapacity);
            Array.Fill(_jointColorIndicesById, -1, previousCapacity, newCapacity - previousCapacity);
            Array.Fill(_jointLocalIndicesById, -1, previousCapacity, newCapacity - previousCapacity);
            Array.Clear(_jointConstraintGraphUsedById, previousCapacity, newCapacity - previousCapacity);
        }

        private void SetJointConstraintGraphIndex(JointHandle handle, int colorIndex, int localIndex)
        {
            int jointId = handle.Id;
            if (jointId < 0)
            {
                return;
            }

            EnsureJointConstraintGraphCapacity(jointId + 1);
            _jointColorIndicesById[jointId] = colorIndex;
            _jointLocalIndicesById[jointId] = localIndex;
            _jointConstraintGraphUsedById[jointId] = true;
        }

        private void ClearJointConstraintGraphIndex(JointHandle handle)
        {
            int jointId = handle.Id;
            if ((uint)jointId >= (uint)_jointConstraintGraphUsedById.Length)
            {
                return;
            }

            _jointConstraintGraphUsedById[jointId] = false;
            _jointColorIndicesById[jointId] = -1;
            _jointLocalIndicesById[jointId] = -1;
        }

        private void ClearJointConstraintGraphIndices()
        {
            Array.Fill(_jointColorIndicesById, -1);
            Array.Fill(_jointLocalIndicesById, -1);
            Array.Clear(_jointConstraintGraphUsedById, 0, _jointConstraintGraphUsedById.Length);
        }

        private bool TryGetJointConstraintGraphIndex(JointHandle handle, out int colorIndex, out int localIndex)
        {
            int jointId = handle.Id;
            if ((uint)jointId < (uint)_jointConstraintGraphUsedById.Length && _jointConstraintGraphUsedById[jointId])
            {
                colorIndex = _jointColorIndicesById[jointId];
                localIndex = _jointLocalIndicesById[jointId];
                return true;
            }

            colorIndex = -1;
            localIndex = -1;
            return false;
        }

        internal bool TryGetJointSolverSetType(JointHandle handle, out SolverSetType type)
        {
            int jointId = handle.Id;
            if ((uint)jointId < (uint)_jointSolverSetUsedById.Length && _jointSolverSetUsedById[jointId])
            {
                type = _jointSolverSetTypesById[jointId];
                return true;
            }

            type = default;
            return false;
        }

        internal bool TryGetJointSolverSetId(JointHandle handle, out int setId)
        {
            int jointId = handle.Id;
            if ((uint)jointId < (uint)_jointSolverSetUsedById.Length && _jointSolverSetUsedById[jointId])
            {
                setId = _jointSolverSetIdsById[jointId];
                return true;
            }

            setId = 0;
            return false;
        }

        private void TryAddContactToSleepingSet(Contact contact, Body bodyA, Body bodyB)
        {
            if (!_def.EnableSleep)
            {
                return;
            }
            if (!TryGetBodyIslandId(bodyA, out int islandId) &&
                !TryGetBodyIslandId(bodyB, out islandId))
            {
                return;
            }
            if (islandId < 0 || islandId >= _lastIslands.Count)
            {
                return;
            }

            Island island = _lastIslands[islandId];
            if (island.IsAwake)
            {
                return;
            }
            if (_sleepingSetIdByIslandId.TryGetValue(island.Id, out int setId) &&
                _sleepingIslandSets.TryGetValue(setId, out SolverSet? set))
            {
                if (!set.Contacts.Contains(contact))
                {
                    set.Contacts.Add(contact);
                    contact.SolverSetType = SolverSetType.Sleeping;
                    contact.SolverSetId = setId;
                }
            }
        }

        private void TryAddJointToSleepingSet(JointHandle handle, Body bodyA, Body bodyB)
        {
            if (!_def.EnableSleep)
            {
                return;
            }
            if (!TryGetBodyIslandId(bodyA, out int islandId) &&
                !TryGetBodyIslandId(bodyB, out islandId))
            {
                return;
            }
            if (islandId < 0 || islandId >= _lastIslands.Count)
            {
                return;
            }

            Island island = _lastIslands[islandId];
            if (island.IsAwake)
            {
                return;
            }
            if (_sleepingSetIdByIslandId.TryGetValue(island.Id, out int setId) &&
                _sleepingIslandSets.TryGetValue(setId, out SolverSet? set))
            {
                if (!set.Joints.Contains(handle))
                {
                    set.Joints.Add(handle);
                                        SetJointSolverSet(handle, SolverSetType.Sleeping, setId);
                }
            }
        }

        private static Body GetJointBodyA(object joint)
        {
            return joint switch
            {
                DistanceJoint j => j.BodyA,
                RevoluteJoint j => j.BodyA,
                PrismaticJoint j => j.BodyA,
                WheelJoint j => j.BodyA,
                PulleyJoint j => j.BodyA,
                WeldJoint j => j.BodyA,
                MotorJoint j => j.BodyA,
                GearJoint j => j.BodyA,
                RopeJoint j => j.BodyA,
                FrictionJoint j => j.BodyA,
                _ => throw new System.InvalidOperationException("Unknown joint type.")
            };
        }

        private static Body GetJointBodyB(object joint)
        {
            return joint switch
            {
                DistanceJoint j => j.BodyB,
                RevoluteJoint j => j.BodyB,
                PrismaticJoint j => j.BodyB,
                WheelJoint j => j.BodyB,
                PulleyJoint j => j.BodyB,
                WeldJoint j => j.BodyB,
                MotorJoint j => j.BodyB,
                GearJoint j => j.BodyB,
                RopeJoint j => j.BodyB,
                FrictionJoint j => j.BodyB,
                _ => throw new System.InvalidOperationException("Unknown joint type.")
            };
        }

        private void HandleJointCreationAwakeState(Body bodyA, Body bodyB)
        {
            if (!_def.EnableSleep)
            {
                return;
            }

            if (bodyA.Type == BodyType.Static && bodyB.Type == BodyType.Static)
            {
                return;
            }

            bool awakeA = bodyA.Type != BodyType.Static && bodyA.Awake;
            bool awakeB = bodyB.Type != BodyType.Static && bodyB.Awake;

            if (awakeA && !awakeB)
            {
                bodyB.SetAwake(true);
                return;
            }

            if (!awakeA && awakeB)
            {
                bodyA.SetAwake(true);
                return;
            }

            if (!awakeA && !awakeB)
            {
                MergeSleepingIslands(bodyA, bodyB);
            }
        }

        private void MergeSleepingIslands(Body bodyA, Body bodyB)
        {
            BuildIslands(awakeOnly: false);

            if (!TryGetBodyIslandId(bodyA, out int islandAId) ||
                !TryGetBodyIslandId(bodyB, out int islandBId) ||
                islandAId == islandBId ||
                islandAId < 0 || islandBId < 0 ||
                islandAId >= _lastIslands.Count || islandBId >= _lastIslands.Count)
            {
                return;
            }

            Island islandA = _lastIslands[islandAId];
            Island islandB = _lastIslands[islandBId];

            if (islandA.IsAwake || islandB.IsAwake)
            {
                return;
            }

            bool aIsPrimary = islandA.Bodies.Count >= islandB.Bodies.Count;
            if (islandA.Bodies.Count == islandB.Bodies.Count)
            {
                aIsPrimary = islandA.Id <= islandB.Id;
            }

            Island primary = aIsPrimary ? islandA : islandB;
            Island secondary = primary == islandA ? islandB : islandA;

            RemoveIslandFromSet(secondary, _sleepingSet);
            _lastIslands.Remove(secondary);
            if (_sleepingSetIdByIslandId.TryGetValue(secondary.Id, out int secondarySetId) &&
                _sleepingIslandSets.TryGetValue(secondarySetId, out SolverSet? secondarySet))
            {
                _sleepingIslandSets.Remove(secondarySetId);
                _sleepingSetIdByIslandId.Remove(secondary.Id);
                _solverSetIdPool.Free(secondarySetId);
                if (!_sleepingSetIdByIslandId.TryGetValue(primary.Id, out int primarySetId) ||
                    !_sleepingIslandSets.TryGetValue(primarySetId, out SolverSet? primarySet))
                {
                    CreateSleepingSetForIsland(primary);
                    primarySetId = _sleepingSetIdByIslandId[primary.Id];
                    _sleepingIslandSets.TryGetValue(primarySetId, out primarySet);
                }
                if (primarySet != null)
                {
                    AddIslandToSet(secondary, primarySet);
                    AddIslandBodiesToSet(secondary, primarySet);
                    SortSolverSet(primarySet);
                    for (int i = 0; i < secondary.Contacts.Count; ++i)
                    {
                        secondary.Contacts[i].SolverSetType = SolverSetType.Sleeping;
                        secondary.Contacts[i].SolverSetId = primarySetId;
                    }
                    for (int i = 0; i < secondary.Joints.Count; ++i)
                    {
                                                SetJointSolverSet(secondary.Joints[i], SolverSetType.Sleeping, primarySetId);
                    }
                }
            }
            else if (_sleepingSetIdByIslandId.TryGetValue(primary.Id, out int existingPrimarySetId) &&
                     _sleepingIslandSets.TryGetValue(existingPrimarySetId, out SolverSet? existingPrimarySet))
            {
                AddIslandToSet(secondary, existingPrimarySet);
                AddIslandBodiesToSet(secondary, existingPrimarySet);
                SortSolverSet(existingPrimarySet);
                for (int i = 0; i < secondary.Contacts.Count; ++i)
                {
                    secondary.Contacts[i].SolverSetType = SolverSetType.Sleeping;
                    secondary.Contacts[i].SolverSetId = existingPrimarySetId;
                }
                for (int i = 0; i < secondary.Joints.Count; ++i)
                {
                                        SetJointSolverSet(secondary.Joints[i], SolverSetType.Sleeping, existingPrimarySetId);
                }
            }

            for (int i = 0; i < secondary.Bodies.Count; ++i)
            {
                Body body = secondary.Bodies[i];
                primary.Bodies.Add(body);
            }

            for (int i = 0; i < secondary.Contacts.Count; ++i)
            {
                Contact contact = secondary.Contacts[i];
                primary.Contacts.Add(contact);
            }

            for (int i = 0; i < secondary.Joints.Count; ++i)
            {
                JointHandle handle = secondary.Joints[i];
                primary.Joints.Add(handle);
            }

            primary.ConstraintRemoveCount += secondary.ConstraintRemoveCount;

            AddIslandToSet(primary, _sleepingSet);
            RebuildIslandIds();
        }


        private void RebuildIslandIds()
        {
            Array.Fill(_bodyIslandIdsByBody, -1, 0, _bodyCount);
            for (int i = 0; i < _lastIslands.Count; ++i)
            {
                Island island = _lastIslands[i];
                for (int j = 0; j < island.Bodies.Count; ++j)
                {
                    _bodyIslandIdsByBody[island.Bodies[j].Id] = i;
                }
            }
        }

        private void AddIslandToSet(Island island, SolverSet set)
        {
            if (!set.Islands.Contains(island))
            {
                set.Islands.Add(island);
            }

            for (int i = 0; i < island.Contacts.Count; ++i)
            {
                Contact contact = island.Contacts[i];
                if (!set.Contacts.Contains(contact))
                {
                    set.Contacts.Add(contact);
                }
            }

            for (int i = 0; i < island.Joints.Count; ++i)
            {
                JointHandle handle = island.Joints[i];
                if (!set.Joints.Contains(handle))
                {
                    set.Joints.Add(handle);
                }
            }
        }

        private void AddIslandBodiesToSet(Island island, SolverSet set)
        {
            for (int i = 0; i < island.Bodies.Count; ++i)
            {
                Body body = island.Bodies[i];
                if (body.Type == BodyType.Static)
                {
                    continue;
                }
                if (!set.Bodies.Contains(body))
                {
                    set.Bodies.Add(body);
                }
            }
        }

        private void RemoveIslandFromSet(Island island, SolverSet set)
        {
            set.Islands.Remove(island);

            for (int i = 0; i < island.Contacts.Count; ++i)
            {
                set.Contacts.Remove(island.Contacts[i]);
            }

            for (int i = 0; i < island.Joints.Count; ++i)
            {
                set.Joints.Remove(island.Joints[i]);
            }
        }

        private void WakeIslandFromSleepingSet(Island island)
        {
            MoveIslandBetweenSets(island, _sleepingSet, _awakeSet);

            if (_sleepingSetIdByIslandId.TryGetValue(island.Id, out int setId) &&
                _sleepingIslandSets.TryGetValue(setId, out SolverSet? islandSet))
            {
                for (int i = 0; i < islandSet.Bodies.Count; ++i)
                {
                    Body body = islandSet.Bodies[i];
                    if (body.Type == BodyType.Static)
                    {
                        continue;
                    }
                    body.SolverSetType = SolverSetType.Awake;
                    if (!_awakeSet.Bodies.Contains(body))
                    {
                        _awakeSet.Bodies.Add(body);
                    }
                    _sleepingSet.Bodies.Remove(body);
                }

                for (int i = 0; i < islandSet.Contacts.Count; ++i)
                {
                    Contact contact = islandSet.Contacts[i];
                    if (!_awakeSet.Contacts.Contains(contact))
                    {
                        _awakeSet.Contacts.Add(contact);
                    }
                    _sleepingSet.Contacts.Remove(contact);
                    contact.SolverSetType = SolverSetType.Awake;
                    contact.SolverSetId = 0;
                }

                for (int i = 0; i < islandSet.Joints.Count; ++i)
                {
                    JointHandle handle = islandSet.Joints[i];
                    if (!_awakeSet.Joints.Contains(handle))
                    {
                        _awakeSet.Joints.Add(handle);
                    }
                    _sleepingSet.Joints.Remove(handle);
                                        SetJointSolverSet(handle, SolverSetType.Awake, 0);
                }
            }

            SortSolverSet(_awakeSet);
        }

        private void CreateSleepingSetForIsland(Island island)
        {
            SolverSet set = new SolverSet();
            AddIslandToSet(island, set);
            AddIslandBodiesToSet(island, set);
            SortSolverSet(set);
            int setId = _solverSetIdPool.Alloc();
            _sleepingIslandSets[setId] = set;
            _sleepingSetIdByIslandId[island.Id] = setId;
            for (int i = 0; i < set.Contacts.Count; ++i)
            {
                set.Contacts[i].SolverSetType = SolverSetType.Sleeping;
                set.Contacts[i].SolverSetId = setId;
            }
            for (int i = 0; i < set.Joints.Count; ++i)
            {
                                SetJointSolverSet(set.Joints[i], SolverSetType.Sleeping, setId);
            }
        }

        private void RemoveSleepingSetForIsland(Island island)
        {
            if (_sleepingSetIdByIslandId.TryGetValue(island.Id, out int setId))
            {
                _sleepingSetIdByIslandId.Remove(island.Id);
                if (_sleepingIslandSets.Remove(setId))
                {
                    _solverSetIdPool.Free(setId);
                }
            }
        }

        private void RemoveJointFromSolverSets(JointType type, int jointId)
        {
            JointHandle handle = new JointHandle(type, jointId);
            _awakeSet.Joints.Remove(handle);
            _sleepingSet.Joints.Remove(handle);
            RemoveJointFromSleepingSets(handle);
            ClearJointSolverSet(handle);
        }

        private void RemoveJointFromSleepingSets(JointHandle handle)
        {
            if (TryGetJointSolverSetId(handle, out int setId) &&
                setId != 0 &&
                _sleepingIslandSets.TryGetValue(setId, out SolverSet? set))
            {
                set.Joints.Remove(handle);
                return;
            }

            foreach (System.Collections.Generic.KeyValuePair<int, SolverSet> pair in _sleepingIslandSets)
            {
                pair.Value.Joints.Remove(handle);
            }
        }

        private int FindSleepingSetIdForJoint(JointHandle handle)
        {
            foreach (System.Collections.Generic.KeyValuePair<int, SolverSet> pair in _sleepingIslandSets)
            {
                if (pair.Value.Joints.Contains(handle))
                {
                    return pair.Key;
                }
            }

            return 0;
        }

        private int GetSleepingSetId(Island island)
        {
            return _sleepingSetIdByIslandId.TryGetValue(island.Id, out int setId) ? setId : 0;
        }

        private void RemoveContactFromSleepingSets(Contact contact)
        {
            int setId = contact.SolverSetId;
            if (setId != 0 && _sleepingIslandSets.TryGetValue(setId, out SolverSet? set))
            {
                set.Contacts.Remove(contact);
                return;
            }

            foreach (System.Collections.Generic.KeyValuePair<int, SolverSet> pair in _sleepingIslandSets)
            {
                pair.Value.Contacts.Remove(contact);
            }
        }

        private void RemoveNonTouchingContact(Contact contact)
        {
            _awakeSet.NonTouchingContacts.Remove(contact);
            _sleepingSet.NonTouchingContacts.Remove(contact);
            _disabledSet.NonTouchingContacts.Remove(contact);
        }

        private void RebuildNonTouchingContacts()
        {
            _awakeSet.NonTouchingContacts.Clear();
            _sleepingSet.NonTouchingContacts.Clear();
            _disabledSet.NonTouchingContacts.Clear();

            System.Collections.Generic.List<ContactKey> keys = new System.Collections.Generic.List<ContactKey>(_contactMap.Count);
            foreach (var pair in _contactMap)
            {
                keys.Add(pair.Key);
            }
            keys.Sort((a, b) => a.A != b.A ? a.A.CompareTo(b.A) : a.B.CompareTo(b.B));

            for (int i = 0; i < keys.Count; ++i)
            {
                if (!_contactMap.TryGetValue(keys[i], out Contact? contact))
                {
                    continue;
                }
                if (contact.Manifold.PointCount > 0)
                {
                    continue;
                }

                Fixture? fixtureA = contact.FixtureA;
                Fixture? fixtureB = contact.FixtureB;
                if (fixtureA == null || fixtureB == null)
                {
                    continue;
                }

                SolverSetType setType = SolverSetType.Awake;
                if (_def.EnableSleep &&
                    fixtureA.Body.Type != BodyType.Static &&
                    fixtureB.Body.Type != BodyType.Static &&
                    fixtureA.Body.SolverSetType == SolverSetType.Sleeping &&
                    fixtureB.Body.SolverSetType == SolverSetType.Sleeping)
                {
                    setType = SolverSetType.Disabled;
                }

                contact.SolverSetType = setType;
                contact.SolverSetId = 0;
                contact.ColorIndex = -1;
                contact.LocalIndex = -1;
                if (setType == SolverSetType.Disabled)
                {
                    _disabledSet.NonTouchingContacts.Add(contact);
                }
                else
                {
                    _awakeSet.NonTouchingContacts.Add(contact);
                }
            }
        }

        private static JointType GetJointType(object joint)
        {
            return joint switch
            {
                DistanceJoint => JointType.Distance,
                RevoluteJoint => JointType.Revolute,
                PrismaticJoint => JointType.Prismatic,
                WheelJoint => JointType.Wheel,
                PulleyJoint => JointType.Pulley,
                WeldJoint => JointType.Weld,
                MotorJoint => JointType.Motor,
                GearJoint => JointType.Gear,
                RopeJoint => JointType.Rope,
                FrictionJoint => JointType.Friction,
                _ => throw new System.InvalidOperationException("Unknown joint type.")
            };
        }

        private void MoveIslandBetweenSets(Island island, SolverSet source, SolverSet target)
        {
            RemoveIslandFromSet(island, source);
            AddIslandToSet(island, target);

            SolverSetType targetType = ReferenceEquals(target, _awakeSet) ? SolverSetType.Awake : SolverSetType.Sleeping;
            for (int i = 0; i < island.Contacts.Count; ++i)
            {
                island.Contacts[i].SolverSetType = targetType;
                island.Contacts[i].SolverSetId = targetType == SolverSetType.Awake ? 0 : island.Id;
            }
            for (int i = 0; i < island.Joints.Count; ++i)
            {
                                SetJointSolverSet(island.Joints[i], targetType, targetType == SolverSetType.Awake ? 0 : island.Id);
            }
        }

        private Body? GetJointOtherBody(JointHandle handle, Body body)
        {
            if (!TryGetJointIndex(handle, out int index))
            {
                return null;
            }

            return handle.Type switch
            {
                JointType.Distance => GetOtherBody(_distanceJointsData[index], body),
                JointType.Revolute => GetOtherBody(_revoluteJointsData[index], body),
                JointType.Prismatic => GetOtherBody(_prismaticJointsData[index], body),
                JointType.Wheel => GetOtherBody(_wheelJointsData[index], body),
                JointType.Pulley => GetOtherBody(_pulleyJointsData[index], body),
                JointType.Weld => GetOtherBody(_weldJointsData[index], body),
                JointType.Motor => GetOtherBody(_motorJointsData[index], body),
                JointType.Gear => GetOtherBody(_gearJointsData[index], body),
                JointType.Rope => GetOtherBody(_ropeJointsData[index], body),
                JointType.Friction => GetOtherBody(_frictionJointsData[index], body),
                _ => null
            };
        }

        private Body GetOtherBody(DistanceJointData joint, Body body) => GetBody(body.Id == joint.BodyA ? joint.BodyB : joint.BodyA);
        private Body GetOtherBody(RevoluteJointData joint, Body body) => GetBody(body.Id == joint.BodyA ? joint.BodyB : joint.BodyA);
        private Body GetOtherBody(PrismaticJointData joint, Body body) => GetBody(body.Id == joint.BodyA ? joint.BodyB : joint.BodyA);
        private Body GetOtherBody(WheelJointData joint, Body body) => GetBody(body.Id == joint.BodyA ? joint.BodyB : joint.BodyA);
        private Body GetOtherBody(PulleyJointData joint, Body body) => GetBody(body.Id == joint.BodyA ? joint.BodyB : joint.BodyA);
        private Body GetOtherBody(WeldJointData joint, Body body) => GetBody(body.Id == joint.BodyA ? joint.BodyB : joint.BodyA);
        private Body GetOtherBody(MotorJointData joint, Body body) => GetBody(body.Id == joint.BodyA ? joint.BodyB : joint.BodyA);

        private bool TryGetGearRepresentativeBodies(ref GearJointData joint, out Body bodyA, out Body bodyB)
        {
            bodyA = GetBody(joint.BodyB);
            bodyB = GetBody(joint.BodyD);
            if (!IsBothStatic(bodyA, bodyB))
            {
                return true;
            }

            bodyA = GetBody(joint.BodyA);
            bodyB = GetBody(joint.BodyB);
            if (!IsBothStatic(bodyA, bodyB))
            {
                return true;
            }

            bodyA = GetBody(joint.BodyC);
            bodyB = GetBody(joint.BodyD);
            if (!IsBothStatic(bodyA, bodyB))
            {
                return true;
            }

            bodyA = GetBody(joint.BodyA);
            bodyB = GetBody(joint.BodyC);
            return true;
        }

        private static bool IsBothStatic(Body bodyA, Body bodyB)
        {
            return bodyA.Type == BodyType.Static && bodyB.Type == BodyType.Static;
        }

        private static Body PickGearTraversalBody(Body current, Body first, Body second, Body third)
        {
            if (!ReferenceEquals(first, current) && first.Type != BodyType.Static)
            {
                return first;
            }
            if (!ReferenceEquals(second, current) && second.Type != BodyType.Static)
            {
                return second;
            }
            if (!ReferenceEquals(third, current) && third.Type != BodyType.Static)
            {
                return third;
            }
            if (!ReferenceEquals(first, current))
            {
                return first;
            }
            if (!ReferenceEquals(second, current))
            {
                return second;
            }
            return third;
        }

        private Body GetOtherBody(GearJointData joint, Body body)
        {
            Body bodyA = GetBody(joint.BodyA);
            Body bodyB = GetBody(joint.BodyB);
            Body bodyC = GetBody(joint.BodyC);
            Body bodyD = GetBody(joint.BodyD);

            if (body.Id == joint.BodyA)
            {
                return PickGearTraversalBody(bodyA, bodyB, bodyD, bodyC);
            }
            if (body.Id == joint.BodyB)
            {
                return PickGearTraversalBody(bodyB, bodyD, bodyA, bodyC);
            }
            if (body.Id == joint.BodyC)
            {
                return PickGearTraversalBody(bodyC, bodyD, bodyB, bodyA);
            }
            if (body.Id == joint.BodyD)
            {
                return PickGearTraversalBody(bodyD, bodyB, bodyC, bodyA);
            }
            return PickGearTraversalBody(body, bodyB, bodyD, bodyA);
        }
        private Body GetOtherBody(RopeJointData joint, Body body) => GetBody(body.Id == joint.BodyA ? joint.BodyB : joint.BodyA);
        private Body GetOtherBody(FrictionJointData joint, Body body) => GetBody(body.Id == joint.BodyA ? joint.BodyB : joint.BodyA);

        private int GetJointIndex(JointHandle handle)
        {
            return handle.Type switch
            {
                JointType.Distance => _distanceJointIndexById[handle.Id],
                JointType.Revolute => _revoluteJointIndexById[handle.Id],
                JointType.Prismatic => _prismaticJointIndexById[handle.Id],
                JointType.Wheel => _wheelJointIndexById[handle.Id],
                JointType.Pulley => _pulleyJointIndexById[handle.Id],
                JointType.Weld => _weldJointIndexById[handle.Id],
                JointType.Motor => _motorJointIndexById[handle.Id],
                JointType.Gear => _gearJointIndexById[handle.Id],
                JointType.Rope => _ropeJointIndexById[handle.Id],
                JointType.Friction => _frictionJointIndexById[handle.Id],
                _ => -1
            };
        }

        private bool TryGetJointIndex(JointHandle handle, out int index)
        {
            switch (handle.Type)
            {
                case JointType.Distance:
                    return _distanceJointIndexById.TryGetValue(handle.Id, out index);
                case JointType.Revolute:
                    return _revoluteJointIndexById.TryGetValue(handle.Id, out index);
                case JointType.Prismatic:
                    return _prismaticJointIndexById.TryGetValue(handle.Id, out index);
                case JointType.Wheel:
                    return _wheelJointIndexById.TryGetValue(handle.Id, out index);
                case JointType.Pulley:
                    return _pulleyJointIndexById.TryGetValue(handle.Id, out index);
                case JointType.Weld:
                    return _weldJointIndexById.TryGetValue(handle.Id, out index);
                case JointType.Motor:
                    return _motorJointIndexById.TryGetValue(handle.Id, out index);
                case JointType.Gear:
                    return _gearJointIndexById.TryGetValue(handle.Id, out index);
                case JointType.Rope:
                    return _ropeJointIndexById.TryGetValue(handle.Id, out index);
                case JointType.Friction:
                    return _frictionJointIndexById.TryGetValue(handle.Id, out index);
                default:
                    index = -1;
                    return false;
            }
        }

        private static int GetJointId<TJoint>(TJoint joint) where TJoint : class
        {
            return joint switch
            {
                DistanceJoint value => value.Id,
                RevoluteJoint value => value.Id,
                PrismaticJoint value => value.Id,
                WheelJoint value => value.Id,
                PulleyJoint value => value.Id,
                WeldJoint value => value.Id,
                MotorJoint value => value.Id,
                GearJoint value => value.Id,
                RopeJoint value => value.Id,
                FrictionJoint value => value.Id,
                _ => -1
            };
        }

        private static void SetJointId<TJoint>(TJoint joint, int id) where TJoint : class
        {
            switch (joint)
            {
                case DistanceJoint value:
                    value.Id = id;
                    break;
                case RevoluteJoint value:
                    value.Id = id;
                    break;
                case PrismaticJoint value:
                    value.Id = id;
                    break;
                case WheelJoint value:
                    value.Id = id;
                    break;
                case PulleyJoint value:
                    value.Id = id;
                    break;
                case WeldJoint value:
                    value.Id = id;
                    break;
                case MotorJoint value:
                    value.Id = id;
                    break;
                case GearJoint value:
                    value.Id = id;
                    break;
                case RopeJoint value:
                    value.Id = id;
                    break;
                case FrictionJoint value:
                    value.Id = id;
                    break;
            }
        }

        internal Fixture CreateFixture(Body body, Shape shape)
        {
            return CreateFixture(body, new FixtureDef(shape));
        }

        internal Fixture CreateFixture(Body body, FixtureDef def)
        {
            Fixture fixture = new Fixture(body, def.Shape)
            {
                Id = _fixtureIdPool.Alloc(),
                Friction = def.Friction,
                Restitution = def.Restitution,
                Density = def.Density,
                IsSensor = def.IsSensor,
                EnableSensorEvents = def.EnableSensorEvents,
                Filter = def.Filter,
                Material = def.ToMaterial(),
                UserData = def.UserData
            };
            fixture.Aabb = ShapeGeometry.ComputeAabb(def.Shape, body.Transform);
            fixture.ProxyId = _broadPhase.CreateProxy(fixture.Aabb, fixture);
            return fixture;
        }

        internal void DestroyFixture(Fixture fixture)
        {
            if (fixture.ProxyId >= 0)
            {
                _broadPhase.DestroyProxy(fixture.ProxyId);
            }
            if (fixture.Id >= 0)
            {
                _fixtureIdPool.Free(fixture.Id);
                fixture.Id = -1;
            }

            for (int i = _contacts.Count - 1; i >= 0; --i)
            {
                Contact contact = _contacts[i];
                if (contact.FixtureA == fixture || contact.FixtureB == fixture)
                {
                    UnlinkContact(contact);
                    RemoveContactFromSleepingSets(contact);
                    _contacts.RemoveAt(i);
                }
            }

            System.Collections.Generic.List<ContactKey> remove = new System.Collections.Generic.List<ContactKey>();
            foreach (var pair in _contactMap)
            {
                Contact contact = pair.Value;
                if (contact.FixtureA == fixture || contact.FixtureB == fixture)
                {
                    UnlinkContact(contact);
                    remove.Add(pair.Key);
                }
            }
            for (int i = 0; i < remove.Count; ++i)
            {
                if (_contactMap.TryGetValue(remove[i], out Contact? contact))
                {
                    if (contact.Id >= 0)
                    {
                        _contactIdPool.Free(contact.Id);
                        contact.Id = -1;
                    }
                }
                _contactMap.Remove(remove[i]);
            }

            if (fixture.ProxyId >= 0 && _sensorPairs.Count > 0)
            {
                System.Collections.Generic.List<ulong> sensorKeys = new System.Collections.Generic.List<ulong>();
                _sensorPairs.CopyKeys(sensorKeys);
                System.Collections.Generic.List<ulong> sensorRemove = new System.Collections.Generic.List<ulong>();
                for (int i = 0; i < sensorKeys.Count; ++i)
                {
                    ulong key = sensorKeys[i];
                    int a = UnpackProxyIdA(key);
                    int b = UnpackProxyIdB(key);
                    if (a == fixture.ProxyId || b == fixture.ProxyId)
                    {
                        sensorRemove.Add(key);
                    }
                }
                for (int i = 0; i < sensorRemove.Count; ++i)
                {
                    ulong key = sensorRemove[i];
                    _sensorPairs.Remove(key);
                    ContactKey contactKey = new ContactKey(UnpackProxyIdA(key), UnpackProxyIdB(key));
                    _sensorUserData.Remove(contactKey);
                }
            }
        }

        public Contact CreateContact(Shape shapeA, Transform transformA, Shape shapeB, Transform transformB)
        {
            Contact contact = new Contact(shapeA, transformA, shapeB, transformB);
            contact.Evaluate();
            return contact;
        }

        public void QueryAabb(Aabb aabb, System.Func<Fixture, bool> callback, QueryFilter? filter = null, bool includeSensors = true)
        {
            QueryFilter useFilter = filter ?? QueryFilter.Default;
            _broadPhase.Query(proxyId =>
            {
                Fixture? fixture = _broadPhase.GetUserData(proxyId);
                if (fixture == null)
                {
                    return true;
                }
                if (!includeSensors && fixture.IsSensor)
                {
                    return true;
                }
                if (!PassesQueryFilter(fixture.Filter, useFilter))
                {
                    return true;
                }
                return callback(fixture);
            }, aabb);
        }

        public System.Collections.Generic.List<Fixture> QueryAabb(Aabb aabb, QueryFilter? filter = null, bool includeSensors = true)
        {
            System.Collections.Generic.List<Fixture> results = new System.Collections.Generic.List<Fixture>();
            QueryAabb(aabb, fixture =>
            {
                results.Add(fixture);
                return true;
            }, filter, includeSensors);
            return results;
        }

        public bool RayCast(RayCastInput input, out RayCastHit hit, QueryFilter? filter = null, bool includeSensors = true)
        {
            if (!Collision.IsValidRay(input))
            {
                hit = default;
                return false;
            }

            QueryFilter useFilter = filter ?? QueryFilter.Default;
            float closest = input.MaxFraction;
            bool found = false;
            RayCastHit bestHit = default;

            _broadPhase.RayCast(proxyId =>
            {
                Fixture? fixture = _broadPhase.GetUserData(proxyId);
                if (fixture == null)
                {
                    return closest;
                }
                if (!includeSensors && fixture.IsSensor)
                {
                    return closest;
                }
                if (!PassesQueryFilter(fixture.Filter, useFilter))
                {
                    return closest;
                }

                if (!TryRayCastFixture(fixture, input, out RayCastOutput output))
                {
                    return closest;
                }

                if (output.Fraction < closest)
                {
                    closest = output.Fraction;
                    found = true;
                    bestHit = new RayCastHit(fixture, output.Point, output.Normal, output.Fraction);
                }

                return closest;
            }, input);

            hit = bestHit;
            return found;
        }

        public System.Collections.Generic.List<RayCastHit> RayCastAll(RayCastInput input, QueryFilter? filter = null, bool includeSensors = true)
        {
            System.Collections.Generic.List<RayCastHit> hits = new System.Collections.Generic.List<RayCastHit>();
            if (!Collision.IsValidRay(input))
            {
                return hits;
            }

            QueryFilter useFilter = filter ?? QueryFilter.Default;
            _broadPhase.RayCast(proxyId =>
            {
                Fixture? fixture = _broadPhase.GetUserData(proxyId);
                if (fixture == null)
                {
                    return input.MaxFraction;
                }
                if (!includeSensors && fixture.IsSensor)
                {
                    return input.MaxFraction;
                }
                if (!PassesQueryFilter(fixture.Filter, useFilter))
                {
                    return input.MaxFraction;
                }
                if (!TryRayCastFixture(fixture, input, out RayCastOutput output))
                {
                    return input.MaxFraction;
                }

                hits.Add(new RayCastHit(fixture, output.Point, output.Normal, output.Fraction));
                return input.MaxFraction;
            }, input);

            hits.Sort((a, b) => a.Fraction.CompareTo(b.Fraction));
            return hits;
        }

        public void UpdateContacts()
        {
            UpdateContacts(includeSensors: true);
        }

        private void UpdateContacts(bool includeSensors)
        {
            bool contactsChanged = false;
            for (int i = 0; i < _bodies.Count; ++i)
            {
                Body body = _bodies[i];
                for (int j = 0; j < body.Fixtures.Count; ++j)
                {
                    Fixture fixture = body.Fixtures[j];
                    Aabb newAabb = ShapeGeometry.ComputeAabb(fixture.Shape, body.Transform);
                    Vec2 oldCenter = fixture.Aabb.Center;
                    Vec2 newCenter = newAabb.Center;
                    Vec2 displacement = newCenter - oldCenter;
                    _broadPhase.MoveProxy(fixture.ProxyId, newAabb, displacement);
                    _broadPhase.TouchProxy(fixture.ProxyId);
                    fixture.Aabb = newAabb;
                }
            }

            _contacts.Clear();
            _awakeSet.Contacts.Clear();
            _sleepingSet.Contacts.Clear();
            _disabledSet.Contacts.Clear();
            HashSet64 seen = new HashSet64(128);
            System.Collections.Generic.List<(ContactKey Key, ContactBeginEvent Event)> beginEvents =
                new System.Collections.Generic.List<(ContactKey, ContactBeginEvent)>();
            System.Collections.Generic.List<(ContactKey Key, ContactEndEvent Event)> endEvents =
                new System.Collections.Generic.List<(ContactKey, ContactEndEvent)>();
            System.Collections.Generic.List<(ContactKey Key, ContactHitEvent Event)> hitEvents =
                new System.Collections.Generic.List<(ContactKey, ContactHitEvent)>();
            System.Collections.Generic.List<(ContactKey Key, Contact Contact, bool Begin, Fixture A, Fixture B)> transitions =
                new System.Collections.Generic.List<(ContactKey, Contact, bool, Fixture, Fixture)>();
            System.Collections.Generic.List<(ContactKey Key, Contact Contact)> changedContacts =
                new System.Collections.Generic.List<(ContactKey, Contact)>();

            System.Collections.Generic.List<(ContactKey Key, Fixture A, Fixture B)> pairs =
                new System.Collections.Generic.List<(ContactKey, Fixture, Fixture)>();
            _broadPhase.UpdatePairs((fixtureA, fixtureB) =>
            {
                if (fixtureA == null || fixtureB == null)
                {
                    return;
                }

                if (fixtureA.IsSensor || fixtureB.IsSensor)
                {
                    return;
                }

                if (!ShouldCollide(fixtureA, fixtureB))
                {
                    return;
                }

                ContactKey key = new ContactKey(fixtureA.ProxyId, fixtureB.ProxyId);
                pairs.Add((key, fixtureA, fixtureB));
            });

            pairs.Sort((a, b) => a.Key.A != b.Key.A ? a.Key.A.CompareTo(b.Key.A) : a.Key.B.CompareTo(b.Key.B));
            for (int i = 0; i < pairs.Count; ++i)
            {
                (ContactKey contactKey, Fixture fixtureA, Fixture fixtureB) = pairs[i];
                seen.Add(MakeProxyPairKey(contactKey.A, contactKey.B));

                if (!_contactMap.TryGetValue(contactKey, out Contact? contact))
                {
                    contact = new Contact(fixtureA, fixtureB);
                    contact.Id = _contactIdPool.Alloc();
                    contact.Evaluate();
                    _contactMap[contactKey] = contact;
                    contactsChanged = true;
                    if (contact.IsTouching != contact.WasTouching)
                    {
                        changedContacts.Add((contactKey, contact));
                    }
                }
                else
                {
                    bool wasTouching = contact.IsTouching;
                    contact.Update(fixtureA.Body.Transform, fixtureB.Body.Transform);
                    if (contact.IsTouching != wasTouching)
                    {
                        changedContacts.Add((contactKey, contact));
                    }
                }

                if (contact.IsTouching)
                {
                    Body bodyA = fixtureA.Body;
                    Body bodyB = fixtureB.Body;
                    if (bodyA.Type != BodyType.Static && bodyB.Type != BodyType.Static)
                    {
                        if (bodyA.Type == BodyType.Kinematic || bodyB.Type == BodyType.Kinematic || bodyA.Awake || bodyB.Awake)
                        {
                            bodyA.SetAwakeFromWorld(true, notifyWorld: false);
                            bodyB.SetAwakeFromWorld(true, notifyWorld: false);
                        }
                        else if (_def.EnableSleep)
                        {
                            MergeSleepingIslands(bodyA, bodyB);
                        }
                    }
                }

                if (contact.Manifold.PointCount > 0)
                {
                    _contacts.Add(contact);
                    RemoveNonTouchingContact(contact);
                    if (_def.EnableSleep &&
                        fixtureA.Body.Type != BodyType.Static &&
                        fixtureB.Body.Type != BodyType.Static)
                    {
                        SolverSetType setType = (fixtureA.Body.SolverSetType == SolverSetType.Awake ||
                                                 fixtureB.Body.SolverSetType == SolverSetType.Awake)
                            ? SolverSetType.Awake
                            : SolverSetType.Sleeping;
                        bool validSleeping = false;
                        int sleepingSetId = 0;
                        if (setType == SolverSetType.Sleeping &&
                            TryGetBodyIslandId(fixtureA.Body, out int islandId) &&
                            TryGetBodyIslandId(fixtureB.Body, out int islandIdB) &&
                            islandId == islandIdB &&
                            islandId >= 0 && islandId < _lastIslands.Count &&
                            !_lastIslands[islandId].IsAwake)
                        {
                            int setId = GetSleepingSetId(_lastIslands[islandId]);
                            if (setId != 0)
                            {
                                validSleeping = true;
                                sleepingSetId = setId;
                            }
                        }
                        else if (setType == SolverSetType.Sleeping)
                        {
                            setType = SolverSetType.Awake;
                        }

                        if (setType == SolverSetType.Awake)
                        {
                            _awakeSet.Contacts.Add(contact);
                        }
                        else
                        {
                            _sleepingSet.Contacts.Add(contact);
                        }
                        contact.SolverSetType = setType;
                        contact.SolverSetId = setType == SolverSetType.Sleeping && validSleeping ? sleepingSetId : 0;
                        contact.ColorIndex = -1;
                        contact.LocalIndex = -1;
                        if (setType == SolverSetType.Sleeping)
                        {
                            if (contact.IsTouching)
                            {
                                TryAddContactToSleepingSet(contact, fixtureA.Body, fixtureB.Body);
                            }
                            else
                            {
                                RemoveContactFromSleepingSets(contact);
                            }
                        }
                    }
                    else
                    {
                        _awakeSet.Contacts.Add(contact);
                        contact.SolverSetType = SolverSetType.Awake;
                        contact.SolverSetId = 0;
                        contact.ColorIndex = -1;
                        contact.LocalIndex = -1;
                    }
                    AddHitEvent(contact, contactKey, hitEvents);
                }
                else
                {
                    RemoveContactFromSleepingSets(contact);
                    SolverSetType setType = SolverSetType.Awake;
                    if (_def.EnableSleep &&
                        fixtureA.Body.Type != BodyType.Static &&
                        fixtureB.Body.Type != BodyType.Static &&
                        fixtureA.Body.SolverSetType == SolverSetType.Sleeping &&
                        fixtureB.Body.SolverSetType == SolverSetType.Sleeping)
                    {
                        setType = SolverSetType.Disabled;
                    }

                    contact.SolverSetType = setType;
                    contact.SolverSetId = 0;
                    contact.ColorIndex = -1;
                    contact.LocalIndex = -1;
                }
            }

            if (changedContacts.Count > 0)
            {
                changedContacts.Sort((a, b) => a.Key.A != b.Key.A ? a.Key.A.CompareTo(b.Key.A) : a.Key.B.CompareTo(b.Key.B));
                for (int i = 0; i < changedContacts.Count; ++i)
                {
                    (ContactKey key, Contact contact) = changedContacts[i];
                    if (contact.IsTouching != contact.WasTouching)
                    {
                        transitions.Add((key, contact, contact.IsTouching, contact.FixtureA!, contact.FixtureB!));
                    }
                }
            }

            if (transitions.Count > 0)
            {
                transitions.Sort((a, b) => a.Key.A != b.Key.A ? a.Key.A.CompareTo(b.Key.A) : a.Key.B.CompareTo(b.Key.B));
                for (int i = 0; i < transitions.Count; ++i)
                {
                    (ContactKey key, Contact contact, bool begin, Fixture fixtureA, Fixture fixtureB) = transitions[i];
                    if (begin)
                    {
                        beginEvents.Add((key, new ContactBeginEvent(fixtureA.UserData, fixtureB.UserData)));
                        LinkContact(contact);
                    }
                    else
                    {
                        endEvents.Add((key, new ContactEndEvent(fixtureA.UserData, fixtureB.UserData)));
                        UnlinkContact(contact);
                        contact.ColorIndex = -1;
                        contact.LocalIndex = -1;
                    }
                    MarkIslandDirty(fixtureA.Body);
                    MarkIslandDirty(fixtureB.Body);
                }
            }

            RebuildNonTouchingContacts();

            int previousCount = _contactMap.Count;
            RemoveStaleContacts(seen, endEvents);
            if (_contactMap.Count != previousCount)
            {
                contactsChanged = true;
            }
            if (includeSensors)
            {
                ProcessSensorPairs();
            }

            // Reconcile linked state with IsTouching. SolveTOI's contact.Update
            // can transition IsTouching silently, bypassing the transitions
            // pipeline above. If we don't relink here, the contact won't be
            // discovered by BuildIslandAdjacency (_bodyContacts[body] stays
            // empty), and BuildConstraintGraph won't colour the contact.
            foreach (var kv in _contactMap)
            {
                Contact c = kv.Value;
                if (c.FixtureA == null || c.FixtureB == null)
                {
                    continue;
                }
                if (c.FixtureA.IsSensor || c.FixtureB.IsSensor)
                {
                    continue;
                }
                bool isLinked = c.EdgeIdA >= 0 && c.EdgeIdB >= 0;
                if (c.IsTouching && !isLinked)
                {
                    LinkContact(c);
                    contactsChanged = true;
                }
                else if (!c.IsTouching && isLinked)
                {
                    UnlinkContact(c);
                    c.ColorIndex = -1;
                    c.LocalIndex = -1;
                    contactsChanged = true;
                }
            }

            if (contactsChanged)
            {
                _islandsDirty = true;
            }

            SortSolverSets();
            _contacts.Sort(CompareContacts);

            foreach (var pair in _contactMap)
            {
                Contact contact = pair.Value;
                if (contact.Manifold.PointCount == 0)
                {
                    continue;
                }
                if (contact.SolverSetType == SolverSetType.Sleeping)
                {
                    bool validSleeping = contact.SolverSetId != 0;
                    if (!validSleeping &&
                        TryGetBodyIslandId(contact.FixtureA!.Body, out int islandId) &&
                        TryGetBodyIslandId(contact.FixtureB!.Body, out int islandIdB) &&
                        islandId == islandIdB &&
                        islandId >= 0 && islandId < _lastIslands.Count &&
                        !_lastIslands[islandId].IsAwake)
                    {
                        int setId = GetSleepingSetId(_lastIslands[islandId]);
                        if (setId != 0)
                        {
                            contact.SolverSetId = setId;
                            validSleeping = true;
                        }
                    }

                    if (!validSleeping)
                    {
                        contact.SolverSetType = SolverSetType.Awake;
                        contact.SolverSetId = 0;
                        if (!_awakeSet.Contacts.Contains(contact))
                        {
                            _awakeSet.Contacts.Add(contact);
                        }
                        _sleepingSet.Contacts.Remove(contact);
                    }
                    else
                    {
                        if (!_sleepingSet.Contacts.Contains(contact))
                        {
                            _sleepingSet.Contacts.Add(contact);
                        }
                        _awakeSet.Contacts.Remove(contact);
                    }
                }
                else
                {
                    if (!_awakeSet.Contacts.Contains(contact))
                    {
                        _awakeSet.Contacts.Add(contact);
                    }
                    _sleepingSet.Contacts.Remove(contact);
                }
            }

            if (beginEvents.Count > 0 || endEvents.Count > 0 || hitEvents.Count > 0)
            {
                beginEvents.Sort((a, b) => a.Key.A != b.Key.A ? a.Key.A.CompareTo(b.Key.A) : a.Key.B.CompareTo(b.Key.B));
                endEvents.Sort((a, b) => a.Key.A != b.Key.A ? a.Key.A.CompareTo(b.Key.A) : a.Key.B.CompareTo(b.Key.B));
                hitEvents.Sort((a, b) => a.Key.A != b.Key.A ? a.Key.A.CompareTo(b.Key.A) : a.Key.B.CompareTo(b.Key.B));

                ContactBeginEvent[] begins = new ContactBeginEvent[beginEvents.Count];
                for (int i = 0; i < beginEvents.Count; ++i)
                {
                    begins[i] = beginEvents[i].Event;
                }
                ContactEndEvent[] ends = new ContactEndEvent[endEvents.Count];
                for (int i = 0; i < endEvents.Count; ++i)
                {
                    ends[i] = endEvents[i].Event;
                }
                ContactHitEvent[] hits = new ContactHitEvent[hitEvents.Count];
                for (int i = 0; i < hitEvents.Count; ++i)
                {
                    hits[i] = hitEvents[i].Event;
                }

                _events.Raise(new ContactEvents(begins, ends, hits));
            }
            // Sensor events are raised by ProcessSensorPairs.

            ValidateContacts();
            ValidateSolverSets();
            ValidateIslands();
        }

        public void Step(float timeStep)
        {
            _solverPipeline.Step(timeStep);
        }

        private void UpdateSensors()
        {
            for (int i = 0; i < _bodies.Count; ++i)
            {
                Body body = _bodies[i];
                for (int j = 0; j < body.Fixtures.Count; ++j)
                {
                    Fixture fixture = body.Fixtures[j];
                    Aabb newAabb = ShapeGeometry.ComputeAabb(fixture.Shape, body.Transform);
                    Vec2 oldCenter = fixture.Aabb.Center;
                    Vec2 newCenter = newAabb.Center;
                    Vec2 displacement = newCenter - oldCenter;
                    _broadPhase.MoveProxy(fixture.ProxyId, newAabb, displacement);
                    _broadPhase.TouchProxy(fixture.ProxyId);
                    fixture.Aabb = newAabb;
                }
            }

            ProcessSensorPairs();
        }

        private void ProcessSensorPairs()
        {
            System.Collections.Generic.List<SensorBeginEvent> sensorBeginEvents = new System.Collections.Generic.List<SensorBeginEvent>();
            System.Collections.Generic.List<SensorEndEvent> sensorEndEvents = new System.Collections.Generic.List<SensorEndEvent>();

            System.Collections.Generic.List<Fixture> sensors = new System.Collections.Generic.List<Fixture>();
            for (int i = 0; i < _bodies.Count; ++i)
            {
                Body body = _bodies[i];
                for (int j = 0; j < body.Fixtures.Count; ++j)
                {
                    Fixture fixture = body.Fixtures[j];
                    if (fixture.IsSensor && fixture.ProxyId >= 0)
                    {
                        sensors.Add(fixture);
                    }
                }
            }

            sensors.Sort((a, b) => a.ProxyId.CompareTo(b.ProxyId));

            System.Collections.Generic.HashSet<int> activeSensors = new System.Collections.Generic.HashSet<int>();
            HashSet64 livePairs = new HashSet64(Math.Max(32, _sensorPairs.Count * 2));
            System.Collections.Generic.Dictionary<ContactKey, (object? A, object? B)> liveUserData =
                new System.Collections.Generic.Dictionary<ContactKey, (object? A, object? B)>();

            for (int i = 0; i < sensors.Count; ++i)
            {
                Fixture sensor = sensors[i];
                int sensorId = sensor.ProxyId;
                if (!sensor.EnableSensorEvents)
                {
                    if (_sensorOverlaps.TryGetValue(sensorId, out System.Collections.Generic.HashSet<Fixture>? previousDisabled) &&
                        previousDisabled.Count > 0)
                    {
                        System.Collections.Generic.List<Fixture> oldFixtures = new System.Collections.Generic.List<Fixture>(previousDisabled);
                        oldFixtures.Sort(CompareFixturesByProxyId);
                        for (int j = 0; j < oldFixtures.Count; ++j)
                        {
                            sensorEndEvents.Add(new SensorEndEvent(sensor.UserData, oldFixtures[j].UserData));
                        }
                        previousDisabled.Clear();
                    }
                    continue;
                }
                activeSensors.Add(sensorId);

                System.Collections.Generic.List<Fixture> overlaps = new System.Collections.Generic.List<Fixture>();
                Aabb queryBounds = sensor.Aabb;
                _broadPhase.Query(proxyId =>
                {
                    Fixture? other = _broadPhase.GetUserData(proxyId);
                    if (other == null || other == sensor)
                    {
                        return true;
                    }
                    if (other.Body == sensor.Body)
                    {
                        return true;
                    }
                    if (!ShouldSensorOverlap(sensor, other))
                    {
                        return true;
                    }
                    if (!TestOverlap(sensor, other))
                    {
                        return true;
                    }

                    overlaps.Add(other);
                    return true;
                }, queryBounds);

                if (overlaps.Count > 1)
                {
                    overlaps.Sort(CompareFixturesByProxyId);
                    int unique = 1;
                    for (int k = 1; k < overlaps.Count; ++k)
                    {
                        if (!ReferenceEquals(overlaps[k], overlaps[unique - 1]))
                        {
                            overlaps[unique++] = overlaps[k];
                        }
                    }
                    if (unique != overlaps.Count)
                    {
                        overlaps.RemoveRange(unique, overlaps.Count - unique);
                    }
                }

                if (!_sensorOverlaps.TryGetValue(sensorId, out System.Collections.Generic.HashSet<Fixture>? previous))
                {
                    previous = new System.Collections.Generic.HashSet<Fixture>();
                    _sensorOverlaps[sensorId] = previous;
                }

                System.Collections.Generic.HashSet<Fixture> current = new System.Collections.Generic.HashSet<Fixture>();
                for (int k = 0; k < overlaps.Count; ++k)
                {
                    Fixture otherFixture = overlaps[k];
                    current.Add(otherFixture);

                    int otherId = otherFixture.ProxyId;
                    ContactKey key = new ContactKey(sensorId, otherId);
                    livePairs.Add(MakeProxyPairKey(sensorId, otherId));
                    liveUserData[key] = (sensor.UserData, otherFixture.UserData);

                    if (!previous.Contains(otherFixture))
                    {
                        sensorBeginEvents.Add(new SensorBeginEvent(sensor.UserData, otherFixture.UserData));
                    }
                }

                foreach (Fixture oldFixture in previous)
                {
                    if (!current.Contains(oldFixture))
                    {
                        sensorEndEvents.Add(new SensorEndEvent(sensor.UserData, oldFixture.UserData));
                    }
                }

                previous.Clear();
                foreach (Fixture fixture in current)
                {
                    previous.Add(fixture);
                }
            }

            if (_sensorOverlaps.Count > activeSensors.Count)
            {
                System.Collections.Generic.List<int> remove = new System.Collections.Generic.List<int>();
                foreach (var pair in _sensorOverlaps)
                {
                    if (!activeSensors.Contains(pair.Key))
                    {
                        Fixture? sensorFixture = _broadPhase.GetUserData(pair.Key);
                        object? sensorUserData = sensorFixture?.UserData;
                        System.Collections.Generic.List<Fixture> oldFixtures = new System.Collections.Generic.List<Fixture>(pair.Value);
                        oldFixtures.Sort(CompareFixturesByProxyId);
                        for (int i = 0; i < oldFixtures.Count; ++i)
                        {
                            sensorEndEvents.Add(new SensorEndEvent(sensorUserData, oldFixtures[i].UserData));
                        }
                        remove.Add(pair.Key);
                    }
                }

                for (int i = 0; i < remove.Count; ++i)
                {
                    _sensorOverlaps.Remove(remove[i]);
                }
            }

            _sensorPairs.Clear();
            MergeSensorPairs(livePairs);
            _sensorUserData.Clear();
            foreach (var pair in liveUserData)
            {
                _sensorUserData[pair.Key] = pair.Value;
            }

            if (sensorBeginEvents.Count > 0 || sensorEndEvents.Count > 0)
            {
                _events.Raise(new SensorEvents(sensorBeginEvents.ToArray(), sensorEndEvents.ToArray()));
            }
        }

        private static int CompareFixturesByProxyId(Fixture a, Fixture b)
        {
            return a.ProxyId.CompareTo(b.ProxyId);
        }

        private int CountFixtures()
        {
            int count = 0;
            for (int i = 0; i < _bodies.Count; ++i)
            {
                count += _bodies[i].Fixtures.Count;
            }
            return count;
        }

        private int CountJoints()
        {
            return _distanceJointCount +
                   _revoluteJointCount +
                   _prismaticJointCount +
                   _wheelJointCount +
                   _pulleyJointCount +
                   _weldJointCount +
                   _motorJointCount +
                   _gearJointCount +
                   _ropeJointCount +
                   _frictionJointCount;
        }

        [System.Diagnostics.Conditional("DEBUG")]
        private void ValidateContacts()
        {
            System.Collections.Generic.HashSet<Contact> awakeTouching = new System.Collections.Generic.HashSet<Contact>(_awakeSet.Contacts);
            System.Collections.Generic.HashSet<Contact> sleepingTouching = new System.Collections.Generic.HashSet<Contact>(_sleepingSet.Contacts);
            System.Collections.Generic.HashSet<Contact> disabledNonTouching = new System.Collections.Generic.HashSet<Contact>(_disabledSet.NonTouchingContacts);
            System.Collections.Generic.HashSet<Contact> awakeNonTouching = new System.Collections.Generic.HashSet<Contact>(_awakeSet.NonTouchingContacts);
            System.Collections.Generic.HashSet<Contact> sleepingNonTouching = new System.Collections.Generic.HashSet<Contact>(_sleepingSet.NonTouchingContacts);

            foreach (var pair in _contactMap)
            {
                Contact contact = pair.Value;
                bool touching = contact.Manifold.PointCount > 0;
                bool spansAwake = false;
                if (_def.EnableSleep && contact.FixtureA != null && contact.FixtureB != null)
                {
                    Body bodyA = contact.FixtureA.Body;
                    Body bodyB = contact.FixtureB.Body;
                    if (bodyA.Type != BodyType.Static && bodyB.Type != BodyType.Static)
                    {
                        spansAwake = bodyA.SolverSetType == SolverSetType.Awake || bodyB.SolverSetType == SolverSetType.Awake;
                    }
                }

                if (touching)
                {
                    if (contact.SolverSetType == SolverSetType.Sleeping)
                    {
                        System.Diagnostics.Debug.Assert(sleepingTouching.Contains(contact));
                        System.Diagnostics.Debug.Assert(contact.ColorIndex == -1);
                        System.Diagnostics.Debug.Assert(contact.LocalIndex == -1);
                    }
                    else
                    {
                        System.Diagnostics.Debug.Assert(awakeTouching.Contains(contact));
                    }
                    System.Diagnostics.Debug.Assert(!disabledNonTouching.Contains(contact));
                    System.Diagnostics.Debug.Assert(!awakeNonTouching.Contains(contact));
                    System.Diagnostics.Debug.Assert(!sleepingNonTouching.Contains(contact));
                    if (contact.SolverSetType == SolverSetType.Sleeping)
                    {
                        System.Diagnostics.Debug.Assert(!spansAwake);
                    }
                }
                else
                {
                    if (contact.SolverSetType == SolverSetType.Disabled)
                    {
                        System.Diagnostics.Debug.Assert(disabledNonTouching.Contains(contact));
                    }
                    else
                    {
                        System.Diagnostics.Debug.Assert(awakeNonTouching.Contains(contact));
                    }
                    System.Diagnostics.Debug.Assert(!awakeTouching.Contains(contact));
                    System.Diagnostics.Debug.Assert(!sleepingTouching.Contains(contact));
                    System.Diagnostics.Debug.Assert(contact.ColorIndex == -1);
                    System.Diagnostics.Debug.Assert(contact.LocalIndex == -1);
                }
            }
        }

        [System.Diagnostics.Conditional("DEBUG")]
        private void ValidateSolverSets()
        {
            if (_islandsDirty)
            {
                return;
            }
            System.Collections.Generic.HashSet<Body> staticBodies = new System.Collections.Generic.HashSet<Body>(_staticSet.Bodies);
            System.Collections.Generic.HashSet<Body> awakeBodies = new System.Collections.Generic.HashSet<Body>(_awakeSet.Bodies);
            System.Collections.Generic.HashSet<Body> sleepingBodies = new System.Collections.Generic.HashSet<Body>(_sleepingSet.Bodies);
            System.Collections.Generic.HashSet<Body> disabledBodies = new System.Collections.Generic.HashSet<Body>(_disabledSet.Bodies);

            for (int i = 0; i < _bodies.Count; ++i)
            {
                Body body = _bodies[i];
                if (body.Type == BodyType.Static)
                {
                    System.Diagnostics.Debug.Assert(staticBodies.Contains(body));
                    System.Diagnostics.Debug.Assert(!awakeBodies.Contains(body));
                    System.Diagnostics.Debug.Assert(!sleepingBodies.Contains(body));
                    System.Diagnostics.Debug.Assert(!disabledBodies.Contains(body));
                    continue;
                }

                if (body.SolverSetType == SolverSetType.Awake)
                {
                    System.Diagnostics.Debug.Assert(awakeBodies.Contains(body));
                }
                else if (body.SolverSetType == SolverSetType.Sleeping)
                {
                    System.Diagnostics.Debug.Assert(sleepingBodies.Contains(body));
                }
            }

            foreach (var pair in _contactMap)
            {
                Contact contact = pair.Value;
                bool touching = contact.Manifold.PointCount > 0;
                if (touching)
                {
                    if (contact.SolverSetType == SolverSetType.Sleeping)
                    {
                        System.Diagnostics.Debug.Assert(_sleepingSet.Contacts.Contains(contact));
                        System.Diagnostics.Debug.Assert(contact.SolverSetId != 0);
                    }
                    else
                    {
                        System.Diagnostics.Debug.Assert(_awakeSet.Contacts.Contains(contact));
                        System.Diagnostics.Debug.Assert(contact.SolverSetId == 0);
                    }
                }
                else
                {
                    if (contact.SolverSetType == SolverSetType.Disabled)
                    {
                        System.Diagnostics.Debug.Assert(_disabledSet.NonTouchingContacts.Contains(contact));
                    }
                    else
                    {
                        System.Diagnostics.Debug.Assert(_awakeSet.NonTouchingContacts.Contains(contact));
                    }
                    System.Diagnostics.Debug.Assert(contact.SolverSetId == 0);
                }
            }

            for (int i = 0; i < _awakeSet.Joints.Count; ++i)
            {
                JointHandle handle = _awakeSet.Joints[i];
                System.Diagnostics.Debug.Assert(TryGetJointSolverSetType(handle, out SolverSetType type) && type == SolverSetType.Awake);
                System.Diagnostics.Debug.Assert(TryGetJointSolverSetId(handle, out int setId) && setId == 0);
            }

            for (int i = 0; i < _sleepingSet.Joints.Count; ++i)
            {
                JointHandle handle = _sleepingSet.Joints[i];
                System.Diagnostics.Debug.Assert(TryGetJointSolverSetType(handle, out SolverSetType type) && type == SolverSetType.Sleeping);
            }

            ValidateIdPoolMembership();
        }

        [System.Diagnostics.Conditional("DEBUG")]
        private void ValidateIdPoolMembership()
        {
            /*
            ValidatePool(
                _bodyIdPool,
                _bodyIdPool.Capacity,
                () =>
                {
                    bool[] used = new bool[_bodyIdPool.Capacity];
                    for (int i = 0; i < _bodies.Count; ++i)
                    {
                        int id = _bodies[i].Id;
                        if (id >= 0)
                        {
                            System.Diagnostics.Debug.Assert(id < used.Length);
                            used[id] = true;
                        }
                    }
                    return used;
                });
            */

            ValidatePool(
                _fixtureIdPool,
                _fixtureIdPool.Capacity,
                () =>
                {
                    bool[] used = new bool[_fixtureIdPool.Capacity];
                    for (int i = 0; i < _bodies.Count; ++i)
                    {
                        var fixtures = _bodies[i].Fixtures;
                        for (int j = 0; j < fixtures.Count; ++j)
                        {
                            int id = fixtures[j].Id;
                            if (id >= 0)
                            {
                                System.Diagnostics.Debug.Assert(id < used.Length);
                                used[id] = true;
                            }
                        }
                    }
                    return used;
                });

            ValidatePool(
                _jointIdPool,
                _jointIdPool.Capacity,
                () =>
                {
                    bool[] used = new bool[_jointIdPool.Capacity];
                    MarkJointIds(used, (System.Collections.Generic.IList<DistanceJoint>)_distanceJoints, static joint => joint.Id);
                    MarkJointIds(used, _revoluteJoints, static joint => joint.Id);
                    MarkJointIds(used, _prismaticJoints, static joint => joint.Id);
                    MarkJointIds(used, _wheelJoints, static joint => joint.Id);
                    MarkJointIds(used, _pulleyJoints, static joint => joint.Id);
                    MarkJointIds(used, _weldJoints, static joint => joint.Id);
                    MarkJointIds(used, _motorJoints, static joint => joint.Id);
                    MarkJointIds(used, _gearJoints, static joint => joint.Id);
                    MarkJointIds(used, _ropeJoints, static joint => joint.Id);
                    MarkJointIds(used, _frictionJoints, static joint => joint.Id);
                    return used;
                });

            ValidatePool(
                _contactIdPool,
                _contactIdPool.Capacity,
                () =>
                {
                    bool[] used = new bool[_contactIdPool.Capacity];
                    foreach (var pair in _contactMap)
                    {
                        int id = pair.Value.Id;
                        if (id >= 0)
                        {
                            System.Diagnostics.Debug.Assert(id < used.Length);
                            used[id] = true;
                        }
                    }
                    return used;
                });

            ValidatePool(
                _contactEdgeIdPool,
                _contactEdgeIdPool.Capacity,
                () =>
                {
                    bool[] used = new bool[_contactEdgeIdPool.Capacity];
                    foreach (var pair in _contactMap)
                    {
                        Contact contact = pair.Value;
                        if (contact.EdgeA != null)
                        {
                            int id = contact.EdgeA.Id;
                            System.Diagnostics.Debug.Assert(0 <= id && id < used.Length);
                            used[id] = true;
                        }
                        if (contact.EdgeB != null)
                        {
                            int id = contact.EdgeB.Id;
                            System.Diagnostics.Debug.Assert(0 <= id && id < used.Length);
                            used[id] = true;
                        }
                    }
                    return used;
                });

            ValidatePool(
                _solverSetIdPool,
                _solverSetIdPool.Capacity,
                () =>
                {
                    bool[] used = new bool[_solverSetIdPool.Capacity];
                    used[0] = true;
                    foreach (var pair in _sleepingIslandSets)
                    {
                        int id = pair.Key;
                        System.Diagnostics.Debug.Assert(0 <= id && id < used.Length);
                        used[id] = true;
                    }
                    return used;
                });
        }

        [System.Diagnostics.Conditional("DEBUG")]
        private void ValidateConstraintGraph()
        {
            System.Collections.Generic.Dictionary<int, Body> bodiesById = new System.Collections.Generic.Dictionary<int, Body>(_bodies.Count);
            for (int i = 0; i < _bodies.Count; ++i)
            {
                Body body = _bodies[i];
                if (body.Id >= 0)
                {
                    bodiesById[body.Id] = body;
                }
            }

            for (int colorIndex = 0; colorIndex < Constants.GraphColorCount; ++colorIndex)
            {
                ConstraintGraph.GraphColor color = _constraintGraph.Colors[colorIndex];
                System.Collections.Generic.HashSet<int> expectedDynamicBodyIds = new System.Collections.Generic.HashSet<int>();

                for (int i = 0; i < color.Contacts.Count; ++i)
                {
                    Contact contact = color.Contacts[i];
                    System.Diagnostics.Debug.Assert(contact.SolverSetType == SolverSetType.Awake);
                    System.Diagnostics.Debug.Assert(contact.SolverSetId == 0);
                    System.Diagnostics.Debug.Assert(contact.ColorIndex == colorIndex);
                    System.Diagnostics.Debug.Assert(contact.LocalIndex == i);
                    if (contact.FixtureA != null && contact.FixtureA.Body.Type != BodyType.Static)
                    {
                        expectedDynamicBodyIds.Add(contact.FixtureA.Body.Id);
                    }
                    if (contact.FixtureB != null && contact.FixtureB.Body.Type != BodyType.Static)
                    {
                        expectedDynamicBodyIds.Add(contact.FixtureB.Body.Id);
                    }
                }

                for (int i = 0; i < color.Joints.Count; ++i)
                {
                    JointHandle handle = color.Joints[i];
                    System.Diagnostics.Debug.Assert(TryGetJointConstraintGraphIndex(handle, out int jointColor, out int localIndex));
                    System.Diagnostics.Debug.Assert(jointColor == colorIndex);
                    System.Diagnostics.Debug.Assert(localIndex == i);
                    if (TryGetJointBodies(handle, out Body bodyA, out Body bodyB))
                    {
                        if (bodyA.Type != BodyType.Static)
                        {
                            expectedDynamicBodyIds.Add(bodyA.Id);
                        }
                        if (bodyB.Type != BodyType.Static)
                        {
                            expectedDynamicBodyIds.Add(bodyB.Id);
                        }
                    }
                }

                if (colorIndex == Constants.GraphOverflowIndex)
                {
                    continue;
                }

                System.Diagnostics.Debug.Assert(color.BodySet != null);
                BitSet bodySet = color.BodySet!;
                foreach (int bodyId in expectedDynamicBodyIds)
                {
                    System.Diagnostics.Debug.Assert(bodySet.GetBit((uint)bodyId));
                }

                int bitCount = bodySet.CountSetBits();
                System.Diagnostics.Debug.Assert(bitCount == expectedDynamicBodyIds.Count);
                for (int bodyId = 0; bodyId < _bodyCount; ++bodyId)
                {
                    bool present = bodySet.GetBit((uint)bodyId);
                    if (!present)
                    {
                        continue;
                    }

                    System.Diagnostics.Debug.Assert(expectedDynamicBodyIds.Contains(bodyId));
                    System.Diagnostics.Debug.Assert(bodiesById.TryGetValue(bodyId, out Body? body));
                    System.Diagnostics.Debug.Assert(body.Type != BodyType.Static);
                }
            }
        }

        private static void MarkJointIds<TJoint>(
            bool[] used,
            System.Collections.Generic.IList<TJoint> joints,
            System.Func<TJoint, int> getId) where TJoint : class
        {
            for (int i = 0; i < joints.Count; ++i)
            {
                if (joints[i] == null) continue;
                int id = getId(joints[i]);
                if (id < 0)
                {
                    continue;
                }
                System.Diagnostics.Debug.Assert(id < used.Length);
                used[id] = true;
            }
        }

        private static void ValidatePool(IdPool pool, int capacity, System.Func<bool[]> buildUsed)
        {
            bool[] used = buildUsed();
            System.Diagnostics.Debug.Assert(used.Length == capacity);
            int usedCount = 0;
            for (int i = 0; i < used.Length; ++i)
            {
                if (used[i])
                {
                    usedCount += 1;
                    pool.ValidateUsedId(i);
                }
                else
                {
                    pool.ValidateFreeId(i);
                }
            }
            System.Diagnostics.Debug.Assert(usedCount == pool.Count);
        }

        [System.Diagnostics.Conditional("DEBUG")]
        private void ValidateIslands()
        {
            if (_islandsDirty)
            {
                return;
            }

            for (int i = 0; i < _lastIslands.Count; ++i)
            {
                Island island = _lastIslands[i];
                System.Diagnostics.Debug.Assert(island.Bodies.Count > 0);

                bool isAwake = island.IsAwake;
                for (int b = 0; b < island.Bodies.Count; ++b)
                {
                    Body body = island.Bodies[b];
                    System.Diagnostics.Debug.Assert(body.Type != BodyType.Static);
                    if (isAwake)
                    {
                        System.Diagnostics.Debug.Assert(_awakeSet.Islands.Contains(island));
                    }
                    else
                    {
                        System.Diagnostics.Debug.Assert(_sleepingSet.Islands.Contains(island));
                    }

                    if (TryGetBodyIslandId(body, out int islandId))
                    {
                        System.Diagnostics.Debug.Assert(islandId == i);
                    }
                }

                for (int c = 0; c < island.Contacts.Count; ++c)
                {
                    Contact contact = island.Contacts[c];
                    System.Diagnostics.Debug.Assert(contact.Manifold.PointCount > 0);
                    System.Diagnostics.Debug.Assert(contact.SolverSetType == SolverSetType.Awake || contact.SolverSetType == SolverSetType.Sleeping);
                }

                for (int j = 0; j < island.Joints.Count; ++j)
                {
                    JointHandle handle = island.Joints[j];
                    System.Diagnostics.Debug.Assert(TryGetJointSolverSetType(handle, out SolverSetType type));
                    if (isAwake)
                    {
                        System.Diagnostics.Debug.Assert(type == SolverSetType.Awake);
                    }
                    else
                    {
                        System.Diagnostics.Debug.Assert(type == SolverSetType.Sleeping);
                    }

                    if (TryGetJointSolverSetId(handle, out int setId))
                    {
                        if (isAwake)
                        {
                            System.Diagnostics.Debug.Assert(setId == 0);
                        }
                        else
                        {
                            System.Diagnostics.Debug.Assert(setId == GetSleepingSetId(island));
                        }
                    }
                }
            }
        }

        internal void ResetSweeps()
        {
            for (int i = 0; i < _bodies.Count; ++i)
            {
                Body body = _bodies[i];
                Vec2 center = body.GetWorldCenter();
                float angle = body.Transform.Q.Angle;
                body.Sweep = new Sweep(body.LocalCenter, center, center, angle, angle, 0f);
                _bodyAwakeAtStepStart[body.Id] = _bodyAwakes[body.Id];
                // Phase 2.5 of TIER4_PARITY_PLAN — Stage A: clear within-step
                // delta at start of each outer Step. Sub-steps inside the Step
                // loop accumulate into these arrays. Consumers (added in
                // subsequent stages) read `body.Position + delta` so they
                // see the predicted post-integration anchor offset.
                _bodyDeltaPositions[body.Id] = Vec2.Zero;
                _bodyDeltaRotations[body.Id] = Rot.Identity;
                // Stage A.2: snapshot the body's pose at the top of the outer
                // Step. Stays fixed across all sub-steps so consumers can
                // recover step-start when the future migration needs it (e.g.
                // soft-joint DeltaCenter re-captured each Init, cpp v3 style).
                _bodyStepStartPositions[body.Id] = _bodyPositions[body.Id];
                _bodyStepStartRotations[body.Id] = _bodyRotations[body.Id];
            }
        }

        internal void RaiseBodyEvents()
        {
            if (_bodies.Count == 0)
            {
                return;
            }

            System.Collections.Generic.List<BodyEvent> moves = new System.Collections.Generic.List<BodyEvent>();
            for (int i = 0; i < _bodies.Count; ++i)
            {
                Body body = _bodies[i];
                if (body.Type == BodyType.Static)
                {
                    continue;
                }

                bool wasAwake = _bodyAwakeAtStepStart[body.Id];
                bool isAwake = _bodyAwakes[body.Id];
                bool fellAsleep = wasAwake && !isAwake;
                if (!isAwake && !fellAsleep)
                {
                    continue;
                }

                moves.Add(new BodyEvent(body.Definition.UserData, body.Transform, fellAsleep));
            }

            if (moves.Count > 0)
            {
                _events.Raise(new BodyEvents(moves.ToArray()));
            }
        }

        internal void RaiseJointEvents(float invDt)
        {
            float forceThreshold = _def.JointForceThreshold;
            float torqueThreshold = _def.JointTorqueThreshold;
            if (forceThreshold >= float.MaxValue && torqueThreshold >= float.MaxValue)
            {
                return;
            }

            System.Collections.Generic.List<JointEvent> events = new System.Collections.Generic.List<JointEvent>();

            void Consider(Body bodyA, Body bodyB, Vec2 reactionForce, float reactionTorque)
            {
                if (bodyA == null || bodyB == null)
                {
                    return;
                }
                if (!_bodyAwakes[bodyA.Id] && !_bodyAwakes[bodyB.Id])
                {
                    return;
                }
                float force = reactionForce.Length;
                float torque = MathF.Abs(reactionTorque);
                if (force >= forceThreshold || torque >= torqueThreshold)
                {
                    events.Add(new JointEvent(bodyA.Definition.UserData, bodyB.Definition.UserData, force, torque));
                }
            }

            for (int i = 0; i < _distanceJointCount; ++i)
            {
                DistanceJoint j = _distanceJoints[i];
                Consider(j.BodyA, j.BodyB, j.GetReactionForce(invDt), j.GetReactionTorque(invDt));
            }
            for (int i = 0; i < _revoluteJointCount; ++i)
            {
                RevoluteJoint j = _revoluteJoints[i];
                Consider(j.BodyA, j.BodyB, j.GetReactionForce(invDt), j.GetReactionTorque(invDt));
            }
            for (int i = 0; i < _prismaticJointCount; ++i)
            {
                PrismaticJoint j = _prismaticJoints[i];
                Consider(j.BodyA, j.BodyB, j.GetReactionForce(invDt), j.GetReactionTorque(invDt));
            }
            for (int i = 0; i < _wheelJointCount; ++i)
            {
                WheelJoint j = _wheelJoints[i];
                Consider(j.BodyA, j.BodyB, j.GetReactionForce(invDt), j.GetReactionTorque(invDt));
            }
            for (int i = 0; i < _pulleyJointCount; ++i)
            {
                PulleyJoint j = _pulleyJoints[i];
                Consider(j.BodyA, j.BodyB, j.GetReactionForce(invDt), j.GetReactionTorque(invDt));
            }
            for (int i = 0; i < _weldJointCount; ++i)
            {
                WeldJoint j = _weldJoints[i];
                Consider(j.BodyA, j.BodyB, j.GetReactionForce(invDt), j.GetReactionTorque(invDt));
            }
            for (int i = 0; i < _motorJointCount; ++i)
            {
                MotorJoint j = _motorJoints[i];
                Consider(j.BodyA, j.BodyB, j.GetReactionForce(invDt), j.GetReactionTorque(invDt));
            }
            for (int i = 0; i < _ropeJointCount; ++i)
            {
                RopeJoint j = _ropeJoints[i];
                Consider(j.BodyA, j.BodyB, j.GetReactionForce(invDt), j.GetReactionTorque(invDt));
            }
            for (int i = 0; i < _frictionJointCount; ++i)
            {
                FrictionJoint j = _frictionJoints[i];
                Consider(j.BodyA, j.BodyB, j.GetReactionForce(invDt), j.GetReactionTorque(invDt));
            }
            // Gear joints don't accumulate impulse — reaction always zero.

            if (events.Count > 0)
            {
                _events.Raise(new JointEvents(events.ToArray()));
            }
        }

        internal void SyncSweeps()
        {
            for (int i = 0; i < _bodies.Count; ++i)
            {
                Body body = _bodies[i];
                if (body.Type == BodyType.Static)
                {
                    continue;
                }

                Vec2 center = body.GetWorldCenter();
                float angle = body.Transform.Q.Angle;
                body.Sweep = new Sweep(body.LocalCenter, body.Sweep.C0, center, body.Sweep.A0, angle, body.Sweep.Alpha0);
            }
        }

        private void RaiseContactImpulseEvents()
        {
            if (_awakeSet.Contacts.Count == 0)
            {
                return;
            }

            System.Collections.Generic.List<ContactImpulseEvent> impulseEvents = new System.Collections.Generic.List<ContactImpulseEvent>();
            for (int i = 0; i < _awakeSet.Contacts.Count; ++i)
            {
                Contact contact = _awakeSet.Contacts[i];
                if (contact.FixtureA == null || contact.FixtureB == null)
                {
                    continue;
                }

                if (contact.Manifold.PointCount == 0)
                {
                    continue;
                }

                Fixture fixtureA = contact.FixtureA;
                Fixture fixtureB = contact.FixtureB;
                Body bodyA = fixtureA.Body;
                Body bodyB = fixtureB.Body;

                float radiusA = GetShapeRadius(fixtureA.Shape);
                float radiusB = GetShapeRadius(fixtureB.Shape);
                WorldManifold worldManifold = new WorldManifold();
                worldManifold.Initialize(contact.Manifold, bodyA.Transform, radiusA, bodyB.Transform, radiusB);

                Vec2 normal = worldManifold.Normal;
                for (int p = 0; p < contact.Manifold.PointCount; ++p)
                {
                    ManifoldPoint mp = contact.Manifold.Points[p];
                    Vec2 point = worldManifold.Points[p];
                    impulseEvents.Add(new ContactImpulseEvent(
                        fixtureA.UserData,
                        fixtureB.UserData,
                        point,
                        normal,
                        mp.NormalImpulse,
                        mp.TangentImpulse));
                }
            }

            if (impulseEvents.Count > 0)
            {
                _events.Raise(new ContactImpulseEvents(impulseEvents.ToArray()));
            }
        }

        private void SolveContacts(float timeStep, bool warmStart)
        {
            SolveContacts(timeStep, warmStart, _contacts);
        }

        private void SolveContacts(float timeStep, bool warmStart, System.Collections.Generic.IReadOnlyList<Contact> contacts)
        {
            for (int i = 0; i < contacts.Count; ++i)
            {
                Contact contact = contacts[i];
                if (contact.FixtureA == null || contact.FixtureB == null)
                {
                    continue;
                }

                Fixture fixtureA = contact.FixtureA;
                Fixture fixtureB = contact.FixtureB;
                Body bodyA = fixtureA.Body;
                Body bodyB = fixtureB.Body;

                if (bodyA.Type == BodyType.Static && bodyB.Type == BodyType.Static)
                {
                    continue;
                }

                float radiusA = GetShapeRadius(fixtureA.Shape);
                float radiusB = GetShapeRadius(fixtureB.Shape);

                WorldManifold worldManifold = new WorldManifold();
                worldManifold.Initialize(contact.Manifold, bodyA.Transform, radiusA, bodyB.Transform, radiusB);

                Vec2 normal = worldManifold.Normal;
                Vec2 tangent = new Vec2(-normal.Y, normal.X);

                float friction = MixFriction(fixtureA, fixtureB);
                float invMassA = bodyA.InverseMass;
                float invMassB = bodyB.InverseMass;
                float invIA = bodyA.InverseInertia;
                float invIB = bodyB.InverseInertia;
                int pointCount = contact.Manifold.PointCount;
                int solvePointCount = pointCount;
                int primaryIndex = 0;
                bool useBlockSolver = false;
                float k11 = 0f;
                float k22 = 0f;
                float k12 = 0f;
                float det = 0f;

                if (pointCount == 2)
                {
                    SolverPoint sp1 = contact.SolverPoints[0];
                    SolverPoint sp2 = contact.SolverPoints[1];

                    Vec2 rA1 = sp1.RA;
                    Vec2 rB1 = sp1.RB;
                    Vec2 rA2 = sp2.RA;
                    Vec2 rB2 = sp2.RB;

                    float rnA1 = Vec2.Cross(rA1, normal);
                    float rnB1 = Vec2.Cross(rB1, normal);
                    float rnA2 = Vec2.Cross(rA2, normal);
                    float rnB2 = Vec2.Cross(rB2, normal);

                    k11 = invMassA + invMassB + invIA * rnA1 * rnA1 + invIB * rnB1 * rnB1;
                    k22 = invMassA + invMassB + invIA * rnA2 * rnA2 + invIB * rnB2 * rnB2;
                    k12 = invMassA + invMassB + invIA * rnA1 * rnA2 + invIB * rnB1 * rnB2;

                    det = k11 * k22 - k12 * k12;
                    const float kMaxConditionNumber = 1000f;
                    if (det > Constants.Epsilon && k11 * k11 < kMaxConditionNumber * det)
                    {
                        useBlockSolver = true;
                    }
                    if (!useBlockSolver)
                    {
                        solvePointCount = 1;
                        PositionManifold pm0 = ComputePositionManifold(contact, bodyA.Transform, bodyB.Transform, radiusA, radiusB, 0);
                        PositionManifold pm1 = ComputePositionManifold(contact, bodyA.Transform, bodyB.Transform, radiusA, radiusB, 1);
                        if (pm1.Separation < pm0.Separation)
                        {
                            primaryIndex = 1;
                        }

                        int otherIndex = 1 - primaryIndex;
                        ManifoldPoint otherMp = contact.Manifold.Points[otherIndex];
                        if (otherMp.NormalImpulse != 0f || otherMp.TangentImpulse != 0f)
                        {
                            contact.Manifold.Points[otherIndex] = new ManifoldPoint(otherMp.LocalPoint, 0f, 0f, otherMp.Id);
                        }
                    }
                }

                // Warm start
                if (warmStart)
                {
                    if (solvePointCount == 1)
                    {
                        int index = pointCount == 2 ? primaryIndex : 0;
                        SolverPoint sp = contact.SolverPoints[index];
                        Vec2 rA = sp.RA;
                        Vec2 rB = sp.RB;
                        ManifoldPoint mp = contact.Manifold.Points[index];
                        if (mp.NormalImpulse != 0f || mp.TangentImpulse != 0f)
                        {
                            Vec2 warm = mp.NormalImpulse * normal + mp.TangentImpulse * tangent;
                            bodyA.LinearVelocity -= invMassA * warm;
                            bodyB.LinearVelocity += invMassB * warm;
                            bodyA.AngularVelocity -= invIA * Vec2.Cross(rA, warm);
                            bodyB.AngularVelocity += invIB * Vec2.Cross(rB, warm);
                        }
                    }
                    else
                    {
                        for (int p = 0; p < pointCount; ++p)
                        {
                            SolverPoint sp = contact.SolverPoints[p];
                            Vec2 rA = sp.RA;
                            Vec2 rB = sp.RB;
                            ManifoldPoint mp = contact.Manifold.Points[p];
                            if (mp.NormalImpulse == 0f && mp.TangentImpulse == 0f)
                            {
                                continue;
                            }

                            Vec2 warm = mp.NormalImpulse * normal + mp.TangentImpulse * tangent;
                            bodyA.LinearVelocity -= invMassA * warm;
                            bodyB.LinearVelocity += invMassB * warm;
                            bodyA.AngularVelocity -= invIA * Vec2.Cross(rA, warm);
                            bodyB.AngularVelocity += invIB * Vec2.Cross(rB, warm);
                        }
                    }
                }

                // Friction constraints
                if (solvePointCount == 1)
                {
                    int index = pointCount == 2 ? primaryIndex : 0;
                    SolverPoint sp = contact.SolverPoints[index];
                    Vec2 rA = sp.RA;
                    Vec2 rB = sp.RB;
                    ManifoldPoint mp = contact.Manifold.Points[index];

                    Vec2 vA = bodyA.LinearVelocity + Vec2.Cross(bodyA.AngularVelocity, rA);
                    Vec2 vB = bodyB.LinearVelocity + Vec2.Cross(bodyB.AngularVelocity, rB);
                    Vec2 dv = vB - vA;
                    float vt = Vec2.Dot(dv, tangent);

                    if (sp.TangentMass > 0f)
                    {
                        float tangentImpulse = -vt * sp.TangentMass;
                        float maxFriction = friction * mp.NormalImpulse;
                        float newTangentImpulse = MathFng.Clamp(mp.TangentImpulse + tangentImpulse, -maxFriction, maxFriction);
                        float appliedTangent = newTangentImpulse - mp.TangentImpulse;
                        Vec2 Pt = appliedTangent * tangent;

                        bodyA.LinearVelocity -= invMassA * Pt;
                        bodyB.LinearVelocity += invMassB * Pt;
                        bodyA.AngularVelocity -= invIA * Vec2.Cross(rA, Pt);
                        bodyB.AngularVelocity += invIB * Vec2.Cross(rB, Pt);

                        mp = new ManifoldPoint(mp.LocalPoint, mp.NormalImpulse, newTangentImpulse, mp.Id);
                        contact.Manifold.Points[index] = mp;
                    }
                }
                else
                {
                    for (int p = 0; p < pointCount; ++p)
                    {
                        SolverPoint sp = contact.SolverPoints[p];
                        Vec2 rA = sp.RA;
                        Vec2 rB = sp.RB;
                        ManifoldPoint mp = contact.Manifold.Points[p];

                        Vec2 vA = bodyA.LinearVelocity + Vec2.Cross(bodyA.AngularVelocity, rA);
                        Vec2 vB = bodyB.LinearVelocity + Vec2.Cross(bodyB.AngularVelocity, rB);
                        Vec2 dv = vB - vA;
                        float vt = Vec2.Dot(dv, tangent);

                        if (sp.TangentMass > 0f)
                        {
                            float tangentImpulse = -vt * sp.TangentMass;
                            float maxFriction = friction * mp.NormalImpulse;
                            float newTangentImpulse = MathFng.Clamp(mp.TangentImpulse + tangentImpulse, -maxFriction, maxFriction);
                            float appliedTangent = newTangentImpulse - mp.TangentImpulse;
                            Vec2 Pt = appliedTangent * tangent;

                            bodyA.LinearVelocity -= invMassA * Pt;
                            bodyB.LinearVelocity += invMassB * Pt;
                            bodyA.AngularVelocity -= invIA * Vec2.Cross(rA, Pt);
                            bodyB.AngularVelocity += invIB * Vec2.Cross(rB, Pt);

                            mp = new ManifoldPoint(mp.LocalPoint, mp.NormalImpulse, newTangentImpulse, mp.Id);
                            contact.Manifold.Points[p] = mp;
                        }
                    }
                }

                // Normal constraints
                if (solvePointCount == 1)
                {
                    int index = pointCount == 2 ? primaryIndex : 0;
                    SolverPoint sp = contact.SolverPoints[index];
                    Vec2 rA = sp.RA;
                    Vec2 rB = sp.RB;
                    ManifoldPoint mp = contact.Manifold.Points[index];

                    Vec2 vA = bodyA.LinearVelocity + Vec2.Cross(bodyA.AngularVelocity, rA);
                    Vec2 vB = bodyB.LinearVelocity + Vec2.Cross(bodyB.AngularVelocity, rB);
                    Vec2 dv = vB - vA;

                    float vn = Vec2.Dot(dv, normal);
                    float impulse = -(vn - sp.VelocityBias + sp.Bias + sp.Softness * mp.NormalImpulse) * sp.NormalMass;
                    float newNormalImpulse = MathF.Max(mp.NormalImpulse + impulse, 0f);
                    float appliedNormal = newNormalImpulse - mp.NormalImpulse;
                    Vec2 Pn = appliedNormal * normal;

                    bodyA.LinearVelocity -= invMassA * Pn;
                    bodyB.LinearVelocity += invMassB * Pn;
                    bodyA.AngularVelocity -= invIA * Vec2.Cross(rA, Pn);
                    bodyB.AngularVelocity += invIB * Vec2.Cross(rB, Pn);

                    mp = new ManifoldPoint(mp.LocalPoint, newNormalImpulse, mp.TangentImpulse, mp.Id);
                    contact.Manifold.Points[index] = mp;
                }
                else if (useBlockSolver)
                {
                    SolverPoint sp1 = contact.SolverPoints[0];
                    SolverPoint sp2 = contact.SolverPoints[1];
                    ManifoldPoint mp1 = contact.Manifold.Points[0];
                    ManifoldPoint mp2 = contact.Manifold.Points[1];

                    Vec2 rA1 = sp1.RA;
                    Vec2 rB1 = sp1.RB;
                    Vec2 rA2 = sp2.RA;
                    Vec2 rB2 = sp2.RB;

                    float k11Soft = k11 + sp1.Softness;
                    float k22Soft = k22 + sp2.Softness;
                    float detSoft = k11Soft * k22Soft - k12 * k12;
                    float invDet = detSoft != 0f ? 1f / detSoft : 0f;
                    Mat22 normalMass = new Mat22(
                        new Vec2(invDet * k22Soft, -invDet * k12),
                        new Vec2(-invDet * k12, invDet * k11Soft));

                    Vec2 vA1 = bodyA.LinearVelocity + Vec2.Cross(bodyA.AngularVelocity, rA1);
                    Vec2 vB1 = bodyB.LinearVelocity + Vec2.Cross(bodyB.AngularVelocity, rB1);
                    Vec2 vA2 = bodyA.LinearVelocity + Vec2.Cross(bodyA.AngularVelocity, rA2);
                    Vec2 vB2 = bodyB.LinearVelocity + Vec2.Cross(bodyB.AngularVelocity, rB2);

                    float vn1 = Vec2.Dot(vB1 - vA1, normal);
                    float vn2 = Vec2.Dot(vB2 - vA2, normal);

                    float b1 = vn1 - sp1.VelocityBias + sp1.Bias + sp1.Softness * mp1.NormalImpulse;
                    float b2 = vn2 - sp2.VelocityBias + sp2.Bias + sp2.Softness * mp2.NormalImpulse;

                    float ax = mp1.NormalImpulse;
                    float ay = mp2.NormalImpulse;

                    // Compute b' = b - K * a
                    b1 -= k11Soft * ax + k12 * ay;
                    b2 -= k12 * ax + k22Soft * ay;

                    // Case 1: both impulses positive
                    Vec2 x = -Mat22.Mul(normalMass, new Vec2(b1, b2));
                    if (x.X >= 0f && x.Y >= 0f)
                    {
                        ApplyNormalBlock(x.X - ax, x.Y - ay, normal, rA1, rB1, rA2, rB2, ref bodyA, ref bodyB, invMassA, invIA, invMassB, invIB);
                        mp1 = new ManifoldPoint(mp1.LocalPoint, x.X, mp1.TangentImpulse, mp1.Id);
                        mp2 = new ManifoldPoint(mp2.LocalPoint, x.Y, mp2.TangentImpulse, mp2.Id);
                        contact.Manifold.Points[0] = mp1;
                        contact.Manifold.Points[1] = mp2;
                        continue;
                    }

                    // Case 2: x2 = 0
                    float x1 = k11Soft > 0f ? -b1 / k11Soft : 0f;
                    float vn2Case2 = k12 * x1 + b2;
                    if (x1 >= 0f && vn2Case2 >= 0f)
                    {
                        ApplyNormalBlock(x1 - ax, -ay, normal, rA1, rB1, rA2, rB2, ref bodyA, ref bodyB, invMassA, invIA, invMassB, invIB);
                        mp1 = new ManifoldPoint(mp1.LocalPoint, x1, mp1.TangentImpulse, mp1.Id);
                        mp2 = new ManifoldPoint(mp2.LocalPoint, 0f, mp2.TangentImpulse, mp2.Id);
                        contact.Manifold.Points[0] = mp1;
                        contact.Manifold.Points[1] = mp2;
                        continue;
                    }

                    // Case 3: x1 = 0
                    float x2 = k22Soft > 0f ? -b2 / k22Soft : 0f;
                    float vn1Case3 = k12 * x2 + b1;
                    if (x2 >= 0f && vn1Case3 >= 0f)
                    {
                        ApplyNormalBlock(-ax, x2 - ay, normal, rA1, rB1, rA2, rB2, ref bodyA, ref bodyB, invMassA, invIA, invMassB, invIB);
                        mp1 = new ManifoldPoint(mp1.LocalPoint, 0f, mp1.TangentImpulse, mp1.Id);
                        mp2 = new ManifoldPoint(mp2.LocalPoint, x2, mp2.TangentImpulse, mp2.Id);
                        contact.Manifold.Points[0] = mp1;
                        contact.Manifold.Points[1] = mp2;
                    }
                }
            }
        }

        private static void ApplyNormalBlock(
            float dx1,
            float dx2,
            Vec2 normal,
            Vec2 rA1,
            Vec2 rB1,
            Vec2 rA2,
            Vec2 rB2,
            ref Body bodyA,
            ref Body bodyB,
            float invMassA,
            float invIA,
            float invMassB,
            float invIB)
        {
            Vec2 P1 = dx1 * normal;
            Vec2 P2 = dx2 * normal;
            Vec2 P = P1 + P2;

            bodyA.LinearVelocity -= invMassA * P;
            bodyB.LinearVelocity += invMassB * P;
            bodyA.AngularVelocity -= invIA * (Vec2.Cross(rA1, P1) + Vec2.Cross(rA2, P2));
            bodyB.AngularVelocity += invIB * (Vec2.Cross(rB1, P1) + Vec2.Cross(rB2, P2));
        }

        private void SolveTOI()
        {
            if (_def.EnableContinuous == false)
            {
                return;
            }

            float timeStep = _prevTimeStep;
            if (timeStep <= 0f)
            {
                return;
            }

            if (_def.UsePerBodyCCD)
            {
                SolveTOIPerBody(timeStep);
                return;
            }

            HashSet64 processed = new HashSet64(256);
            System.Collections.Generic.List<SensorHitEvent> sensorHitEvents = new System.Collections.Generic.List<SensorHitEvent>();
            HashSet64 sensorHitKeys = new HashSet64(128);

            System.Collections.Generic.List<Contact> orderedContacts = new System.Collections.Generic.List<Contact>(_contactMap.Count);
            foreach (Contact contact in _contactMap.Values)
            {
                orderedContacts.Add(contact);
            }

            orderedContacts.Sort(CompareContacts);
            for (int i = 0; i < orderedContacts.Count; ++i)
            {
                Contact contact = orderedContacts[i];
                if (contact.FixtureA == null || contact.FixtureB == null)
                {
                    continue;
                }

                processed.Add(MakeProxyPairKey(contact.FixtureA.ProxyId, contact.FixtureB.ProxyId));
                ProcessTOI(contact, timeStep);
            }

            for (int i = 0; i < _bodies.Count; ++i)
            {
                Body movingBody = _bodies[i];
                if (movingBody.Type == BodyType.Static)
                {
                    continue;
                }

                Vec2 translation = movingBody.Sweep.C - movingBody.Sweep.C0;
                float minTranslation = MathF.Max(Constants.LinearSlop, 0.25f * movingBody.Sweep.LocalCenter.Length);
                if (translation.LengthSquared <= minTranslation * minTranslation)
                {
                    continue;
                }

                for (int f = 0; f < movingBody.Fixtures.Count; ++f)
                {
                    Fixture fixture = movingBody.Fixtures[f];
                    if (fixture.IsSensor)
                    {
                        continue;
                    }
                    if (!fixture.EnableSensorEvents)
                    {
                        continue;
                    }

                    Transform startTransform = movingBody.Sweep.GetTransform(0f);
                    ShapeProxy proxy = BuildWorldProxy(fixture.Shape, startTransform);
                    ShapeCastInput input = new ShapeCastInput(proxy, translation, 1f, false);
                    float minSolidFraction = 1f;
                    _broadPhase.ShapeCast(otherId =>
                    {
                        Fixture? otherFixture = _broadPhase.GetUserData(otherId);
                        if (otherFixture == null)
                        {
                            return 1f;
                        }

                        if (otherFixture == fixture)
                        {
                            return 1f;
                        }

                        if (otherFixture.Body == fixture.Body)
                        {
                            return 1f;
                        }

                        if (otherFixture.IsSensor)
                        {
                            if (!ShouldSensorOverlap(otherFixture, fixture))
                            {
                                return 1f;
                            }

                            if (!TryShapeCast(otherFixture, fixture, startTransform, translation, out CastOutput cast))
                            {
                                return 1f;
                            }

                            if (!cast.Hit || cast.Fraction > minSolidFraction)
                            {
                                return 1f;
                            }

                            ulong hitKey = MakeProxyPairKey(otherFixture.ProxyId, fixture.ProxyId);
                            if (!sensorHitKeys.Add(hitKey))
                            {
                                sensorHitEvents.Add(new SensorHitEvent(
                                    otherFixture.UserData,
                                    fixture.UserData,
                                    cast.Point,
                                    cast.Normal,
                                    cast.Fraction));
                            }
                            return 1f;
                        }

                        if (!ShouldCollide(fixture, otherFixture))
                        {
                            return 1f;
                        }

                        if (TryShapeCast(otherFixture, fixture, startTransform, translation, out CastOutput solidCast) &&
                            solidCast.Hit)
                        {
                            if (solidCast.Fraction < minSolidFraction)
                            {
                                minSolidFraction = solidCast.Fraction;
                            }
                        }

                        ulong key = MakeProxyPairKey(fixture.ProxyId, otherFixture.ProxyId);
                        if (processed.Contains(key))
                        {
                            return 1f;
                        }

                        processed.Add(key);
                        Contact temp = new Contact(fixture, otherFixture);
                        temp.Update(fixture.Body.Transform, otherFixture.Body.Transform);
                        ProcessTOI(temp, timeStep);
                        return 1f;
                    }, input);
                }
            }

            if (sensorHitEvents.Count > 0)
            {
                _events.Raise(new SensorHitEvents(sensorHitEvents.ToArray()));
            }
        }

        // Phase 3 of TIER4_PARITY_PLAN: per-body CCD matching cpp v3's
        // `b2SolveContinuous` ([box2d-cpp/src/solver.c:379](../box2d-cpp/src/solver.c)).
        // For each fast or bullet dynamic body, sweep its shapes against the
        // broadphase and advance the body's transform to the earliest TOI. No
        // per-contact impulse work — the next step's regular solver handles
        // contacts. Keeps joint-coupled structures consistent (welded chains,
        // breakable bodies) because no body moves while its joint partners stay
        // behind.
        private void SolveTOIPerBody(float timeStep)
        {
            System.Collections.Generic.List<SensorHitEvent> sensorHitEvents =
                new System.Collections.Generic.List<SensorHitEvent>();
            HashSet64 sensorHitKeys = new HashSet64(128);

            for (int i = 0; i < _bodies.Count; ++i)
            {
                Body body = _bodies[i];
                if (body.Type != BodyType.Dynamic)
                {
                    continue;
                }
                if (_def.EnableSleep && !body.Awake)
                {
                    continue;
                }

                bool isBullet = body.Definition.Bullet;
                if (!isBullet && !IsFastBody(body, timeStep))
                {
                    continue;
                }

                SolveContinuousBody(body, isBullet, sensorHitEvents, sensorHitKeys);
            }

            if (sensorHitEvents.Count > 0)
            {
                _events.Raise(new SensorHitEvents(sensorHitEvents.ToArray()));
            }
        }

        /// <summary>
        /// "Fast" criterion from cpp: body motion this step exceeds half the
        /// smallest fixture extent. Bullets bypass this check (always run CCD).
        /// </summary>
        private static bool IsFastBody(Body body, float timeStep)
        {
            Vec2 translation = body.Sweep.C - body.Sweep.C0;
            float maxDeltaPosition = translation.Length;
            float maxVelocity = body.LinearVelocity.Length * timeStep;
            float motion = MathF.Max(maxDeltaPosition, maxVelocity);

            float minExtent = float.MaxValue;
            for (int i = 0; i < body.Fixtures.Count; ++i)
            {
                Aabb a = body.Fixtures[i].Aabb;
                float halfMin = MathF.Min(a.UpperBound.X - a.LowerBound.X, a.UpperBound.Y - a.LowerBound.Y) * 0.5f;
                if (halfMin < minExtent) minExtent = halfMin;
            }
            if (minExtent >= float.MaxValue) return false;

            const float safetyFactor = 0.5f;
            return motion > safetyFactor * minExtent;
        }

        private void SolveContinuousBody(Body body, bool isBullet,
            System.Collections.Generic.List<SensorHitEvent> sensorHitEvents,
            HashSet64 sensorHitKeys)
        {
            Sweep bodySweep = body.Sweep;
            float minFraction = 1f;

            for (int f = 0; f < body.Fixtures.Count; ++f)
            {
                Fixture fixture = body.Fixtures[f];
                if (fixture.IsSensor)
                {
                    continue;
                }

                Transform startXf = bodySweep.GetTransform(0f);
                Transform endXf = bodySweep.GetTransform(1f);
                Aabb startAabb = ShapeGeometry.ComputeAabb(fixture.Shape, startXf);
                Aabb endAabb = ShapeGeometry.ComputeAabb(fixture.Shape, endXf);
                Aabb sweptAabb = Aabb.Union(startAabb, endAabb);

                Fixture localFixture = fixture;
                Sweep localBodySweep = bodySweep;
                float localMinFraction = minFraction;
                Vec2 translation = bodySweep.C - bodySweep.C0;

                _broadPhase.Query(otherId =>
                {
                    Fixture? other = _broadPhase.GetUserData(otherId);
                    if (other == null) return true;
                    if (other.Body == body) return true;

                    // Non-bullet fast bodies only sweep against static partners
                    // (cpp's b2SolveContinuous restricts non-bullet queries to
                    // the static tree). Bullets sweep against everything.
                    if (!isBullet && other.Body.Type != BodyType.Static) return true;

                    if (other.IsSensor)
                    {
                        if (localFixture.EnableSensorEvents && ShouldSensorOverlap(other, localFixture))
                        {
                            if (TryShapeCast(other, localFixture, startXf, translation, out CastOutput cast) && cast.Hit)
                            {
                                ulong hitKey = MakeProxyPairKey(other.ProxyId, localFixture.ProxyId);
                                // HashSet64.Add returns true if key was ALREADY
                                // present (inverted from .NET HashSet<T>.Add).
                                // Fire the event only on the first sighting.
                                if (!sensorHitKeys.Add(hitKey))
                                {
                                    sensorHitEvents.Add(new SensorHitEvent(
                                        other.UserData,
                                        localFixture.UserData,
                                        cast.Point,
                                        cast.Normal,
                                        cast.Fraction));
                                }
                            }
                        }
                        return true;
                    }

                    if (!ShouldCollide(localFixture, other)) return true;

                    ShapeProxy proxyA = ShapeGeometry.ToProxy(localFixture.Shape);
                    ShapeProxy proxyB = ShapeGeometry.ToProxy(other.Shape);
                    ToiInput input = new ToiInput(proxyA, proxyB, localBodySweep, other.Body.Sweep, localMinFraction);
                    ToiOutput output = TimeOfImpact.Compute(input);

                    if (output.State == ToiState.Hit && output.Fraction > 0f && output.Fraction < localMinFraction)
                    {
                        localMinFraction = output.Fraction;
                    }
                    return true;
                }, sweptAabb);

                if (localMinFraction < minFraction) minFraction = localMinFraction;
            }

            if (minFraction < 1f)
            {
                AdvanceBodyToTOI(body, minFraction);

                // Refresh AABBs for the advanced position so the next step's
                // broadphase / narrowphase sees the correct bounds.
                for (int f = 0; f < body.Fixtures.Count; ++f)
                {
                    Fixture fixture = body.Fixtures[f];
                    Aabb newAabb = ShapeGeometry.ComputeAabb(fixture.Shape, body.Transform);
                    Vec2 oldCenter = fixture.Aabb.Center;
                    Vec2 newCenter = newAabb.Center;
                    _broadPhase.MoveProxy(fixture.ProxyId, newAabb, newCenter - oldCenter);
                    fixture.Aabb = newAabb;
                }
            }
        }

        private void AdvanceBodyToTOI(Body body, float alpha)
        {
            if (body.Type == BodyType.Static)
            {
                return;
            }

            Sweep sweep = body.Sweep.Advance(alpha);
            body.SetTransformFromSweep(sweep);
        }

        private void ProcessTOI(Contact contact, float timeStep)
        {
            Body? bodyA = contact.FixtureA?.Body;
            Body? bodyB = contact.FixtureB?.Body;

            if (bodyA == null || bodyB == null)
            {
                return;
            }

            if (contact.FixtureA!.IsSensor || contact.FixtureB!.IsSensor)
            {
                return;
            }

            if (bodyA.Type == BodyType.Static && bodyB.Type == BodyType.Static)
            {
                return;
            }

            if (bodyA.Type == BodyType.Kinematic && bodyB.Type == BodyType.Kinematic &&
                !bodyA.Definition.Bullet && !bodyB.Definition.Bullet)
            {
                return;
            }

            if (contact.IsTouching && !bodyA.Definition.Bullet && !bodyB.Definition.Bullet)
            {
                return;
            }

            if (contact.ToiCount > _def.MaxSubSteps)
            {
                return;
            }

            bool collideA = bodyA.Definition.Bullet || bodyA.Type != BodyType.Dynamic;
            bool collideB = bodyB.Definition.Bullet || bodyB.Type != BodyType.Dynamic;
            if (!collideA && !collideB)
            {
                return;
            }


            float toi;
            if (contact.HasToi)
            {
                toi = contact.Toi;
            }
            else
            {
                ShapeProxy proxyA = ShapeGeometry.ToProxy(contact.ShapeA);
                ShapeProxy proxyB = ShapeGeometry.ToProxy(contact.ShapeB);
                ToiInput toiInput = new ToiInput(proxyA, proxyB, bodyA.Sweep, bodyB.Sweep, 1f);
                ToiOutput output = TimeOfImpact.Compute(toiInput);

                // Match the cpp behaviour at box2d-cpp/src/solver.c:340 — only the
                // open interval (0, MaxFraction) counts as a useful hit. Overlapped
                // and zero-fraction hits mean the bodies are already touching; let
                // the regular contact pipeline resolve them.
                if (output.State != ToiState.Hit || output.Fraction <= 0f || output.Fraction > 1f)
                {
                    return;
                }

                toi = output.Fraction;
                contact.CacheToi(toi);
            }

            AdvanceBodyToTOI(bodyA, toi);
            AdvanceBodyToTOI(bodyB, toi);

            if (bodyA.Type != BodyType.Static)
            {
                bodyA.SetAwake(true);
            }
            if (bodyB.Type != BodyType.Static)
            {
                bodyB.SetAwake(true);
            }

            contact.Update(bodyA.Transform, bodyB.Transform);
            if (contact.Manifold.PointCount == 0)
            {
                return;
            }

            int subSteps = _def.MaxSubSteps;
            float remaining = timeStep * (1f - toi);
            if (remaining <= 0f)
            {
                return;
            }

            float subDt = remaining / subSteps;
            for (int s = 0; s < subSteps; ++s)
            {
                IntegrateForTOI(bodyA, subDt);
                IntegrateForTOI(bodyB, subDt);

                contact.Update(bodyA.Transform, bodyB.Transform);
                if (contact.Manifold.PointCount == 0)
                {
                    continue;
                }

                // Each sub-step is a fresh integrate-and-solve over a short dt.
                // Zero cached impulses so the velocity-iteration loop drives
                // vn -> 0 cleanly within this sub-step without inheriting state.
                for (int p = 0; p < contact.Manifold.PointCount; ++p)
                {
                    ManifoldPoint mp = contact.Manifold.Points[p];
                    contact.Manifold.Points[p] = new ManifoldPoint(mp.LocalPoint, 0f, 0f, mp.Id);
                }

                PrepareContact(contact, subDt, 1f);
                for (int it = 0; it < _def.VelocityIterations; ++it)
                {
                    SolveContact(contact, warmStart: false);
                }
                for (int it = 0; it < _def.PositionIterations; ++it)
                {
                    SolvePositionContact(contact);
                }
            }

            contact.IncrementToiCount();
        }

        private void IntegrateForTOI(Body body, float dt)
        {
            if (body.Type == BodyType.Static)
            {
                return;
            }

            Vec2 oldCenter = body.GetWorldCenter();
            float oldAngle = body.Transform.Q.Angle;

            if (body.Type == BodyType.Dynamic)
            {
                Vec2 accel = body.GravityScale * Gravity;
                if (body.InverseMass > 0f)
                {
                    accel += body.InverseMass * body.Force;
                }
                body.LinearVelocity += dt * accel;
                if (body.InverseInertia > 0f)
                {
                    body.AngularVelocity += dt * body.InverseInertia * body.Torque;
                }
                body.LinearVelocity = ApplyLinearDamping(body.LinearVelocity, body.LinearDamping, dt);
                body.AngularVelocity = ApplyAngularDamping(body.AngularVelocity, body.AngularDamping, dt);
            }

            Vec2 newCenter = oldCenter + dt * body.LinearVelocity;
            float newAngle = oldAngle + dt * body.AngularVelocity;
            body.SetTransformFromCenter(newCenter, newAngle);
            body.Sweep = new Sweep(body.LocalCenter, oldCenter, newCenter, oldAngle, newAngle, 0f);
        }

        private static ShapeProxy BuildWorldProxy(Shape shape, Transform transform)
        {
            ShapeProxy localProxy = ShapeGeometry.ToProxy(shape);
            Vec2[] points = new Vec2[localProxy.Count];
            for (int i = 0; i < localProxy.Count; ++i)
            {
                points[i] = Transform.Mul(transform, localProxy.Points[i]);
            }
            return new ShapeProxy(points, localProxy.Count, localProxy.Radius);
        }

        private static bool TryShapeCast(Fixture stationary, Fixture moving, Transform movingStart, Vec2 translation, out CastOutput output)
        {
            ShapeProxy proxyA = ShapeGeometry.ToProxy(stationary.Shape);
            ShapeProxy proxyB = ShapeGeometry.ToProxy(moving.Shape);
            ShapeCastPairInput pairInput = new ShapeCastPairInput(
                proxyA,
                proxyB,
                stationary.Body.Transform,
                movingStart,
                translation,
                1f,
                false);
            output = ShapeCast.Cast(pairInput);
            return true;
        }

        private Body? FindBodyForShape(Shape shape)
        {
            for (int i = 0; i < _bodies.Count; ++i)
            {
                Body body = _bodies[i];
                for (int j = 0; j < body.Fixtures.Count; ++j)
                {
                    if (body.Fixtures[j].Shape == shape)
                    {
                        return body;
                    }
                }
            }
            return null;
        }

        private void PrepareContact(Contact contact, float timeStep, float dtRatio)
        {
            if (contact.FixtureA == null || contact.FixtureB == null)
            {
                return;
            }

            Fixture fixtureA = contact.FixtureA;
            Fixture fixtureB = contact.FixtureB;
            Body bodyA = fixtureA.Body;
            Body bodyB = fixtureB.Body;

            float radiusA = GetShapeRadius(fixtureA.Shape);
            float radiusB = GetShapeRadius(fixtureB.Shape);

            WorldManifold worldManifold = new WorldManifold();
            worldManifold.Initialize(contact.Manifold, bodyA.Transform, radiusA, bodyB.Transform, radiusB);

            Vec2 normal = worldManifold.Normal;
            Vec2 centerA = bodyA.GetWorldCenter();
            Vec2 centerB = bodyB.GetWorldCenter();
            for (int p = 0; p < contact.Manifold.PointCount; ++p)
            {
                ManifoldPoint mp = contact.Manifold.Points[p];
                if (dtRatio != 1f)
                {
                    mp = new ManifoldPoint(mp.LocalPoint, mp.NormalImpulse * dtRatio, mp.TangentImpulse * dtRatio, mp.Id);
                    contact.Manifold.Points[p] = mp;
                }

                Vec2 point = worldManifold.Points[p];
                Vec2 rA = point - centerA;
                Vec2 rB = point - centerB;

                float rnA = Vec2.Cross(rA, normal);
                float rnB = Vec2.Cross(rB, normal);
                float kNormal = bodyA.InverseMass + bodyB.InverseMass +
                                bodyA.InverseInertia * rnA * rnA + bodyB.InverseInertia * rnB * rnB;
                float normalMass = kNormal > 0f ? 1f / kNormal : 0f;

                Vec2 tangent = new Vec2(-normal.Y, normal.X);
                float rtA = Vec2.Cross(rA, tangent);
                float rtB = Vec2.Cross(rB, tangent);
                float kTangent = bodyA.InverseMass + bodyB.InverseMass +
                                 bodyA.InverseInertia * rtA * rtA + bodyB.InverseInertia * rtB * rtB;
                float tangentMass = kTangent > 0f ? 1f / kTangent : 0f;

                Vec2 vA = bodyA.LinearVelocity + Vec2.Cross(bodyA.AngularVelocity, rA);
                Vec2 vB = bodyB.LinearVelocity + Vec2.Cross(bodyB.AngularVelocity, rB);
                float vn = Vec2.Dot(vB - vA, normal);

                float velocityBias = 0f;
                float softness = 0f;
                float bias = 0f;
                float restitution = MixRestitution(fixtureA, fixtureB);
                if (vn < -_def.RestitutionThreshold)
                {
                    velocityBias = -restitution * vn;
                }
                if (_def.EnableContactSoftening)
                {
                    float separation = Vec2.Dot((centerB + rB) - (centerA + rA), normal);
                    if (_def.UseSoftConstraints)
                    {
                        ComputeContactSoftness(kNormal, timeStep, separation, out bias, out softness);
                        if (softness > 0f)
                        {
                            normalMass = 1f / (kNormal + softness);
                        }
                    }
                    else
                    {
                        float baumgarte = Constants.Baumgarte * (separation + Constants.LinearSlop) / timeStep;
                        float maxBias = MathF.Max(_def.ContactSpeed, 0f);
                        float legacyBias = MathFng.Clamp(baumgarte, -maxBias, 0f);
                        float baumgarteClamped = MathFng.Clamp(legacyBias, -Constants.MaxLinearCorrection, 0f);
                        velocityBias -= baumgarteClamped;
                    }
                }

                contact.SolverPoints[p] = new SolverPoint
                {
                    RA = rA,
                    RB = rB,
                    NormalMass = normalMass,
                    TangentMass = tangentMass,
                    VelocityBias = velocityBias,
                    Softness = softness,
                    Bias = bias
                };
            }
        }

        private void SolveContact(Contact contact, bool warmStart)
        {
            if (contact.FixtureA == null || contact.FixtureB == null)
            {
                return;
            }

            Fixture fixtureA = contact.FixtureA;
            Fixture fixtureB = contact.FixtureB;
            Body bodyA = fixtureA.Body;
            Body bodyB = fixtureB.Body;

            if (bodyA.Type == BodyType.Static && bodyB.Type == BodyType.Static)
            {
                return;
            }

            float radiusA = GetShapeRadius(fixtureA.Shape);
            float radiusB = GetShapeRadius(fixtureB.Shape);

            WorldManifold worldManifold = new WorldManifold();
            worldManifold.Initialize(contact.Manifold, bodyA.Transform, radiusA, bodyB.Transform, radiusB);

            Vec2 normal = worldManifold.Normal;
            Vec2 tangent = new Vec2(-normal.Y, normal.X);

            float friction = MathF.Sqrt(fixtureA.Friction * fixtureB.Friction);
            float invMassA = bodyA.InverseMass;
            float invMassB = bodyB.InverseMass;
            float invIA = bodyA.InverseInertia;
            float invIB = bodyB.InverseInertia;

            for (int p = 0; p < contact.Manifold.PointCount; ++p)
            {
                SolverPoint sp = contact.SolverPoints[p];
                Vec2 rA = sp.RA;
                Vec2 rB = sp.RB;

                Vec2 vA = bodyA.LinearVelocity + Vec2.Cross(bodyA.AngularVelocity, rA);
                Vec2 vB = bodyB.LinearVelocity + Vec2.Cross(bodyB.AngularVelocity, rB);
                Vec2 dv = vB - vA;

                ManifoldPoint mp = contact.Manifold.Points[p];

                if (warmStart && (mp.NormalImpulse != 0f || mp.TangentImpulse != 0f))
                {
                    Vec2 warm = mp.NormalImpulse * normal + mp.TangentImpulse * tangent;
                    bodyA.LinearVelocity -= invMassA * warm;
                    bodyB.LinearVelocity += invMassB * warm;
                    bodyA.AngularVelocity -= invIA * Vec2.Cross(rA, warm);
                    bodyB.AngularVelocity += invIB * Vec2.Cross(rB, warm);
                }

                Vec2 vA2 = bodyA.LinearVelocity + Vec2.Cross(bodyA.AngularVelocity, rA);
                Vec2 vB2 = bodyB.LinearVelocity + Vec2.Cross(bodyB.AngularVelocity, rB);
                Vec2 dv2 = vB2 - vA2;
                float vn = Vec2.Dot(dv2, normal);
                if (vn > 0f)
                {
                    continue;
                }

                float impulse = -(vn - sp.VelocityBias + sp.Bias + sp.Softness * mp.NormalImpulse) * sp.NormalMass;
                float newNormalImpulse = MathF.Max(mp.NormalImpulse + impulse, 0f);
                float appliedNormal = newNormalImpulse - mp.NormalImpulse;
                Vec2 Pn = appliedNormal * normal;
                bodyA.LinearVelocity -= invMassA * Pn;
                bodyB.LinearVelocity += invMassB * Pn;
                bodyA.AngularVelocity -= invIA * Vec2.Cross(rA, Pn);
                bodyB.AngularVelocity += invIB * Vec2.Cross(rB, Pn);

                Vec2 vA3 = bodyA.LinearVelocity + Vec2.Cross(bodyA.AngularVelocity, rA);
                Vec2 vB3 = bodyB.LinearVelocity + Vec2.Cross(bodyB.AngularVelocity, rB);
                Vec2 dv3 = vB3 - vA3;
                float vt = Vec2.Dot(dv3, tangent);

                if (sp.TangentMass > 0f)
                {
                    float tangentImpulse = -vt * sp.TangentMass;
                    float maxFriction = friction * newNormalImpulse;
                    float newTangentImpulse = MathFng.Clamp(mp.TangentImpulse + tangentImpulse, -maxFriction, maxFriction);
                    float appliedTangent = newTangentImpulse - mp.TangentImpulse;
                    Vec2 Pt = appliedTangent * tangent;

                    bodyA.LinearVelocity -= invMassA * Pt;
                    bodyB.LinearVelocity += invMassB * Pt;
                    bodyA.AngularVelocity -= invIA * Vec2.Cross(rA, Pt);
                    bodyB.AngularVelocity += invIB * Vec2.Cross(rB, Pt);

                    mp = new ManifoldPoint(mp.LocalPoint, newNormalImpulse, newTangentImpulse, mp.Id);
                }
                else
                {
                    mp = new ManifoldPoint(mp.LocalPoint, newNormalImpulse, mp.TangentImpulse, mp.Id);
                }

                contact.Manifold.Points[p] = mp;
            }
        }

        private void SolvePositionContact(Contact contact)
        {
            if (contact.FixtureA == null || contact.FixtureB == null)
            {
                return;
            }

            Fixture fixtureA = contact.FixtureA;
            Fixture fixtureB = contact.FixtureB;
            Body bodyA = fixtureA.Body;
            Body bodyB = fixtureB.Body;

            if (bodyA.Type == BodyType.Static && bodyB.Type == BodyType.Static)
            {
                return;
            }

            float radiusA = GetShapeRadius(fixtureA.Shape);
            float radiusB = GetShapeRadius(fixtureB.Shape);

            WorldManifold worldManifold = new WorldManifold();
            worldManifold.Initialize(contact.Manifold, bodyA.Transform, radiusA, bodyB.Transform, radiusB);

            Vec2 centerA = bodyA.GetWorldCenter();
            Vec2 centerB = bodyB.GetWorldCenter();
            for (int p = 0; p < contact.Manifold.PointCount; ++p)
            {
                PositionManifold pm = ComputePositionManifold(contact, bodyA.Transform, bodyB.Transform, radiusA, radiusB, p);
                Vec2 normal = pm.Normal;
                Vec2 point = pm.Point;
                float separation = pm.Separation;

                Vec2 rA = point - centerA;
                Vec2 rB = point - centerB;

                float C = MathFng.Clamp(Constants.Baumgarte * (separation + Constants.LinearSlop), -Constants.MaxLinearCorrection, 0f);

                float invMassA = bodyA.InverseMass;
                float invMassB = bodyB.InverseMass;
                float invIA = bodyA.InverseInertia;
                float invIB = bodyB.InverseInertia;

                float rnA = Vec2.Cross(rA, normal);
                float rnB = Vec2.Cross(rB, normal);
                float k = invMassA + invMassB + invIA * rnA * rnA + invIB * rnB * rnB;
                if (k <= 0f)
                {
                    continue;
                }

                float impulse = -C / k;
                Vec2 P = impulse * normal;

                Vec2 newCenterA = centerA - invMassA * P;
                Vec2 newCenterB = centerB + invMassB * P;
                float newAngleA = bodyA.Transform.Q.Angle - invIA * Vec2.Cross(rA, P);
                float newAngleB = bodyB.Transform.Q.Angle + invIB * Vec2.Cross(rB, P);

                bodyA.SetTransformFromCenter(newCenterA, newAngleA);
                bodyB.SetTransformFromCenter(newCenterB, newAngleB);
                centerA = newCenterA;
                centerB = newCenterB;
            }
        }

        private static float GetShapeRadius(Shape shape)
        {
            switch (shape.Type)
            {
                case ShapeType.Circle:
                    return ((CircleShape)shape).Radius;
                case ShapeType.Capsule:
                    return ((CapsuleShape)shape).Radius;
                case ShapeType.Segment:
                    return Constants.PolygonRadius;
                case ShapeType.Polygon:
                    return ((PolygonShape)shape).Radius;
                case ShapeType.ChainSegment:
                    return Constants.PolygonRadius;
                default:
                    return 0f;
            }
        }

        private static Vec2 ApplyLinearDamping(Vec2 velocity, float damping, float timeStep)
        {
            float factor = MathF.Max(0f, 1f - timeStep * damping);
            return velocity * factor;
        }

        private void ComputeContactSoftness(float kNormal, float timeStep, float separation, out float bias, out float softness)
        {
            bias = 0f;
            softness = 0f;

            if (kNormal <= 0f)
            {
                return;
            }

            float hertz = _stepContactHertz > 0f ? _stepContactHertz : _def.ContactHertz;
            if (hertz <= 0f)
            {
                return;
            }

            float mass = 1f / kNormal;
            float omega = 2f * MathFng.Pi * hertz;
            float d = 2f * mass * _def.ContactDampingRatio * omega;
            float k = mass * omega * omega;

            float gamma = timeStep * (d + timeStep * k);
            if (gamma > 0f)
            {
                gamma = 1f / gamma;
            }

            float c = MathF.Min(separation + Constants.LinearSlop, 0f);
            bias = c * timeStep * k * gamma;

            float maxBias = MathF.Max(_def.ContactSpeed, 0f);
            bias = MathFng.Clamp(bias, -maxBias, 0f);

            softness = gamma;
        }

        private float MixFriction(Fixture a, Fixture b)
        {
            if (_def.FrictionCallback != null)
            {
                return _def.FrictionCallback(a.Friction, a.Filter.CategoryBits, b.Friction, b.Filter.CategoryBits);
            }
            return MathF.Sqrt(a.Friction * b.Friction);
        }

        private float MixRestitution(Fixture a, Fixture b)
        {
            if (_def.RestitutionCallback != null)
            {
                return _def.RestitutionCallback(a.Restitution, a.Filter.CategoryBits, b.Restitution, b.Filter.CategoryBits);
            }
            return 0.5f * (a.Restitution + b.Restitution);
        }

        private static Vec2 ClampLinearSpeed(Vec2 velocity, float maxSpeed)
        {
            float speedSqr = velocity.LengthSquared;
            float maxSqr = maxSpeed * maxSpeed;
            if (speedSqr > maxSqr && speedSqr > 0f)
            {
                float scale = maxSpeed / MathF.Sqrt(speedSqr);
                return velocity * scale;
            }

            return velocity;
        }

        private static float ClampAngularSpeed(float angularVelocity, float maxSpeed)
        {
            float abs = MathF.Abs(angularVelocity);
            if (abs > maxSpeed && abs > 0f)
            {
                return MathF.Sign(angularVelocity) * maxSpeed;
            }
            return angularVelocity;
        }

        private static Vec2 ClampTranslation(Vec2 translation, float maxTranslation)
        {
            float lengthSqr = translation.LengthSquared;
            float maxSqr = maxTranslation * maxTranslation;
            if (lengthSqr > maxSqr && lengthSqr > 0f)
            {
                float scale = maxTranslation / MathF.Sqrt(lengthSqr);
                return translation * scale;
            }
            return translation;
        }

        private static float ClampRotation(float rotation, float maxRotation)
        {
            float abs = MathF.Abs(rotation);
            if (abs > maxRotation && abs > 0f)
            {
                return MathF.Sign(rotation) * maxRotation;
            }
            return rotation;
        }

        private static float ApplyAngularDamping(float angularVelocity, float damping, float timeStep)
        {
            float factor = MathF.Max(0f, 1f - timeStep * damping);
            return angularVelocity * factor;
        }

        private bool ShouldCollide(Fixture a, Fixture b)
        {
            if (a.IsSensor || b.IsSensor)
            {
                return false;
            }
            if (ReferenceEquals(a.Body, b.Body))
            {
                return false;
            }
            if (IsCollisionDisabledByJoint(a.Body, b.Body))
            {
                return false;
            }

            if (a.Filter.GroupIndex != 0 && a.Filter.GroupIndex == b.Filter.GroupIndex)
            {
                return a.Filter.GroupIndex > 0;
            }

            if ((a.Filter.MaskBits & b.Filter.CategoryBits) == 0 || (b.Filter.MaskBits & a.Filter.CategoryBits) == 0)
            {
                return false;
            }

            return true;
        }

        private bool IsCollisionDisabledByJoint(Body bodyA, Body bodyB)
        {
            System.Collections.Generic.List<JointHandle>? joints = _bodyJoints[bodyA.Id];
            if (joints == null)
            {
                return false;
            }

            for (int i = 0; i < joints.Count; ++i)
            {
                JointHandle handle = joints[i];
                if (handle.Type == JointType.Gear)
                {
                    if (TryGetJointIndex(handle, out int gearIndex) &&
                        IsGearPrimaryBodyPair(gearIndex, bodyA, bodyB) &&
                        !GetJointCollideConnected(handle))
                    {
                        return true;
                    }
                    continue;
                }

                Body? other = GetJointOtherBody(handle, bodyA);
                if (!ReferenceEquals(other, bodyB))
                {
                    continue;
                }
                if (!GetJointCollideConnected(handle))
                {
                    return true;
                }
            }

            return false;
        }

        private bool IsGearPrimaryBodyPair(int gearIndex, Body bodyA, Body bodyB)
        {
            ref GearJointData joint = ref _gearJointsData[gearIndex];
            TryGetGearRepresentativeBodies(ref joint, out Body primaryA, out Body primaryB);
            return (bodyA.Id == primaryA.Id && bodyB.Id == primaryB.Id) ||
                   (bodyA.Id == primaryB.Id && bodyB.Id == primaryA.Id);
        }

        private bool GetJointCollideConnected(JointHandle handle)
        {
            if (!TryGetJointIndex(handle, out int index))
            {
                return false;
            }

            return handle.Type switch
            {
                JointType.Distance => _distanceJointsData[index].CollideConnected,
                JointType.Revolute => _revoluteJointsData[index].CollideConnected,
                JointType.Prismatic => _prismaticJointsData[index].CollideConnected,
                JointType.Wheel => _wheelJointsData[index].CollideConnected,
                JointType.Pulley => _pulleyJointsData[index].CollideConnected,
                JointType.Weld => _weldJointsData[index].CollideConnected,
                JointType.Motor => _motorJointsData[index].CollideConnected,
                JointType.Gear => _gearJointsData[index].CollideConnected,
                JointType.Rope => _ropeJointsData[index].CollideConnected,
                JointType.Friction => _frictionJointsData[index].CollideConnected,
                _ => false
            };
        }

        private static bool ShouldSensorOverlap(Fixture sensor, Fixture other)
        {
            if (!sensor.EnableSensorEvents || !other.EnableSensorEvents)
            {
                return false;
            }

            if (sensor.Filter.GroupIndex != 0 && sensor.Filter.GroupIndex == other.Filter.GroupIndex)
            {
                return sensor.Filter.GroupIndex > 0;
            }

            if ((sensor.Filter.MaskBits & other.Filter.CategoryBits) == 0 ||
                (other.Filter.MaskBits & sensor.Filter.CategoryBits) == 0)
            {
                return false;
            }

            return true;
        }

        private void SolvePositionConstraints()
        {
            SolvePositionConstraints(_contacts);
        }

        private void SolvePositionConstraints(System.Collections.Generic.IReadOnlyList<Contact> contacts)
        {
            for (int i = 0; i < contacts.Count; ++i)
            {
                Contact contact = contacts[i];
                if (contact.FixtureA == null || contact.FixtureB == null)
                {
                    continue;
                }

                Fixture fixtureA = contact.FixtureA;
                Fixture fixtureB = contact.FixtureB;
                Body bodyA = fixtureA.Body;
                Body bodyB = fixtureB.Body;

                if (bodyA.Type == BodyType.Static && bodyB.Type == BodyType.Static)
                {
                    continue;
                }

                float radiusA = GetShapeRadius(fixtureA.Shape);
                float radiusB = GetShapeRadius(fixtureB.Shape);

                WorldManifold worldManifold = new WorldManifold();
                worldManifold.Initialize(contact.Manifold, bodyA.Transform, radiusA, bodyB.Transform, radiusB);

                Vec2 centerA = bodyA.GetWorldCenter();
                Vec2 centerB = bodyB.GetWorldCenter();
                for (int p = 0; p < contact.Manifold.PointCount; ++p)
                {
                    PositionManifold pm = ComputePositionManifold(contact, bodyA.Transform, bodyB.Transform, radiusA, radiusB, p);
                    Vec2 normal = pm.Normal;
                    Vec2 point = pm.Point;
                    float separation = pm.Separation;

                    Vec2 rA = point - centerA;
                    Vec2 rB = point - centerB;

                    float C = MathFng.Clamp(Constants.Baumgarte * (separation + Constants.LinearSlop), -Constants.MaxLinearCorrection, 0f);

                    float invMassA = bodyA.InverseMass;
                    float invMassB = bodyB.InverseMass;
                    float invIA = bodyA.InverseInertia;
                    float invIB = bodyB.InverseInertia;

                    float rnA = Vec2.Cross(rA, normal);
                    float rnB = Vec2.Cross(rB, normal);
                    float k = invMassA + invMassB + invIA * rnA * rnA + invIB * rnB * rnB;
                    if (k <= 0f)
                    {
                        continue;
                    }

                    float impulse = -C / k;
                    Vec2 P = impulse * normal;

                    Vec2 newCenterA = centerA - invMassA * P;
                    Vec2 newCenterB = centerB + invMassB * P;
                    float newAngleA = bodyA.Transform.Q.Angle - invIA * Vec2.Cross(rA, P);
                    float newAngleB = bodyB.Transform.Q.Angle + invIB * Vec2.Cross(rB, P);

                    bodyA.SetTransformFromCenter(newCenterA, newAngleA);
                    bodyB.SetTransformFromCenter(newCenterB, newAngleB);
                    centerA = newCenterA;
                    centerB = newCenterB;
                }
            }
        }

        private readonly struct PositionManifold
        {
            public readonly Vec2 Normal;
            public readonly Vec2 Point;
            public readonly float Separation;

            public PositionManifold(Vec2 normal, Vec2 point, float separation)
            {
                Normal = normal;
                Point = point;
                Separation = separation;
            }
        }

        private static PositionManifold ComputePositionManifold(Contact contact, Transform xfA, Transform xfB, float radiusA, float radiusB, int index)
        {
            Manifold manifold = contact.Manifold;
            switch (manifold.Type)
            {
                case ManifoldType.Circles:
                    {
                        Vec2 pointA = Transform.Mul(xfA, manifold.LocalPoint);
                        Vec2 pointB = Transform.Mul(xfB, manifold.Points[0].LocalPoint);
                        Vec2 normal = pointB - pointA;
                        float distance = normal.Length;
                        if (distance > Constants.Epsilon)
                        {
                            normal = normal / distance;
                        }
                        else
                        {
                            normal = new Vec2(1f, 0f);
                            distance = 0f;
                        }
                        Vec2 point = 0.5f * (pointA + pointB);
                        float separation = distance - (radiusA + radiusB);
                        return new PositionManifold(normal, point, separation);
                    }
                case ManifoldType.FaceA:
                    {
                        Vec2 normal = Rot.Mul(xfA.Q, manifold.LocalNormal);
                        Vec2 planePoint = Transform.Mul(xfA, manifold.LocalPoint);
                        Vec2 clipPoint = Transform.Mul(xfB, manifold.Points[index].LocalPoint);
                        float separation = Vec2.Dot(clipPoint - planePoint, normal) - radiusA - radiusB;
                        return new PositionManifold(normal, clipPoint, separation);
                    }
                case ManifoldType.FaceB:
                    {
                        Vec2 normal = Rot.Mul(xfB.Q, manifold.LocalNormal);
                        Vec2 planePoint = Transform.Mul(xfB, manifold.LocalPoint);
                        Vec2 clipPoint = Transform.Mul(xfA, manifold.Points[index].LocalPoint);
                        float separation = Vec2.Dot(clipPoint - planePoint, normal) - radiusA - radiusB;
                        normal = -normal;
                        return new PositionManifold(normal, clipPoint, separation);
                    }
                default:
                    return new PositionManifold(new Vec2(1f, 0f), Vec2.Zero, 0f);
            }
        }

        private void PrepareContacts(float timeStep, float dtRatio)
        {
            PrepareContacts(timeStep, dtRatio, _contacts);
        }

        private void PrepareContacts(float timeStep, float dtRatio, System.Collections.Generic.IReadOnlyList<Contact> contacts)
        {
            for (int i = 0; i < contacts.Count; ++i)
            {
                Contact contact = contacts[i];
                if (contact.FixtureA == null || contact.FixtureB == null)
                {
                    continue;
                }

                Fixture fixtureA = contact.FixtureA;
                Fixture fixtureB = contact.FixtureB;
                Body bodyA = fixtureA.Body;
                Body bodyB = fixtureB.Body;

                float radiusA = GetShapeRadius(fixtureA.Shape);
                float radiusB = GetShapeRadius(fixtureB.Shape);

                WorldManifold worldManifold = new WorldManifold();
                worldManifold.Initialize(contact.Manifold, bodyA.Transform, radiusA, bodyB.Transform, radiusB);

                Vec2 normal = worldManifold.Normal;
                Vec2 centerA = bodyA.GetWorldCenter();
                Vec2 centerB = bodyB.GetWorldCenter();
                for (int p = 0; p < contact.Manifold.PointCount; ++p)
                {
                    ManifoldPoint mp = contact.Manifold.Points[p];
                    if (dtRatio != 1f)
                    {
                        mp = new ManifoldPoint(mp.LocalPoint, mp.NormalImpulse * dtRatio, mp.TangentImpulse * dtRatio, mp.Id);
                        contact.Manifold.Points[p] = mp;
                    }

                    Vec2 point = worldManifold.Points[p];
                    Vec2 rA = point - centerA;
                    Vec2 rB = point - centerB;

                    float rnA = Vec2.Cross(rA, normal);
                    float rnB = Vec2.Cross(rB, normal);
                    float kNormal = bodyA.InverseMass + bodyB.InverseMass +
                                    bodyA.InverseInertia * rnA * rnA + bodyB.InverseInertia * rnB * rnB;
                    float normalMass = kNormal > 0f ? 1f / kNormal : 0f;

                    Vec2 tangent = new Vec2(-normal.Y, normal.X);
                    float rtA = Vec2.Cross(rA, tangent);
                    float rtB = Vec2.Cross(rB, tangent);
                    float kTangent = bodyA.InverseMass + bodyB.InverseMass +
                                     bodyA.InverseInertia * rtA * rtA + bodyB.InverseInertia * rtB * rtB;
                    float tangentMass = kTangent > 0f ? 1f / kTangent : 0f;

                    Vec2 vA = bodyA.LinearVelocity + Vec2.Cross(bodyA.AngularVelocity, rA);
                    Vec2 vB = bodyB.LinearVelocity + Vec2.Cross(bodyB.AngularVelocity, rB);
                    float vn = Vec2.Dot(vB - vA, normal);

                    float velocityBias = 0f;
                    float softness = 0f;
                    float bias = 0f;
                    float restitution = MixRestitution(fixtureA, fixtureB);
                    if (vn < -_def.RestitutionThreshold)
                    {
                        velocityBias = -restitution * vn;
                    }
                    if (_def.EnableContactSoftening)
                    {
                        float separation = Vec2.Dot((centerB + rB) - (centerA + rA), normal);
                        if (_def.UseSoftConstraints)
                        {
                            ComputeContactSoftness(kNormal, timeStep, separation, out bias, out softness);
                            if (softness > 0f)
                            {
                                normalMass = 1f / (kNormal + softness);
                            }
                        }
                        else
                        {
                            float baumgarte = Constants.Baumgarte * (separation + Constants.LinearSlop) / timeStep;
                            float maxBias = MathF.Max(_def.ContactSpeed, 0f);
                            float legacyBias = MathFng.Clamp(baumgarte, -maxBias, 0f);
                            float baumgarteClamped = MathFng.Clamp(legacyBias, -Constants.MaxLinearCorrection, 0f);
                            velocityBias -= baumgarteClamped;
                        }
                    }

                    contact.SolverPoints[p] = new SolverPoint
                    {
                        RA = rA,
                        RB = rB,
                        NormalMass = normalMass,
                        TangentMass = tangentMass,
                        VelocityBias = velocityBias,
                        Softness = softness,
                        Bias = bias
                    };
                }
            }
        }

        private void RemoveStaleContacts(HashSet64 live, System.Collections.Generic.List<(ContactKey Key, ContactEndEvent Event)> endEvents)
        {
            if (_contactMap.Count == 0)
            {
                return;
            }

            System.Collections.Generic.List<ContactKey> remove = new System.Collections.Generic.List<ContactKey>();
            foreach (var pair in _contactMap)
            {
                if (!live.Contains(MakeProxyPairKey(pair.Key.A, pair.Key.B)))
                {
                    remove.Add(pair.Key);
                }
            }
            remove.Sort((a, b) =>
            {
                int cmp = a.A.CompareTo(b.A);
                return cmp != 0 ? cmp : a.B.CompareTo(b.B);
            });

            for (int i = 0; i < remove.Count; ++i)
            {
                if (_contactMap.TryGetValue(remove[i], out Contact? contact) && contact.FixtureA != null && contact.FixtureB != null)
                {
                    if (contact.IsTouching || contact.WasTouching)
                    {
                        endEvents.Add((remove[i], new ContactEndEvent(contact.FixtureA.UserData, contact.FixtureB.UserData)));
                    }
                    _awakeSet.Contacts.Remove(contact);
                    _sleepingSet.Contacts.Remove(contact);
                    _disabledSet.Contacts.Remove(contact);
                    _awakeSet.NonTouchingContacts.Remove(contact);
                    _sleepingSet.NonTouchingContacts.Remove(contact);
                    _disabledSet.NonTouchingContacts.Remove(contact);
                    UnlinkContact(contact);
                    RemoveContactFromSleepingSets(contact);
                }
                if (contact != null && contact.Id >= 0)
                {
                    _contactIdPool.Free(contact.Id);
                    contact.Id = -1;
                }
                _contactMap.Remove(remove[i]);
            }
        }

        private void RemoveStaleSensors(HashSet64 live, System.Collections.Generic.List<SensorEndEvent> endEvents)
        {
            if (_sensorPairs.Count == 0)
            {
                MergeSensorPairs(live);
                return;
            }

            System.Collections.Generic.List<ulong> previous = new System.Collections.Generic.List<ulong>();
            _sensorPairs.CopyKeys(previous);
            System.Collections.Generic.List<ulong> remove = new System.Collections.Generic.List<ulong>();
            for (int i = 0; i < previous.Count; ++i)
            {
                ulong key = previous[i];
                if (!live.Contains(key))
                {
                    remove.Add(key);
                }
            }
            remove.Sort(CompareProxyPairKeys);

            for (int i = 0; i < remove.Count; ++i)
            {
                ulong key = remove[i];
                _sensorPairs.Remove(key);
                ContactKey contactKey = new ContactKey(UnpackProxyIdA(key), UnpackProxyIdB(key));
                if (_sensorUserData.TryGetValue(contactKey, out var data))
                {
                    endEvents.Add(new SensorEndEvent(data.A, data.B));
                    _sensorUserData.Remove(contactKey);
                }
                else
                {
                    endEvents.Add(new SensorEndEvent(null, null));
                }
            }

            MergeSensorPairs(live);
        }

        private static bool TestOverlap(Fixture a, Fixture b)
        {
            ShapeProxy proxyA = ShapeGeometry.ToProxy(a.Shape);
            ShapeProxy proxyB = ShapeGeometry.ToProxy(b.Shape);
            DistanceInput input = new DistanceInput(proxyA, proxyB, a.Body.Transform, b.Body.Transform, true);
            DistanceOutput output = Distance.Compute(input);
            return output.Distance <= 0f;
        }

        private static bool PassesQueryFilter(Filter fixtureFilter, QueryFilter queryFilter)
        {
            if ((fixtureFilter.MaskBits & queryFilter.CategoryBits) == 0)
            {
                return false;
            }
            if ((queryFilter.MaskBits & fixtureFilter.CategoryBits) == 0)
            {
                return false;
            }
            return true;
        }

        private static bool TryRayCastFixture(Fixture fixture, RayCastInput input, out RayCastOutput output)
        {
            Transform xf = fixture.Body.Transform;
            Vec2 localOrigin = Transform.MulT(xf, input.Origin);
            Vec2 localTranslation = Rot.MulT(xf.Q, input.Translation);
            RayCastInput localInput = new RayCastInput(localOrigin, localTranslation, input.MaxFraction);

            CastOutput cast;
            switch (fixture.Shape.Type)
            {
                case ShapeType.Circle:
                    cast = Collision.RayCastCircle(ShapeGeometry.ToCircle((CircleShape)fixture.Shape), localInput);
                    break;
                case ShapeType.Capsule:
                    cast = Collision.RayCastCapsule(ShapeGeometry.ToCapsule((CapsuleShape)fixture.Shape), localInput);
                    break;
                case ShapeType.Segment:
                    cast = Collision.RayCastSegment(ShapeGeometry.ToSegment((SegmentShape)fixture.Shape), localInput);
                    break;
                case ShapeType.Polygon:
                    cast = Collision.RayCastPolygon(ShapeGeometry.ToPolygon((PolygonShape)fixture.Shape), localInput);
                    break;
                case ShapeType.ChainSegment:
                    cast = Collision.RayCastChainSegment(ShapeGeometry.ToChainSegment((ChainSegmentShape)fixture.Shape), localInput);
                    break;
                default:
                    cast = new CastOutput(Vec2.Zero, Vec2.Zero, 0f, 0, false);
                    break;
            }

            if (!cast.Hit)
            {
                output = default;
                return false;
            }

            Vec2 worldPoint = Transform.Mul(xf, cast.Point);
            Vec2 worldNormal = Rot.Mul(xf.Q, cast.Normal).Normalize();
            output = new RayCastOutput(worldPoint, worldNormal, cast.Fraction);
            return true;
        }

        public readonly struct RayCastHit
        {
            public readonly Fixture Fixture;
            public readonly Vec2 Point;
            public readonly Vec2 Normal;
            public readonly float Fraction;

            public RayCastHit(Fixture fixture, Vec2 point, Vec2 normal, float fraction)
            {
                Fixture = fixture;
                Point = point;
                Normal = normal;
                Fraction = fraction;
            }
        }

        private void AddHitEvent(Contact contact, ContactKey key, System.Collections.Generic.List<(ContactKey Key, ContactHitEvent Event)> hitEvents)
        {
            if (contact.FixtureA == null || contact.FixtureB == null)
            {
                return;
            }

            Fixture fixtureA = contact.FixtureA;
            Fixture fixtureB = contact.FixtureB;
            Body bodyA = fixtureA.Body;
            Body bodyB = fixtureB.Body;

            float radiusA = GetShapeRadius(fixtureA.Shape);
            float radiusB = GetShapeRadius(fixtureB.Shape);
            WorldManifold worldManifold = new WorldManifold();
            worldManifold.Initialize(contact.Manifold, bodyA.Transform, radiusA, bodyB.Transform, radiusB);

            Vec2 normal = worldManifold.Normal;
            if (contact.Manifold.PointCount == 0)
            {
                return;
            }

            Vec2 point = worldManifold.Points[0];
            Vec2 centerA = bodyA.GetWorldCenter();
            Vec2 centerB = bodyB.GetWorldCenter();
            Vec2 rA = point - centerA;
            Vec2 rB = point - centerB;
            Vec2 vA = bodyA.LinearVelocity + Vec2.Cross(bodyA.AngularVelocity, rA);
            Vec2 vB = bodyB.LinearVelocity + Vec2.Cross(bodyB.AngularVelocity, rB);
            float speed = MathF.Abs(Vec2.Dot(vB - vA, normal));

            if (speed >= _def.HitEventThreshold)
            {
                hitEvents.Add((key, new ContactHitEvent(fixtureA.UserData, fixtureB.UserData, point, normal, speed)));
            }
        }

        private void UpdateSleep(float timeStep)
        {
            if (_awakeSet.Islands.Count == 0)
            {
                return;
            }

            float linearTolSqr = Constants.LinearSleepTolerance * Constants.LinearSleepTolerance;
            float angularTol = Constants.AngularSleepTolerance;

            for (int i = 0; i < _awakeSet.Islands.Count; ++i)
            {
                Island island = _awakeSet.Islands[i];
                float minSleepTime = float.MaxValue;

                for (int j = 0; j < island.Bodies.Count; ++j)
                {
                    Body body = island.Bodies[j];
                    if (body.Type == BodyType.Static)
                    {
                        continue;
                    }

                    if (body.AllowSleep == false ||
                        body.LinearVelocity.LengthSquared > linearTolSqr ||
                        MathF.Abs(body.AngularVelocity) > angularTol)
                    {
                        body.SleepTime = 0f;
                        minSleepTime = 0f;
                    }
                    else
                    {
                        body.SleepTime += timeStep;
                        minSleepTime = MathF.Min(minSleepTime, body.SleepTime);
                    }
                }

                if (minSleepTime >= Constants.TimeToSleep)
                {
                    island.IsAwake = false;
                    MoveIslandBetweenSets(island, _awakeSet, _sleepingSet);
                    CreateSleepingSetForIsland(island);
                    i--;

                    for (int j = 0; j < island.Bodies.Count; ++j)
                    {
                        Body body = island.Bodies[j];
                        if (body.Type == BodyType.Static || body.AllowSleep == false || body.Awake == false)
                        {
                            continue;
                        }
                        body.SetAwakeFromWorld(false, notifyWorld: false);
                        body.SolverSetType = SolverSetType.Sleeping;
                        _awakeSet.Bodies.Remove(body);
                        if (!_sleepingSet.Bodies.Contains(body))
                        {
                            _sleepingSet.Bodies.Add(body);
                        }
                    }
                }
            }
        }

        private readonly struct ContactKey : System.IEquatable<ContactKey>
        {
            public readonly int A;
            public readonly int B;

            public ContactKey(int proxyIdA, int proxyIdB)
            {
                if (proxyIdA < proxyIdB)
                {
                    A = proxyIdA;
                    B = proxyIdB;
                }
                else
                {
                    A = proxyIdB;
                    B = proxyIdA;
                }
            }

            public bool Equals(ContactKey other) => A == other.A && B == other.B;
            public override bool Equals(object? obj) => obj is ContactKey other && Equals(other);
            public override int GetHashCode()
            {
                unchecked
                {
                    int hash = 17;
                    hash = (hash * 31) + A.GetHashCode();
                    hash = (hash * 31) + B.GetHashCode();
                    return hash;
                }
            }
        }

        private static ulong MakeProxyPairKey(int proxyIdA, int proxyIdB)
        {
            uint a = (uint)(proxyIdA + 1);
            uint b = (uint)(proxyIdB + 1);
            return a < b ? ((ulong)a << 32) | b : ((ulong)b << 32) | a;
        }

        private static int UnpackProxyIdA(ulong key)
        {
            return (int)((key >> 32) & 0xFFFFFFFF) - 1;
        }

        private static int UnpackProxyIdB(ulong key)
        {
            return (int)(key & 0xFFFFFFFF) - 1;
        }

        private static int CompareProxyPairKeys(ulong a, ulong b)
        {
            int a0 = UnpackProxyIdA(a);
            int a1 = UnpackProxyIdB(a);
            int b0 = UnpackProxyIdA(b);
            int b1 = UnpackProxyIdB(b);
            if (a0 != b0)
            {
                return a0.CompareTo(b0);
            }
            return a1.CompareTo(b1);
        }

        private void MergeSensorPairs(HashSet64 live)
        {
            System.Collections.Generic.List<ulong> keys = new System.Collections.Generic.List<ulong>();
            live.CopyKeys(keys);
            for (int i = 0; i < keys.Count; ++i)
            {
                _sensorPairs.Add(keys[i]);
            }
        }
    }
}



