using System;

namespace Box2DNG
{
    public sealed class DistanceJoint
    {
        internal readonly World _world;
        
        internal DistanceJoint(World world, DistanceJointDef def)
        {
            _world = world;
            // Id and Index are set by World immediately after creation
        }

        public int Id { get; internal set; } = -1;
        internal int Index { get; set; } = -1;
        
        public Body BodyA => _world.Bodies[_world._distanceJointsData[Index].BodyA];
        public Body BodyB => _world.Bodies[_world._distanceJointsData[Index].BodyB];
        
        public Vec2 LocalAnchorA => _world._distanceJointsData[Index].LocalAnchorA;
        public Vec2 LocalAnchorB => _world._distanceJointsData[Index].LocalAnchorB;
        
        public float Length 
        { 
            get => _world._distanceJointsData[Index].Length;
            set => _world._distanceJointsData[Index].Length = value;
        }
        
        public float FrequencyHz 
        { 
            get => _world._distanceJointsData[Index].FrequencyHz;
            set => _world._distanceJointsData[Index].FrequencyHz = value;
        }
        
        public float DampingRatio 
        { 
            get => _world._distanceJointsData[Index].DampingRatio;
            set => _world._distanceJointsData[Index].DampingRatio = value;
        }
        
        public bool CollideConnected => _world._distanceJointsData[Index].CollideConnected; // Usually immutable

        public float GetCurrentLength()
        {
            Vec2 pA = Transform.Mul(BodyA.Transform, LocalAnchorA);
            Vec2 pB = Transform.Mul(BodyB.Transform, LocalAnchorB);
            return (pB - pA).Length;
        }

        public Vec2 GetReactionForce(float invDt)
        {
            ref DistanceJointData data = ref _world._distanceJointsData[Index];
            return (data.Impulse * invDt) * data.U;
        }

        public float GetReactionTorque(float invDt) => 0f;
    }
}
