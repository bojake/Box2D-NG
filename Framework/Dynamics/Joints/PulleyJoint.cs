using System;

namespace Box2DNG
{
    public sealed class PulleyJoint
    {
        internal World World;
        internal int Id;
        internal int Index;

        internal PulleyJoint(World world, int id, int index)
        {
            World = world;
            Id = id;
            Index = index;
        }

        public Body BodyA => World.GetBody(World._pulleyJointsData[Index].BodyA);
        public Body BodyB => World.GetBody(World._pulleyJointsData[Index].BodyB);
        public Vec2 GroundAnchorA => World._pulleyJointsData[Index].GroundAnchorA;
        public Vec2 GroundAnchorB => World._pulleyJointsData[Index].GroundAnchorB;
        public Vec2 LocalAnchorA => World._pulleyJointsData[Index].LocalAnchorA;
        public Vec2 LocalAnchorB => World._pulleyJointsData[Index].LocalAnchorB;
        public float LengthA => World._pulleyJointsData[Index].LengthA;
        public float LengthB => World._pulleyJointsData[Index].LengthB;
        public float Ratio => World._pulleyJointsData[Index].Ratio;
        public bool CollideConnected => World._pulleyJointsData[Index].CollideConnected;

        public float GetCurrentLengthA()
        {
            Vec2 p = Transform.Mul(BodyA.Transform, LocalAnchorA);
            return (p - GroundAnchorA).Length;
        }

        public float GetCurrentLengthB()
        {
            Vec2 p = Transform.Mul(BodyB.Transform, LocalAnchorB);
            return (p - GroundAnchorB).Length;
        }

        public Vec2 GetReactionForce(float invDt)
        {
            ref PulleyJointData data = ref World._pulleyJointsData[Index];
            return (data.Impulse * invDt) * data.UB;
        }

        public float GetReactionTorque(float invDt) => 0f;
    }
}
