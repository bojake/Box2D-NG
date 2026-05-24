using System;

namespace Box2DNG
{
    public sealed class RopeJoint
    {
        internal World World;
        internal int Id;
        internal int Index;

        internal RopeJoint(World world, int id, int index)
        {
            World = world;
            Id = id;
            Index = index;
        }

        public Body BodyA => World.GetBody(World._ropeJointsData[Index].BodyA);
        public Body BodyB => World.GetBody(World._ropeJointsData[Index].BodyB);
        public Vec2 LocalAnchorA => World._ropeJointsData[Index].LocalAnchorA;
        public Vec2 LocalAnchorB => World._ropeJointsData[Index].LocalAnchorB;
        public float MaxLength => World._ropeJointsData[Index].MaxLength;
        public bool CollideConnected => World._ropeJointsData[Index].CollideConnected;

        public void SetMaxLength(float maxLength)
        {
            World._ropeJointsData[Index].MaxLength = MathF.Max(Constants.LinearSlop, maxLength);
        }

        public float GetCurrentLength()
        {
            Vec2 pA = Transform.Mul(BodyA.Transform, LocalAnchorA);
            Vec2 pB = Transform.Mul(BodyB.Transform, LocalAnchorB);
            return (pB - pA).Length;
        }

        public Vec2 GetReactionForce(float invDt)
        {
            ref RopeJointData data = ref World._ropeJointsData[Index];
            return (data.Impulse * invDt) * data.U;
        }

        public float GetReactionTorque(float invDt) => 0f;
    }
}
