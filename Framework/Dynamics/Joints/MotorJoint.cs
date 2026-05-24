using System;

namespace Box2DNG
{
    public sealed class MotorJoint
    {
        internal World World;
        internal int Id;
        internal int Index;

        internal MotorJoint(World world, int id, int index)
        {
            World = world;
            Id = id;
            Index = index;
        }

        public Body BodyA => World.GetBody(World._motorJointsData[Index].BodyA);
        public Body BodyB => World.GetBody(World._motorJointsData[Index].BodyB);

        public Vec2 LinearOffset => World._motorJointsData[Index].LinearOffset;
        public float AngularOffset => World._motorJointsData[Index].AngularOffset;
        public float MaxForce => World._motorJointsData[Index].MaxForce;
        public float MaxTorque => World._motorJointsData[Index].MaxTorque;
        public float CorrectionFactor => World._motorJointsData[Index].CorrectionFactor;
        public bool CollideConnected => World._motorJointsData[Index].CollideConnected;

        public Vec2 GetReactionForce(float invDt)
        {
            return invDt * World._motorJointsData[Index].LinearImpulse;
        }

        public float GetReactionTorque(float invDt)
        {
            return invDt * World._motorJointsData[Index].AngularImpulse;
        }
    }
}
