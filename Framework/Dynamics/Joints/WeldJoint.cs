using System;

namespace Box2DNG
{
    public sealed class WeldJoint
    {
        internal World World;
        internal int Id;
        internal int Index;

        internal WeldJoint(World world, int id, int index)
        {
            World = world;
            Id = id;
            Index = index;
        }

        public Body BodyA => World.GetBody(World._weldJointsData[Index].BodyA);
        public Body BodyB => World.GetBody(World._weldJointsData[Index].BodyB);
        public Vec2 LocalAnchorA => World._weldJointsData[Index].LocalAnchorA;
        public Vec2 LocalAnchorB => World._weldJointsData[Index].LocalAnchorB;
        public float ReferenceAngle => World._weldJointsData[Index].ReferenceAngle;
        public bool CollideConnected => World._weldJointsData[Index].CollideConnected;

        public Vec2 GetReactionForce(float invDt)
        {
            return invDt * World._weldJointsData[Index].Impulse;
        }

        public float GetReactionTorque(float invDt)
        {
            return invDt * World._weldJointsData[Index].AngularImpulse;
        }
    }
}
