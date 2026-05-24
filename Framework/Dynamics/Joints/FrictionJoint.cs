using System;

namespace Box2DNG
{
    public sealed class FrictionJoint
    {
        internal World World;
        internal int Id;
        internal int Index;

        internal FrictionJoint(World world, int id, int index)
        {
            World = world;
            Id = id;
            Index = index;
        }

        public Body BodyA => World.GetBody(World._frictionJointsData[Index].BodyA);
        public Body BodyB => World.GetBody(World._frictionJointsData[Index].BodyB);
        public Vec2 LocalAnchorA => World._frictionJointsData[Index].LocalAnchorA;
        public Vec2 LocalAnchorB => World._frictionJointsData[Index].LocalAnchorB;
        public float MaxForce => World._frictionJointsData[Index].MaxForce;
        public float MaxTorque => World._frictionJointsData[Index].MaxTorque;
        public bool CollideConnected => World._frictionJointsData[Index].CollideConnected;

        public void SetMaxForce(float maxForce)
        {
            World._frictionJointsData[Index].MaxForce = MathF.Max(0f, maxForce);
        }

        public void SetMaxTorque(float maxTorque)
        {
            World._frictionJointsData[Index].MaxTorque = MathF.Max(0f, maxTorque);
        }

        public Vec2 GetReactionForce(float invDt)
        {
            return invDt * World._frictionJointsData[Index].LinearImpulse;
        }

        public float GetReactionTorque(float invDt)
        {
            return invDt * World._frictionJointsData[Index].AngularImpulse;
        }
    }
}
