using System;

namespace Box2DNG
{
    public sealed class RevoluteJoint
    {
        internal World World;
        internal int Id;
        internal int Index;

        internal RevoluteJoint(World world, int id, int index)
        {
            World = world;
            Id = id;
            Index = index;
        }

        public Body BodyA => World.GetBody(World._revoluteJointsData[Index].BodyA);
        public Body BodyB => World.GetBody(World._revoluteJointsData[Index].BodyB);

        public Vec2 LocalAnchorA => World._revoluteJointsData[Index].LocalAnchorA;
        public Vec2 LocalAnchorB => World._revoluteJointsData[Index].LocalAnchorB;
        public float ReferenceAngle => World._revoluteJointsData[Index].ReferenceAngle;

        public bool EnableMotor
        {
            get => World._revoluteJointsData[Index].EnableMotor;
            set => World._revoluteJointsData[Index].EnableMotor = value;
        }

        public float MotorSpeed
        {
            get => World._revoluteJointsData[Index].MotorSpeed;
            set
            {
                World._revoluteJointsData[Index].MotorSpeed = value;
                // Wake bodies
                BodyA.SetAwake(true);
                BodyB.SetAwake(true);
            }
        }

        public float MaxMotorTorque
        {
            get => World._revoluteJointsData[Index].MaxMotorTorque;
            set => World._revoluteJointsData[Index].MaxMotorTorque = value;
        }

        public bool EnableLimit
        {
            get => World._revoluteJointsData[Index].EnableLimit;
            set
            {
                World._revoluteJointsData[Index].EnableLimit = value;
                BodyA.SetAwake(true);
                BodyB.SetAwake(true);
            }
        }

        public float LowerAngle => World._revoluteJointsData[Index].LowerAngle;
        public float UpperAngle => World._revoluteJointsData[Index].UpperAngle;
        public bool CollideConnected => World._revoluteJointsData[Index].CollideConnected;

        public float GetMotorTorque(float invDt)
        {
            return World._revoluteJointsData[Index].MotorImpulse * invDt;
        }

        public Vec2 GetReactionForce(float invDt)
        {
            return invDt * World._revoluteJointsData[Index].Impulse;
        }

        public float GetReactionTorque(float invDt)
        {
            ref RevoluteJointData data = ref World._revoluteJointsData[Index];
            return invDt * (data.MotorImpulse + data.LimitImpulse);
        }

        public float GetJointAngle()
        {
             Body bA = BodyA;
             Body bB = BodyB;
             return bB.Rotation.Angle - bA.Rotation.Angle - ReferenceAngle;
        }

        public float GetJointSpeed()
        {
            Body bA = BodyA;
            Body bB = BodyB;
            return bB.AngularVelocity - bA.AngularVelocity;
        }

        public void SetMotorSpeed(float speed)
        {
            MotorSpeed = speed;
        }

        public void SetMotorEnabled(bool flag)
        {
            EnableMotor = flag;
        }

        public void SetLimits(float lower, float upper)
        {
            ref RevoluteJointData data = ref World._revoluteJointsData[Index];
            if (lower != data.LowerAngle || upper != data.UpperAngle)
            {
                data.LowerAngle = lower;
                data.UpperAngle = upper;
                data.Impulse = Vec2.Zero;
                data.LimitImpulse = 0f;
                BodyA.SetAwake(true);
                BodyB.SetAwake(true);
            }
        }
    }
}
