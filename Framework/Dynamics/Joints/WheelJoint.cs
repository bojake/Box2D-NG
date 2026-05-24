using System;

namespace Box2DNG
{
    public sealed class WheelJoint
    {
        internal World World;
        internal int Id;
        internal int Index;

        internal WheelJoint(World world, int id, int index)
        {
            World = world;
            Id = id;
            Index = index;
        }

        public Body BodyA => World.GetBody(World._wheelJointsData[Index].BodyA);
        public Body BodyB => World.GetBody(World._wheelJointsData[Index].BodyB);
        public Vec2 LocalAnchorA => World._wheelJointsData[Index].LocalAnchorA;
        public Vec2 LocalAnchorB => World._wheelJointsData[Index].LocalAnchorB;
        public Vec2 LocalAxisA => World._wheelJointsData[Index].LocalAxisA;
        public bool EnableMotor => World._wheelJointsData[Index].EnableMotor;
        public float MotorSpeed => World._wheelJointsData[Index].MotorSpeed;
        public float MaxMotorTorque => World._wheelJointsData[Index].MaxMotorTorque;
        public float FrequencyHz => World._wheelJointsData[Index].FrequencyHz;
        public float DampingRatio => World._wheelJointsData[Index].DampingRatio;
        public bool EnableLimit => World._wheelJointsData[Index].EnableLimit;
        public float LowerTranslation => World._wheelJointsData[Index].LowerTranslation;
        public float UpperTranslation => World._wheelJointsData[Index].UpperTranslation;
        public bool CollideConnected => World._wheelJointsData[Index].CollideConnected;

        public void SetMotorEnabled(bool enable) => World._wheelJointsData[Index].EnableMotor = enable;
        public void SetMotorSpeed(float speed) => World._wheelJointsData[Index].MotorSpeed = speed;
        public void SetMaxMotorTorque(float torque) => World._wheelJointsData[Index].MaxMotorTorque = MathF.Max(0f, torque);
        public void SetSpringFrequencyHz(float hz) => World._wheelJointsData[Index].FrequencyHz = MathF.Max(0f, hz);
        public void SetSpringDampingRatio(float ratio) => World._wheelJointsData[Index].DampingRatio = MathF.Max(0f, ratio);
        public void SetLimitsEnabled(bool enable) => World._wheelJointsData[Index].EnableLimit = enable;

        public void SetLimits(float lower, float upper)
        {
            ref WheelJointData data = ref World._wheelJointsData[Index];
            data.EnableLimit = true;
            data.LowerTranslation = MathF.Min(lower, upper);
            data.UpperTranslation = MathF.Max(lower, upper);
        }

        public Vec2 GetReactionForce(float invDt)
        {
            ref WheelJointData data = ref World._wheelJointsData[Index];
            Vec2 axis = Rot.Mul(BodyA.Transform.Q, data.LocalAxisA);
            Vec2 perp = new Vec2(-axis.Y, axis.X);
            float axialImpulse = data.SpringImpulse + data.LowerImpulse - data.UpperImpulse;
            return invDt * (data.Impulse * perp + axialImpulse * axis);
        }

        public float GetReactionTorque(float invDt)
        {
            return invDt * World._wheelJointsData[Index].MotorImpulse;
        }

        public float GetMotorTorque(float invDt) => invDt * World._wheelJointsData[Index].MotorImpulse;
    }
}
