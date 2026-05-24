using System;

namespace Box2DNG
{
    public sealed class PrismaticJoint
    {
        internal World World;
        internal int Id;
        internal int Index;

        internal PrismaticJoint(World world, int id, int index)
        {
            World = world;
            Id = id;
            Index = index;
        }

        public Body BodyA => World.GetBody(World._prismaticJointsData[Index].BodyA);
        public Body BodyB => World.GetBody(World._prismaticJointsData[Index].BodyB);

        public Vec2 LocalAnchorA => World._prismaticJointsData[Index].LocalAnchorA;
        public Vec2 LocalAnchorB => World._prismaticJointsData[Index].LocalAnchorB;
        public Vec2 LocalAxisA => World._prismaticJointsData[Index].LocalAxisA;
        public float ReferenceAngle => World._prismaticJointsData[Index].ReferenceAngle;

        public bool EnableSpring
        {
            get => World._prismaticJointsData[Index].EnableSpring;
            set
            {
                World._prismaticJointsData[Index].EnableSpring = value;
                BodyA.SetAwake(true);
                BodyB.SetAwake(true);
            }
        }

        public float FrequencyHz
        {
            get => World._prismaticJointsData[Index].FrequencyHz;
            set => World._prismaticJointsData[Index].FrequencyHz = Math.Max(0f, value);
        }

        public float DampingRatio
        {
            get => World._prismaticJointsData[Index].DampingRatio;
            set => World._prismaticJointsData[Index].DampingRatio = Math.Max(0f, value);
        }

        public float TargetTranslation
        {
            get => World._prismaticJointsData[Index].TargetTranslation;
            set
            {
                World._prismaticJointsData[Index].TargetTranslation = value;
                BodyA.SetAwake(true);
                BodyB.SetAwake(true);
            }
        }

        public bool EnableLimit
        {
            get => World._prismaticJointsData[Index].EnableLimit;
            set
            {
                World._prismaticJointsData[Index].EnableLimit = value;
                BodyA.SetAwake(true);
                BodyB.SetAwake(true);
            }
        }

        public float LowerTranslation
        {
            get => World._prismaticJointsData[Index].LowerTranslation;
            set
            {
                ref PrismaticJointData data = ref World._prismaticJointsData[Index];
                data.LowerTranslation = value;
                data.LowerImpulse = 0f;
                // data.UpperImpulse = 0f; // Original didn't reset upper when lower changed, but SetLimits did reset both.
                // Keeping consistent with individual property setter if it exists, but previously it was SetLimits.
                BodyA.SetAwake(true);
                BodyB.SetAwake(true);
            }
        }

        public float UpperTranslation
        {
            get => World._prismaticJointsData[Index].UpperTranslation;
            set
            {
                ref PrismaticJointData data = ref World._prismaticJointsData[Index];
                data.UpperTranslation = value;
                data.UpperImpulse = 0f;
                BodyA.SetAwake(true);
                BodyB.SetAwake(true);
            }
        }

        public bool EnableMotor
        {
            get => World._prismaticJointsData[Index].EnableMotor;
            set
            {
                World._prismaticJointsData[Index].EnableMotor = value;
                BodyA.SetAwake(true);
                BodyB.SetAwake(true);
            }
        }

        public float MotorSpeed
        {
            get => World._prismaticJointsData[Index].MotorSpeed;
            set
            {
                World._prismaticJointsData[Index].MotorSpeed = value;
                BodyA.SetAwake(true);
                BodyB.SetAwake(true);
            }
        }

        public float MaxMotorForce
        {
            get => World._prismaticJointsData[Index].MaxMotorForce;
            set => World._prismaticJointsData[Index].MaxMotorForce = Math.Max(0f, value);
        }
        
        public bool CollideConnected => World._prismaticJointsData[Index].CollideConnected;

        public float ConstraintHertz
        {
            get => World._prismaticJointsData[Index].ConstraintHertz;
            set => World._prismaticJointsData[Index].ConstraintHertz = Math.Max(0f, value);
        }

        public float ConstraintDampingRatio
        {
            get => World._prismaticJointsData[Index].ConstraintDampingRatio;
            set => World._prismaticJointsData[Index].ConstraintDampingRatio = Math.Max(0f, value);
        }

        public void SetSpringEnabled(bool enable) => EnableSpring = enable;
        public void SetSpringFrequencyHz(float hz) => FrequencyHz = hz;
        public void SetSpringDampingRatio(float ratio) => DampingRatio = ratio;
        public void SetTargetTranslation(float translation) => TargetTranslation = translation;
        public void SetMotorEnabled(bool enable) => EnableMotor = enable;
        public void SetMotorSpeed(float speed) => MotorSpeed = speed;
        public void SetMaxMotorForce(float force) => MaxMotorForce = force;
        public void SetLimitEnabled(bool enable) => EnableLimit = enable;
        
        public void SetConstraintTuning(float hertz, float dampingRatio)
        {
            ConstraintHertz = hertz;
            ConstraintDampingRatio = dampingRatio;
        }

        public void SetLimits(float lower, float upper)
        {
            ref PrismaticJointData data = ref World._prismaticJointsData[Index];
            data.EnableLimit = true;
            data.LowerTranslation = Math.Min(lower, upper);
            data.UpperTranslation = Math.Max(lower, upper);
            data.LowerImpulse = 0f;
            data.UpperImpulse = 0f;
            BodyA.SetAwake(true);
            BodyB.SetAwake(true);
        }

        public float GetTranslation()
        {
            Body bA = BodyA;
            Body bB = BodyB;
            Vec2 pA = Transform.Mul(bA.Transform, LocalAnchorA);
            Vec2 pB = Transform.Mul(bB.Transform, LocalAnchorB);
            Vec2 d = pB - pA;
            Vec2 axis = Rot.Mul(bA.Transform.Q, LocalAxisA);
            return Vec2.Dot(d, axis);
        }

        public float GetSpeed()
        {
            Body bA = BodyA;
            Body bB = BodyB;
            
            Vec2 axis = Rot.Mul(bA.Transform.Q, LocalAxisA);
            Vec2 rA = Rot.Mul(bA.Transform.Q, LocalAnchorA - bA.LocalCenter);
            Vec2 rB = Rot.Mul(bB.Transform.Q, LocalAnchorB - bB.LocalCenter);
            Vec2 d = (bB.GetWorldCenter() - bA.GetWorldCenter()) + (rB - rA);

            Vec2 vA = bA.LinearVelocity + Vec2.Cross(bA.AngularVelocity, rA);
            Vec2 vB = bB.LinearVelocity + Vec2.Cross(bB.AngularVelocity, rB);
            Vec2 vRel = vB - vA;

            return Vec2.Dot(d, Vec2.Cross(bA.AngularVelocity, axis)) + Vec2.Dot(axis, vRel);
        }

        public float GetMotorForce(float invDt)
        {
            return World._prismaticJointsData[Index].MotorImpulse * invDt;
        }

        public Vec2 GetReactionForce(float invDt)
        {
            ref PrismaticJointData data = ref World._prismaticJointsData[Index];
            float axialImpulse = data.MotorImpulse + data.SpringImpulse + data.LowerImpulse - data.UpperImpulse;
            Vec2 force = invDt * (data.Impulse.X * data.Perp + axialImpulse * data.Axis);
            return force;
        }

        public float GetReactionTorque(float invDt)
        {
            return invDt * World._prismaticJointsData[Index].Impulse.Y;
        }
    }
}
