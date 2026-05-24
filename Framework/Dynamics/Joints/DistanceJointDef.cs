using System;

namespace Box2DNG
{
    public sealed class DistanceJointDef
    {
        public Body BodyA { get; private set; }
        public Body BodyB { get; private set; }
        public Vec2 LocalAnchorA { get; private set; }
        public Vec2 LocalAnchorB { get; private set; }
        public float Length { get; private set; }
        public float FrequencyHz { get; private set; } = 0f;
        public float DampingRatio { get; private set; } = 0f;
        public bool CollideConnected { get; private set; }

        public DistanceJointDef(Body bodyA, Body bodyB, Vec2 worldAnchorA, Vec2 worldAnchorB)
        {
            BodyA = bodyA ?? throw new ArgumentNullException(nameof(bodyA));
            BodyB = bodyB ?? throw new ArgumentNullException(nameof(bodyB));
            LocalAnchorA = Transform.MulT(bodyA.Transform, worldAnchorA);
            LocalAnchorB = Transform.MulT(bodyB.Transform, worldAnchorB);
            Length = (worldAnchorB - worldAnchorA).Length;
        }

        public DistanceJointDef WithLength(float length) { Length = length; return this; }
        public DistanceJointDef WithFrequency(float hz) { FrequencyHz = hz; return this; }
        public DistanceJointDef WithFrequency(float hz, float dampingRatio)
        {
            FrequencyHz = hz;
            DampingRatio = dampingRatio;
            return this;
        }
        public DistanceJointDef WithDampingRatio(float ratio) { DampingRatio = ratio; return this; }
        public DistanceJointDef WithCollideConnected(bool collideConnected = true) { CollideConnected = collideConnected; return this; }
    }
}
