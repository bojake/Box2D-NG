using System;

namespace Box2DNG
{
    public sealed class WeldJointDef
    {
        public Body BodyA { get; private set; }
        public Body BodyB { get; private set; }
        public Vec2 LocalAnchorA { get; private set; }
        public Vec2 LocalAnchorB { get; private set; }
        public float ReferenceAngle { get; private set; }
        public bool CollideConnected { get; private set; }

        // Soft-constraint tuning (Phase 1 of TIER4_PARITY_PLAN). Default Hertz=0
        // means "inherit world's JointHertz fallback", which defaults to rigid.
        // Set non-zero to make the weld behave as a damped spring instead of
        // a hard constraint — mirrors cpp box2d v3's WeldJointDef linear/angular
        // hertz + dampingRatio.
        public float LinearHertz { get; private set; }
        public float LinearDampingRatio { get; private set; }
        public float AngularHertz { get; private set; }
        public float AngularDampingRatio { get; private set; }

        public WeldJointDef WithCollideConnected(bool collideConnected = true)
        {
            CollideConnected = collideConnected;
            return this;
        }

        /// <summary>Configure the linear constraint as a Hertz-driven soft spring.</summary>
        public WeldJointDef WithLinearSpring(float hertz, float dampingRatio)
        {
            LinearHertz = Math.Max(0f, hertz);
            LinearDampingRatio = Math.Max(0f, dampingRatio);
            return this;
        }

        /// <summary>Configure the angular constraint as a Hertz-driven soft spring.</summary>
        public WeldJointDef WithAngularSpring(float hertz, float dampingRatio)
        {
            AngularHertz = Math.Max(0f, hertz);
            AngularDampingRatio = Math.Max(0f, dampingRatio);
            return this;
        }

        public WeldJointDef(Body bodyA, Body bodyB, Vec2 worldAnchor)
        {
            BodyA = bodyA ?? throw new ArgumentNullException(nameof(bodyA));
            BodyB = bodyB ?? throw new ArgumentNullException(nameof(bodyB));
            LocalAnchorA = Transform.MulT(bodyA.Transform, worldAnchor);
            LocalAnchorB = Transform.MulT(bodyB.Transform, worldAnchor);
            ReferenceAngle = bodyB.Transform.Q.Angle - bodyA.Transform.Q.Angle;
        }
    }
}
