using System;

namespace Box2DNG
{
    /// <summary>
    /// Soft-constraint coefficients used by joints (and Phase 1 of TIER4_PARITY_PLAN
    /// onwards). Matches cpp box2d v3's <c>b2Softness</c>
    /// ([solver.h:127](../box2d-cpp/src/solver.h:127)).
    ///
    /// Constraint equation per velocity iteration:
    ///   impulse = -massScale * (Cdot + biasRate*C) - impulseScale * accImpulse
    ///
    /// where C is the position error and Cdot is the velocity error.
    /// </summary>
    public readonly struct Softness : IEquatable<Softness>
    {
        public readonly float BiasRate;
        public readonly float MassScale;
        public readonly float ImpulseScale;

        public Softness(float biasRate, float massScale, float impulseScale)
        {
            BiasRate = biasRate;
            MassScale = massScale;
            ImpulseScale = impulseScale;
        }

        /// <summary>Rigid constraint (no soft response): full mass scale, no impulse decay, no bias.</summary>
        public static readonly Softness Rigid = new Softness(0f, 1f, 0f);

        /// <summary>All zeros. Used as the "no constraint" sentinel before
        /// callers decide whether to fall back to <see cref="Rigid"/>.</summary>
        public static readonly Softness Zero = default;

        /// <summary>
        /// Construct a soft-spring constraint from natural frequency
        /// <paramref name="hertz"/>, damping ratio <paramref name="dampingRatio"/>
        /// (1 = critically damped), and sub-step duration <paramref name="h"/>.
        /// Returns <see cref="Zero"/> when <paramref name="hertz"/> is &lt;= 0 so
        /// the caller can substitute its own behaviour.
        ///
        /// Math: matches cpp's <c>b2MakeSoft</c> ([solver.h:239](../box2d-cpp/src/solver.h:239)):
        ///   omega = 2π·hertz
        ///   biasRate = omega / (2·ratio + h·omega)
        ///   massScale = h·omega·(2·ratio + h·omega) / (1 + h·omega·(2·ratio + h·omega))
        ///   impulseScale = 1 / (1 + h·omega·(2·ratio + h·omega))
        ///   massScale + impulseScale == 1 in all cases.
        /// </summary>
        public static Softness Make(float hertz, float dampingRatio, float h)
        {
            if (hertz <= 0f)
            {
                return Zero;
            }
            float omega = 2f * MathF.PI * hertz;
            float a1 = 2f * dampingRatio + h * omega;
            float a2 = h * omega * a1;
            float a3 = 1f / (1f + a2);
            return new Softness(omega / a1, a2 * a3, a3);
        }

        public bool IsZero => BiasRate == 0f && MassScale == 0f && ImpulseScale == 0f;

        public bool Equals(Softness other) =>
            BiasRate == other.BiasRate && MassScale == other.MassScale && ImpulseScale == other.ImpulseScale;
        public override bool Equals(object? obj) => obj is Softness other && Equals(other);
        public override int GetHashCode() => HashCode.Combine(BiasRate, MassScale, ImpulseScale);
        public static bool operator ==(Softness a, Softness b) => a.Equals(b);
        public static bool operator !=(Softness a, Softness b) => !a.Equals(b);
        public override string ToString() => $"Softness(bias={BiasRate:F3} mass={MassScale:F3} impulse={ImpulseScale:F3})";
    }
}
