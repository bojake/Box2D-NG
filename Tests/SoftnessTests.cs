using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Box2DNG.Tests
{
    /// <summary>
    /// Phase 1 foundation: the Hertz-driven soft-constraint coefficients used
    /// by joints. Pin the algebraic identities so any future port-level change
    /// can't accidentally drift from cpp's <c>b2MakeSoft</c>.
    /// </summary>
    [TestClass]
    public class SoftnessTests
    {
        private const float Eps = 1e-5f;

        [TestMethod]
        public void Make_HertzZero_ReturnsZero()
        {
            Softness s = Softness.Make(0f, 0.5f, 1f / 60f);
            Assert.IsTrue(s.IsZero, $"Expected Zero, got {s}");
            Assert.AreEqual(Softness.Zero, s);
        }

        [TestMethod]
        public void Make_HertzNegative_ReturnsZero()
        {
            Softness s = Softness.Make(-5f, 0.5f, 1f / 60f);
            Assert.IsTrue(s.IsZero);
        }

        [TestMethod]
        public void Make_MassPlusImpulseScale_EqualsOne()
        {
            // cpp's invariant: massScale + impulseScale == 1 always.
            foreach (float hz in new[] { 1f, 5f, 30f, 120f, 1000f })
            foreach (float ratio in new[] { 0f, 0.1f, 0.5f, 1f, 2f })
            foreach (float h in new[] { 1f / 60f, 1f / 240f, 1f / 30f })
            {
                Softness s = Softness.Make(hz, ratio, h);
                Assert.AreEqual(1f, s.MassScale + s.ImpulseScale, Eps,
                    $"massScale + impulseScale != 1 for hz={hz} ratio={ratio} h={h}: {s}");
            }
        }

        [TestMethod]
        public void Make_ZeroDamping_MatchesClosedForm()
        {
            // cpp comment: "If z == 0:
            //   bias = 1/h, massScale = hw^2 / (1 + hw^2), impulseScale = 1 / (1 + hw^2)"
            float hertz = 30f;
            float h = 1f / 60f;
            Softness s = Softness.Make(hertz, 0f, h);

            float omega = 2f * MathF.PI * hertz;
            float hw = h * omega;
            float expectedBias = 1f / h;
            float expectedImpulse = 1f / (1f + hw * hw);
            float expectedMass = (hw * hw) / (1f + hw * hw);

            Assert.AreEqual(expectedBias, s.BiasRate, Eps * MathF.Abs(expectedBias) + Eps);
            Assert.AreEqual(expectedImpulse, s.ImpulseScale, Eps);
            Assert.AreEqual(expectedMass, s.MassScale, Eps);
        }

        [TestMethod]
        public void Make_HighHertz_ApproachesRigid()
        {
            // As hertz -> infinity: massScale -> 1, impulseScale -> 0, bias -> 1/h.
            Softness s = Softness.Make(10000f, 0.5f, 1f / 60f);
            Assert.IsTrue(s.MassScale > 0.999f, $"massScale {s.MassScale} should approach 1");
            Assert.IsTrue(s.ImpulseScale < 0.001f, $"impulseScale {s.ImpulseScale} should approach 0");
        }

        [TestMethod]
        public void Rigid_FullMassScale()
        {
            Assert.AreEqual(0f, Softness.Rigid.BiasRate);
            Assert.AreEqual(1f, Softness.Rigid.MassScale);
            Assert.AreEqual(0f, Softness.Rigid.ImpulseScale);
        }

        [TestMethod]
        public void Equality_StructSemantics()
        {
            Softness a = Softness.Make(30f, 0.5f, 1f / 60f);
            Softness b = Softness.Make(30f, 0.5f, 1f / 60f);
            Softness c = Softness.Make(15f, 0.5f, 1f / 60f);
            Assert.AreEqual(a, b);
            Assert.AreNotEqual(a, c);
            Assert.AreEqual(a.GetHashCode(), b.GetHashCode());
            Assert.IsTrue(a == b);
            Assert.IsTrue(a != c);
        }

        [TestMethod]
        public void Make_KnownCppCalibration_Pi4()
        {
            // From cpp b2MakeSoft comment:
            //   if w = π/4 * inv_h: massScale ~= 0.38, impulseScale ~= 0.62
            // Solve for hertz: 2π·hertz = π/(4·h) -> hertz = 1/(8·h)
            float h = 1f / 60f;
            float hertz = 1f / (8f * h);
            Softness s = Softness.Make(hertz, 0f, h);
            // Hand-computed: pi^2/(16+pi^2) ≈ 0.38133, 16/(16+pi^2) ≈ 0.61867
            Assert.AreEqual(0.38133f, s.MassScale, 0.001f);
            Assert.AreEqual(0.61867f, s.ImpulseScale, 0.001f);
        }
    }
}
