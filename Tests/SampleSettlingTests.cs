using Box2DNG.Viewer.Samples;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Box2DNG.Tests
{
    /// <summary>
    /// Phase 0 baseline: for each sample whose expected steady state is "at
    /// rest", run a fixed window and pin the late-window peak speed. These
    /// numbers will move under the Phase 1-3 refactors (soft joints, sub-step,
    /// per-body CCD) — improvements should drop the peak, regressions should
    /// raise it.
    ///
    /// The thresholds here are deliberately *loose* — they're tuned to catch
    /// catastrophic divergence rather than to nail today's exact numbers.
    /// [BASELINE.md] records the actual current values; this test just enforces
    /// that no single number explodes.
    /// </summary>
    [TestClass]
    public class SampleSettlingTests
    {
        // 10 seconds of simulated time. Long enough for stacks and chains to
        // settle; short enough to keep the suite fast.
        private const int Steps = 600;

        [TestMethod]
        public void Cantilever_LateWindowBounded()
        {
            // Cantilever has welded chains. The viewer sample currently uses
            // body-level damping to suppress per-contact CCD energy buildup
            // (CantileverSample.cs). Phase 1+3 will replace that with soft
            // welds + per-body CCD; until then, the bounded late-window peak
            // is what we pin.
            var m = new SampleMetrics(new CantileverSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states: {m.NonFiniteBodyCount}. {m}");
            Assert.IsTrue(m.LateWindowPeakSpeed < 5f, $"Late-window peak {m.LateWindowPeakSpeed} too high. {m}");
            Assert.IsTrue(m.MinY > -5f, $"Bodies fell through: minY={m.MinY}. {m}");
        }

        [TestMethod]
        public void Pyramid_LateWindowBounded()
        {
            // KNOWN ISSUE (Phase 0 baseline): the Pyramid sample's stack
            // doesn't fully settle in 10s — peakV≈12.6 m/s. Phase 1 (soft
            // contacts/joints) + Phase 2 (sub-stepping) should drop this.
            // BASELINE.md tracks the exact value.
            var m = new SampleMetrics(new PyramidSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states: {m.NonFiniteBodyCount}. {m}");
            Assert.IsTrue(m.LateWindowPeakSpeed < 20f, $"Pyramid late window. {m}");
            Assert.IsTrue(m.MinY > -5f, $"Bodies fell through: minY={m.MinY}. {m}");
        }

        [TestMethod]
        public void Bridge_LateWindowBounded()
        {
            // Bridge is a row of planks connected by revolute joints. With
            // soft revolutes (Phase 1) the bridge should sag and settle; with
            // hard revolutes it oscillates a bit longer. Either way, no
            // explosions.
            var m = new SampleMetrics(new BridgeSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.LateWindowPeakSpeed < 10f, $"Bridge late window. {m}");
            Assert.IsTrue(m.MinY > -10f, $"Bridge minY. {m}");
        }

        [TestMethod]
        public void Dominos_LateWindowBounded()
        {
            // KNOWN ISSUE: 3 bodies fall through to ~y=-60. Phase 1+3 target.
            var m = new SampleMetrics(new DominosSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.LateWindowPeakSpeed < 50f, $"Dominos late window. {m}");
        }

        [TestMethod]
        public void CompoundShapes_LateWindowBounded()
        {
            // KNOWN ISSUE: 2 bodies fall through to y≈-450. Likely a sample-
            // level scene geometry issue (terrain gaps) or a contact stability
            // problem. Phase 1+3 target.
            var m = new SampleMetrics(new CompoundShapesSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.LateWindowPeakSpeed < 120f, $"CompoundShapes late window. {m}");
        }

        [TestMethod]
        public void Confined_LateWindowBounded()
        {
            // Confined is many bodies bouncing in a box. Bodies stay bounded
            // by the geometry; we just check no explosion.
            var m = new SampleMetrics(new ConfinedSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.LateWindowPeakSpeed < 30f, $"Confined late window. {m}");
        }

        [TestMethod]
        public void CircleStress_LateWindowBounded()
        {
            // KNOWN ISSUE: stack hits the world MaxLinearSpeed cap (120 m/s)
            // during settling and the late-window peak is ~64. This is the
            // same iterative-solver stack-stability story as Pyramid.
            // Phase 1+2 target.
            var m = new SampleMetrics(new CircleStressSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.LateWindowPeakSpeed < 100f, $"CircleStress late window. {m}");
        }

        [TestMethod]
        public void Chain_LateWindowBounded()
        {
            // ChainSegmentShape terrain - validates the chain-segment narrowphase
            // we ported in Tier 3. Bodies should settle on the terrain.
            var m = new SampleMetrics(new ChainSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.LateWindowPeakSpeed < 10f, $"Chain late window. {m}");
        }

        [TestMethod]
        public void CharacterCollision_LateWindowBounded()
        {
            // KNOWN ISSUE: 1 character falls through (y≈-297) — likely the
            // chain-segment cusp case from Tier 3 that needs the Solve3 +
            // chain-vs-polygon fixes to apply to the sample geometry. The
            // ChainCollisionParityTests cover the algorithm; the sample
            // still trips it.
            var m = new SampleMetrics(new CharacterCollisionSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.LateWindowPeakSpeed < 100f, $"CharacterCollision late window. {m}");
        }

        [TestMethod]
        public void EdgeShapes_LateWindowBounded()
        {
            // KNOWN ISSUE: 4 bodies fall through to y≈-306. The EdgeShapes
            // sample uses CW chain winding; Tier 3 tests use CCW (required
            // by the chain-segment-vs-polygon narrowphase). Sample needs
            // updating, but pinning the current behaviour.
            var m = new SampleMetrics(new EdgeShapesSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.LateWindowPeakSpeed < 100f, $"EdgeShapes late window. {m}");
        }

        [TestMethod]
        public void CollisionFiltering_LateWindowBounded()
        {
            var m = new SampleMetrics(new CollisionFilteringSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.LateWindowPeakSpeed < 15f, $"CollisionFiltering late window. {m}");
        }

        [TestMethod]
        public void VaryingFriction_LateWindowBounded()
        {
            var m = new SampleMetrics(new VaryingFrictionSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.LateWindowPeakSpeed < 15f, $"VaryingFriction late window. {m}");
        }

        [TestMethod]
        public void VaryingRestitution_LateWindowBounded()
        {
            // Restitution scene — bodies should keep bouncing but not explode.
            var m = new SampleMetrics(new VaryingRestitutionSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.LateWindowPeakSpeed < 30f, $"VaryingRestitution late window. {m}");
        }
    }
}
