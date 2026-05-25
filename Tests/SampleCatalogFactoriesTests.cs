using System;
using Box2DNG.Viewer.Samples;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Box2DNG.Tests
{
    /// <summary>
    /// Task #83 regression test. The original <see cref="SampleCatalog.All"/>
    /// returned singleton <see cref="ISample"/> instances; any probe that
    /// called <c>Build()</c> on the same instance more than once accumulated
    /// hidden instance state (e.g. <c>CircleStressSample._rng</c>'s
    /// <c>Random(1234)</c> advances every <c>Build</c>), producing different
    /// post-step world states for the same nominal configuration. That
    /// inflated reported regressions across the cause-investigation probes.
    ///
    /// These tests pin:
    ///   1. <see cref="SampleCatalog.Factories"/> is the same length as
    ///      <see cref="SampleCatalog.All"/> and shares names in the same order.
    ///   2. Each factory returns a FRESH instance every call.
    ///   3. Building the same sample twice via factories produces an
    ///      identical post-step world state, while building twice on the
    ///      singleton from <see cref="SampleCatalog.All"/> diverges for
    ///      samples with internal RNG state (demonstrating why factories
    ///      are required).
    /// </summary>
    [TestClass]
    public class SampleCatalogFactoriesTests
    {
        [TestMethod]
        public void Factories_MatchAll_InOrder_ByName()
        {
            Assert.AreEqual(SampleCatalog.All.Count, SampleCatalog.Factories.Count,
                "Factories and All must have the same length.");
            for (int i = 0; i < SampleCatalog.Factories.Count; ++i)
            {
                ISample fromFactory = SampleCatalog.Factories[i]();
                Assert.AreEqual(SampleCatalog.All[i].Name, fromFactory.Name,
                    $"Factory[{i}] name mismatch with All[{i}].");
            }
        }

        [TestMethod]
        public void Factory_ReturnsFreshInstanceEachCall()
        {
            for (int i = 0; i < SampleCatalog.Factories.Count; ++i)
            {
                ISample a = SampleCatalog.Factories[i]();
                ISample b = SampleCatalog.Factories[i]();
                Assert.AreNotSame(a, b, $"Factory[{i}] ({a.Name}) returned the same instance twice — leak risk.");
            }
        }

        /// <summary>
        /// Demonstrates the leak. Build <see cref="CircleStressSample"/> twice
        /// on the SAME instance — its <c>Random(1234)</c> field advances, so
        /// the second build produces a different scene. Same sample built
        /// via two SEPARATE factory calls produces identical scenes.
        /// </summary>
        [TestMethod]
        public void CircleStress_SingletonLeak_VersusFreshInstance()
        {
            // Path A: singleton — build twice on the same instance.
            ISample singleton = new CircleStressSample();
            float singletonFirst  = BuildAndSumDynamicMasses(singleton);
            float singletonSecond = BuildAndSumDynamicMasses(singleton);

            // Path B: fresh instances — build once each on two separate instances.
            float freshA = BuildAndSumDynamicMasses(new CircleStressSample());
            float freshB = BuildAndSumDynamicMasses(new CircleStressSample());

            Assert.AreEqual(freshA, freshB, 0f,
                "Two fresh CircleStressSample instances must produce identical " +
                "summed dynamic mass. If this fails, sample state is leaking through " +
                "static or shared mutable state — not just instance RNG.");

            Assert.AreNotEqual(singletonFirst, singletonSecond,
                "Sanity check: building the same CircleStressSample twice must produce " +
                "DIFFERENT total mass because its Random(1234) advances. If this fails, " +
                "the leak may have been fixed inside CircleStressSample (good!) and the " +
                "assertion should be relaxed.");
        }

        /// <summary>
        /// Build the sample into a fresh world and return the summed mass of
        /// every dynamic body. CircleStress's randomized loop creates 615
        /// circles with `radius = 1f ± 0.5f * RandomRange(0.5f, 1f)` and
        /// `density = radius * 1.5f`, so the total mass depends on the
        /// instance's `Random` state. A second Build() on the same instance
        /// produces a different total; a second Build() on a fresh instance
        /// produces an identical total.
        /// </summary>
        private static float BuildAndSumDynamicMasses(ISample sample)
        {
            World world = new World(sample.CreateWorldDef());
            sample.Build(world);
            float total = 0f;
            for (int i = 0; i < world.Bodies.Count; ++i)
            {
                Body b = world.Bodies[i];
                if (b.Type == BodyType.Dynamic) total += b.Mass;
            }
            return total;
        }
    }
}
