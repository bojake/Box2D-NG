using System;
using System.Collections.Generic;
using Box2DNG.Viewer.Samples;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Box2DNG.Tests
{
    /// <summary>
    /// Phase 0 of the cpp v3 parity plan ([TIER4_PARITY_PLAN.md]). For every
    /// sample in the viewer catalog, run the sample's Build + Step path
    /// (same code path the viewer uses) for a short window and pin the
    /// invariants that should hold *regardless* of solver tuning:
    ///
    ///   - no NaN/Infinity in any body's transform or velocity
    ///   - no body exceeds the world's `MaximumLinearSpeed` cap (a finite proxy
    ///     for "didn't explode")
    ///   - the catalog itself loads and every sample's Build() succeeds
    ///
    /// Phase 1-3 will move numbers in many samples; this test is the safety
    /// net that catches catastrophic regressions while the per-sample tests
    /// (settling, fall-through, etc.) pin the scene-specific quality.
    /// </summary>
    [TestClass]
    public class AllSamplesRegressionTests
    {
        // Run for 1 second of simulated time. Long enough to surface explosions
        // and finite-failure modes; short enough to keep the suite fast.
        private const int Steps = 60;

        [TestMethod]
        public void AllSamples_StayFinite_FromCatalog()
        {
            int failureCount = 0;
            var failures = new System.Text.StringBuilder();

            foreach (ISample sample in SampleCatalog.All)
            {
                try
                {
                    SampleMetrics m = new SampleMetrics(sample);
                    m.Run(Steps);
                    if (m.NonFiniteBodyCount > 0)
                    {
                        failureCount++;
                        failures.AppendLine($"  {sample.Name}: {m.NonFiniteBodyCount} non-finite body snapshots");
                    }

                    float cap = m.World.Def.MaximumLinearSpeed;
                    if (m.PeakLinearSpeed >= cap)
                    {
                        failureCount++;
                        failures.AppendLine($"  {sample.Name}: peak speed {m.PeakLinearSpeed:F1} hit world cap {cap}");
                    }
                }
                catch (Exception ex)
                {
                    failureCount++;
                    failures.AppendLine($"  {sample.Name}: {ex.GetType().Name}: {ex.Message}");
                }
            }

            Assert.AreEqual(0, failureCount, $"Sample regression failures:\n{failures}");
        }

        /// <summary>
        /// Each ISample must construct cleanly and report the metadata the viewer
        /// uses (Name, SubSteps). Catches stray null refs and broken SubSteps
        /// configuration before the viewer scenes are touched.
        /// </summary>
        [TestMethod]
        public void AllSamples_HaveValidMetadata()
        {
            foreach (ISample sample in SampleCatalog.All)
            {
                Assert.IsFalse(string.IsNullOrWhiteSpace(sample.Name), "Sample.Name must be set.");
                Assert.IsTrue(sample.SubSteps >= 1, $"{sample.Name}: SubSteps must be >= 1, was {sample.SubSteps}.");
                WorldDef def = sample.CreateWorldDef();
                Assert.IsNotNull(def, $"{sample.Name}: CreateWorldDef returned null.");
            }
        }
    }
}
