using System;
using Box2DNG.Viewer.Samples;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Box2DNG.Tests
{
    /// <summary>
    /// Phase 0 deliverable: emit the current metric values for every sample
    /// to stdout. Run once, paste output into [BASELINE.md], then re-run
    /// after each Phase 1-3 milestone to compare. Not run in normal CI —
    /// gated by an environment variable so it's opt-in.
    ///
    /// Run with:
    ///   B2_BASELINE=1 dotnet test --filter "FullyQualifiedName~BaselineRecorder"
    /// </summary>
    [TestClass]
    public class BaselineRecorder
    {
        private const int Steps = 600;

        [TestMethod]
        public void RecordAllSampleMetrics()
        {
            if (Environment.GetEnvironmentVariable("B2_BASELINE") != "1")
            {
                Assert.Inconclusive("Set B2_BASELINE=1 to run this collector.");
                return;
            }

            Console.WriteLine();
            Console.WriteLine("| Sample | peakV | lateV | peakW | minY | fellThrough |");
            Console.WriteLine("|--------|------:|------:|------:|-----:|------------:|");

            foreach (ISample sample in SampleCatalog.All)
            {
                try
                {
                    SampleMetrics m = new SampleMetrics(sample);
                    m.Run(Steps);
                    Console.WriteLine(
                        $"| {sample.Name} | {m.PeakLinearSpeed:F2} | {m.LateWindowPeakSpeed:F2} | " +
                        $"{m.PeakAngularSpeed:F2} | {m.MinY:F2} | {m.FellThroughCount} |");
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"| {sample.Name} | EXCEPTION | | | | | {ex.Message} |");
                }
            }
        }
    }
}
