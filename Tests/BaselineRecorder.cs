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

            int subStepCount = 1;
            string? envSub = Environment.GetEnvironmentVariable("B2_SUBSTEP");
            if (envSub != null && int.TryParse(envSub, out int n))
            {
                subStepCount = Math.Max(1, n);
            }
            bool perBodyCCD = Environment.GetEnvironmentVariable("B2_PERBODY_CCD") == "1";
            // Phase 2.5 toggle — flag-on enables the cpp-v3 delta-position
            // model (Body.Transform = step-start + delta, joints write to
            // delta arrays, soft-joint DeltaCenter re-captured each Init).
            bool useDelta = Environment.GetEnvironmentVariable("B2_DELTA") == "1";

            Console.WriteLine();
            Console.WriteLine($"# Sample metrics — SubStepCount={subStepCount}, UsePerBodyCCD={perBodyCCD}, UseDeltaPositionTracking={useDelta}");
            Console.WriteLine("| Sample | peakV | lateV | peakW | minY | fellThrough |");
            Console.WriteLine("|--------|------:|------:|------:|-----:|------------:|");

            foreach (ISample sample in SampleCatalog.All)
            {
                try
                {
                    SampleMetrics m = new SampleMetrics(new TunedSample(sample, subStepCount, perBodyCCD, useDelta));
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

        /// <summary>Wraps a sample to override its world def's SubStepCount + UsePerBodyCCD + UseDeltaPositionTracking.</summary>
        private sealed class TunedSample : ISample
        {
            private readonly ISample _inner;
            private readonly int _subStepCount;
            private readonly bool _perBodyCCD;
            private readonly bool _useDelta;
            public TunedSample(ISample inner, int subStepCount, bool perBodyCCD, bool useDelta)
            {
                _inner = inner;
                _subStepCount = subStepCount;
                _perBodyCCD = perBodyCCD;
                _useDelta = useDelta;
            }
            public string Name => _inner.Name;
            public int SubSteps => _inner.SubSteps;
            public WorldDef CreateWorldDef()
            {
                WorldDef def = _inner.CreateWorldDef().WithSubStepCount(_subStepCount);
                if (_perBodyCCD) def = def.UsePerBodyContinuous();
                if (_useDelta) def = def.UseDeltaPositions();
                return def;
            }
            public void Build(World world) => _inner.Build(world);
            public void Step(World world, float dt) => _inner.Step(world, dt);
            public void OnKey(char key) => _inner.OnKey(key);
        }
    }
}
