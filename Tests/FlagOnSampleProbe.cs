using System;
using System.Collections.Generic;
using System.Text;
using Box2DNG.Viewer.Samples;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Box2DNG.Tests
{
    /// <summary>
    /// Phase 2.5 cause investigation — sample-by-sample probe at flag-on N=1.
    ///
    /// After the cause #1 (Cantilever Stage L revert) and cause #4 (AddPair Sweep
    /// fix) corrections, the headline samples (Pyramid, Dominos, CompoundShapes)
    /// improved at flag-on N=1. This probe sweeps the *entire* viewer catalog at
    /// flag-on N=1 and reports per-sample lateV / fall-through deltas so we can:
    ///
    ///   - confirm no other sample regresses at flag-on N=1 (post-fix)
    ///   - identify which samples still need attention before flipping the default
    ///   - measure cause #2 seed (`UseBiasOnlyContacts`) per-sample interaction
    ///     in a follow-up probe
    ///
    /// Probes are not assertions — they emit a table to stderr (no Assert.Fail
    /// unless something diverges to NaN). Numbers are recorded into BASELINE.md
    /// once we like the read.
    ///
    /// Uses <see cref="SampleCatalog.Factories"/> rather than <see
    /// cref="SampleCatalog.All"/> so every <c>Build()</c> gets a fresh
    /// sample instance. Pre-fix this probe re-used the catalog singletons
    /// and consumed shared <c>Random(1234)</c> state across configurations,
    /// inflating regression numbers (CircleStress reported +9.53 lateV when
    /// the true number with a fresh instance is +24.99, and Chain reported
    /// +0.53 when fresh-instance is actually −0.53). See task #83.
    /// </summary>
    [TestClass]
    public class FlagOnSampleProbe
    {
        private const int TotalSteps = 600;
        private const int LateWindowSteps = 60;
        private const float FallThroughY = -2f;

        private struct Result
        {
            public string Name;
            public float LateV;
            public float PeakV;
            public int FellThrough;
            public int NonFinite;
            public bool Exploded;
            public string Error;
        }

        [TestMethod, Timeout(600000)]
        public void AllSamples_FlagOff_vs_FlagOn_N1()
        {
            var off = new Dictionary<string, Result>();
            var on = new Dictionary<string, Result>();
            var names = new List<string>();

            foreach (Func<ISample> factory in SampleCatalog.Factories)
            {
                ISample s = factory();
                names.Add(s.Name);
                off[s.Name] = Probe(s, useDelta: false, biasOnly: false, n: 1);
            }
            foreach (Func<ISample> factory in SampleCatalog.Factories)
            {
                ISample s = factory();
                on[s.Name] = Probe(s, useDelta: true, biasOnly: false, n: 1);
            }

            var sb = new StringBuilder();
            sb.AppendLine();
            sb.AppendLine("Phase 2.5 flag-on N=1 sample-by-sample probe");
            sb.AppendLine("=============================================");
            sb.AppendLine($"{"Sample",-26} {"off lateV",10} {"on lateV",10} {"Δ lateV",10} {"off FT",6} {"on FT",6} {"notes",-20}");
            foreach (string name in names)
            {
                Result o = off[name];
                Result n = on[name];
                string notes = "";
                if (n.Exploded || o.Exploded) notes += "EXPLODE ";
                if (n.NonFinite > 0 || o.NonFinite > 0) notes += "NaN ";
                if (!string.IsNullOrEmpty(n.Error)) notes += $"on-err:{n.Error} ";
                if (!string.IsNullOrEmpty(o.Error)) notes += $"off-err:{o.Error} ";
                float delta = n.LateV - o.LateV;
                sb.AppendLine($"{name,-26} {o.LateV,10:F2} {n.LateV,10:F2} {delta,10:F2} {o.FellThrough,6} {n.FellThrough,6} {notes,-20}");
            }
            Console.Error.WriteLine(sb.ToString());
        }

        [TestMethod, Timeout(600000)]
        public void AllSamples_FlagOn_N1_BiasOnly()
        {
            var on = new Dictionary<string, Result>();
            var onBias = new Dictionary<string, Result>();
            var names = new List<string>();

            foreach (Func<ISample> factory in SampleCatalog.Factories)
            {
                ISample s = factory();
                names.Add(s.Name);
                on[s.Name] = Probe(s, useDelta: true, biasOnly: false, n: 1);
            }
            foreach (Func<ISample> factory in SampleCatalog.Factories)
            {
                ISample s = factory();
                onBias[s.Name] = Probe(s, useDelta: true, biasOnly: true, n: 1);
            }

            var sb = new StringBuilder();
            sb.AppendLine();
            sb.AppendLine("Phase 2.5 cause #2 seed: bias-only vs NGS, flag-on N=1");
            sb.AppendLine("=======================================================");
            sb.AppendLine($"{"Sample",-26} {"on lateV",10} {"+bias lateV",12} {"Δ lateV",10} {"on FT",6} {"+bias FT",8} {"notes",-20}");
            foreach (string name in names)
            {
                Result n = on[name];
                Result b = onBias[name];
                string notes = "";
                if (b.Exploded) notes += "EXPLODE ";
                if (b.NonFinite > 0) notes += "NaN ";
                if (!string.IsNullOrEmpty(b.Error)) notes += $"err:{b.Error}";
                float delta = b.LateV - n.LateV;
                sb.AppendLine($"{name,-26} {n.LateV,10:F2} {b.LateV,12:F2} {delta,10:F2} {n.FellThrough,6} {b.FellThrough,8} {notes,-20}");
            }
            Console.Error.WriteLine(sb.ToString());
        }

        [TestMethod, Timeout(120000)]
        public void Pyramid_BiasOnly_SubStep_Probe()
        {
            // Fresh PyramidSample() per probe — no shared state inside Pyramid,
            // but use the factory pattern consistently to keep the probe leak-immune
            // if Pyramid ever grows state.
            Result off_n1     = Probe(new PyramidSample(), useDelta: false, biasOnly: false, n: 1);
            Result on_n1      = Probe(new PyramidSample(), useDelta: true,  biasOnly: false, n: 1);
            Result on_n4      = Probe(new PyramidSample(), useDelta: true,  biasOnly: false, n: 4);
            Result on_n4_bias = Probe(new PyramidSample(), useDelta: true,  biasOnly: true,  n: 4);
            Result on_n1_bias = Probe(new PyramidSample(), useDelta: true,  biasOnly: true,  n: 1);

            Console.Error.WriteLine(
                $"Pyramid lateV: off+N1={off_n1.LateV:F2} (FT {off_n1.FellThrough})" +
                $"  on+N1={on_n1.LateV:F2} (FT {on_n1.FellThrough})" +
                $"  on+N4={on_n4.LateV:F2} (FT {on_n4.FellThrough})" +
                $"  on+N4+bias={on_n4_bias.LateV:F2} (FT {on_n4_bias.FellThrough})" +
                $"  on+N1+bias={on_n1_bias.LateV:F2} (FT {on_n1_bias.FellThrough})");
        }

        private static Result Probe(ISample sample, bool useDelta, bool biasOnly, int n)
        {
            var r = new Result { Name = sample.Name };
            try
            {
                WorldDef def = sample.CreateWorldDef();
                // Force exact N regardless of sample's declared SubSteps. We're isolating
                // the flag effect, not the sample's preferred sub-step count.
                def = def.WithSubStepCount(n);
                if (useDelta) def = def.UseDeltaPositions();
                if (biasOnly) def = def.WithBiasOnlyContacts();
                var world = new World(def);
                sample.Build(world);
                int lateStart = TotalSteps - LateWindowSteps;
                var fellThroughIds = new HashSet<int>();
                for (int i = 0; i < TotalSteps; ++i)
                {
                    sample.Step(world, 1f / 60f);
                    world.Step(1f / 60f);
                    for (int b = 0; b < world.Bodies.Count; ++b)
                    {
                        Body body = world.Bodies[b];
                        if (body.Type != BodyType.Dynamic) continue;
                        Vec2 p = body.Transform.P;
                        Vec2 v = body.LinearVelocity;
                        if (!IsFinite(p) || !IsFinite(v))
                        {
                            r.NonFinite++;
                            continue;
                        }
                        float speed = v.Length;
                        if (speed > r.PeakV) r.PeakV = speed;
                        if (i >= lateStart && speed > r.LateV) r.LateV = speed;
                        if (p.Y < FallThroughY) fellThroughIds.Add(body.Id);
                    }
                    if (r.PeakV >= world.Def.MaximumLinearSpeed * 0.99f) r.Exploded = true;
                }
                r.FellThrough = fellThroughIds.Count;
            }
            catch (Exception ex)
            {
                r.Error = ex.GetType().Name;
            }
            return r;
        }

        private static bool IsFinite(Vec2 v) =>
            !(float.IsNaN(v.X) || float.IsInfinity(v.X) || float.IsNaN(v.Y) || float.IsInfinity(v.Y));
    }
}
