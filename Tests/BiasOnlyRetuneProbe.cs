using System;
using System.Collections.Generic;
using System.Text;
using Box2DNG.Viewer.Samples;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Box2DNG.Tests
{
    /// <summary>
    /// Phase 2.5 task #80 — retune `ContactHertz` / `ContactDampingRatio`
    /// under flag-on + bias-only mode to recover the small Chain / Tumbler /
    /// VaryingFriction / Bridge regressions without losing the
    /// TheoJansen / EdgeShapes / CompoundShapes wins.
    ///
    /// Two probes:
    ///   1. `Regressions_FreshInstance_Recheck` — re-measure the 4 supposedly
    ///      regressing samples with fresh `new XxxSample()` instances. The
    ///      CircleStress bisect showed the singleton-leak issue can both
    ///      inflate and mask numbers, so we verify the regressions are even
    ///      real before tuning.
    ///   2. `HzDampingGrid_BiasOnly` — coarse grid sweep over
    ///      ContactHertz ∈ {30, 60, 120, 180} and ratio ∈ {1, 2, 5, 10}
    ///      against a mix of regression-prone and win-bearing samples.
    ///      Identifies a global tuning point that preserves wins and
    ///      recovers regressions.
    /// </summary>
    [TestClass]
    public class BiasOnlyRetuneProbe
    {
        private const int TotalSteps = 600;
        private const int LateWindowSteps = 60;
        private const float Dt = 1f / 60f;

        /// <summary>Factory map — each entry creates a FRESH sample instance every call.</summary>
        private static readonly Dictionary<string, Func<ISample>> Factories = new()
        {
            // Win samples (must not regress under retune)
            { "Pyramid",         () => new PyramidSample() },
            { "Dominos",         () => new DominosSample() },
            { "CompoundShapes",  () => new CompoundShapesSample() },
            { "TheoJansen",      () => new TheoJansenSample() },
            { "EdgeShapes",      () => new EdgeShapesSample() },
            { "BulletTest",      () => new BulletTestSample() },
            { "SliderCrank",     () => new SliderCrankSample() },
            { "CircleStress",    () => new CircleStressSample() },
            // Suspected regressions (recheck + recover)
            { "Chain",           () => new ChainSample() },
            { "Tumbler",         () => new TumblerSample() },
            { "VaryingFriction", () => new VaryingFrictionSample() },
            { "Bridge",          () => new BridgeSample() },
        };

        [TestMethod, Timeout(300000)]
        public void Regressions_FreshInstance_Recheck()
        {
            // For each suspect sample, fresh-instance measure:
            //   off + N1
            //   on + N1   (NGS path)
            //   on + N1 + bias-only
            // and confirm whether the original probe's regression holds up.
            string[] suspects = { "Chain", "Tumbler", "VaryingFriction", "Bridge" };
            var sb = new StringBuilder();
            sb.AppendLine();
            sb.AppendLine("Fresh-instance recheck of bias-only regressions");
            sb.AppendLine("===============================================");
            sb.AppendLine($"{"Sample",-18} {"off+N1",10} {"on+N1",10} {"on+N1+bias",12} {"Δ bias-off",10} {"verdict",-30}");
            foreach (string name in suspects)
            {
                var off  = Run(Factories[name](), useDelta: false, biasOnly: false);
                var on   = Run(Factories[name](), useDelta: true,  biasOnly: false);
                var onB  = Run(Factories[name](), useDelta: true,  biasOnly: true);
                float dBias = onB.LateV - off.LateV;
                string verdict;
                if (MathF.Abs(dBias) < 0.5f) verdict = "noise (< 0.5)";
                else if (dBias < 0) verdict = "WIN under bias-only";
                else verdict = "real regression";
                sb.AppendLine($"{name,-18} {off.LateV,10:F2} {on.LateV,10:F2} {onB.LateV,12:F2} {dBias,10:F2} {verdict,-30}");
            }
            Console.Error.WriteLine(sb.ToString());
        }

        [TestMethod, Timeout(900000)]
        public void HzDampingGrid_BiasOnly()
        {
            // Coarse (Hz × ratio) sweep with bias-only on every sample.
            // Baseline: off+N1 (target for regression samples to "recover" toward).
            // Reference: on+N1+bias-only with current defaults (120, 1) — already in
            // the first row of the grid.
            float[] hzs    = { 30f, 60f, 120f, 180f };
            float[] ratios = { 1f, 2f, 5f, 10f };

            // Off baseline per sample (lateV target).
            var offBaseline = new Dictionary<string, float>();
            foreach (var (name, factory) in Factories)
            {
                offBaseline[name] = Run(factory(), useDelta: false, biasOnly: false).LateV;
            }

            var sb = new StringBuilder();
            sb.AppendLine();
            sb.AppendLine("(Hz × ratio) grid sweep, flag-on + bias-only — lateV per sample");
            sb.AppendLine("================================================================");
            sb.AppendLine("Format: Δ vs off-baseline (negative = improvement, positive = regression)");
            sb.AppendLine("Current defaults: Hz=120, ratio=1");
            sb.AppendLine();

            // Header rows
            sb.Append($"{"Sample",-18} {"off",8} ");
            foreach (float hz in hzs)
                foreach (float r in ratios)
                    sb.Append($" H{hz,4:F0}r{r,4:F1}");
            sb.AppendLine();

            foreach (var (name, factory) in Factories)
            {
                sb.Append($"{name,-18} {offBaseline[name],8:F2} ");
                foreach (float hz in hzs)
                {
                    foreach (float r in ratios)
                    {
                        float lateV = Run(factory(), useDelta: true, biasOnly: true, contactHertz: hz, dampingRatio: r).LateV;
                        float delta = lateV - offBaseline[name];
                        sb.Append($" {delta,9:F2}");
                    }
                }
                sb.AppendLine();
            }

            Console.Error.WriteLine(sb.ToString());
        }

        private struct RunResult
        {
            public float LateV;
            public float PeakV;
            public int FellThrough;
            public int NonFinite;
        }

        private static RunResult Run(
            ISample sample,
            bool useDelta,
            bool biasOnly,
            float contactHertz = 120f,
            float dampingRatio = 1f)
        {
            WorldDef def = sample.CreateWorldDef()
                .WithSubStepCount(1)
                .WithContactHertz(contactHertz)
                .WithContactDamping(dampingRatio);
            if (useDelta) def = def.UseDeltaPositions();
            if (biasOnly) def = def.WithBiasOnlyContacts();
            var world = new World(def);
            sample.Build(world);

            var r = new RunResult();
            int lateStart = TotalSteps - LateWindowSteps;
            var fellThroughIds = new HashSet<int>();
            for (int i = 0; i < TotalSteps; ++i)
            {
                sample.Step(world, Dt);
                world.Step(Dt);
                for (int b = 0; b < world.Bodies.Count; ++b)
                {
                    Body body = world.Bodies[b];
                    if (body.Type != BodyType.Dynamic) continue;
                    Vec2 v = body.LinearVelocity;
                    Vec2 p = body.Transform.P;
                    if (float.IsNaN(v.X) || float.IsInfinity(v.X) || float.IsNaN(v.Y) || float.IsInfinity(v.Y))
                    {
                        r.NonFinite++;
                        continue;
                    }
                    float speed = v.Length;
                    if (speed > r.PeakV) r.PeakV = speed;
                    if (i >= lateStart && speed > r.LateV) r.LateV = speed;
                    if (p.Y < -2f) fellThroughIds.Add(body.Id);
                }
            }
            r.FellThrough = fellThroughIds.Count;
            return r;
        }
    }
}
