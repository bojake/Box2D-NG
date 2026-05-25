using System;
using System.Text;
using Box2DNG.Viewer.Samples;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Box2DNG.Tests
{
    /// <summary>
    /// Phase 2.5 task #84 — diagnose VaryingFriction explosions under
    /// bias-only + under-damped soft-spring tunings. The (Hz × ratio)
    /// grid sweep (task #80) surfaced 8 of 16 cells where VaryingF lateV
    /// blows up to 33-91 m/s (vs the flag-off baseline of 1.58). The scene
    /// is 5 boxes (frictions 0.75, 0.5, 0.35, 0.1, 0.0) falling onto a
    /// flat horizontal ground — no slope, no joints, no restitution. The
    /// explosion shouldn't be possible from the physics; it must be a
    /// numerical instability in the bias↔friction-cap coupling.
    ///
    /// Per-step trace at H30r1 (explosion) vs H120r1 (current default,
    /// stable). Identifies which body, at which step, starts the runaway.
    /// </summary>
    [TestClass]
    public class VaryingFrictionExplosionProbe
    {
        private const int TotalSteps = 600;
        private const float Dt = 1f / 60f;

        [TestMethod, Timeout(60000)]
        public void VaryingFriction_H30r1_vs_H120r1_DivergenceTrace()
        {
            BodyTrace[] stable = TraceVaryingFriction(hertz: 120f, damping: 1f);
            BodyTrace[] exploding = TraceVaryingFriction(hertz: 30f, damping: 1f);

            var sb = new StringBuilder();
            sb.AppendLine();
            sb.AppendLine("VaryingFriction explosion trace: H120r1 (stable) vs H30r1 (exploding)");
            sb.AppendLine("=====================================================================");
            sb.AppendLine($"Scene: 5 boxes (friction 0.75, 0.5, 0.35, 0.1, 0.0) falling on horizontal ground");
            sb.AppendLine();

            // Find first step where exploding > 5 m/s (boxes should be settled by step 60).
            int firstWild = -1;
            for (int i = 0; i < TotalSteps; ++i)
            {
                float maxV = 0f;
                for (int b = 0; b < 5; ++b)
                {
                    float v = exploding[i].Speeds[b];
                    if (v > maxV) maxV = v;
                }
                if (i > 60 && maxV > 5f)
                {
                    firstWild = i;
                    break;
                }
            }
            sb.AppendLine($"First step after settling window (i>60) where any exploding body v > 5 m/s: {firstWild}");

            // Print key checkpoints + the divergence window
            int[] checkpoints = { 0, 15, 30, 45, 60, 100, 150, 200, 300, 400, 500, 599 };
            sb.AppendLine();
            sb.AppendLine($"{"step",5} | {"H120r1 v (per-box)",-50} | {"H30r1 v (per-box)",-50}");
            sb.AppendLine($"{"     ",5} | {"box0(μ=0.75) box1(μ=0.5) box2(μ=0.35) box3(μ=0.1) box4(μ=0)",-50} | (same)");
            sb.AppendLine();
            foreach (int i in checkpoints)
            {
                sb.AppendLine($"{i,5} | {SpeedRow(stable[i]),-50} | {SpeedRow(exploding[i]),-50}");
            }

            // If found a divergence, zoom in
            if (firstWild >= 0)
            {
                sb.AppendLine();
                sb.AppendLine($"=== Detailed window around divergence (step {firstWild}) ===");
                int lo = Math.Max(0, firstWild - 5);
                int hi = Math.Min(TotalSteps - 1, firstWild + 10);
                for (int i = lo; i <= hi; ++i)
                {
                    sb.AppendLine($"{i,5} | {SpeedRow(stable[i]),-50} | {SpeedRow(exploding[i]),-50}");
                }
                sb.AppendLine();
                sb.AppendLine($"=== Position + angular velocity at step {firstWild} ===");
                for (int b = 0; b < 5; ++b)
                {
                    sb.AppendLine(
                        $"box{b} (μ={Frictions[b]:F2}): " +
                        $"H120r1 pos=({stable[firstWild].PositionsX[b]:F2},{stable[firstWild].PositionsY[b]:F2}) v={stable[firstWild].Speeds[b]:F2} w={stable[firstWild].AngularVel[b]:F2}  |  " +
                        $"H30r1 pos=({exploding[firstWild].PositionsX[b]:F2},{exploding[firstWild].PositionsY[b]:F2}) v={exploding[firstWild].Speeds[b]:F2} w={exploding[firstWild].AngularVel[b]:F2}");
                }
            }

            // Late-window peaks per box
            sb.AppendLine();
            sb.AppendLine("=== Late-window (steps 540-599) per-box peak speed ===");
            for (int b = 0; b < 5; ++b)
            {
                float stPeak = 0, exPeak = 0;
                for (int i = TotalSteps - 60; i < TotalSteps; ++i)
                {
                    if (stable[i].Speeds[b] > stPeak) stPeak = stable[i].Speeds[b];
                    if (exploding[i].Speeds[b] > exPeak) exPeak = exploding[i].Speeds[b];
                }
                sb.AppendLine($"box{b} (μ={Frictions[b]:F2}): H120r1={stPeak:F2}  H30r1={exPeak:F2}");
            }

            Console.Error.WriteLine(sb.ToString());
        }

        [TestMethod, Timeout(60000)]
        public void VaryingFriction_NGS_vs_BiasOnly_AtH30r1()
        {
            // Does the v2-NGS backstop rescue VaryingFriction at H30r1 (which
            // explodes in bias-only mode)? If yes, the explosion is purely a
            // bias-only-mode artifact — the NGS pass dampens whatever runaway
            // the friction-bias coupling produces. If no, the (Hz, ratio)
            // resonance is independent of NGS and inherent to the soft path.
            BodyTrace[] biasOnly = TraceVaryingFriction(hertz: 30f, damping: 1f);
            BodyTrace[] ngsAtH30 = TraceVaryingFriction(hertz: 30f, damping: 1f, biasOnly: false);

            var sb = new StringBuilder();
            sb.AppendLine();
            sb.AppendLine("VaryingFriction H30r1: bias-only vs NGS backstop");
            sb.AppendLine("=================================================");
            float biasPeak = 0, ngsPeak = 0;
            for (int i = TotalSteps - 60; i < TotalSteps; ++i)
            {
                for (int b = 0; b < 5; ++b)
                {
                    if (biasOnly[i].Speeds[b] > biasPeak) biasPeak = biasOnly[i].Speeds[b];
                    if (ngsAtH30[i].Speeds[b] > ngsPeak) ngsPeak = ngsAtH30[i].Speeds[b];
                }
            }
            sb.AppendLine($"Late-window peak speed: bias-only={biasPeak:F2}  NGS-backstop={ngsPeak:F2}");
            sb.AppendLine();
            sb.AppendLine($"{"step",5} | {"bias-only (per-box v)",30} | {"NGS (per-box v)",30}");
            int[] checkpoints = { 30, 60, 100, 200, 300, 400, 500, 599 };
            foreach (int i in checkpoints)
            {
                sb.AppendLine($"{i,5} | {SpeedRow(biasOnly[i]),-30} | {SpeedRow(ngsAtH30[i]),-30}");
            }
            Console.Error.WriteLine(sb.ToString());
        }

        [TestMethod, Timeout(60000)]
        public void VaryingFriction_DampingRatio_Sweep_AtHz30()
        {
            // Sweep damping ratio at Hz=30 to see where the explosion threshold lies.
            float[] ratios = { 0.5f, 1f, 2f, 3f, 5f, 7.5f, 10f };
            var sb = new StringBuilder();
            sb.AppendLine();
            sb.AppendLine("VaryingFriction damping-ratio sweep at Hz=30, bias-only");
            sb.AppendLine("========================================================");
            sb.AppendLine($"{"ratio",6} | {"box0(μ=0.75)",13} {"box1(μ=0.5)",12} {"box2(μ=0.35)",13} {"box3(μ=0.1)",12} {"box4(μ=0)",10} | {"maxV",6}");
            foreach (float r in ratios)
            {
                BodyTrace[] trace = TraceVaryingFriction(hertz: 30f, damping: r);
                float[] peaks = new float[5];
                float maxV = 0f;
                for (int i = TotalSteps - 60; i < TotalSteps; ++i)
                {
                    for (int b = 0; b < 5; ++b)
                    {
                        if (trace[i].Speeds[b] > peaks[b]) peaks[b] = trace[i].Speeds[b];
                        if (trace[i].Speeds[b] > maxV) maxV = trace[i].Speeds[b];
                    }
                }
                sb.AppendLine($"{r,6:F1} | {peaks[0],13:F2} {peaks[1],12:F2} {peaks[2],13:F2} {peaks[3],12:F2} {peaks[4],10:F2} | {maxV,6:F2}");
            }
            Console.Error.WriteLine(sb.ToString());
        }

        private static readonly float[] Frictions = { 0.75f, 0.5f, 0.35f, 0.1f, 0.0f };

        private struct BodyTrace
        {
            public float[] Speeds;
            public float[] AngularVel;
            public float[] PositionsX;
            public float[] PositionsY;
        }

        private static string SpeedRow(BodyTrace t)
        {
            var sb = new StringBuilder();
            for (int b = 0; b < 5; ++b)
            {
                sb.Append($"{t.Speeds[b],6:F1} ");
            }
            return sb.ToString();
        }

        private static BodyTrace[] TraceVaryingFriction(float hertz, float damping, bool biasOnly = true)
        {
            ISample sample = new VaryingFrictionSample();
            WorldDef def = sample.CreateWorldDef()
                .WithSubStepCount(1)
                .UseDeltaPositions()
                .WithContactHertz(hertz)
                .WithContactDamping(damping);
            if (biasOnly) def = def.WithBiasOnlyContacts();
            var world = new World(def);
            sample.Build(world);

            var traces = new BodyTrace[TotalSteps];
            // Snapshot dynamic body indices in build order (skip the static ground).
            int[] dynIds = new int[5];
            int dyn = 0;
            for (int b = 0; b < world.Bodies.Count && dyn < 5; ++b)
            {
                if (world.Bodies[b].Type == BodyType.Dynamic)
                {
                    dynIds[dyn++] = b;
                }
            }

            for (int i = 0; i < TotalSteps; ++i)
            {
                sample.Step(world, Dt);
                world.Step(Dt);
                var t = new BodyTrace
                {
                    Speeds = new float[5],
                    AngularVel = new float[5],
                    PositionsX = new float[5],
                    PositionsY = new float[5]
                };
                for (int b = 0; b < 5; ++b)
                {
                    Body body = world.Bodies[dynIds[b]];
                    Vec2 v = body.LinearVelocity;
                    Vec2 p = body.Transform.P;
                    t.Speeds[b] = float.IsNaN(v.X) ? float.NaN : v.Length;
                    t.AngularVel[b] = body.AngularVelocity;
                    t.PositionsX[b] = p.X;
                    t.PositionsY[b] = p.Y;
                }
                traces[i] = t;
            }
            return traces;
        }
    }
}
