using System;
using System.Collections.Generic;
using System.Text;
using Box2DNG.Viewer.Samples;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Box2DNG.Tests
{
    /// <summary>
    /// Phase 2.5 cause #5 — bisect the CircleStress flag-on N=1 regression
    /// (off lateV 64.26 → on lateV 73.79, Δ +9.53). The only meaningful
    /// flag-on N=1 regression across the 35-sample probe.
    ///
    /// Tracks per-step max-dynamic-body speed for off+N1 vs on+N1 in
    /// lockstep, prints the divergence point, and identifies which body
    /// goes off the rails first.
    ///
    /// Each probe instantiates `new CircleStressSample()` so the sample's
    /// `_rng = new Random(1234)` field initializer fires fresh — avoiding
    /// the singleton-leakage issue tracked in task #83.
    /// </summary>
    [TestClass]
    public class CircleStressBisectProbe
    {
        private const int TotalSteps = 600;
        private const float Dt = 1f / 60f;

        private struct StepSnapshot
        {
            public float MaxSpeed;
            public int MaxSpeedBodyId;
            public Vec2 MaxSpeedBodyPos;
            public int CountOver30;
            public int CountOver60;
            public int CountOver90;
            public int FellThrough;     // bodies below y=-2
            public int NonFinite;
        }

        [TestMethod, Timeout(180000)]
        public void CircleStress_FlagOff_vs_FlagOn_DivergenceTrace()
        {
            StepSnapshot[] off = RunCircleStress(useDelta: false, biasOnly: false);
            StepSnapshot[] on  = RunCircleStress(useDelta: true,  biasOnly: false);
            StepSnapshot[] onBias = RunCircleStress(useDelta: true, biasOnly: true);

            // Find the first step where on's MaxSpeed diverges from off by >5 m/s
            int divergeStep = -1;
            for (int i = 0; i < TotalSteps; ++i)
            {
                if (MathF.Abs(on[i].MaxSpeed - off[i].MaxSpeed) > 5f)
                {
                    divergeStep = i;
                    break;
                }
            }

            // Find the first step where things visibly settle/explode
            var sb = new StringBuilder();
            sb.AppendLine();
            sb.AppendLine("CircleStress flag-off vs flag-on N=1 divergence trace");
            sb.AppendLine("======================================================");
            sb.AppendLine($"First |Δ MaxSpeed| > 5 m/s at step: {divergeStep}");
            sb.AppendLine();
            sb.AppendLine($"{"step",5} {"off MaxV",10} {"on MaxV",10} {"on+b MaxV",10} {"Δ on-off",10} {"off >60",8} {"on >60",8} {"off FT",8} {"on FT",8}");

            // Print every 30 steps + the divergence area in detail
            void PrintRow(int i)
            {
                sb.AppendLine($"{i,5} {off[i].MaxSpeed,10:F2} {on[i].MaxSpeed,10:F2} {onBias[i].MaxSpeed,10:F2} {(on[i].MaxSpeed - off[i].MaxSpeed),10:F2} {off[i].CountOver60,8} {on[i].CountOver60,8} {off[i].FellThrough,8} {on[i].FellThrough,8}");
            }

            for (int i = 0; i < TotalSteps; i += 30)
            {
                PrintRow(i);
            }
            sb.AppendLine();
            if (divergeStep >= 0)
            {
                sb.AppendLine($"=== Detailed window around divergence (step {divergeStep}) ===");
                int lo = Math.Max(0, divergeStep - 5);
                int hi = Math.Min(TotalSteps - 1, divergeStep + 15);
                for (int i = lo; i <= hi; ++i) PrintRow(i);

                sb.AppendLine();
                sb.AppendLine($"=== Body identity at divergence ===");
                sb.AppendLine($"off step {divergeStep}: max-speed body id={off[divergeStep].MaxSpeedBodyId} v={off[divergeStep].MaxSpeed:F2} pos=({off[divergeStep].MaxSpeedBodyPos.X:F2}, {off[divergeStep].MaxSpeedBodyPos.Y:F2})");
                sb.AppendLine($"on  step {divergeStep}: max-speed body id={on[divergeStep].MaxSpeedBodyId}  v={on[divergeStep].MaxSpeed:F2} pos=({on[divergeStep].MaxSpeedBodyPos.X:F2}, {on[divergeStep].MaxSpeedBodyPos.Y:F2})");
            }

            // Final state
            sb.AppendLine();
            sb.AppendLine($"=== Final state (step {TotalSteps - 1}) ===");
            PrintRow(TotalSteps - 1);

            // Late-window peak
            int lateStart = TotalSteps - 60;
            float lateOff = 0, lateOn = 0, lateOnBias = 0;
            for (int i = lateStart; i < TotalSteps; ++i)
            {
                if (off[i].MaxSpeed > lateOff) lateOff = off[i].MaxSpeed;
                if (on[i].MaxSpeed > lateOn) lateOn = on[i].MaxSpeed;
                if (onBias[i].MaxSpeed > lateOnBias) lateOnBias = onBias[i].MaxSpeed;
            }
            sb.AppendLine($"Late-window peak: off={lateOff:F2} on={lateOn:F2} on+bias={lateOnBias:F2}");

            Console.Error.WriteLine(sb.ToString());
        }

        private static StepSnapshot[] RunCircleStress(bool useDelta, bool biasOnly)
        {
            ISample sample = new CircleStressSample();
            WorldDef def = sample.CreateWorldDef().WithSubStepCount(1);
            if (useDelta) def = def.UseDeltaPositions();
            if (biasOnly) def = def.WithBiasOnlyContacts();
            var world = new World(def);
            sample.Build(world);

            var snaps = new StepSnapshot[TotalSteps];
            for (int i = 0; i < TotalSteps; ++i)
            {
                sample.Step(world, Dt);
                world.Step(Dt);

                var s = new StepSnapshot();
                for (int b = 0; b < world.Bodies.Count; ++b)
                {
                    Body body = world.Bodies[b];
                    if (body.Type != BodyType.Dynamic) continue;
                    Vec2 v = body.LinearVelocity;
                    Vec2 p = body.Transform.P;
                    if (float.IsNaN(v.X) || float.IsInfinity(v.X) || float.IsNaN(v.Y) || float.IsInfinity(v.Y))
                    {
                        s.NonFinite++;
                        continue;
                    }
                    float speed = v.Length;
                    if (speed > s.MaxSpeed)
                    {
                        s.MaxSpeed = speed;
                        s.MaxSpeedBodyId = body.Id;
                        s.MaxSpeedBodyPos = p;
                    }
                    if (speed > 30) s.CountOver30++;
                    if (speed > 60) s.CountOver60++;
                    if (speed > 90) s.CountOver90++;
                    if (p.Y < -2f) s.FellThrough++;
                }
                snaps[i] = s;
            }
            return snaps;
        }
    }
}
