using System;
using System.Collections.Generic;
using System.Text;
using Box2DNG.Viewer.Samples;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Box2DNG.Tests
{
    /// <summary>
    /// Phase 3 / Task #86 — diagnose UsePerBodyCCD regressions.
    ///
    /// Task #85's flip attempt found per-body CCD catastrophically fails
    /// CircleStress (8 bodies to y=-3290), Cantilever (8 through),
    /// FrictionJoint (1 through). Now that delta-tracking is the default
    /// and the Sweep tracking bug is fixed, re-run with per-body CCD ON
    /// (other deferred defaults OFF) to isolate the per-body CCD failure.
    ///
    /// For each failing scene:
    ///   - Sample-by-sample finite check at flag-on N=1 + per-body CCD
    ///   - Identify which bodies tunnel and at what step
    ///   - Determine root cause: missed broadphase / wrong Sweep / TOI false
    ///     negative / AdvanceBodyToTOI race
    /// </summary>
    [TestClass]
    public class PerBodyCCDDiagnosticProbe
    {
        private const int TotalSteps = 300;
        private const float Dt = 1f / 60f;
        private const float FallThroughY = -2f;

        private struct Snap
        {
            public int Step;
            public int FellThrough;
            public float MaxV;
            public int MaxVBodyId;
            public float MinY;
            public int MinYBodyId;
        }

        private static readonly Dictionary<string, Func<ISample>> Suspects = new()
        {
            { "CircleStress",   () => new CircleStressSample() },
            { "Cantilever",     () => new CantileverSample() },
            { "FrictionJoint",  () => new FrictionJointSample() },
            { "SliderCrank",    () => new SliderCrankSample() },
            { "Pyramid",        () => new PyramidSample() },
            { "Dominos",        () => new DominosSample() },
        };

        [TestMethod, Timeout(180000)]
        public void PerBodyCCD_On_vs_Off_SusceptSamples()
        {
            var sb = new StringBuilder();
            sb.AppendLine();
            sb.AppendLine("Per-body CCD ON vs OFF (delta-tracking on, bias-only off, N=1)");
            sb.AppendLine("================================================================");
            sb.AppendLine($"{"Sample",-15} {"off MaxV",10} {"off MinY",10} {"off FT",6} | {"on MaxV",10} {"on MinY",10} {"on FT",6} {"Δ FT",6}");
            foreach (var (name, factory) in Suspects)
            {
                Snap off = Run(factory(), perBodyCCD: false);
                Snap on  = Run(factory(), perBodyCCD: true);
                sb.AppendLine($"{name,-15} {off.MaxV,10:F2} {off.MinY,10:F2} {off.FellThrough,6} | {on.MaxV,10:F2} {on.MinY,10:F2} {on.FellThrough,6} {(on.FellThrough - off.FellThrough),6}");
            }
            Console.Error.WriteLine(sb.ToString());
        }

        [TestMethod, Timeout(60000)]
        public void CircleStress_PerBodyCCD_DivergenceTrace()
        {
            // Per-step tracking — when does CircleStress start falling through?
            ISample sample = new CircleStressSample();
            WorldDef def = sample.CreateWorldDef()
                .UsePerBodyContinuous();
            var world = new World(def);
            sample.Build(world);

            var sb = new StringBuilder();
            sb.AppendLine();
            sb.AppendLine("CircleStress + per-body CCD divergence trace");
            sb.AppendLine("=============================================");
            sb.AppendLine($"{"step",5} {"maxV",8} {"minY",8} {"FT",4} {"awake",6}");

            int firstFT = -1;
            for (int i = 0; i < TotalSteps; ++i)
            {
                sample.Step(world, Dt);
                world.Step(Dt);

                float maxV = 0f, minY = float.MaxValue;
                int ft = 0, awake = 0;
                for (int b = 0; b < world.Bodies.Count; ++b)
                {
                    Body body = world.Bodies[b];
                    if (body.Type != BodyType.Dynamic) continue;
                    if (body.Awake) awake++;
                    Vec2 v = body.LinearVelocity;
                    Vec2 p = body.Transform.P;
                    if (float.IsNaN(v.X) || float.IsInfinity(v.X)) continue;
                    float speed = v.Length;
                    if (speed > maxV) maxV = speed;
                    if (p.Y < minY) minY = p.Y;
                    if (p.Y < FallThroughY) ft++;
                }
                if (ft > 0 && firstFT < 0) firstFT = i;

                if (i % 30 == 0 || (firstFT >= 0 && i - firstFT < 8))
                {
                    sb.AppendLine($"{i,5} {maxV,8:F2} {minY,8:F2} {ft,4} {awake,6}");
                }
            }
            sb.AppendLine();
            sb.AppendLine($"First fall-through at step: {firstFT}");
            Console.Error.WriteLine(sb.ToString());
        }

        private static Snap Run(ISample sample, bool perBodyCCD)
        {
            WorldDef def = sample.CreateWorldDef();
            if (perBodyCCD) def = def.UsePerBodyContinuous();
            var world = new World(def);
            sample.Build(world);

            var s = new Snap { MinY = float.MaxValue };
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
                    if (float.IsNaN(v.X) || float.IsInfinity(v.X)) continue;
                    float speed = v.Length;
                    if (speed > s.MaxV) { s.MaxV = speed; s.MaxVBodyId = body.Id; }
                    if (p.Y < s.MinY) { s.MinY = p.Y; s.MinYBodyId = body.Id; }
                    if (p.Y < FallThroughY) s.FellThrough++;
                }
            }
            s.Step = TotalSteps;
            return s;
        }
    }
}
