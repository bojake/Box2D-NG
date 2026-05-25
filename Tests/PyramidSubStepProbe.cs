using System;
using System.Text;
using Box2DNG.Viewer.Samples;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Box2DNG.Tests
{
    /// <summary>
    /// Phase 2.5 task #81 — cause #3: characterize how Pyramid's late-window
    /// peak speed and fall-through count scale with SubStepCount under
    /// flag-on (Pyramid is the headline scene that regresses at N>1).
    ///
    /// Hypothesis under investigation: the contact softness bias is computed
    /// in `PrepareConstraints(h=outerDt/N)` using the SUB-STEP timestep, but
    /// then applied N times in the velocity-solve loop. The biasRate scales
    /// roughly as `omega / (2·ratio + h·omega)` — smaller h gives BIGGER
    /// biasRate. So at N=4 the per-sub-step bias is ~2.8× larger than at
    /// N=1, and the per-outer-step total is ~11× larger. This over-correction
    /// pumps energy that breaks the stack.
    ///
    /// Probe scans N ∈ {1, 2, 3, 4, 6, 8} for three configurations and
    /// reports lateV + FT. If the hypothesis is correct, we expect monotonic
    /// degradation with N. If not, the degradation should look chaotic /
    /// resonant (like VaryingFriction's tuning sensitivity).
    /// </summary>
    [TestClass]
    public class PyramidSubStepProbe
    {
        private const int TotalSteps = 600;
        private const int LateWindowSteps = 60;

        [TestMethod, Timeout(300000)]
        public void Pyramid_SubStepCount_Scaling()
        {
            int[] Ns = { 1, 2, 3, 4, 6, 8 };
            var sb = new StringBuilder();
            sb.AppendLine();
            sb.AppendLine("Pyramid lateV + FT scaling vs SubStepCount");
            sb.AppendLine("============================================");
            sb.AppendLine($"{"N",3} | {"off lateV",10} {"off FT",6} | {"on+NGS lateV",13} {"on+NGS FT",10} | {"on+bias lateV",14} {"on+bias FT",11}");
            foreach (int n in Ns)
            {
                Result off = Run(useDelta: false, biasOnly: false, n: n);
                Result on  = Run(useDelta: true,  biasOnly: false, n: n);
                Result onB = Run(useDelta: true,  biasOnly: true,  n: n);
                sb.AppendLine($"{n,3} | {off.LateV,10:F2} {off.FellThrough,6} | {on.LateV,13:F2} {on.FellThrough,10} | {onB.LateV,14:F2} {onB.FellThrough,11}");
            }
            Console.Error.WriteLine(sb.ToString());
        }

        /// <summary>
        /// Same probe but tuned to cpp v3 defaults (Hz=30, ratio=10). If the
        /// over-correction is the issue, cpp v3's much-larger ratio should
        /// reduce per-sub-step bias proportionally and rescue the stack.
        /// </summary>
        [TestMethod, Timeout(300000)]
        public void Pyramid_SubStepCount_Scaling_CppDefaults()
        {
            int[] Ns = { 1, 2, 3, 4, 6, 8 };
            var sb = new StringBuilder();
            sb.AppendLine();
            sb.AppendLine("Pyramid scaling at cpp v3 defaults (Hz=30, ratio=10)");
            sb.AppendLine("=====================================================");
            sb.AppendLine($"{"N",3} | {"off lateV",10} {"off FT",6} | {"on+NGS lateV",13} {"on+NGS FT",10} | {"on+bias lateV",14} {"on+bias FT",11}");
            foreach (int n in Ns)
            {
                Result off = Run(useDelta: false, biasOnly: false, n: n, hertz: 30f, damping: 10f);
                Result on  = Run(useDelta: true,  biasOnly: false, n: n, hertz: 30f, damping: 10f);
                Result onB = Run(useDelta: true,  biasOnly: true,  n: n, hertz: 30f, damping: 10f);
                sb.AppendLine($"{n,3} | {off.LateV,10:F2} {off.FellThrough,6} | {on.LateV,13:F2} {on.FellThrough,10} | {onB.LateV,14:F2} {onB.FellThrough,11}");
            }
            Console.Error.WriteLine(sb.ToString());
        }

        private struct Result
        {
            public float LateV;
            public float PeakV;
            public int FellThrough;
            public int NonFinite;
        }

        private static Result Run(bool useDelta, bool biasOnly, int n, float hertz = 120f, float damping = 1f)
        {
            ISample sample = new PyramidSample();
            WorldDef def = sample.CreateWorldDef()
                .WithSubStepCount(n)
                .WithContactHertz(hertz)
                .WithContactDamping(damping);
            if (useDelta) def = def.UseDeltaPositions();
            if (biasOnly) def = def.WithBiasOnlyContacts();
            var world = new World(def);
            sample.Build(world);

            var r = new Result();
            int lateStart = TotalSteps - LateWindowSteps;
            var fellThroughIds = new System.Collections.Generic.HashSet<int>();
            for (int i = 0; i < TotalSteps; ++i)
            {
                sample.Step(world, 1f / 60f);
                world.Step(1f / 60f);
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
