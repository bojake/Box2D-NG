using System;
using System.Collections.Generic;
using Box2DNG.Viewer.Samples;

namespace Box2DNG.Tests
{
    /// <summary>
    /// Helper for the per-sample regression tests that pin viewer-scene quality.
    /// Drives an <see cref="ISample"/> through its own Step/Build API (same path the
    /// viewer uses) and records metrics that the Tier-4 parity phases will move:
    /// peak speeds, fall-through counts, settling time, NaN/Inf detection.
    /// </summary>
    public sealed class SampleMetrics
    {
        public ISample Sample { get; }
        public World World { get; }

        public float PeakLinearSpeed { get; private set; }
        public float PeakAngularSpeed { get; private set; }
        public int PeakLinearBodyId { get; private set; } = -1;
        public int PeakAngularBodyId { get; private set; } = -1;
        public float MinY { get; private set; } = float.MaxValue;
        public float MaxY { get; private set; } = float.MinValue;
        public int StepsRun { get; private set; }
        public int NonFiniteBodyCount { get; private set; }

        /// <summary>Late-window peak: max speed observed in the final <see cref="LateWindowSteps"/> steps of the run.</summary>
        public float LateWindowPeakSpeed { get; private set; }
        public int LateWindowSteps { get; }

        private readonly float _dt;
        private readonly float _fallThroughY;
        private readonly HashSet<int> _fellThroughBodyIds = new();

        public IReadOnlyCollection<int> FellThroughBodyIds => _fellThroughBodyIds;
        public int FellThroughCount => _fellThroughBodyIds.Count;

        public SampleMetrics(
            ISample sample,
            float dt = 1f / 60f,
            int lateWindowSteps = 60,
            float fallThroughY = -2f)
        {
            Sample = sample;
            World = new World(sample.CreateWorldDef());
            sample.Build(World);
            _dt = dt;
            LateWindowSteps = lateWindowSteps;
            _fallThroughY = fallThroughY;
        }

        /// <summary>
        /// Run the sample for <paramref name="stepCount"/> outer steps, honouring its
        /// declared SubSteps just like the viewer's tick. Records metrics throughout.
        /// </summary>
        public void Run(int stepCount)
        {
            int subSteps = Math.Max(1, Sample.SubSteps);
            float subDt = _dt / subSteps;

            for (int i = 0; i < stepCount; ++i)
            {
                for (int s = 0; s < subSteps; ++s)
                {
                    Sample.Step(World, subDt);
                    World.Step(subDt);
                }
                SampleStep(i, stepCount);
            }
        }

        private void SampleStep(int stepIndex, int totalSteps)
        {
            float frameMaxLinear = 0f;
            float frameMaxAngular = 0f;
            int frameLinearBody = -1;
            int frameAngularBody = -1;

            for (int b = 0; b < World.Bodies.Count; ++b)
            {
                Body body = World.Bodies[b];
                Vec2 p = body.Transform.P;
                if (!IsFinite(p) || !IsFinite(body.LinearVelocity) || !IsFinite(body.AngularVelocity))
                {
                    NonFiniteBodyCount++;
                    continue;
                }

                if (body.Type == BodyType.Dynamic)
                {
                    float v = body.LinearVelocity.Length;
                    if (v > frameMaxLinear) { frameMaxLinear = v; frameLinearBody = b; }
                    float w = MathF.Abs(body.AngularVelocity);
                    if (w > frameMaxAngular) { frameMaxAngular = w; frameAngularBody = b; }
                }

                if (p.Y < MinY) MinY = p.Y;
                if (p.Y > MaxY) MaxY = p.Y;

                if (body.Type == BodyType.Dynamic && p.Y < _fallThroughY)
                {
                    _fellThroughBodyIds.Add(body.Id);
                }
            }

            if (frameMaxLinear > PeakLinearSpeed)
            {
                PeakLinearSpeed = frameMaxLinear;
                PeakLinearBodyId = frameLinearBody;
            }
            if (frameMaxAngular > PeakAngularSpeed)
            {
                PeakAngularSpeed = frameMaxAngular;
                PeakAngularBodyId = frameAngularBody;
            }

            if (stepIndex >= totalSteps - LateWindowSteps)
            {
                if (frameMaxLinear > LateWindowPeakSpeed)
                {
                    LateWindowPeakSpeed = frameMaxLinear;
                }
            }

            StepsRun = stepIndex + 1;
        }

        /// <summary>
        /// Steps the world until either the max dynamic body speed drops below
        /// <paramref name="speedThreshold"/> for <paramref name="settleHoldSteps"/>
        /// consecutive steps, or <paramref name="maxSteps"/> is reached.
        /// Returns the step index at which settling was confirmed, or -1 if it
        /// never settled within <paramref name="maxSteps"/>.
        /// </summary>
        public int RunUntilSettled(float speedThreshold, int settleHoldSteps = 30, int maxSteps = 1200)
        {
            int subSteps = Math.Max(1, Sample.SubSteps);
            float subDt = _dt / subSteps;
            int consecutiveBelow = 0;

            for (int i = 0; i < maxSteps; ++i)
            {
                for (int s = 0; s < subSteps; ++s)
                {
                    Sample.Step(World, subDt);
                    World.Step(subDt);
                }
                SampleStep(i, maxSteps);

                float currentMax = 0f;
                for (int b = 0; b < World.Bodies.Count; ++b)
                {
                    Body body = World.Bodies[b];
                    if (body.Type != BodyType.Dynamic) continue;
                    float v = body.LinearVelocity.Length;
                    if (v > currentMax) currentMax = v;
                }

                if (currentMax < speedThreshold)
                {
                    consecutiveBelow++;
                    if (consecutiveBelow >= settleHoldSteps)
                    {
                        return i + 1;
                    }
                }
                else
                {
                    consecutiveBelow = 0;
                }
            }

            return -1;
        }

        public override string ToString()
        {
            return $"Sample={Sample.Name} steps={StepsRun} peakV={PeakLinearSpeed:F3}(body {PeakLinearBodyId}) peakW={PeakAngularSpeed:F3}(body {PeakAngularBodyId}) " +
                   $"y=[{MinY:F2},{MaxY:F2}] lateV={LateWindowPeakSpeed:F3} fellThrough={FellThroughCount} nonFinite={NonFiniteBodyCount}";
        }

        private static bool IsFinite(Vec2 v) => !(float.IsNaN(v.X) || float.IsInfinity(v.X) || float.IsNaN(v.Y) || float.IsInfinity(v.Y));
        private static bool IsFinite(float v) => !(float.IsNaN(v) || float.IsInfinity(v));
    }
}
