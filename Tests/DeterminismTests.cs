using System;
using System.Collections.Generic;
using System.Text;
using Box2DNG.Viewer.Samples;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Box2DNG.Tests
{
    /// <summary>
    /// Phase 0 baseline: identical sample + identical step count must produce
    /// identical body positions across two runs. cpp box2d v3 emphasizes
    /// run-to-run determinism; we should preserve it through the Phase 1-3
    /// refactors (soft joints, sub-stepping, per-body CCD). If a future change
    /// accidentally introduces a HashSet/Dictionary iteration order dependency
    /// or a closure-allocation race, these tests catch it.
    ///
    /// We hash the (position, rotation) tuple of every body after N steps,
    /// then compare bytes. We don't pin specific *values* (those change with
    /// Phase 1-3 refactors); only that two runs produce the *same* values.
    /// </summary>
    [TestClass]
    public class DeterminismTests
    {
        private const int Steps = 120;

        [TestMethod]
        public void Cantilever_IsDeterministic()
        {
            AssertDeterministic(() => new CantileverSample());
        }

        [TestMethod]
        public void Pyramid_IsDeterministic()
        {
            AssertDeterministic(() => new PyramidSample());
        }

        [TestMethod]
        public void Breakable_IsDeterministic()
        {
            AssertDeterministic(() => new BreakableSample());
        }

        [TestMethod]
        public void Pinball_IsDeterministic()
        {
            AssertDeterministic(() => new PinballSample());
        }

        [TestMethod]
        public void AddPair_IsDeterministic()
        {
            AssertDeterministic(() => new AddPairSample());
        }

        [TestMethod]
        public void Bridge_IsDeterministic()
        {
            AssertDeterministic(() => new BridgeSample());
        }

        [TestMethod]
        public void Car_IsDeterministic()
        {
            AssertDeterministic(() => new CarSample());
        }

        [TestMethod]
        public void Tumbler_IsDeterministic()
        {
            AssertDeterministic(() => new TumblerSample());
        }

        [TestMethod]
        public void TheoJansen_IsDeterministic()
        {
            AssertDeterministic(() => new TheoJansenSample());
        }

        [TestMethod]
        public void WeldJoint_IsDeterministic()
        {
            AssertDeterministic(() => new WeldJointSample());
        }

        private static void AssertDeterministic(Func<ISample> factory)
        {
            string sigA = Snapshot(factory(), Steps);
            string sigB = Snapshot(factory(), Steps);
            Assert.AreEqual(sigA, sigB,
                $"Two runs of {factory().Name} after {Steps} steps produced different body states (non-deterministic).");
        }

        /// <summary>
        /// Run the sample for N steps then produce a stable signature of every
        /// body's position and rotation. The signature is the body-id-ordered
        /// concatenation of float bytes — exact equality matters, not numeric
        /// closeness (we want byte-for-byte determinism).
        /// </summary>
        private static string Snapshot(ISample sample, int steps)
        {
            World world = new World(sample.CreateWorldDef());
            sample.Build(world);
            int subSteps = Math.Max(1, sample.SubSteps);
            float subDt = (1f / 60f) / subSteps;
            for (int i = 0; i < steps; ++i)
            {
                for (int s = 0; s < subSteps; ++s)
                {
                    sample.Step(world, subDt);
                    world.Step(subDt);
                }
            }

            // Sort by body id so insertion-order differences don't leak in.
            var bodies = new List<Body>(world.Bodies);
            bodies.Sort((a, b) => a.Id.CompareTo(b.Id));

            StringBuilder sb = new StringBuilder(bodies.Count * 40);
            foreach (Body body in bodies)
            {
                sb.Append(body.Id).Append(':');
                sb.Append(BitConverter.ToUInt32(BitConverter.GetBytes(body.Transform.P.X)).ToString("X8")).Append(',');
                sb.Append(BitConverter.ToUInt32(BitConverter.GetBytes(body.Transform.P.Y)).ToString("X8")).Append(',');
                sb.Append(BitConverter.ToUInt32(BitConverter.GetBytes(body.Transform.Q.Angle)).ToString("X8")).Append(';');
            }
            return sb.ToString();
        }
    }
}
