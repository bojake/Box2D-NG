using Box2DNG.Viewer.Samples;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Box2DNG.Tests
{
    /// <summary>
    /// Phase 0 baseline for samples whose expected steady state is "moving"
    /// (motors, restitution-driven, kinematic-driven). Settling assertions
    /// don't apply; we instead pin finite-state and a bounded peak speed.
    ///
    /// These thresholds will move under the Phase 1-3 refactors. Cpp v3's
    /// soft joints + sub-stepping should reduce the spikes that motorized
    /// scenes exhibit during initial transients.
    /// </summary>
    [TestClass]
    public class SampleActiveTests
    {
        private const int Steps = 600;

        [TestMethod]
        public void ApplyForce_StaysFinite()
        {
            var m = new SampleMetrics(new ApplyForceSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.PeakLinearSpeed < 100f, $"ApplyForce peak. {m}");
        }

        [TestMethod]
        public void BodyTypes_StaysFinite()
        {
            // BodyTypes cycles between static / kinematic / dynamic via OnKey.
            // The non-keyed scene should still stay bounded over 10s.
            var m = new SampleMetrics(new BodyTypesSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.PeakLinearSpeed < 60f, $"BodyTypes peak. {m}");
        }

        [TestMethod]
        public void Car_StaysFinite()
        {
            // Car is wheel-joint driven with a motor. Existing CarSampleTests
            // cover the car's motion; this just adds the universal invariant.
            var m = new SampleMetrics(new CarSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.PeakLinearSpeed < 60f, $"Car peak. {m}");
        }

        [TestMethod]
        public void Tumbler_StaysFinite()
        {
            // KNOWN ISSUE: 32 bodies fall through (y≈-563). The kinematic-
            // rotated container leaks debris through its walls — the
            // per-contact CCD doesn't detect the kinematic-vs-dynamic
            // sweep correctly. Phase 3 (per-body CCD with proper bullet/
            // kinematic handling) target.
            var m = new SampleMetrics(new TumblerSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.PeakLinearSpeed < 130f, $"Tumbler peak. {m}");
        }

        [TestMethod]
        public void TheoJansen_StaysFinite()
        {
            // KNOWN ISSUE: 1 body fell through (y≈-766). Theo Jansen has many
            // welded segments — Phase 1 (soft welds) target.
            var m = new SampleMetrics(new TheoJansenSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.PeakLinearSpeed < 130f, $"TheoJansen peak. {m}");
        }

        [TestMethod]
        public void SliderCrank_StaysFinite()
        {
            // Slider-crank covered by SliderCrankSampleTests; this adds the
            // universal invariant.
            var m = new SampleMetrics(new SliderCrankSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.PeakLinearSpeed < 60f, $"SliderCrank peak. {m}");
        }

        [TestMethod]
        public void MotorJoint_StaysFinite()
        {
            // KNOWN ISSUE: 1 body fell through (y≈-473). Sample-specific
            // geometry/joint interaction. Phase 1+3 target.
            var m = new SampleMetrics(new MotorJointSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.PeakLinearSpeed < 120f, $"MotorJoint peak. {m}");
        }

        [TestMethod]
        public void Revolute_StaysFinite()
        {
            var m = new SampleMetrics(new RevoluteSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.PeakLinearSpeed < 60f, $"Revolute peak. {m}");
        }

        [TestMethod]
        public void Prismatic_StaysFinite()
        {
            var m = new SampleMetrics(new PrismaticSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.PeakLinearSpeed < 60f, $"Prismatic peak. {m}");
        }

        [TestMethod]
        public void MultiplePrismatic_StaysFinite()
        {
            var m = new SampleMetrics(new MultiplePrismaticSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.PeakLinearSpeed < 60f, $"MultiplePrismatic peak. {m}");
        }

        [TestMethod]
        public void UnstablePrismaticJoints_StaysFinite()
        {
            // Sample name implies instability — capture the bounded peak.
            var m = new SampleMetrics(new UnstablePrismaticJointsSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.PeakLinearSpeed < 200f, $"UnstablePrismaticJoints peak. {m}");
        }

        [TestMethod]
        public void WeldJoint_StaysFinite()
        {
            // Direct welded scene — central to Phase 1.
            var m = new SampleMetrics(new WeldJointSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.PeakLinearSpeed < 100f, $"WeldJoint peak. {m}");
        }

        [TestMethod]
        public void DistanceJoint_StaysFinite()
        {
            var m = new SampleMetrics(new DistanceJointSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.PeakLinearSpeed < 60f, $"DistanceJoint peak. {m}");
        }

        [TestMethod]
        public void FrictionJoint_StaysFinite()
        {
            var m = new SampleMetrics(new FrictionJointSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.PeakLinearSpeed < 60f, $"FrictionJoint peak. {m}");
        }

        [TestMethod]
        public void Pulleys_StaysFinite()
        {
            var m = new SampleMetrics(new PulleysSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.PeakLinearSpeed < 60f, $"Pulleys peak. {m}");
        }

        [TestMethod]
        public void Rope_StaysFinite()
        {
            // Rope sample uses the rope physics primitive (separate from joint).
            var m = new SampleMetrics(new RopeSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.PeakLinearSpeed < 60f, $"Rope peak. {m}");
        }

        [TestMethod]
        public void RopeJoint_StaysFinite()
        {
            var m = new SampleMetrics(new RopeJointSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.PeakLinearSpeed < 60f, $"RopeJoint peak. {m}");
        }

        [TestMethod]
        public void AddPair_StaysFinite()
        {
            // AddPair is the bullet-crowd scene. The bullet hits a crowd of
            // small bodies; speeds will be high but bounded.
            var m = new SampleMetrics(new AddPairSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.PeakLinearSpeed < 200f, $"AddPair peak. {m}");
        }

        [TestMethod]
        public void BulletTest_StaysFinite()
        {
            var m = new SampleMetrics(new BulletTestSample());
            m.Run(Steps);
            Assert.IsTrue(m.NonFiniteBodyCount == 0, $"Non-finite states. {m}");
            Assert.IsTrue(m.PeakLinearSpeed < 200f, $"BulletTest peak. {m}");
        }
    }
}
