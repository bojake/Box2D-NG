using System;
using System.Collections.Generic;
using System.Linq;

namespace Box2DNG.Viewer.Samples
{
    public static class SampleCatalog
    {
        /// <summary>
        /// Factory delegates — each call returns a FRESH <see cref="ISample"/>
        /// instance. Use this for any code that calls <see cref="ISample.Build"/>
        /// more than once per sample (probes, sweeps, grid tests), so the
        /// sample's instance state (e.g. <c>CircleStressSample</c>'s
        /// <c>Random(1234)</c>, <c>BreakableSample</c>'s break-event flags)
        /// doesn't leak between runs. See task #83.
        /// </summary>
        public static IReadOnlyList<Func<ISample>> Factories { get; } = new Func<ISample>[]
        {
            () => new AddPairSample(),
            () => new ApplyForceSample(),
            () => new BodyTypesSample(),
            () => new BreakableSample(),
            () => new BridgeSample(),
            () => new BulletTestSample(),
            () => new CantileverSample(),
            () => new CarSample(),
            () => new ChainSample(),
            () => new CharacterCollisionSample(),
            () => new CircleStressSample(),
            () => new CollisionFilteringSample(),
            () => new CompoundShapesSample(),
            () => new ConfinedSample(),
            () => new PyramidSample(),
            () => new DominosSample(),
            () => new EdgeShapesSample(),
            () => new PrismaticSample(),
            () => new UnstablePrismaticJointsSample(),
            () => new MultiplePrismaticSample(),
            () => new PulleysSample(),
            () => new TumblerSample(),
            () => new RevoluteSample(),
            () => new GearsSample(),
            () => new RopeSample(),
            () => new RopeJointSample(),
            () => new DistanceJointSample(),
            () => new WeldJointSample(),
            () => new FrictionJointSample(),
            () => new MotorJointSample(),
            () => new SliderCrankSample(),
            () => new TheoJansenSample(),
            () => new PinballSample(),
            () => new VaryingFrictionSample(),
            () => new VaryingRestitutionSample()
        };

        /// <summary>
        /// Singleton list of every sample, materialized once at static init.
        /// Use this for the viewer (which wants stable instances so it can
        /// preserve per-sample UI state across navigation) and for tests
        /// that only call <see cref="ISample.Build"/> exactly once per
        /// sample. For multi-run probes, prefer <see cref="Factories"/>.
        /// </summary>
        public static IReadOnlyList<ISample> All { get; } = Factories.Select(f => f()).ToArray();

        /// <summary>
        /// Convenience: materialize a fresh list of all samples. Equivalent
        /// to <c>Factories.Select(f =&gt; f()).ToArray()</c>. Use when you need
        /// a freshly-instantiated catalog at the call site.
        /// </summary>
        public static ISample[] CreateAll() => Factories.Select(f => f()).ToArray();
    }
}
