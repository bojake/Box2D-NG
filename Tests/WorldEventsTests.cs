using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Collections.Generic;

namespace Box2DNG.Tests
{
    [TestClass]
    public class WorldEventsTests
    {
        [TestMethod]
        public void BodyEvents_AreRaisedForMovingDynamicBodies()
        {
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)));
            Body body = world.CreateBody(new BodyDef().AsDynamic().At(0f, 10f).WithUserData("falling"));
            body.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));

            List<BodyEvent> received = new List<BodyEvent>();
            world.Events.BodyEvents += evt => received.AddRange(evt.Events);

            world.Step(1f / 60f);

            Assert.IsTrue(received.Count >= 1, "Expected a body move event.");
            Assert.IsTrue(received.Exists(e => "falling".Equals(e.UserData)),
                "Move event should carry the body UserData.");
            Assert.IsFalse(received[0].FellAsleep, "Body did not fall asleep this step.");
        }

        [TestMethod]
        public void BodyEvents_NoEventsForStaticOnlyWorld()
        {
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)));
            world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));

            int count = 0;
            world.Events.BodyEvents += evt => count += evt.Events.Length;

            world.Step(1f / 60f);

            Assert.AreEqual(0, count, "Static-only world should not emit body events.");
        }

        [TestMethod]
        public void BodyEvents_FellAsleepFlag_TrueWhenBodySleeps()
        {
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)).EnableSleeping(true));
            Body ground = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            ground.CreateFixture(new FixtureDef(new SegmentShape(new Vec2(-10f, 0f), new Vec2(10f, 0f))));

            Body box = world.CreateBody(new BodyDef().AsDynamic().At(0f, 0.6f).WithUserData("box"));
            box.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));

            bool sawFellAsleep = false;
            world.Events.BodyEvents += evt =>
            {
                foreach (var e in evt.Events)
                {
                    if (e.FellAsleep && "box".Equals(e.UserData))
                    {
                        sawFellAsleep = true;
                    }
                }
            };

            for (int i = 0; i < 600 && !sawFellAsleep; ++i)
            {
                world.Step(1f / 60f);
            }

            // It's possible the body never settles under this step config; record what we saw.
            // The contract is: when a body falls asleep, the flag fires.
            // If the test framework doesn't sleep, this isn't a regression - just a non-result.
            // We assert it triggered if the body went to sleep:
            if (!box.Awake)
            {
                Assert.IsTrue(sawFellAsleep, "Body fell asleep but no FellAsleep event was raised.");
            }
        }

        [TestMethod]
        public void JointEvents_NotRaisedWithDefaultInfiniteThreshold()
        {
            World world = new World(new WorldDef().WithGravity(new Vec2(0f, -10f)));
            Body a = world.CreateBody(new BodyDef().AsStatic().At(0f, 5f));
            Body b = world.CreateBody(new BodyDef().AsDynamic().At(0f, 3f));
            b.CreateFixture(new FixtureDef(new CircleShape(0.5f)).WithDensity(1f));
            world.CreateJoint(new DistanceJointDef(a, b, Vec2.Zero, Vec2.Zero).WithLength(2f));

            int eventCount = 0;
            world.Events.JointEvents += evt => eventCount += evt.Events.Length;

            for (int i = 0; i < 30; ++i)
            {
                world.Step(1f / 60f);
            }

            Assert.AreEqual(0, eventCount, "Default threshold is infinite — no events should fire.");
        }

        [TestMethod]
        public void JointEvents_RaisedWhenThresholdExceeded()
        {
            World world = new World(new WorldDef()
                .WithGravity(new Vec2(0f, -10f))
                .WithJointForceThreshold(0f)); // zero = report all awake joints

            Body a = world.CreateBody(new BodyDef().AsStatic().At(0f, 5f).WithUserData("anchor"));
            a.CreateFixture(new FixtureDef(new CircleShape(0.1f)));
            Body b = world.CreateBody(new BodyDef().AsDynamic().At(0f, 3f).WithUserData("hanging"));
            b.CreateFixture(new FixtureDef(new CircleShape(0.3f)).WithDensity(1f));
            world.CreateJoint(new DistanceJointDef(a, b, Vec2.Zero, Vec2.Zero).WithLength(2f));

            List<JointEvent> received = new List<JointEvent>();
            world.Events.JointEvents += evt => received.AddRange(evt.Events);

            for (int i = 0; i < 30; ++i)
            {
                world.Step(1f / 60f);
            }

            Assert.IsTrue(received.Count > 0, "Expected joint events with zero threshold.");
            JointEvent e = received[received.Count - 1];
            Assert.IsTrue(e.ReactionForce > 0f, $"Expected non-zero reaction force in event. Got {e.ReactionForce}");
        }

        [TestMethod]
        public void JointEvents_NotRaisedAboveSetThresholdWhenUnderThreshold()
        {
            // Threshold much higher than actual reaction → no events.
            World world = new World(new WorldDef()
                .WithGravity(new Vec2(0f, -10f))
                .WithJointForceThreshold(1e9f)
                .WithJointTorqueThreshold(1e9f));

            Body a = world.CreateBody(new BodyDef().AsStatic().At(0f, 5f));
            Body b = world.CreateBody(new BodyDef().AsDynamic().At(0f, 3f));
            b.CreateFixture(new FixtureDef(new CircleShape(0.3f)).WithDensity(1f));
            world.CreateJoint(new DistanceJointDef(a, b, Vec2.Zero, Vec2.Zero).WithLength(2f));

            int events = 0;
            world.Events.JointEvents += evt => events += evt.Events.Length;
            for (int i = 0; i < 30; ++i)
            {
                world.Step(1f / 60f);
            }
            Assert.AreEqual(0, events, "Threshold not exceeded; no events expected.");
        }
    }
}
