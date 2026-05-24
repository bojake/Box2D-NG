using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Collections.Generic;

namespace Box2DNG.Tests
{
    [TestClass]
    public class QueryFilterTests
    {
        private static World BuildWorldWithCategories()
        {
            World world = new World(new WorldDef());
            Body redBody = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            redBody.CreateFixture(new FixtureDef(new CircleShape(1f))
                .WithFilter(new Filter(categoryBits: 0x0001, maskBits: ulong.MaxValue, groupIndex: 0))
                .WithUserData("red"));

            Body blueBody = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            blueBody.CreateFixture(new FixtureDef(new CircleShape(1f))
                .WithFilter(new Filter(categoryBits: 0x0002, maskBits: ulong.MaxValue, groupIndex: 0))
                .WithUserData("blue"));
            return world;
        }

        [TestMethod]
        public void QueryAabb_DefaultFilter_MatchesAll()
        {
            World world = BuildWorldWithCategories();
            Aabb area = new Aabb(new Vec2(-2f, -2f), new Vec2(2f, 2f));
            List<Fixture> results = world.QueryAabb(area);
            Assert.AreEqual(2, results.Count);
        }

        [TestMethod]
        public void QueryAabb_FilterByCategory_NarrowsResults()
        {
            World world = BuildWorldWithCategories();
            Aabb area = new Aabb(new Vec2(-2f, -2f), new Vec2(2f, 2f));
            QueryFilter redOnly = new QueryFilter(categoryBits: 0x0001, maskBits: 0x0001);
            List<Fixture> results = world.QueryAabb(area, redOnly);
            Assert.AreEqual(1, results.Count);
            Assert.AreEqual("red", results[0].UserData);
        }

        [TestMethod]
        public void RayCast_FilterExcludesByMask()
        {
            World world = new World(new WorldDef());
            Body red = world.CreateBody(new BodyDef().AsStatic().At(0f, 0f));
            red.CreateFixture(new FixtureDef(new PolygonShape(new[]
                {
                    new Vec2(-0.5f, -0.5f), new Vec2(0.5f, -0.5f),
                    new Vec2(0.5f, 0.5f), new Vec2(-0.5f, 0.5f)
                }))
                .WithFilter(new Filter(0x0001, ulong.MaxValue, 0))
                .WithUserData("red"));

            RayCastInput input = new RayCastInput(new Vec2(-3f, 0f), new Vec2(6f, 0f), 1f);
            // Filter mask doesn't include 0x0001 → no hit.
            QueryFilter noRed = new QueryFilter(categoryBits: 0x0002, maskBits: 0x0002);
            Assert.IsFalse(world.RayCast(input, out _, noRed));
            // Default filter → hit.
            Assert.IsTrue(world.RayCast(input, out World.RayCastHit hit));
            Assert.AreEqual("red", hit.Fixture.UserData);
        }
    }
}
