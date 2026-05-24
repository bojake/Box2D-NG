using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Collections.Generic;

namespace Box2DNG.Tests
{
    [TestClass]
    public class CoreAndAllocatorTests
    {
        // ----- Box2DVersion -----

        [TestMethod]
        public void Box2DVersion_CarriesMajorMinorRevision()
        {
            Box2DVersion v = new Box2DVersion(3, 2, 1);
            Assert.AreEqual(3, v.Major);
            Assert.AreEqual(2, v.Minor);
            Assert.AreEqual(1, v.Revision);
            Assert.AreEqual("3.2.1", v.ToString());
        }

        [TestMethod]
        public void Box2D_GetVersion_ReturnsCurrentVersion()
        {
            Box2DVersion v = Box2D.GetVersion();
            Assert.IsTrue(v.Major >= 3, $"Expected major >= 3, got {v.Major}");
        }

        // ----- Box2D facade: world create/destroy/lookup -----

        [TestMethod]
        public void Box2D_CreateAndDestroyWorld()
        {
            WorldId id = Box2D.CreateWorld(new WorldDef().WithGravity(Vec2.Zero));
            try
            {
                Assert.IsTrue(Box2D.IsValid(id));
                Assert.IsTrue(Box2D.TryGetWorld(id, out World? world));
                Assert.IsNotNull(world);
                Assert.AreSame(world, Box2D.GetWorld(id));
            }
            finally
            {
                Assert.IsTrue(Box2D.DestroyWorld(id));
            }
            Assert.IsFalse(Box2D.IsValid(id));
        }

        [TestMethod]
        public void Box2D_GetWorld_ThrowsOnInvalidId()
        {
            WorldId bogus = default;
            Assert.ThrowsException<ArgumentException>(() => Box2D.GetWorld(bogus));
        }

        [TestMethod]
        public void Box2D_DestroyTwice_SecondCallReturnsFalse()
        {
            WorldId id = Box2D.CreateWorld(new WorldDef());
            Assert.IsTrue(Box2D.DestroyWorld(id));
            Assert.IsFalse(Box2D.DestroyWorld(id));
        }

        // ----- Box2D static settings -----

        [TestMethod]
        public void Box2D_SetLengthUnitsPerMeter_AndRead()
        {
            float old = Box2D.LengthUnitsPerMeter;
            try
            {
                Box2D.SetLengthUnitsPerMeter(2.5f);
                Assert.AreEqual(2.5f, Box2D.LengthUnitsPerMeter);
            }
            finally
            {
                Box2D.SetLengthUnitsPerMeter(old);
            }
        }

        [TestMethod]
        public void Box2D_SetLengthUnitsPerMeter_RejectsInvalid()
        {
            Assert.ThrowsException<ArgumentOutOfRangeException>(() => Box2D.SetLengthUnitsPerMeter(0f));
            Assert.ThrowsException<ArgumentOutOfRangeException>(() => Box2D.SetLengthUnitsPerMeter(-1f));
            Assert.ThrowsException<ArgumentOutOfRangeException>(() => Box2D.SetLengthUnitsPerMeter(float.NaN));
        }

        [TestMethod]
        public void Box2D_SetHandlers_RejectsNull()
        {
            Assert.ThrowsException<ArgumentNullException>(() => Box2D.SetAssertHandler(null!));
            Assert.ThrowsException<ArgumentNullException>(() => Box2D.SetLogHandler(null!));
        }

        [TestMethod]
        public void Box2D_HashAccumulates()
        {
            byte[] data = new byte[] { 1, 2, 3, 4 };
            uint h1 = Box2D.Hash(5381u, data);
            uint h2 = Box2D.Hash(5381u, data, 4);
            Assert.AreEqual(h1, h2);
            // Empty slice yields the seed.
            Assert.AreEqual(5381u, Box2D.Hash(5381u, new byte[0], 0));
        }

        [TestMethod]
        public void Box2D_Timer_GetTicksAndMilliseconds()
        {
            ulong start = Box2D.GetTicks();
            // Burn a small amount of work to get a measurable delta.
            int spin = 0;
            for (int i = 0; i < 100_000; ++i) spin += i;
            Assert.IsTrue(spin > 0); // keep optimizer honest
            float ms = Box2D.GetMilliseconds(start);
            Assert.IsTrue(ms >= 0f, $"ms was {ms}");

            ulong cursor = start;
            float resetMs = Box2D.GetMillisecondsAndReset(ref cursor);
            Assert.IsTrue(resetMs >= 0f);
            Assert.IsTrue(cursor >= start);
        }

        // ----- Box2DAllocator (custom IArrayAllocator hook) -----

        private sealed class CountingAllocator : IArrayAllocator
        {
            public int AllocCount;
            public int FreeCount;
            public T[] Allocate<T>(int count)
            {
                AllocCount++;
                return new T[count];
            }
            public void Free<T>(T[] array)
            {
                FreeCount++;
            }
        }

        [TestMethod]
        public void Box2DAllocator_HonorsCustomAllocator()
        {
            CountingAllocator custom = new CountingAllocator();
            Box2D.SetAllocator(custom);
            try
            {
                int[] buf = Box2DAllocator.Alloc<int>(64);
                Assert.IsNotNull(buf);
                Assert.AreEqual(64, buf.Length);
                Assert.AreEqual(1, custom.AllocCount);

                Box2DAllocator.Free(buf);
                Assert.AreEqual(1, custom.FreeCount);
            }
            finally
            {
                // Reinstate the default allocator path by setting a no-op which the
                // alloc fallback bypasses (custom != null still). Tests below depend
                // on the default path, so use a fresh CountingAllocator and ignore it.
                Box2D.SetAllocator(new CountingAllocator());
            }
        }

        [TestMethod]
        public void Box2DAllocator_GrowCopiesAndFreesOld()
        {
            int[] a = Box2DAllocator.Alloc<int>(8);
            for (int i = 0; i < 8; ++i) a[i] = i + 1;

            int[] grown = Box2DAllocator.Grow(a, 16);
            Assert.AreEqual(16, grown.Length);
            for (int i = 0; i < 8; ++i)
            {
                Assert.AreEqual(i + 1, grown[i]);
            }
            Box2DAllocator.Free(grown);
        }

        [TestMethod]
        public void Box2DAllocator_FreeNullOrEmptyIsNoOp()
        {
            Box2DAllocator.Free<int>(null);
            Box2DAllocator.Free(Array.Empty<int>());
            // No exception, nothing to assert beyond "didn't throw".
        }

        [TestMethod]
        public void Box2DAllocator_AllocZeroReturnsEmptyArray()
        {
            int[] zero = Box2DAllocator.Alloc<int>(0);
            Assert.AreSame(Array.Empty<int>(), zero);
        }

        // ----- ArenaAllocator -----

        [TestMethod]
        public void Arena_AllocBytes_FromInternalCapacity()
        {
            ArenaAllocator arena = new ArenaAllocator(1024);
            ArenaBlock block = arena.AllocateBytes(64);
            Assert.IsFalse(block.UsedHeap);
            Assert.AreEqual(64, block.Size);
            arena.FreeBytes(block);
            Assert.AreEqual(0, arena.Allocation);
            arena.Destroy();
        }

        [TestMethod]
        public void Arena_AllocBytes_SpillsToHeapWhenOverCapacity()
        {
            ArenaAllocator arena = new ArenaAllocator(16);
            ArenaBlock big = arena.AllocateBytes(128); // larger than internal capacity
            Assert.IsTrue(big.UsedHeap);
            arena.FreeBytes(big);
            arena.Destroy();
        }

        [TestMethod]
        public void Arena_AllocateArray_RoundTrip()
        {
            ArenaAllocator arena = new ArenaAllocator(1024);
            int[] a = arena.AllocateArray<int>(16);
            Assert.AreEqual(16, a.Length);
            arena.FreeArray(a);
            arena.Destroy();
        }

        [TestMethod]
        public void Arena_AllocateArray_ZeroReturnsEmpty()
        {
            ArenaAllocator arena = new ArenaAllocator(1024);
            int[] zero = arena.AllocateArray<int>(0);
            Assert.AreEqual(0, zero.Length);
            arena.Destroy();
        }

        [TestMethod]
        public void Arena_Grow_AdvancesCapacityToFitMax()
        {
            ArenaAllocator arena = new ArenaAllocator(32);
            int initial = arena.Capacity;
            // Force a heap spill so MaxAllocation > initial Capacity.
            ArenaBlock big = arena.AllocateBytes(256);
            arena.FreeBytes(big);
            arena.Grow();
            Assert.IsTrue(arena.Capacity > initial);
            arena.Destroy();
        }

        [TestMethod]
        public void ArenaBlock_SpanLengthMatchesSize()
        {
            ArenaAllocator arena = new ArenaAllocator(256);
            ArenaBlock block = arena.AllocateBytes(48);
            // Align32 rounds up to 64.
            Assert.AreEqual(64, block.Size);
            Assert.AreEqual(64, block.Span.Length);
            arena.FreeBytes(block);
            arena.Destroy();
        }
    }
}
