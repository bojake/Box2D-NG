using System;

namespace Box2DNG
{
    public readonly struct Aabb : IEquatable<Aabb>
    {
        public readonly Vec2 LowerBound;
        public readonly Vec2 UpperBound;

        public Aabb(Vec2 lower, Vec2 upper)
        {
            LowerBound = lower;
            UpperBound = upper;
        }

        public bool Equals(Aabb other) => LowerBound.Equals(other.LowerBound) && UpperBound.Equals(other.UpperBound);

        public override bool Equals(object? obj) => obj is Aabb other && Equals(other);

        public override int GetHashCode()
        {
            unchecked
            {
                int hash = 17;
                hash = (hash * 31) + LowerBound.GetHashCode();
                hash = (hash * 31) + UpperBound.GetHashCode();
                return hash;
            }
        }

        public Vec2 Center => (LowerBound + UpperBound) * 0.5f;
        public Vec2 Extents => (UpperBound - LowerBound) * 0.5f;
        public float Perimeter => 2f * (UpperBound.X - LowerBound.X + UpperBound.Y - LowerBound.Y);

        public bool Contains(Aabb other)
        {
            return LowerBound.X <= other.LowerBound.X &&
                   LowerBound.Y <= other.LowerBound.Y &&
                   other.UpperBound.X <= UpperBound.X &&
                   other.UpperBound.Y <= UpperBound.Y;
        }

        public static Aabb Union(Aabb a, Aabb b)
        {
            Vec2 lower = new Vec2(MathF.Min(a.LowerBound.X, b.LowerBound.X), MathF.Min(a.LowerBound.Y, b.LowerBound.Y));
            Vec2 upper = new Vec2(MathF.Max(a.UpperBound.X, b.UpperBound.X), MathF.Max(a.UpperBound.Y, b.UpperBound.Y));
            return new Aabb(lower, upper);
        }

        public static bool Overlaps(Aabb a, Aabb b)
        {
            return !(b.LowerBound.X > a.UpperBound.X ||
                     b.LowerBound.Y > a.UpperBound.Y ||
                     a.LowerBound.X > b.UpperBound.X ||
                     a.LowerBound.Y > b.UpperBound.Y);
        }

        public override string ToString() => $"[{LowerBound} -> {UpperBound}]";
    }
}
