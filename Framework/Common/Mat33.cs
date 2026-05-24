using System;

namespace Box2DNG
{
    public readonly struct Mat33 : IEquatable<Mat33>
    {
        public readonly Vec3 Ex;
        public readonly Vec3 Ey;
        public readonly Vec3 Ez;

        public Mat33(Vec3 ex, Vec3 ey, Vec3 ez)
        {
            Ex = ex;
            Ey = ey;
            Ez = ez;
        }

        public static Mat33 Identity => new Mat33(new Vec3(1f, 0f, 0f), new Vec3(0f, 1f, 0f), new Vec3(0f, 0f, 1f));

        public bool Equals(Mat33 other) => Ex.Equals(other.Ex) && Ey.Equals(other.Ey) && Ez.Equals(other.Ez);

        public override bool Equals(object? obj) => obj is Mat33 other && Equals(other);

        public override int GetHashCode()
        {
            unchecked
            {
                int hash = 17;
                hash = (hash * 31) + Ex.GetHashCode();
                hash = (hash * 31) + Ey.GetHashCode();
                hash = (hash * 31) + Ez.GetHashCode();
                return hash;
            }
        }

        public static Vec3 Mul(Mat33 a, Vec3 v)
        {
            return v.X * a.Ex + v.Y * a.Ey + v.Z * a.Ez;
        }

        public static Vec2 Mul22(Mat33 a, Vec2 v)
        {
            return new Vec2(a.Ex.X * v.X + a.Ey.X * v.Y, a.Ex.Y * v.X + a.Ey.Y * v.Y);
        }

        public Vec3 Solve33(Vec3 b)
        {
            float det = Vec3.Dot(Ex, Vec3.Cross(Ey, Ez));
            if (det != 0f)
            {
                det = 1f / det;
            }
            return new Vec3(
                det * Vec3.Dot(b, Vec3.Cross(Ey, Ez)),
                det * Vec3.Dot(Ex, Vec3.Cross(b, Ez)),
                det * Vec3.Dot(Ex, Vec3.Cross(Ey, b)));
        }

        public Vec2 Solve22(Vec2 b)
        {
            float a11 = Ex.X, a12 = Ey.X, a21 = Ex.Y, a22 = Ey.Y;
            float det = a11 * a22 - a12 * a21;
            if (det != 0f)
            {
                det = 1f / det;
            }
            return new Vec2(det * (a22 * b.X - a12 * b.Y), det * (a11 * b.Y - a21 * b.X));
        }

        public override string ToString() => $"[{Ex} {Ey} {Ez}]";
    }
}
