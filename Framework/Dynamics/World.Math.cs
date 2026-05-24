using System;

namespace Box2DNG
{
    public sealed partial class World
    {
        private static Vec2 Solve22(Mat22 A, Vec2 b) => A.Solve(b);

        private static Vec3 Solve33(Mat33 A, Vec3 b) => A.Solve33(b);
    }
}
