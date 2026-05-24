using System;

namespace Box2DNG
{
    public sealed class Fixture
    {
        public Body Body { get; }
        public Shape Shape { get; }
        public float Friction { get; internal set; }
        public float Restitution { get; internal set; }
        public float Density { get; internal set; }
        public bool IsSensor { get; internal set; }
        public bool EnableSensorEvents { get; internal set; } = true;
        public Filter Filter { get; internal set; }
        public SurfaceMaterial Material { get; internal set; }
        public object? UserData { get; internal set; }
        public int Id { get; internal set; } = -1;
        public int ProxyId { get; internal set; } = -1;
        public Aabb Aabb { get; internal set; }

        internal Fixture(Body body, Shape shape)
        {
            Body = body ?? throw new ArgumentNullException(nameof(body));
            Shape = shape ?? throw new ArgumentNullException(nameof(shape));
            Friction = 0.2f;
            Restitution = 0f;
            Density = 1f;
            IsSensor = false;
            Filter = Filter.Default;
            Material = new SurfaceMaterial(Friction, Restitution, 0f, 0f, 0, 0);
        }

        public void SetSensorEventsEnabled(bool enable)
        {
            EnableSensorEvents = enable;
        }
    }
}
