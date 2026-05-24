using System;

namespace Box2DNG
{
    public sealed class WorldEvents
    {
        public event Action<BodyEvents>? BodyEvents;
        public event Action<ContactEvents>? ContactEvents;
        public event Action<ContactImpulseEvents>? ContactImpulseEvents;
        public event Action<SensorEvents>? SensorEvents;
        public event Action<SensorHitEvents>? SensorHitEvents;
        public event Action<JointEvents>? JointEvents;

        internal void Raise(BodyEvents events) => BodyEvents?.Invoke(events);
        internal void Raise(ContactEvents events) => ContactEvents?.Invoke(events);
        internal void Raise(ContactImpulseEvents events) => ContactImpulseEvents?.Invoke(events);
        internal void Raise(SensorEvents events) => SensorEvents?.Invoke(events);
        internal void Raise(SensorHitEvents events) => SensorHitEvents?.Invoke(events);
        internal void Raise(JointEvents events) => JointEvents?.Invoke(events);
    }

    public readonly struct BodyEvents
    {
        public readonly BodyEvent[] Events;

        public BodyEvents(BodyEvent[] events)
        {
            Events = events;
        }
    }

    public readonly struct ContactEvents
    {
        public readonly ContactBeginEvent[] Begin;
        public readonly ContactEndEvent[] End;
        public readonly ContactHitEvent[] Hit;

        public ContactEvents(ContactBeginEvent[] begin, ContactEndEvent[] end, ContactHitEvent[] hit)
        {
            Begin = begin;
            End = end;
            Hit = hit;
        }
    }

    public readonly struct ContactImpulseEvents
    {
        public readonly ContactImpulseEvent[] Events;

        public ContactImpulseEvents(ContactImpulseEvent[] events)
        {
            Events = events;
        }
    }

    public readonly struct SensorEvents
    {
        public readonly SensorBeginEvent[] Begin;
        public readonly SensorEndEvent[] End;

        public SensorEvents(SensorBeginEvent[] begin, SensorEndEvent[] end)
        {
            Begin = begin;
            End = end;
        }
    }

    public readonly struct SensorHitEvents
    {
        public readonly SensorHitEvent[] Events;

        public SensorHitEvents(SensorHitEvent[] events)
        {
            Events = events;
        }
    }

    public readonly struct JointEvents
    {
        public readonly JointEvent[] Events;

        public JointEvents(JointEvent[] events)
        {
            Events = events;
        }
    }

    public readonly struct BodyEvent
    {
        public readonly object? UserData;
        public readonly Transform Transform;
        public readonly bool FellAsleep;

        public BodyEvent(object? userData, Transform transform, bool fellAsleep)
        {
            UserData = userData;
            Transform = transform;
            FellAsleep = fellAsleep;
        }

        public Vec2 Position => Transform.P;
    }

    public readonly struct ContactBeginEvent
    {
        public readonly object? UserDataA;
        public readonly object? UserDataB;

        public ContactBeginEvent(object? userDataA, object? userDataB)
        {
            UserDataA = userDataA;
            UserDataB = userDataB;
        }
    }

    public readonly struct ContactEndEvent
    {
        public readonly object? UserDataA;
        public readonly object? UserDataB;

        public ContactEndEvent(object? userDataA, object? userDataB)
        {
            UserDataA = userDataA;
            UserDataB = userDataB;
        }
    }

    public readonly struct ContactHitEvent
    {
        public readonly object? UserDataA;
        public readonly object? UserDataB;
        public readonly Vec2 Point;
        public readonly Vec2 Normal;
        public readonly float Speed;

        public ContactHitEvent(object? userDataA, object? userDataB, Vec2 point, Vec2 normal, float speed)
        {
            UserDataA = userDataA;
            UserDataB = userDataB;
            Point = point;
            Normal = normal;
            Speed = speed;
        }
    }

    public readonly struct ContactImpulseEvent
    {
        public readonly object? UserDataA;
        public readonly object? UserDataB;
        public readonly Vec2 Point;
        public readonly Vec2 Normal;
        public readonly float NormalImpulse;
        public readonly float TangentImpulse;

        public ContactImpulseEvent(object? userDataA, object? userDataB, Vec2 point, Vec2 normal, float normalImpulse, float tangentImpulse)
        {
            UserDataA = userDataA;
            UserDataB = userDataB;
            Point = point;
            Normal = normal;
            NormalImpulse = normalImpulse;
            TangentImpulse = tangentImpulse;
        }
    }

    public readonly struct SensorBeginEvent
    {
        public readonly object? UserDataA;
        public readonly object? UserDataB;

        public SensorBeginEvent(object? userDataA, object? userDataB)
        {
            UserDataA = userDataA;
            UserDataB = userDataB;
        }
    }

    public readonly struct SensorEndEvent
    {
        public readonly object? UserDataA;
        public readonly object? UserDataB;

        public SensorEndEvent(object? userDataA, object? userDataB)
        {
            UserDataA = userDataA;
            UserDataB = userDataB;
        }
    }

    public readonly struct SensorHitEvent
    {
        public readonly object? SensorUserData;
        public readonly object? VisitorUserData;
        public readonly Vec2 Point;
        public readonly Vec2 Normal;
        public readonly float Fraction;

        public SensorHitEvent(object? sensorUserData, object? visitorUserData, Vec2 point, Vec2 normal, float fraction)
        {
            SensorUserData = sensorUserData;
            VisitorUserData = visitorUserData;
            Point = point;
            Normal = normal;
            Fraction = fraction;
        }
    }

    public readonly struct JointEvent
    {
        public readonly object? UserDataA;
        public readonly object? UserDataB;
        public readonly float ReactionForce;
        public readonly float ReactionTorque;

        public JointEvent(object? userDataA, object? userDataB, float reactionForce, float reactionTorque)
        {
            UserDataA = userDataA;
            UserDataB = userDataB;
            ReactionForce = reactionForce;
            ReactionTorque = reactionTorque;
        }
    }
}
