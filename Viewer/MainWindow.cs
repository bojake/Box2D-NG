using System;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Layout;
using Avalonia.Media;
using Avalonia.Media.Immutable;
using Avalonia.Threading;

namespace Box2DNG.Viewer
{
    public sealed class MainWindow : Window
    {
        private const float PixelsPerMeter = 60f;

        private readonly Samples.ISample[] _samples;
        private readonly ComboBox _sampleSelector;
        private readonly WorldCanvas _canvas;
        private readonly DispatcherTimer _timer;

        private World _world;
        private Samples.ISample _sample;

        public MainWindow()
        {
            Title = "Box2D-NG Viewer";
            Width = 1000;
            Height = 700;
            Background = Brushes.White;

            _samples = (Samples.ISample[])Samples.SampleCatalog.All;
            _sample = _samples[0];
            _world = CreateWorld(_sample);

            _canvas = new WorldCanvas(this) { Focusable = true };
            _canvas.SetWorld(_world);

            _sampleSelector = new ComboBox
            {
                Width = 220,
                Margin = new Thickness(10, 10, 0, 0),
                HorizontalAlignment = HorizontalAlignment.Left,
                VerticalAlignment = VerticalAlignment.Top,
            };
            foreach (Samples.ISample sample in _samples)
            {
                _sampleSelector.Items.Add(sample.Name);
            }
            _sampleSelector.SelectedIndex = 0;
            _sampleSelector.SelectionChanged += (_, __) =>
            {
                int index = _sampleSelector.SelectedIndex;
                if (index < 0 || index >= _samples.Length)
                {
                    return;
                }
                _sample = _samples[index];
                _world = CreateWorld(_sample);
                _canvas.SetWorld(_world);
            };

            Grid root = new Grid();
            root.Children.Add(_canvas);
            root.Children.Add(_sampleSelector);
            Content = root;

            KeyDown += (_, e) =>
            {
                string s = e.Key.ToString();
                if (s.Length == 1)
                {
                    _sample.OnKey(char.ToLowerInvariant(s[0]));
                }
            };

            _timer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(16) };
            _timer.Tick += (_, __) =>
            {
                int subSteps = Math.Max(1, _sample.SubSteps);
                float dt = (1f / 60f) / subSteps;
                for (int i = 0; i < subSteps; ++i)
                {
                    _sample.Step(_world, dt);
                    _world.Step(dt);
                }
                _canvas.InvalidateVisual();
            };
            _timer.Start();
        }

        private static World CreateWorld(Samples.ISample sample)
        {
            World world = new World(sample.CreateWorldDef());
            sample.Build(world);
            return world;
        }

        private sealed class WorldCanvas : Control
        {
            private static readonly IPen ShapePen = new ImmutablePen(Brushes.Black, 2);
            private static readonly IBrush ShapeFill = new ImmutableSolidColorBrush(Color.FromArgb(120, 70, 140, 200));
            private static readonly IBrush DiagBg = new ImmutableSolidColorBrush(Color.FromArgb(180, 20, 20, 20));
            private static readonly IBrush DiagFg = Brushes.White;
            private static readonly IPen AxisPen = new ImmutablePen(new ImmutableSolidColorBrush(Color.FromArgb(120, 80, 80, 80)), 2);
            private static readonly IPen LimitPen = new ImmutablePen(new ImmutableSolidColorBrush(Color.FromArgb(180, 70, 160, 70)), 2);
            private static readonly IBrush PointBrush = new ImmutableSolidColorBrush(Color.FromArgb(200, 80, 80, 80));
            private static readonly Typeface DiagFont = new Typeface(FontFamily.Default);

            private readonly MainWindow _owner;
            private World? _world;

            public WorldCanvas(MainWindow owner)
            {
                _owner = owner;
                ClipToBounds = true;
            }

            public void SetWorld(World world)
            {
                _world = world;
                InvalidateVisual();
            }

            public override void Render(DrawingContext context)
            {
                base.Render(context);
                if (_world == null)
                {
                    return;
                }

                foreach (Body body in _world.Bodies)
                {
                    foreach (Fixture fixture in body.Fixtures)
                    {
                        DrawFixture(context, body, fixture);
                    }
                }

                DrawPrismaticJoints(context);
                DrawDiagnostics(context);
            }

            private void DrawFixture(DrawingContext g, Body body, Fixture fixture)
            {
                switch (fixture.Shape.Type)
                {
                    case ShapeType.Circle:
                        DrawCircle(g, body, (CircleShape)fixture.Shape);
                        break;
                    case ShapeType.Polygon:
                        DrawPolygon(g, body, (PolygonShape)fixture.Shape);
                        break;
                    case ShapeType.Capsule:
                        DrawCapsule(g, body, (CapsuleShape)fixture.Shape);
                        break;
                    case ShapeType.Segment:
                        DrawSegment(g, body, (SegmentShape)fixture.Shape);
                        break;
                    case ShapeType.ChainSegment:
                        DrawChainSegment(g, body, (ChainSegmentShape)fixture.Shape);
                        break;
                }
            }

            private void DrawCircle(DrawingContext g, Body body, CircleShape shape)
            {
                Vec2 center = Transform.Mul(body.Transform, shape.Center);
                Point c = ToScreen(center);
                double r = shape.Radius * PixelsPerMeter;
                g.DrawEllipse(ShapeFill, ShapePen, c, r, r);
            }

            private void DrawPolygon(DrawingContext g, Body body, PolygonShape shape)
            {
                if (shape.Vertices.Count == 0)
                {
                    return;
                }
                StreamGeometry geom = new StreamGeometry();
                using (var ctx = geom.Open())
                {
                    Vec2 v0 = Transform.Mul(body.Transform, shape.Vertices[0]);
                    ctx.BeginFigure(ToScreen(v0), isFilled: true);
                    for (int i = 1; i < shape.Vertices.Count; ++i)
                    {
                        Vec2 v = Transform.Mul(body.Transform, shape.Vertices[i]);
                        ctx.LineTo(ToScreen(v));
                    }
                    ctx.EndFigure(isClosed: true);
                }
                g.DrawGeometry(ShapeFill, ShapePen, geom);
            }

            private void DrawCapsule(DrawingContext g, Body body, CapsuleShape shape)
            {
                Vec2 c1 = Transform.Mul(body.Transform, shape.Center1);
                Vec2 c2 = Transform.Mul(body.Transform, shape.Center2);
                double r = shape.Radius * PixelsPerMeter;
                Point p1 = ToScreen(c1);
                Point p2 = ToScreen(c2);
                g.DrawLine(ShapePen, p1, p2);
                g.DrawEllipse(ShapeFill, ShapePen, p1, r, r);
                g.DrawEllipse(ShapeFill, ShapePen, p2, r, r);
            }

            private void DrawSegment(DrawingContext g, Body body, SegmentShape shape)
            {
                Vec2 p1 = Transform.Mul(body.Transform, shape.Point1);
                Vec2 p2 = Transform.Mul(body.Transform, shape.Point2);
                g.DrawLine(ShapePen, ToScreen(p1), ToScreen(p2));
            }

            private void DrawChainSegment(DrawingContext g, Body body, ChainSegmentShape shape)
            {
                Vec2 p1 = Transform.Mul(body.Transform, shape.Point1);
                Vec2 p2 = Transform.Mul(body.Transform, shape.Point2);
                g.DrawLine(ShapePen, ToScreen(p1), ToScreen(p2));
            }

            private void DrawPrismaticJoints(DrawingContext g)
            {
                if (_world == null || _world.PrismaticJoints.Count == 0)
                {
                    return;
                }

                foreach (PrismaticJoint joint in _world.PrismaticJoints)
                {
                    Vec2 anchorA = joint.BodyA.GetWorldPoint(joint.LocalAnchorA);
                    Vec2 anchorB = joint.BodyB.GetWorldPoint(joint.LocalAnchorB);
                    Vec2 axis = Rot.Mul(joint.BodyA.Transform.Q, joint.LocalAxisA).Normalize();

                    Point pA = ToScreen(anchorA);
                    Point pB = ToScreen(anchorB);
                    g.DrawLine(AxisPen, pA, pB);

                    float scale = 0.25f;
                    Vec2 perp = new Vec2(-axis.Y, axis.X);
                    Vec2 sp1 = anchorA - scale * perp;
                    Vec2 sp2 = anchorA + scale * perp;
                    g.DrawLine(AxisPen, ToScreen(sp1), ToScreen(sp2));

                    if (joint.EnableLimit)
                    {
                        Vec2 lower = anchorA + joint.LowerTranslation * axis;
                        Vec2 upper = anchorA + joint.UpperTranslation * axis;
                        g.DrawLine(LimitPen, ToScreen(lower), ToScreen(upper));
                        g.DrawLine(LimitPen, ToScreen(lower - scale * perp), ToScreen(lower + scale * perp));
                        g.DrawLine(LimitPen, ToScreen(upper - scale * perp), ToScreen(upper + scale * perp));
                    }

                    double radius = 3;
                    g.DrawEllipse(PointBrush, null, pA, radius, radius);
                    g.DrawEllipse(PointBrush, null, pB, radius, radius);
                }
            }

            private void DrawDiagnostics(DrawingContext g)
            {
                if (_world == null)
                {
                    return;
                }
                FormattedText text = new FormattedText(
                    $"Contacts: {_world.Contacts.Count}",
                    System.Globalization.CultureInfo.InvariantCulture,
                    FlowDirection.LeftToRight,
                    DiagFont,
                    13,
                    DiagFg);
                Rect rect = new Rect(10, 40, text.Width + 8, text.Height + 6);
                g.FillRectangle(DiagBg, rect);
                g.DrawText(text, new Point(rect.X + 4, rect.Y + 3));
            }

            private Point ToScreen(Vec2 worldPoint)
            {
                double w = Bounds.Width;
                double h = Bounds.Height;
                double x = w * 0.5 + worldPoint.X * PixelsPerMeter;
                double y = h * 0.75 - worldPoint.Y * PixelsPerMeter;
                return new Point(x, y);
            }
        }
    }
}
