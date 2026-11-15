// Grasshopper Script Instance
using System;
using System.Collections.Generic;
using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Rhino.Geometry;

// Center the raw profile and normalize its longest world bounding-box side to Size.
// Apply the dimensionless Scale factor, then sweep along start / middle / end triples.
// ArcPts uses List access.
// All other inputs use Item access. T is normalized arc length and Ang is in degrees.
// OffX and OffY are signed distances in the first rail's normal plane, in model units.
// At zero offset the profile point selected by T lies on the rail.
// Srf contains the exact surface for each swept segment. PatchL contains its
// quad mesh. LayersL groups the uncolored strips by original segment index.
public class Script_Instance : GH_ScriptInstance
{
    private void RunScript(
		List<Point3d> ArcPts,
		Curve Prof,
		double Size,
		double Scale,
		double T,
		double Ang,
		double OffX,
		double OffY,
		double EdgeLen,
		ref object Srf,
		ref object PatchL,
		ref object LayersL)
    {
        Srf = PatchL = LayersL = null;
        if (Prof == null || !Prof.IsValid)
        {
            Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Supply a valid profile curve.");
            return;
        }
        if (ArcPts == null || ArcPts.Count < 3 || ArcPts.Count % 3 != 0)
        {
            Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Error,
                "ArcPts requires a start, middle and end point for each segment.");
            return;
        }

        Curve profile = PrepareProfile(Prof, Size, Scale);
        if (profile == null) return;
        Plane source = ProfileFrame(profile, T);
        var surfaces = new List<Brep>();
        var meshes = new List<Mesh>();
        var layers = new DataTree<Mesh>();
        Curve section = null;

        for (int i = 0; i < ArcPts.Count / 3; i++)
        {
            int first = 3 * i;
            Rail rail;
            if (!Rail.TryCreate(ArcPts[first], ArcPts[first + 1], ArcPts[first + 2], out rail))
                continue;

            // Place the profile once. Every later segment starts with its
            // predecessor's end section, including transitions to straight rails.
            if (section == null)
            {
                var frame = new Plane(rail.Start, rail.Radial, rail.Transverse);
                Transform placement = Transform.PlaneToPlane(source, frame);
                Transform spin = Transform.Rotation(Ang * Math.PI / 180.0, rail.Tangent, rail.Start);
                // Translate after the spin so Ang can rotate the profile at a fixed offset.
                Transform offset = Transform.Translation(rail.Radial * OffX + rail.Transverse * OffY);
                section = profile.DuplicateCurve();
                section.Transform(offset * spin * placement);
            }

            surfaces.Add(rail.SweepSurface(section));
            if (EdgeLen > 0.0)
            {
                int across = Math.Max(1, (int)Math.Round(section.GetLength() / EdgeLen));
                int along = rail.StepCount(section, EdgeLen, across);
                Point3d[] samples = SampleProfile(section, across);
                var grid = new Point3d[along + 1, samples.Length];
                for (int u = 0; u <= along; u++)
                {
                    Transform motion = rail.Motion(u, along);
                    for (int v = 0; v < samples.Length; v++)
                    {
                        Point3d point = samples[v];
                        point.Transform(motion);
                        grid[u, v] = point;
                    }
                }

                meshes.Add(PatchMesh(grid));
                var path = new GH_Path(i);
                for (int v = 0; v < samples.Length - 1; v++)
                    layers.Add(LayerMesh(grid, v), path);
            }

            section = section.DuplicateCurve();
            section.Transform(rail.Motion(1, 1));
        }

        Srf = surfaces;
        PatchL = meshes;
        LayersL = layers;
    }

    private Curve PrepareProfile(Curve original, double size, double scale)
    {
        BoundingBox box = original.GetBoundingBox(true);
        if (!box.IsValid)
        {
            Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "The profile has no valid bounding box.");
            return null;
        }

        double target = size > 0.0 ? size : 1.0;
        Vector3d span = box.Diagonal;
        double longest = Math.Max(span.X, Math.Max(span.Y, span.Z));
        Transform normalize = Transform.Translation(Point3d.Origin - box.Center);
        if (longest < 1e-12)
        {
            Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Warning,
                "The profile has negligible extent. Size normalization was skipped.");
        }
        else
        {
            normalize = Transform.Scale(Point3d.Origin, target / longest) * normalize;
        }

        Curve prepared = original.DuplicateCurve();
        if (prepared == null || !prepared.Transform(normalize))
        {
            Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Could not normalize the profile.");
            return null;
        }

        // The Scale component takes its pivot from the already-normalized curve.
        // The sweep's attachment point T is evaluated after both transforms.
        Point3d pivot = prepared.GetBoundingBox(true).Center;
        if (!prepared.Transform(Transform.Scale(pivot, scale)) || !prepared.IsValid)
        {
            Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Scale produced an invalid profile.");
            return null;
        }
        return prepared;
    }

    private Plane ProfileFrame(Curve profile, double t)
    {
        Point3d anchor = profile.PointAtNormalizedLength(Math.Max(0.0, Math.Min(1.0, t)));
        Plane plane;
        if (!profile.TryGetPlane(out plane))
        {
            Point3d[] points = SampleProfile(profile, 20);
            if (points.Length < 3 || Plane.FitPlaneToPoints(points, out plane) != PlaneFitResult.Success)
                plane = Plane.WorldXY;
            Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                "Used an estimated placement plane for the nonplanar profile.");
        }
        return new Plane(anchor, plane.XAxis, plane.YAxis);
    }

    private static Point3d[] SampleProfile(Curve section, int divisions)
    {
        double[] parameters = section.DivideByCount(divisions, true);
        if (parameters == null || parameters.Length < 2)
            return new[] { section.PointAtStart, section.PointAtEnd };
        return Array.ConvertAll(parameters, section.PointAt);
    }

    private struct Rail
    {
        public Point3d Start;
        public Vector3d Radial, Transverse, Tangent;
        private bool isArc;
        private Point3d center;
        private Vector3d axis, displacement;
        private double angle;

        public static bool TryCreate(Point3d start, Point3d middle, Point3d end, out Rail rail)
        {
            rail = new Rail { Start = start, displacement = end - start };
            var arc = new Arc(start, middle, end);
            rail.isArc = arc.IsValid && arc.Radius > 1e-9 && arc.Radius < 1e12;
            if (rail.isArc)
            {
                rail.center = arc.Center;
                rail.angle = arc.Angle;
                rail.axis = arc.Plane.Normal;
                rail.axis.Unitize();
                rail.Radial = start - arc.Center;
                if (!rail.Radial.Unitize()) rail.Radial = arc.Plane.XAxis;
                rail.Tangent = Vector3d.CrossProduct(rail.axis, rail.Radial);
                rail.Tangent.Unitize();
            }
            else
            {
                if (rail.displacement.Length < 1e-12) return false;
                rail.Tangent = rail.displacement;
                rail.Tangent.Unitize();
                rail.Radial.PerpendicularTo(rail.Tangent);
                rail.Radial.Unitize();
            }
            rail.Transverse = Vector3d.CrossProduct(rail.Tangent, rail.Radial);
            rail.Transverse.Unitize();
            return true;
        }

        public Transform Motion(int station, int steps)
        {
            return isArc
                ? Transform.Rotation(angle * station / steps, axis, center)
                : Transform.Translation(displacement * ((double)station / steps));
        }

        public Brep SweepSurface(Curve section)
        {
            Surface surface = isArc
                ? (Surface)RevSurface.Create(section, new Line(center, center + axis), 0.0, angle)
                : Surface.CreateExtrusion(section, displacement);
            return surface == null ? null : surface.ToBrep();
        }

        public int StepCount(Curve section, double edgeLength, int across)
        {
            if (!isArc)
                return Math.Max(1, (int)Math.Round(displacement.Length / edgeLength));

            // Fit the nearest whole number of chords at the largest sampled radius.
            var axisLine = new Line(center, center + axis);
            double radius = 0.0;
            foreach (Point3d point in SampleProfile(section, Math.Max(across, 8)))
                radius = Math.Max(radius, axisLine.DistanceTo(point, false));
            double ratio = radius > 1e-9 ? edgeLength / (2.0 * radius) : 2.0;
            return ratio >= 1.0 ? 1 : Math.Max(1, (int)Math.Round(angle / (2.0 * Math.Asin(ratio))));
        }
    }

    private static Mesh PatchMesh(Point3d[,] grid)
    {
        var mesh = new Mesh();
        int stations = grid.GetLength(0);
        int width = grid.GetLength(1);
        for (int u = 0; u < stations; u++)
            for (int v = 0; v < width; v++)
                mesh.Vertices.Add(grid[u, v]);

        for (int u = 0; u < stations - 1; u++)
        {
            for (int v = 0; v < width - 1; v++)
            {
                int a = u * width + v;
                mesh.Faces.AddFace(a, a + width, a + width + 1, a + 1);
            }
        }
        mesh.RebuildNormals();
        return mesh;
    }

    private static Mesh LayerMesh(Point3d[,] grid, int v)
    {
        var strip = new Mesh();
        int stations = grid.GetLength(0);
        for (int u = 0; u < stations; u++)
        {
            strip.Vertices.Add(grid[u, v]);
            strip.Vertices.Add(grid[u, v + 1]);
        }
        for (int u = 0; u < stations - 1; u++)
            strip.Faces.AddFace(2 * u, 2 * u + 2, 2 * u + 3, 2 * u + 1);
        strip.RebuildNormals();
        return strip;
    }
}
