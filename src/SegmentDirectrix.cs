// Grasshopper C# Script (SDK mode).
// Inputs (Item access): C = Curve; Eps = double, absolute fitting tolerance in model units.
// A nonpositive Eps uses 0.001 times the input curve length.
// Outputs: Arcs = ordered arc or line curves; Ctrl = interleaved joints and midpoints.
// For K segments, Ctrl contains 2K+1 points on an open chain and 2K on a closed chain.
// On a closed chain the last segment shares the first joint.
using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using Grasshopper.Kernel;
using Rhino.Geometry;

public class Script_Instance : GH_ScriptInstance
{
    private const int SampleCount = 600;
    private const int MinimumSegmentSamples = 8;

    // The downstream optimizer references these assemblies by name. Load them from
    // this definition's lib folder before Grasshopper requests its control points.
    private void LoadOptimizerLibraries()
    {
        string documentPath = Component.OnPingDocument()?.FilePath;
        if (string.IsNullOrEmpty(documentPath)) return;
        try
        {
            string folder = Path.Combine(Path.GetDirectoryName(documentPath), "lib");
            Assembly.LoadFrom(Path.Combine(folder, "AutoDiff.dll"));
            Assembly.LoadFrom(Path.Combine(folder, "alglib400gpl_net2.dll"));
        }
        catch (Exception ex)
        {
            Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Warning,
                "Could not load the optimizer libraries. Keep the lib folder beside the .gh file. " + ex.Message);
        }
    }

    private void RunScript(Curve C, double Eps, ref object Arcs, ref object Ctrl)
    {
        LoadOptimizerLibraries();
        Arcs = Ctrl = null;
        if (C == null || !C.IsValid)
        {
            Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Supply a valid curve.");
            return;
        }
        double length = C.GetLength();
        if (!(length > 0) || double.IsInfinity(length))
        {
            Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "The curve must have a finite, positive length.");
            return;
        }
        if (double.IsNaN(Eps) || double.IsInfinity(Eps))
        {
            Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Eps must be a finite number.");
            return;
        }
        double eps = Eps > 0 ? Eps : 0.001 * length;
        bool closed = C.IsClosed;
        List<Sample> samples = SampleCurve(C, SampleCount);
        if (samples == null || samples.Count < 10)
        {
            Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Could not sample the directrix.");
            return;
        }
        int n = samples.Count;
        int period = n - 1;
        List<int> forced = ForcedCuts(C, samples);
        var segments = new List<Segment>();

        if (closed && forced.Count == 0)
        {
            // For a smooth loop, choose the fewest segments across several seam positions.
            int seedTrials = Math.Max(1, Math.Min(16, period / 8));
            List<Segment> bestSegs = null;
            for (int s = 0; s < seedTrials; s++)
            {
                int seed = (int)((long)s * period / seedTrials);
                var cand = SweepClosed(samples, seed, eps, MinimumSegmentSamples, period);
                if (cand == null || cand.Count == 0) continue;
                if (bestSegs == null
                    || cand.Count < bestSegs.Count
                    || (cand.Count == bestSegs.Count
                        && Balance(cand, period) < Balance(bestSegs, period)))
                {
                    bestSegs = cand;
                }
            }
            if (bestSegs != null) segments = bestSegs;
        }
        else
        {
            int start = 0;
            var ends = new List<int>(forced); ends.Add(n - 1);
            foreach (int e in ends)
            {
                if (e > start) GreedyBlock(samples, start, e, eps, MinimumSegmentSamples, segments);
                start = Math.Max(start, e);
            }
        }

        // A closed circle fits in one run, but one triple would have identical endpoints.
        // Split that run in two, keeping a valid three-point parameterization.
        if (closed && segments.Count == 1)
        {
            int start = segments[0].Start;
            var rotated = new List<Sample>(period + 1);
            for (int j = 0; j <= period; j++) rotated.Add(samples[(start + j) % period]);
            int half = period / 2;
            segments = new List<Segment> { FitArc(rotated, 0, half), FitArc(rotated, half, period) };
            foreach (Segment seg in segments)
            {
                seg.Start = (start + seg.Start) % period;
                seg.End = (start + seg.End) % period;
                seg.OverTolerance = seg.MaxDeviation > eps;
            }
        }

        int K = segments.Count;
        if (K == 0)
        {
            Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "No segments produced.");
            return;
        }

        int over = 0;
        foreach (Segment g in segments) if (g.OverTolerance) over++;
        if (over > 0)
            Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Warning,
                over + " segment(s) exceed the requested fit tolerance at the minimum sample count. Increase Eps to allow a looser fit.");

        // Each cut supplies one joint shared by both adjacent segments.
        var jointIndices = new List<int>(K + 1);
        jointIndices.Add(segments[0].Start);
        for (int i = 0; i < K; i++) jointIndices.Add(segments[i].End);

        int numJoints = closed ? K : K + 1;

        var joints = new Point3d[numJoints];
        for (int i = 0; i < numJoints; i++) joints[i] = samples[jointIndices[i]].Point;

        var arcs = new List<Curve>(K);
        var ctrl = new List<Point3d>(2 * K + (closed ? 0 : 1));
        for (int i = 0; i < K; i++)
        {
            Point3d start = joints[i];
            Point3d end = joints[(i + 1) % numJoints];
            int a = segments[i].Start, b = segments[i].End;
            if (b <= a) b += period;
            Point3d sample = samples[((a + b) / 2) % period].Point;
            Arc probe = !segments[i].IsLine ? new Arc(start, sample, end) : Arc.Unset;
            Point3d middle = start + (end - start) * 0.5;
            Curve rail = null;
            if (probe.IsValid && probe.Radius > 1e-9)
            {
                Arc arc = new Arc(start, probe.MidPoint, end);
                if (arc.IsValid)
                {
                    middle = probe.MidPoint;
                    rail = new ArcCurve(arc);
                }
            }
            arcs.Add(rail ?? new LineCurve(start, end));
            ctrl.Add(start);
            ctrl.Add(middle);
        }
        if (!closed) ctrl.Add(joints[K]);

        Arcs = arcs;
        Ctrl = ctrl;
    }

    private sealed class Sample
    {
        public double Parameter;
        public Point3d Point;
        public Vector3d Tangent;
    }

    private sealed class Segment
    {
        public int Start, End;
        public bool IsLine;
        public double MaxDeviation = double.MaxValue;
        public bool OverTolerance;
    }

    List<Sample> SampleCurve(Curve c, int n)
    {
        double[] ts = c.DivideByCount(n - 1, true);
        if (ts == null) return null;
        var samples = new List<Sample>(ts.Length);
        foreach (double t in ts)
        {
            Point3d p = c.PointAt(t);
            Vector3d T = c.TangentAt(t);
            if (!T.Unitize()) T = Vector3d.Zero;
            samples.Add(new Sample { Parameter = t, Point = p, Tangent = T });
        }
        return samples;
    }

    List<int> ForcedCuts(Curve c, List<Sample> samples)
    {
        var set = new SortedSet<int>();
        double t0 = c.Domain.T0, td;
        int guard = 0;
        while (c.GetNextDiscontinuity(Continuity.G1_continuous, t0, c.Domain.T1, out td) && guard++ < 1000)
        {
            set.Add(NearestIndex(samples, td));
            t0 = td;
        }
        set.Remove(0); set.Remove(samples.Count - 1);
        return new List<int>(set);
    }

    int NearestIndex(List<Sample> samples, double t)
    {
        int best = 0; double bd = double.MaxValue;
        for (int i = 0; i < samples.Count; i++)
        {
            double d = Math.Abs(samples[i].Parameter - t);
            if (d < bd) { bd = d; best = i; }
        }
        return best;
    }

    // Fit from a trial seam and map the cut indices back to the original sample ring.
    List<Segment> SweepClosed(List<Sample> samples, int seed, double eps, int minPts, int period)
    {
        var R = new List<Sample>(period + 1);
        for (int j = 0; j <= period; j++) R.Add(samples[(seed + j) % period]);   // R[period] closes on R[0]

        var local = new List<Segment>();
        GreedyBlock(R, 0, period, eps, minPts, local);

        foreach (var g in local)
        {
            g.Start = (seed + g.Start) % period;
            g.End = (seed + g.End) % period;
        }
        return local;
    }

    // For equal segment counts, prefer a more even distribution of segment lengths.
    double Balance(List<Segment> segments, int period)
    {
        int lo = int.MaxValue, hi = 1;
        foreach (var g in segments)
        {
            int len = g.End - g.Start;
            if (len <= 0) len += period;
            lo = Math.Min(lo, len); hi = Math.Max(hi, len);
        }
        return (lo > 0) ? (double)hi / lo : double.MaxValue;
    }

    void GreedyBlock(List<Sample> samples, int a, int b, double eps, int minPts, List<Segment> outSegs)
    {
        int i = a;
        while (i < b)
        {
            int jMin = Math.Min(i + minPts - 1, b);
            Segment best = FitArc(samples, i, jMin);
            if (best.MaxDeviation > eps)
            {
                best.OverTolerance = true;
                outSegs.Add(best);
                i = jMin;
                continue;
            }

            // exponential bracketing: double the step while the fit holds
            int good = jMin, bad = -1, step = Math.Max(4, minPts);
            while (good + step <= b)
            {
                Segment f = FitArc(samples, i, good + step);
                if (f.MaxDeviation <= eps) { best = f; good += step; step <<= 1; }
                else { bad = good + step; break; }
            }
            if (bad < 0 && good < b)
            {
                Segment f = FitArc(samples, i, b);
                if (f.MaxDeviation <= eps) { best = f; good = b; }
                else bad = b;
            }
            // bisection on the bracket [good, bad]
            while (bad > 0 && bad - good > 1)
            {
                int mid = (good + bad) / 2;
                Segment f = FitArc(samples, i, mid);
                if (f.MaxDeviation <= eps) { best = f; good = mid; }
                else bad = mid;
            }

            outSegs.Add(best);
            i = good;
        }
    }

    // Best-fit deviation used to decide cuts (Kasa circle in a least-squares plane).
    Segment FitArc(List<Sample> samples, int i0, int i1)
    {
        var g = new Segment { Start = i0, End = i1 };
        int m = i1 - i0 + 1;
        var pts = new Point3d[m];
        for (int k = 0; k < m; k++) pts[k] = samples[i0 + k].Point;

        double turn = 0;
        for (int k = i0; k < i1; k++) turn += Vector3d.VectorAngle(samples[k].Tangent, samples[k + 1].Tangent);
        if (m < 4 || turn < 0.02) return FitLine(g, pts);

        Plane pl;
        if (Plane.FitPlaneToPoints(pts, out pl) != PlaneFitResult.Success) return FitLine(g, pts);
        Vector3d A = pl.Normal;
        if (!A.Unitize()) return FitLine(g, pts);

        double cxs = 0, cys = 0, czs = 0;
        foreach (var p in pts) { cxs += p.X; cys += p.Y; czs += p.Z; }
        var Pc = new Point3d(cxs / m, cys / m, czs / m);
        var fr = new Plane(Pc, A);
        Vector3d U = fr.XAxis, V = fr.YAxis;

        var X = new double[m]; var Y = new double[m]; var Z = new double[m];
        for (int k = 0; k < m; k++)
        {
            Vector3d q = pts[k] - Pc;
            X[k] = q * U; Y[k] = q * V; Z[k] = q * A;
        }

        double Sx = 0, Sy = 0, Sxx = 0, Syy = 0, Sxy = 0, Sw = 0, Swx = 0, Swy = 0;
        for (int k = 0; k < m; k++)
        {
            double w = X[k] * X[k] + Y[k] * Y[k];
            Sx += X[k]; Sy += Y[k];
            Sxx += X[k] * X[k]; Syy += Y[k] * Y[k]; Sxy += X[k] * Y[k];
            Sw += w; Swx += w * X[k]; Swy += w * Y[k];
        }
        double[,] MM = { { Sxx, Sxy, Sx }, { Sxy, Syy, Sy }, { Sx, Sy, (double)m } };
        double[] rhs = { Swx, Swy, Sw };
        double[] sol;
        if (!Solve3(MM, rhs, out sol)) return FitLine(g, pts);
        double ca = 0.5 * sol[0], cb = 0.5 * sol[1];
        double r2 = sol[2] + ca * ca + cb * cb;
        if (!(r2 > 0) || double.IsNaN(r2) || double.IsInfinity(r2)) return FitLine(g, pts);
        double r = Math.Sqrt(r2);

        double dev = 0;
        for (int k = 0; k < m; k++)
        {
            double rad = Math.Sqrt((X[k] - ca) * (X[k] - ca) + (Y[k] - cb) * (Y[k] - cb)) - r;
            double e = Math.Sqrt(rad * rad + Z[k] * Z[k]);
            if (e > dev) dev = e;
        }
        if (double.IsNaN(dev) || double.IsInfinity(dev)) return FitLine(g, pts);

        g.MaxDeviation = dev;
        return g;
    }

    Segment FitLine(Segment g, Point3d[] pts)
    {
        Line ln;
        if (!Line.TryFitLineToPoints(pts, out ln) || !ln.IsValid || ln.Length < 1e-12)
            ln = new Line(pts[0], pts[pts.Length - 1]);
        double dev = 0;
        foreach (var p in pts)
        {
            double d = p.DistanceTo(ln.ClosestPoint(p, false));
            if (d > dev) dev = d;
        }
        g.IsLine = true; g.MaxDeviation = dev;
        return g;
    }

    bool Solve3(double[,] A, double[] b, out double[] x)
    {
        var M = new double[3, 4];
        for (int r = 0; r < 3; r++)
        {
            for (int c = 0; c < 3; c++) M[r, c] = A[r, c];
            M[r, 3] = b[r];
        }
        for (int col = 0; col < 3; col++)
        {
            int piv = col;
            for (int r = col + 1; r < 3; r++)
                if (Math.Abs(M[r, col]) > Math.Abs(M[piv, col])) piv = r;
            if (Math.Abs(M[piv, col]) < 1e-14) { x = null; return false; }
            if (piv != col)
                for (int c = col; c < 4; c++)
                { double tmp = M[col, c]; M[col, c] = M[piv, c]; M[piv, c] = tmp; }
            for (int r = col + 1; r < 3; r++)
            {
                double f = M[r, col] / M[col, col];
                for (int c = col; c < 4; c++) M[r, c] -= f * M[col, c];
            }
        }
        x = new double[3];
        for (int r = 2; r >= 0; r--)
        {
            double s = M[r, 3];
            for (int c = r + 1; c < 3; c++) s -= M[r, c] * x[c];
            x[r] = s / M[r, r];
        }
        return true;
    }

}
