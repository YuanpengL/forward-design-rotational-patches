// Grasshopper Script Instance
//
// Assemblies
#r "AutoDiff.dll"
#r "alglib400gpl_net2.dll"

#region Usings
using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
#endregion
using AutoDiff;

// ============================================================================
// C1ArcOptimizer.cs — second-stage refinement of a shared-endpoint arc chain.
//
// Variables are the shared control net from CurveToG0Arcs (joints + mids),
// arc i using control indices {2i, 2i+1, (2i+2) mod NcPt}. C0 stays exact by
// construction (shared joints); this stage improves G1, closeness to the original curve, and closed-chain frame twist.
//
// The report measures the returned arcs against the original curve. It uses the
// same RMS distance, RMS joint angle and closed-loop frame angle as the paper.
//
// Inputs:
//   C                 Curve       target for closeness and for open/closed detection (C.IsClosed)
//   iConPts           Point3d[]   alternating joint and midpoint for each arc.
//                                An open chain ends with one additional joint.
//   maxIter_C1        int         inner CG iteration cap (default 200 if <= 0)
//   iwC1Continuity    double      G1 weight
//   iwCloseness       double      STARTING closeness weight, annealed to 0
//   iwTwist           double      twist weight (closed only)
//   iwMidCenter       double      mid-point centering weight (0 => off)
//
// Outputs:
//   ocirclePts        Point3d[K,3]  refined {start, mid, end} per arc
//   oReport           string        fit, joint smoothness, closed-loop frame mismatch and optimizer time
// ============================================================================

public class Script_Instance : GH_ScriptInstance
{
    private void RunScript(
		Curve C,
		List<Point3d> iConPts,
		int maxIter_C1,
		double iwC1Continuity,
		double iwCloseness,
		double iwTwist,
		double iwMidCenter,
		ref object ocirclePts,
		ref object oReport)
    {
        ocirclePts = null;
        oReport = null;
        if (C == null || !C.IsValid) { Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Invalid curve."); return; }
        isClosedPolyline = C.IsClosed;
        int NcPt = iConPts == null ? 0 : iConPts.Count;
        int pairedPointCount = NcPt - (isClosedPolyline ? 0 : 1);
        if (NcPt < 3 || pairedPointCount % 2 != 0)
        {
            Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, isClosedPolyline
                ? "A closed curve needs an even number of control points (at least 4). Supply one joint and midpoint per arc, omitting the repeated starting joint."
                : "An open curve needs an odd number of control points (at least 3). Supply one joint and midpoint per arc, then the final endpoint.");
            return;
        }
        int K = pairedPointCount / 2;

        // ---- timing ------------------------------------------------------------
        // System.Diagnostics is fully qualified so the usings block stays untouched.
        var swTotal = System.Diagnostics.Stopwatch.StartNew();
        double tSolve = 0.0;

        targetCurve      = C;
        arcCount         = K;
        wC1Continuity    = iwC1Continuity;
        wCloseness       = iwCloseness;
        wTwist           = iwTwist;
        wMidCenter       = iwMidCenter;

        // control points -> flat variable vector
        Point3d[] cPts = iConPts.ToArray();
        double[] vars = new double[3 * NcPt];
        for (int i = 0; i < NcPt; i++) { vars[3*i] = cPts[i].X; vars[3*i+1] = cPts[i].Y; vars[3*i+2] = cPts[i].Z; }

        // arc -> control-point index triples (shared net; wraps when closed)
        iCirclePts_new = new int[K, 3];
        for (int i = 0; i < K; i++) { iCirclePts_new[i,0] = 2*i; iCirclePts_new[i,1] = 2*i+1; iCirclePts_new[i,2] = (2*i+2) % NcPt; }

        // ---- characteristic length for CG variable scaling ----
        double Lc = 0.0; { int cc = 0;
            for (int i = 0; i + 1 < cPts.Length; i++) { Lc += cPts[i].DistanceTo(cPts[i+1]); cc++; }
            Lc = (cc > 0 && Lc > 1e-12) ? Lc / cc : 1.0; }

        double[] scaleVec = new double[3 * NcPt];
        for (int k = 0; k < scaleVec.Length; k++) scaleVec[k] = Lc;

        // ---- outer majorize-minimize loop with CLOSENESS ANNEALING ----
        int innerIters   = maxIter_C1 > 0 ? maxIter_C1 : 200;
        int annealRounds = 3;
        int settleRounds = 3;
        double annealPow = 1.0;
        double epsg = 1e-10, epsf = 0.0, epsx = 0.0;

        double wClose0 = iwCloseness;

        alglib.mincgstate state;
        alglib.mincgreport rep = new alglib.mincgreport();

        int totalIters = 0;
        int roundsRun  = 0;

        var swSolve = System.Diagnostics.Stopwatch.StartNew();
        for (int round = 0; round < annealRounds + settleRounds; round++)
        {
            double frac;
            if (annealRounds <= 1)            frac = (round == 0) ? 1.0 : 0.0;
            else if (round < annealRounds)    frac = Math.Pow(1.0 - (double)round / (annealRounds - 1), annealPow);
            else                              frac = 0.0;
            wCloseness = wClose0 * Math.Max(0.0, frac);

            double[] prev = (double[])vars.Clone();
            FreezeMajorizer(vars, NcPt);   // feet, tangents, reflex flags
            alglib.mincgcreate(3 * NcPt, vars, out state);
            alglib.mincgsetscale(state, scaleVec);
            alglib.mincgsetprecscale(state);
            alglib.mincgsetcond(state, epsg, epsf, epsx, innerIters);
            alglib.mincgoptimize(state, function_grad, null, null);
            alglib.mincgresults(state, out vars, out rep);

            totalIters += rep.iterationscount;
            roundsRun++;

            double maxMove = 0.0;
            for (int k = 0; k < vars.Length; k++) maxMove = Math.Max(maxMove, Math.Abs(vars[k] - prev[k]));

            // terminationtype: 4 grad converged, 5 maxits, 2/7 step stall,
            // -8 NaN/Inf gradient (degenerate arc -> see mid-centering).
            Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                "round " + round + ": iters=" + rep.iterationscount +
                " term=" + rep.terminationtype +
                " maxMove=" + maxMove.ToString("E2") +
                " wClose=" + wCloseness.ToString("G3"));
        }
        swSolve.Stop(); tSolve = swSolve.Elapsed.TotalSeconds;

        Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
            "MM: " + roundsRun + " round(s), " + totalIters + " inner iterations; closeness " +
            wClose0.ToString("G3") + " -> 0 over " + annealRounds + "; Lc=" + Lc.ToString("G3") +
            "; solve " + tSolve.ToString("F3") + " s.");

        // ---- reconstruct refined arcs ----
        Point3d[] cPts2 = new Point3d[NcPt];
        for (int i = 0; i < NcPt; i++) cPts2[i] = new Point3d(vars[3*i], vars[3*i+1], vars[3*i+2]);

        Point3d[,] circlePts = new Point3d[K, 3];
        for (int i = 0; i < K; i++)
        {
            circlePts[i,0] = cPts2[iCirclePts_new[i,0]];
            circlePts[i,1] = cPts2[iCirclePts_new[i,1]];
            circlePts[i,2] = cPts2[iCirclePts_new[i,2]];
        }

        ocirclePts = circlePts;

        const int samplesPerArc = 16;
        int sampleCount;
        double rmsDistance = MeasureCurveFit(circlePts, samplesPerArc, out sampleCount);
        int measuredJoints, skippedJoints;
        double rmsJointAngle = MeasureJointAngle(circlePts, out measuredJoints, out skippedJoints);
        int expectedSamples = K * samplesPerArc + (isClosedPolyline ? 0 : 1);

        var report = new System.Text.StringBuilder();
        report.AppendLine((isClosedPolyline ? "Closed curve" : "Open curve") + " · " +
            K + (K == 1 ? " arc" : " arcs"));
        report.AppendLine("Root-mean-square errors");
        if (sampleCount > 0)
        {
            report.AppendLine("  Sampled distance to input curve    " + FormatValue(rmsDistance) + " model units");
            BoundingBox box = targetCurve.GetBoundingBox(true);
            if (box.IsValid && box.Diagonal.Length > 1e-12)
                report.AppendLine("    " + FormatValue(100.0 * rmsDistance / box.Diagonal.Length) +
                    "% of the input curve's bounding-box diagonal");
            if (sampleCount < expectedSamples)
                report.AppendLine("    Measured " + sampleCount + " of " + expectedSamples + " sample points.");
        }
        else
        {
            report.AppendLine("  Sampled distance to input curve    Unavailable (no valid samples)");
        }

        if (measuredJoints > 0)
            report.AppendLine("  Angle between joint tangents    " + FormatValue(rmsJointAngle) + "° (0° = smooth)");
        else if (skippedJoints == 0)
            report.AppendLine("  Angle between joint tangents    Not applicable (no joints)");
        else
            report.AppendLine("  Angle between joint tangents    Unavailable (no valid tangents)");
        if (measuredJoints > 0 && skippedJoints > 0)
            report.AppendLine("    Measured " + measuredJoints + " of " + (measuredJoints + skippedJoints) + " joints.");

        if (isClosedPolyline)
        {
            double degrees;
            report.AppendLine("Frame mismatch after one loop    " +
                (TryMeasureFrameMismatch(circlePts, out degrees)
                    ? FormatValue(degrees) + "° (0° = aligned)"
                    : "Unavailable (no valid frame around the loop)"));
        }

        swTotal.Stop();
        report.Append("Optimizer time    " + FormatValue(swTotal.Elapsed.TotalSeconds) + " s");
        oReport = report.ToString();
    }


    // <Custom additional code> ---------------------------------------------------

    Curve targetCurve;
    int[,] iCirclePts_new;
    int arcCount;
    bool isClosedPolyline;
    double wC1Continuity, wCloseness, wTwist, wMidCenter;

    // frozen majorizer state (held fixed across each inner solve)
    Point3d[] frozenFootCP; Vector3d[] frozenTanCP;
    Point3d[,] frozenArcFoot; Vector3d[,] frozenArcTan;
    bool[]   frozenReflex;         // per-arc: true if the sweep exceeds pi
    // Four significant digits keep small angular errors visible without cluttering the panel.
    static string FormatValue(double value)
    {
        return value.ToString("G4", System.Globalization.CultureInfo.InvariantCulture);
    }

    double DistToTarget(Point3d p)
    {
        double t;
        if (!p.IsValid || !targetCurve.ClosestPoint(p, out t)) return double.NaN;
        Point3d foot = targetCurve.PointAt(t);
        return foot.IsValid ? p.DistanceTo(foot) : double.NaN;
    }

    // Sample at equal arc lengths and count each shared endpoint once.
    double MeasureCurveFit(Point3d[,] cp, int nSamp, out int nPts)
    {
        int K = arcCount;
        double sumSq = 0.0;
        nPts = 0;

        int m = Math.Max(2, nSamp);
        for (int i = 0; i < K; i++)
        {
            Arc a = new Arc(cp[i,0], cp[i,1], cp[i,2]);
            if (!a.IsValid) continue;
            Curve ac = a.ToNurbsCurve();
            if (ac == null) continue;

            double[] ts = ac.DivideByCount(m, true);   // m+1 params, both ends
            if (ts == null) continue;

            for (int s = 0; s < ts.Length - 1; s++)     // drop the shared end joint
            {
                double d = DistToTarget(ac.PointAt(ts[s]));
                if (double.IsNaN(d)) continue;
                sumSq += d * d; nPts++;
            }
        }

        if (!isClosedPolyline && K > 0)
        {
            double d = DistToTarget(cp[K-1,2]);
            if (!double.IsNaN(d)) { sumSq += d * d; nPts++; }
        }

        return nPts > 0 ? Math.Sqrt(sumSq / nPts) : double.NaN;
    }

    // Measure both tangents in the direction of travel. Include the seam for closed curves.
    double MeasureJointAngle(Point3d[,] cp, out int nJoint, out int nBad)
    {
        int K = arcCount;
        double sumSq = 0.0;
        nJoint = 0; nBad = 0;

        for (int i = 0; i < K; i++)
        {
            if (!isClosedPolyline && i == K - 1) continue;
            int j = (i + 1) % K;

            Arc a1 = new Arc(cp[i,0], cp[i,1], cp[i,2]);
            Arc a2 = new Arc(cp[j,0], cp[j,1], cp[j,2]);
            if (!a1.IsValid || !a2.IsValid) { nBad++; continue; }

            Curve c1 = a1.ToNurbsCurve(), c2 = a2.ToNurbsCurve();
            if (c1 == null || c2 == null) { nBad++; continue; }

            // both arcs run start -> mid -> end, so these are the two travel
            // directions the G1 term compares
            Vector3d t1 = c1.TangentAtEnd, t2 = c2.TangentAtStart;
            if (!t1.Unitize() || !t2.Unitize()) { nBad++; continue; }

            double alpha = Math.Acos(Math.Max(-1.0, Math.Min(1.0, t1 * t2)));
            sumSq += alpha * alpha;
            nJoint++;
        }

        return nJoint > 0 ? Math.Sqrt(sumSq / nJoint) * 180.0 / Math.PI : double.NaN;
    }

    // Compare the starting frame direction with the same direction transported
    // around the entire chain, using each final arc's actual sweep.
    bool TryMeasureFrameMismatch(Point3d[,] cp, out double phiDeg)
    {
        phiDeg = double.NaN;
        int K = arcCount;

        Vector3d V = Vector3d.Unset, V0 = Vector3d.Unset;

        for (int i = 0; i < K; i++)
        {
            Arc a = new Arc(cp[i,0], cp[i,1], cp[i,2]);
            if (!a.IsValid || a.Radius <= 1e-12) return false;

            Vector3d n = Vector3d.CrossProduct(cp[i,1] - cp[i,0], cp[i,2] - cp[i,0]);
            if (!n.Unitize()) return false;

            if (i == 0) { V = n; V0 = n; continue; }   // arc 0 seeds the frame, as in the term

            double th = a.Angle, c = Math.Cos(th), s = Math.Sin(th);
            V = V * c + Vector3d.CrossProduct(n, V) * s + n * (n * V) * (1.0 - c);
        }

        if (!V0.IsValid || !V.Unitize()) return false;

        double phi = Math.Acos(Math.Max(-1.0, Math.Min(1.0, V0 * V)));
        phiDeg = phi * 180.0 / Math.PI;
        return true;
    }

    // ---- freeze feet / tangents / reflex flags at the current vars ----
    void FreezeMajorizer(double[] vars, int NcPt)
    {
        var cPts = new Point3d[NcPt];
        for (int i = 0; i < NcPt; i++) cPts[i] = new Point3d(vars[3*i], vars[3*i+1], vars[3*i+2]);

        frozenFootCP = new Point3d[NcPt];
        frozenTanCP  = new Vector3d[NcPt];
        for (int j = 0; j < NcPt; j++)
        {
            double t; targetCurve.ClosestPoint(cPts[j], out t);
            frozenFootCP[j] = targetCurve.PointAt(t);
            Vector3d T = targetCurve.TangentAt(t); T.Unitize();
            frozenTanCP[j] = T;
        }

        int K = arcCount;
        frozenArcFoot = new Point3d[K, 2];
        frozenArcTan  = new Vector3d[K, 2];
        frozenReflex  = new bool[K];
        for (int i = 0; i < K; i++)
        {
            Point3d A = cPts[iCirclePts_new[i,0]];
            Point3d B = cPts[iCirclePts_new[i,1]];
            Point3d D = cPts[iCirclePts_new[i,2]];
            Arc arc = new Arc(A, B, D);
            Point3d O = arc.Center; double R = arc.Radius;

            bool ok = arc.IsValid && R > 1e-12 && !double.IsNaN(R) && !double.IsInfinity(R);

            // interior samples, matching the AD sub-arc-midpoint construction
            Point3d q0 = SubArcMid(A, B, O, R);   // ~1/4
            Point3d q1 = SubArcMid(B, D, O, R);   // ~3/4
            Point3d[] qs = { q0, q1 };
            for (int h = 0; h < 2; h++)
            {
                double t; targetCurve.ClosestPoint(qs[h], out t);
                frozenArcFoot[i,h] = targetCurve.PointAt(t);
                Vector3d T = targetCurve.TangentAt(t); T.Unitize();
                frozenArcTan[i,h] = T;
            }

            // TRUE sweep (Rhino handles the major arc); the AD graph gets only the
            // minor angle from acos, so record which branch it must be flipped to.
            double arcAngle = ok ? arc.Angle : 0.0;
            frozenReflex[i] = ok && (arcAngle > Math.PI);
        }
    }

    // midpoint of the (shorter) sub-arc P->Q on the circle center O, radius R.
    // Each half-arc sweeps < pi whenever the full arc sweeps < 2pi, so the
    // bisector construction picks the right side even for a reflex arc.
    Point3d SubArcMid(Point3d P, Point3d Q, Point3d O, double R)
    {
        Vector3d bis = (P - O) + (Q - O);
        if (bis.Length < 1e-12) return P;
        bis.Unitize();
        return O + bis * R;
    }

    public void function_grad(double[] inputVars, ref double func, double[] grad, object obj)
    {
        int NcPt = inputVars.Length / 3;
        int K = arcCount;

        var cPts = new Point3d[NcPt];
        for (int i = 0; i < NcPt; i++) cPts[i] = new Point3d(inputVars[3*i], inputVars[3*i+1], inputVars[3*i+2]);

        var circlePts = new Point3d[K, 3];
        for (int i = 0; i < K; i++)
        {
            circlePts[i,0] = cPts[iCirclePts_new[i,0]];
            circlePts[i,1] = cPts[iCirclePts_new[i,1]];
            circlePts[i,2] = cPts[iCirclePts_new[i,2]];
        }

        var moveVertor = new Vector3d[NcPt];

        var acos = UnaryFunc.Factory(
            x => Math.Acos(Math.Max(-1 + 1e-15, Math.Min(x, 1 - 1e-15))),
            x => -1.0 / Math.Sqrt(Math.Max(1 - x * x, 1e-15)));

        var sin = UnaryFunc.Factory(x => Math.Sin(x), x => Math.Cos(x));

        // ======== 1. G1 continuity =========
        double funcValue1 = 0.0;
        if (wC1Continuity > 0)
        {
            for (int i = 0; i < K; i++)
            {
                if (!isClosedPolyline && (i == K - 1)) continue;

                Point3d pA = circlePts[i,0], pB = circlePts[i,1], pC = circlePts[i,2];
                Point3d pD = circlePts[(i+1)%K, 1], pE = circlePts[(i+1)%K, 2];

                var x1 = new Variable(); var y1 = new Variable(); var z1 = new Variable();
                var x2 = new Variable(); var y2 = new Variable(); var z2 = new Variable();
                var x3 = new Variable(); var y3 = new Variable(); var z3 = new Variable();
                var x4 = new Variable(); var y4 = new Variable(); var z4 = new Variable();
                var x5 = new Variable(); var y5 = new Variable(); var z5 = new Variable();

                TVec A = new TVec(x1,y1,z1), B = new TVec(x2,y2,z2), Cc = new TVec(x3,y3,z3);
                TVec D = new TVec(x4,y4,z4), E = new TVec(x5,y5,z5);

                TVec AC = Cc - A, AB = B - A;
                TVec normalV = TVec.CrossProduct(AB, AC);
                TVec toO1 = (TVec.CrossProduct(normalV, AB) * AC.NormSquared + TVec.CrossProduct(AC, normalV) * AB.NormSquared)
                            * TermBuilder.Power(normalV.NormSquared, -1) * 0.5;
                TVec O1 = A + toO1;
                TVec arc1EndV = -TVec.CrossProduct(Cc - O1, normalV) * TermBuilder.Power(TVec.CrossProduct(Cc - O1, normalV).NormSquared, -0.5);

                TVec CE = E - Cc, CD = D - Cc;
                TVec normalV2 = TVec.CrossProduct(CD, CE);
                TVec toO2 = (TVec.CrossProduct(normalV2, CD) * CE.NormSquared + TVec.CrossProduct(CE, normalV2) * CD.NormSquared)
                            * TermBuilder.Power(normalV2.NormSquared, -1) * 0.5;
                TVec O2 = Cc + toO2;
                TVec arc2StartV = -TVec.CrossProduct(Cc - O2, normalV2) * TermBuilder.Power(TVec.CrossProduct(Cc - O2, normalV2).NormSquared, -0.5);

                Variable[] vs = { x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5 };
                double[] vl = { pA.X,pA.Y,pA.Z, pB.X,pB.Y,pB.Z, pC.X,pC.Y,pC.Z, pD.X,pD.Y,pD.Z, pE.X,pE.Y,pE.Z };

                var func1 = TermBuilder.Power(acos(arc1EndV * arc2StartV), 2) * wC1Continuity;
                funcValue1 += func1.Evaluate(vs, vl);
                double[] g = func1.Differentiate(vs, vl);

                moveVertor[iCirclePts_new[i,0]]       += new Vector3d(g[0],g[1],g[2]);
                moveVertor[iCirclePts_new[i,1]]       += new Vector3d(g[3],g[4],g[5]);
                moveVertor[iCirclePts_new[i,2]]       += new Vector3d(g[6],g[7],g[8]);
                moveVertor[iCirclePts_new[(i+1)%K,1]] += new Vector3d(g[9],g[10],g[11]);
                moveVertor[iCirclePts_new[(i+1)%K,2]] += new Vector3d(g[12],g[13],g[14]);
            }
        }

        // ===== 4. closeness to the CURVE (frozen feet, point-to-tangent) =====
        double funcValue4 = 0.0;
        if (wCloseness > 0)
        {
            for (int j = 0; j < NcPt; j++)
            {
                Point3d P = cPts[j];
                Point3d F = frozenFootCP[j];
                Vector3d T = frozenTanCP[j];
                Vector3d d = F - P;
                Vector3d dperp = d - (d * T) * T;
                funcValue4 += (dperp * dperp) * wCloseness;
                moveVertor[j] += -2.0 * wCloseness * dperp;
            }

            for (int i = 0; i < K; i++)
            {
                Point3d pA = circlePts[i,0], pB = circlePts[i,1], pC = circlePts[i,2];

                for (int h = 0; h < 2; h++)
                {
                    var x1 = new Variable(); var y1 = new Variable(); var z1 = new Variable();
                    var x2 = new Variable(); var y2 = new Variable(); var z2 = new Variable();
                    var x3 = new Variable(); var y3 = new Variable(); var z3 = new Variable();
                    TVec A = new TVec(x1,y1,z1), B = new TVec(x2,y2,z2), Cc = new TVec(x3,y3,z3);

                    TVec AC = Cc - A, AB = B - A;
                    TVec normalV = TVec.CrossProduct(AB, AC);
                    TVec toO = (TVec.CrossProduct(normalV, AB) * AC.NormSquared + TVec.CrossProduct(AC, normalV) * AB.NormSquared)
                               * TermBuilder.Power(normalV.NormSquared, -1) * 0.5;
                    TVec O = A + toO;
                    Term R = TermBuilder.Power(new TVec(A - O).NormSquared, 0.5);

                    TVec uP = (h == 0) ? (A - O) : (B - O);
                    TVec uQ = (h == 0) ? (B - O) : (Cc - O);
                    TVec sumPQ = uP + uQ;
                    TVec dir = sumPQ * TermBuilder.Power(sumPQ.NormSquared, -0.5);
                    TVec Ppt = O + dir * R;

                    Point3d F = frozenArcFoot[i,h];
                    Vector3d T = frozenArcTan[i,h];
                    TVec dvec = new TVec(F.X, F.Y, F.Z) - Ppt;
                    Term dT = dvec.X * T.X + dvec.Y * T.Y + dvec.Z * T.Z;
                    TVec dperp = dvec - new TVec(T.X, T.Y, T.Z) * dT;
                    var func4 = dperp.NormSquared * wCloseness;

                    Variable[] vs = { x1,y1,z1, x2,y2,z2, x3,y3,z3 };
                    double[] vl = { pA.X,pA.Y,pA.Z, pB.X,pB.Y,pB.Z, pC.X,pC.Y,pC.Z };

                    funcValue4 += func4.Evaluate(vs, vl);
                    double[] g = func4.Differentiate(vs, vl);
                    moveVertor[iCirclePts_new[i,0]] += new Vector3d(g[0],g[1],g[2]);
                    moveVertor[iCirclePts_new[i,1]] += new Vector3d(g[3],g[4],g[5]);
                    moveVertor[iCirclePts_new[i,2]] += new Vector3d(g[6],g[7],g[8]);
                }
            }
        }

        // ====== 5. twist (closed only) ======
        double funcValue5 = 0.0;
        if (wTwist > 0 && isClosedPolyline)
        {
            Variable[] vs = new Variable[NcPt * 3];
            for (int i = 0; i < NcPt * 3; i++) vs[i] = new Variable();
            double[] vl = new double[3 * NcPt];
            for (int i = 0; i < NcPt; i++) { vl[3*i] = cPts[i].X; vl[3*i+1] = cPts[i].Y; vl[3*i+2] = cPts[i].Z; }

            TVec V = new TVec(0.0, 0.0, 0.0), Vini = new TVec(0.0, 0.0, 0.0);

            for (int i = 0; i < K; i++)
            {
                var x1 = vs[iCirclePts_new[i,0]*3]; var y1 = vs[iCirclePts_new[i,0]*3+1]; var z1 = vs[iCirclePts_new[i,0]*3+2];
                var x2 = vs[iCirclePts_new[i,1]*3]; var y2 = vs[iCirclePts_new[i,1]*3+1]; var z2 = vs[iCirclePts_new[i,1]*3+2];
                var x3 = vs[iCirclePts_new[i,2]*3]; var y3 = vs[iCirclePts_new[i,2]*3+1]; var z3 = vs[iCirclePts_new[i,2]*3+2];
                TVec A = new TVec(x1,y1,z1), B = new TVec(x2,y2,z2), Cc = new TVec(x3,y3,z3);

                TVec AC = Cc - A, AB = B - A;
                TVec normalV = TVec.CrossProduct(AB, AC);
                TVec toO = (TVec.CrossProduct(normalV, AB) * AC.NormSquared + TVec.CrossProduct(AC, normalV) * AB.NormSquared)
                           * TermBuilder.Power(normalV.NormSquared, -1) * 0.5;
                TVec O = A + toO;
                Term R2 = new TVec(A - O).NormSquared;

                normalV = normalV * TermBuilder.Power(normalV.NormSquared, -0.5);
                TVec AO = (A - O) * TermBuilder.Power(R2, -0.5);
                TVec CO = (Cc - O) * TermBuilder.Power(R2, -0.5);
                Term cosAngle = AO * CO;

                if (i == 0) { V = normalV; Vini = normalV; }
                else
                {
                    // REFLEX BRANCH. The transport angle is the TRUE
                    // sweep: cos(2pi - a) = cos(a), so cosAngle is already right, but
                    // sin(2pi - a) = -sin(a). Without the flip a reflex arc rotates the
                    // frame backwards and F_T chases a holonomy the geometry does not
                    // have. The flag is frozen, so the term stays smooth.
                    Term sinTh = sin(acos(cosAngle));
                    if (frozenReflex != null && i < frozenReflex.Length && frozenReflex[i]) sinTh = sinTh * (-1.0);
                    V = V * cosAngle + TVec.CrossProduct(normalV, V) * sinTh + normalV * (normalV * V) * (1 - cosAngle);
                }
            }

            var func5 = TermBuilder.Power(acos(Vini * V), 2) * wTwist;
            funcValue5 = func5.Evaluate(vs, vl);
            double[] g = func5.Differentiate(vs, vl);
            for (int i = 0; i < NcPt; i++) moveVertor[i] += new Vector3d(g[3*i], g[3*i+1], g[3*i+2]);
        }

        // ====== 6. mid-point centering (kill the slide gauge) ======
        double funcValue6 = 0.0;
        if (wMidCenter > 0)
        {
            for (int i = 0; i < K; i++)
            {
                Point3d pA = circlePts[i,0], pB = circlePts[i,1], pC = circlePts[i,2];

                var x1 = new Variable(); var y1 = new Variable(); var z1 = new Variable();
                var x2 = new Variable(); var y2 = new Variable(); var z2 = new Variable();
                var x3 = new Variable(); var y3 = new Variable(); var z3 = new Variable();
                TVec A = new TVec(x1,y1,z1), B = new TVec(x2,y2,z2), Cc = new TVec(x3,y3,z3);

                Term dAB = (B - A).NormSquared;
                Term dCB = (Cc - B).NormSquared;
                var func6 = TermBuilder.Power(dAB - dCB, 2) * wMidCenter;

                Variable[] vs = { x1,y1,z1, x2,y2,z2, x3,y3,z3 };
                double[] vl = { pA.X,pA.Y,pA.Z, pB.X,pB.Y,pB.Z, pC.X,pC.Y,pC.Z };

                funcValue6 += func6.Evaluate(vs, vl);
                double[] g = func6.Differentiate(vs, vl);
                moveVertor[iCirclePts_new[i,0]] += new Vector3d(g[0],g[1],g[2]);
                moveVertor[iCirclePts_new[i,1]] += new Vector3d(g[3],g[4],g[5]);
                moveVertor[iCirclePts_new[i,2]] += new Vector3d(g[6],g[7],g[8]);
            }
        }

        // ---- assemble func + grad ----
        func = funcValue1 + funcValue4 + funcValue5 + funcValue6;

        for (int i = 0; i < NcPt; i++)
        {
            grad[3*i]   = moveVertor[i].X;
            grad[3*i+1] = moveVertor[i].Y;
            grad[3*i+2] = moveVertor[i].Z;
        }
    }
}
