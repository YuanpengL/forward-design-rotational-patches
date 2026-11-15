using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;

public class Script_Instance : GH_ScriptInstance
{
    private void RunScript(Curve C, double Radius, List<double> U, List<double> V, ref object P)
    {
        P = null;
        if (C == null || !C.IsValid || !(Radius > 0.0) ||
            U == null || U.Count < 2 || V == null || V.Count < 2) return;

        // U measures distance along the rail. V measures turns around each section.
        double[] stations = C.NormalizedLengthParameters(U.ToArray(), 0.0);
        if (stations == null) return;
        var tube = new Mesh();
        int width = V.Count;
        for (int i = 0; i < stations.Length; i++)
        {
            // Transport each section from the same start frame.
            bool atStart = stations[i] == C.Domain.T0;
            var span = new[] { C.Domain.T0, atStart ? C.Domain.T1 : stations[i] };
            Plane[] frames = C.GetPerpendicularFrames(span);
            if (frames == null || frames.Length != 2) return;
            Plane frame = frames[atStart ? 0 : 1];
            var section = new Circle(frame, Radius);
            for (int j = 0; j < width; j++)
            {
                double turn = V[j] == 1.0 ? 0.0 : V[j];
                double angle = 0.25 * Math.PI + 2.0 * Math.PI * turn;
                tube.Vertices.Add(section.PointAt(angle));
                if (i > 0 && j > 0)
                {
                    int corner = i * width + j;
                    tube.Faces.AddFace(corner - width - 1, corner - width, corner, corner - 1);
                }
            }
        }
        tube.Vertices.CombineIdentical(true, true);
        tube.Faces.CullDegenerateFaces();
        tube.Vertices.CullUnused();
        tube.Normals.ComputeNormals();
        tube.Compact();
        if (tube.Faces.Count > 0 && tube.IsValid) P = tube;
    }
}
