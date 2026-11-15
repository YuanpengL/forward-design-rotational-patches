using System;
using Grasshopper.Kernel;
using Rhino.Geometry;

public class Script_Instance : GH_ScriptInstance
{
    private void RunScript(Curve Pl, ref object M)
    {
        M = null;
        if (Pl == null || !Pl.TryGetPolyline(out Polyline boundary) || boundary.Count < 3)
            return;
        if (!boundary.IsClosed) boundary.Add(boundary[0]);

        Mesh face = Mesh.CreateFromClosedPolyline(boundary);
        if (face == null) return;
        face.Faces.ConvertTrianglesToQuads(Math.PI / 90.0, 0.0);
        face.Vertices.CombineIdentical(true, true);
        face.UnifyNormals();
        face.Normals.ComputeNormals();
        face.FaceNormals.ComputeFaceNormals();
        face.Compact();
        M = face;
    }
}
