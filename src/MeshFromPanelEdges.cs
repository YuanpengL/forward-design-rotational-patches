using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;

public class Script_Instance : GH_ScriptInstance
{
    // One branch contains the edges of one mesh. The default maximum valence is four.
    private void RunScript(List<Curve> L, int V, ref object O)
    {
        O = null;
        if (L == null || L.Count == 0) return;
        if (V < 3)
        {
            Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "A mesh face needs at least three edges.");
            return;
        }

        var edges = new List<Curve>(L.Count);
        foreach (Curve edge in L)
        {
            if (edge != null && edge.IsValid) edges.Add(edge);
        }
        if (edges.Count == 0) return;

        double tolerance = Grasshopper.Utility.DocumentTolerance();
        Mesh mesh = BuildMesh(edges.ToArray(), V, tolerance);
        if (mesh == null || mesh.Faces.Count == 0)
        {
            Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "The supplied edges do not form any mesh faces.");
            return;
        }
        OrientFrameMesh(mesh);
        O = mesh;
    }

    private static Mesh BuildMesh(Curve[] edges, int valence, double tolerance)
    {
        // Rhino can fail to resolve a valid network in one traversal order. A second
        // pass from the other end retains the same edges and coincidence tolerance.
        for (int attempt = 0; attempt < 2; attempt++)
        {
            try
            {
                Mesh mesh = Mesh.CreateFromLines(edges, valence, tolerance);
                if (mesh != null && mesh.Faces.Count > 0) return mesh;
            }
            catch (NullReferenceException)
            {
                // Rhino 8 can throw here while discarding non-manifold faces.
            }
            if (attempt == 0) Array.Reverse(edges);
        }
        return null;
    }

    // Keep the side used for the signed frame offset, independent of input line directions.
    // The reference winding starts each face at its lowest vertex index and visits the
    // lower-numbered neighbour next. The first face anchors each connected component.
    private static bool InReferenceOrder(MeshFace face)
    {
        int count = face.IsTriangle ? 3 : 4;
        int first = 0;
        for (int i = 1; i < count; i++)
            if (face[i] < face[first]) first = i;
        return face[(first + 1) % count] < face[(first + count - 1) % count];
    }

    private static void OrientFrameMesh(Mesh mesh)
    {
        int count = mesh.Faces.Count;
        var visited = new bool[count];
        var reverse = new bool[count];
        int opposed = 0;
        for (int first = 0; first < count; first++)
        {
            if (visited[first]) continue;
            bool seedInReferenceOrder = InReferenceOrder(mesh.Faces[first]);
            bool flip = first == 0 ? seedInReferenceOrder : !seedInReferenceOrder;
            foreach (int face in mesh.Faces.GetConnectedFaces(first))
            {
                visited[face] = true;
                reverse[face] = flip;
                if (InReferenceOrder(mesh.Faces[face]) == flip) opposed++;
            }
        }

        // If the whole mesh would point away from its reference winding, use the
        // consistent reference side instead. Preserve face and vertex indices.
        bool allOpposed = opposed == count;
        for (int i = 0; i < count; i++)
        {
            if (reverse[i] == allOpposed) continue;
            MeshFace face = mesh.Faces[i];
            mesh.Faces.SetFace(i, face.IsTriangle
                ? new MeshFace(face.A, face.C, face.B)
                : new MeshFace(face.A, face.D, face.C, face.B));
        }
        mesh.FaceNormals.ComputeFaceNormals();
        mesh.Normals.ComputeNormals();
        if (mesh.IsClosed && HasInwardNormals(mesh)) mesh.Flip(true, true, true);
    }

    private static bool HasInwardNormals(Mesh mesh)
    {
        // Sum signed tetrahedral volumes around a local origin. Division by six
        // is unnecessary because only the sign determines the outward side.
        Point3d origin = mesh.GetBoundingBox(false).Center;
        double volume = 0;
        foreach (MeshFace f in mesh.Faces)
        {
            Vector3d a = mesh.Vertices.Point3dAt(f.A) - origin;
            Vector3d b = mesh.Vertices.Point3dAt(f.B) - origin;
            Vector3d c = mesh.Vertices.Point3dAt(f.C) - origin;
            volume += a * Vector3d.CrossProduct(b, c);
            if (f.IsQuad)
            {
                Vector3d d = mesh.Vertices.Point3dAt(f.D) - origin;
                volume += a * Vector3d.CrossProduct(c, d);
            }
        }
        return volume < 0;
    }
}
