// Grasshopper Script Instance
using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;

// Geo uses List access. Size sets the longest side of the combined world-aligned
// bounding box. The same transform centers and scales every item in the list.
public class Script_Instance : GH_ScriptInstance
{
    private void RunScript(List<GeometryBase> Geo, double Size, ref object oCrv)
    {
        oCrv = null;
        if (Geo == null || Geo.Count == 0)
        {
            Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "No geometry.");
            return;
        }

        BoundingBox bounds = BoundingBox.Empty;
        foreach (GeometryBase item in Geo)
        {
            if (item == null) continue;
            BoundingBox itemBounds = item.GetBoundingBox(true);
            if (!itemBounds.IsValid) continue;
            bounds = bounds.IsValid ? BoundingBox.Union(bounds, itemBounds) : itemBounds;
        }
        if (!bounds.IsValid)
        {
            Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "No valid bounding box.");
            return;
        }

        Vector3d span = bounds.Diagonal;
        double longest = Math.Max(span.X, Math.Max(span.Y, span.Z));
        Transform transform = Transform.Translation(Point3d.Origin - bounds.Center);
        if (longest < 1e-12)
        {
            Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Warning,
                "The bounding box has negligible extent. The geometry was centered without scaling.");
        }
        else
        {
            double target = Size > 0.0 ? Size : 1.0;
            double factor = target / longest;
            transform = Transform.Scale(Point3d.Origin, factor) * transform;
            Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                string.Format("Longest side {0:G4} → {1:G4} (scale {2:G4})", longest, target, factor));
        }

        var result = new List<GeometryBase>(Geo.Count);
        foreach (GeometryBase item in Geo)
        {
            GeometryBase copy = item?.Duplicate();
            if (copy != null) copy.Transform(transform);
            result.Add(copy);
        }

        oCrv = result;
    }
}
