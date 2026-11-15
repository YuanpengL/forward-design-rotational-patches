using System;
using System.Collections.Generic;
using System.Drawing;
using Grasshopper.Kernel;
using Rhino.Geometry;

public class Script_Instance : GH_ScriptInstance
{
    private struct EdgeStyle
    {
        public Curve Edge;
        public Point3d Center;
        public Color Color;
        public double Thickness;
        public bool ModelUnits;
    }

    private readonly List<EdgeStyle> edges = new List<EdgeStyle>();
    private BoundingBox bounds = BoundingBox.Empty;

    public override void BeforeRunScript()
    {
        edges.Clear();
        bounds = BoundingBox.Empty;
    }

    // Item access lets Grasshopper match each edge with its swatch and line width.
    private void RunScript(Curve G, Color S, double T, bool A)
    {
        if (G == null || !G.IsValid) return;
        BoundingBox box = G.GetBoundingBox(false);
        bounds.Union(box);
        edges.Add(new EdgeStyle { Edge = G, Center = box.Center, Color = S, Thickness = T, ModelUnits = A });
    }

    public override BoundingBox ClippingBox => bounds;

    public override void DrawViewportWires(IGH_PreviewArgs args)
    {
        bool selected = Component.Attributes.Selected;
        foreach (EdgeStyle edge in edges)
        {
            double width = edge.Thickness > 0 ? edge.Thickness : args.DefaultCurveThickness;
            double pixelsPerUnit;
            if (edge.ModelUnits && args.Viewport.GetWorldToScreenScale(edge.Center, out pixelsPerUnit))
                width *= pixelsPerUnit;
            args.Display.DrawCurve(edge.Edge, selected ? args.WireColour_Selected : edge.Color, (int)Math.Max(1, width));
        }
    }
}
