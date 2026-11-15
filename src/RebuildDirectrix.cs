using Grasshopper.Kernel;
using Rhino.Geometry;

public class Script_Instance : GH_ScriptInstance
{
    private void RunScript(Curve C, int Degree, ref object R)
    {
        R = null;
        if (C == null || !C.IsValid) return;

        NurbsCurve source = C.ToNurbsCurve();
        if (source == null) return;
        NurbsCurve rebuilt = C.Rebuild(source.Points.Count, Degree, false);
        if (rebuilt == null)
        {
            Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Warning,
                "Could not rebuild the directrix at the selected degree.");
            return;
        }
        rebuilt.Domain = source.Domain;
        R = rebuilt;
    }
}
