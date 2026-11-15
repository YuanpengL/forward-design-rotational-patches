using Grasshopper.Kernel;
using Rhino.Geometry;

public class Script_Instance : GH_ScriptInstance
{
    private void RunScript(Plane Pl, ref object F)
    {
        Pl.Flip();
        F = Pl;
    }
}
