using Grasshopper.Kernel;

public class Script_Instance : GH_ScriptInstance
{
    private void RunScript(double X, ref object Reduction)
    {
        // The upstream division gives the number of strips per panel face.
        Reduction = 1.0 - X;
    }
}
