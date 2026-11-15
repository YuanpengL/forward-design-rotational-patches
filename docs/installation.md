# Run the definition in Rhino 8

Download or clone the complete repository and extract it to a local folder. Keep `arc_vFinal_Release.gh` and the `lib/` directory side by side. Both input curves are embedded in the definition, so the example opens without a separate Rhino model.

## Rhino and Grasshopper

Use Rhino 8 with its included Grasshopper and Kangaroo 2 components. This release was checked with Rhino build `8.37.26307.15002` and Kangaroo 2.5.3. The definition includes the C# components for rebuilding the rail, drawing preview geometry, calculating panel repetition and constructing the supporting-frame meshes.

1. In Rhino, run `Grasshopper` and open [arc_vFinal_Release.gh](../arc_vFinal_Release.gh) from the extracted repository.
2. Use **Zoom Extents** in Rhino. With the saved inputs, the definition produces eight surface patches. The **optimization** component’s `oReport` output gives the fit, joint angle and frame mismatch for this closed example.
3. Open the [workflow and controls](../README.md#how-the-definition-works) to try a different directrix or profile.

## Embedded C# components

These scripts perform the operations that previously used Pufferfish, Human and Weaverbird. They are stored inside the definition. The linked files are the same code in a form you can read on GitHub.

| Canvas operation | C# source |
| --- | --- |
| Rebuild the directrix at the selected degree using its original control-point count and domain. | [RebuildDirectrix.cs](../src/RebuildDirectrix.cs) |
| Draw mesh tubes around the input rail and fitted arcs. | [PipeMesh.cs](../src/PipeMesh.cs) |
| Reverse the plane for the surface direction marker. | [ReversePlane.cs](../src/ReversePlane.cs) |
| Calculate the reduction in distinct panel shapes, counting one family per strip. | [PanelReduction.cs](../src/PanelReduction.cs) |
| Reconstruct the panel-edge mesh and orient the normals used for the frame offset. | [MeshFromPanelEdges.cs](../src/MeshFromPanelEdges.cs) |
| Turn each supporting-frame boundary into a mesh face. | [FrameFace.cs](../src/FrameFace.cs) |
| Draw the supporting-frame edges with the chosen color and line thickness. | [PreviewFrameEdges.cs](../src/PreviewFrameEdges.cs) |

## Numerical libraries

The optimizer uses **AutoDiff 1.2.2** for derivatives and **ALGLIB 4.00.0 Free Edition for C#** for numerical optimization. Both DLLs are supplied in [lib/](../lib/). The upstream segmentation component loads them from the open definition’s folder before the optimizer runs. For versions, source and redistribution terms, see the [third-party notices](../third-party/README.md).

If the optimizer reports that either assembly cannot be found, check the layout below. After restoring a missing DLL or moving the `lib/` folder back into place, restart Rhino and reopen the extracted definition to clear the earlier failed lookup.

```text
arc_vFinal_Release.gh
lib/
  AutoDiff.dll
  alglib400gpl_net2.dll
```

If your Rhino installation still needs explicit references, open a personal copy of the definition and double-click **optimization**. In its Script Editor, remove the two filename-only assembly directives at the top. Choose **Install Package**, set **Package Source** to **DLL Reference**, and select each bundled DLL in turn. Save the script, recompute, and save your personal copy. [McNeel’s C# assembly reference guide](https://developer.rhino3d.com/guides/scripting/scripting-gh-csharp/#assembly-references)
