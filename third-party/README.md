# Optimizer libraries

Keep the `lib` folder beside `arc_vFinal_Release.gh` when copying or moving the definition. The segmentation component loads the two assemblies from that folder before the optimization component receives its control points. Both libraries are included in the repository.

If **optimization** reports a missing assembly, check the folder layout and restart Rhino after restoring the files. The [setup guide](../docs/installation.md#numerical-libraries) also covers adding the two DLL references manually.

| Bundled file | Version | Used for |
| --- | --- | --- |
| `lib/AutoDiff.dll` | AutoDiff 1.2.2 | Differentiating the optimizer’s objective function |
| `lib/alglib400gpl_net2.dll` | ALGLIB 4.00.0, C# Free Edition | Nonlinear conjugate-gradient minimization |

## AutoDiff

AutoDiff is copyright © 2018 Alex Shtof and distributed under the MIT License. The bundled DLL reports assembly version `1.2.2.0`. [License and copyright notice](AutoDiff-LICENSE.txt) · [Upstream source](https://github.com/alexshtf/autodiff) · [NuGet package](https://www.nuget.org/packages/AutoDiff/)

## ALGLIB

ALGLIB is copyright © Sergey Bochkanov (ALGLIB project). The C# Free Edition is distributed under the GNU General Public License, version 2 or later. The included 4.00.0 distribution contains the corresponding C# source under `alglib-csharp/lib-csharp-core/src/`, the original managed DLLs, tests, manual, and both license texts. Its `alglib400gpl_net2.dll` is byte-for-byte identical to the DLL in `lib/`. [Source distribution](alglib-4.00.0.csharp.gpl.zip) · [GPL version 2](ALGLIB-GPL2.txt) · [GPL version 3](ALGLIB-GPL3.txt) · [Upstream project and downloads](https://www.alglib.net/download.php)

## File checksums

| File | SHA-256 |
| --- | --- |
| `AutoDiff.dll` | `c215d516e7965a19e4e28b10066bc4cd0e55be15d5929671a8e8d2f96800e6b5` |
| `alglib400gpl_net2.dll` | `d8071a77737440d87ab2a2e8c50d01cb1a2b93997d005c3f392fa238d8e823c7` |
| `alglib-4.00.0.csharp.gpl.zip` | `e90f2adc9d172f140aa1b0001b937a7dbbf76f5e03b4da8c89c48c320c6ab6f3` |
