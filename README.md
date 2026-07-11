# AutoSurgView

[English](README.md) | [简体中文](README_CN.md)

Open surgical procedures are difficult to document because clinicians frequently occlude the operative field. AutoSurgView addresses this challenge with an autonomous robotic recording system that measures occlusion and repositions the camera to restore visibility.

This repository accompanies *“Autonomous occlusion-aware robotic recording enables continuous visualization of the surgical field in open cardiac surgery.”* It provides the occlusion-aware perception, deep-reinforcement-learning viewpoint planning, and robotic control components of the system.

![Overview of the occlusion-aware perception and autonomous viewpoint planning pipeline](docs/occlusion-aware-viewpoint-planning.png)

> **Research software and safety notice:** this repository controls physical robotic hardware. It is not a medical device and must not be used clinically. Validate limits, emergency-stop behavior, calibration, and collision handling in simulation and on an isolated test rig before enabling actuators.

## Layout

- `robot-controller/` — Windows C++ perception, viewpoint planning, and hardware controller (formerly `下位机`).
- `operator-console/` — Windows WPF operator interface (formerly `上位机`).
- `docs/DEPENDENCIES.md` — dependencies and the remaining unavailable components.

## Current completeness

Build manifests, a configuration example, and the ONNX policy referenced by the occlusion-avoidance code are included. A complete hardware build still requires vendor SDKs and project-owned RRT headers listed in `docs/DEPENDENCIES.md`; they were absent from both source locations and cannot be safely recreated.

## Operator console

Install Visual Studio 2022 with **.NET desktop development** and the .NET Framework 4.8 developer pack:

```powershell
dotnet restore operator-console/AutoSurgView.OperatorConsole.csproj
dotnet build operator-console/AutoSurgView.OperatorConsole.csproj -c Release
```

Camera functions also require licensed Hikvision native SDK binaries in `operator-console/native/`.

## Robot controller

1. Install Visual Studio 2022 with **Desktop development with C++**, CMake, and the dependencies in `docs/DEPENDENCIES.md`.
2. Put non-redistributable headers/libraries under `robot-controller/vendor/` as documented.
3. Copy `robot-controller/config/parameters.example.txt` to `parameters.txt` and validate every setting.
4. Build from the repository root:

```powershell
cmake -S robot-controller -B build/robot-controller
cmake --build build/robot-controller --config Release
```

Set `AUTOSURGVIEW_CONFIG`, `AUTOSURGVIEW_MODEL`, and `AUTOSURGVIEW_LOG_DIR` to override the default relative paths.

## Before publication

- Confirm the copyright holder in `LICENSE`.
- Add paper authors, DOI, BibTeX, exact hardware/firmware/SDK versions, and calibration protocol.
- Publish the missing RRT code if author-owned and cleared for release.
- Do not commit patient data, credentials, serial numbers, or proprietary binaries.

## Citation

If you find this work useful, please cite:

```bibtex
@article{yourname2026autosurgview,
  title   = {Autonomous occlusion-aware robotic recording enables continuous visualization of the surgical field in open cardiac surgery},
  author  = {Haohao Xu¹† and Gang Li¹† and Hao Zhang¹ and Jiongdong Yu¹ and Yueri Cai¹* and Yi Wang² and Ming Gong³ and Hongjia Zhang³},
  journal = {Nature Communications},
  year    = {2026}
}
```

## License

MIT. Third-party and vendor components retain their own licenses.
