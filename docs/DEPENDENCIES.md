# Dependencies and unavailable artifacts

## Robot controller (Windows x64, C++17)

Open-source dependencies inferred directly from includes are OpenCV, Eigen3, Boost.Thread, PCL, Intel RealSense SDK 2, ONNX Runtime, LibTorch, AprilTag 3, Slamtec RPLIDAR SDK, Orocos KDL, `wjwwood/serial`, JsonCpp, OpenMP, and OpenGL/GLUT.

Absent hardware/vendor dependencies (normally not redistributable):

- `ECANVci.h` and its CAN adapter library/DLL.
- `yanHuaDriver.h` and its digital-I/O library/DLL.
- Hikvision SDK runtime for the operator console.

Absent project-owned planning headers:

- `rrt.h`, `rrt_star.h`, `rrt_connect.h`, `rrt_connect_star.h`, `Q_rrt_star.h`.
- `D_rrt.h`, `D_rrt_star.h`, `D_rrt_connect.h`, `D_rrt_connect_star.h`.
- `MP_rrt.h`, `MP_rrt_star.h`, `MP_rrt_connect.h`, `MP_rrt_connect_star.h`.

Expected private layout:

```text
robot-controller/vendor/
  include/
  lib/
  bin/
```

Optional paths also reference an unavailable heart-detection ONNX model, 15 LibTorch compensation models, three SVM models, robot STL meshes, UI icons/audio, and an external Record Tool. Keep those features disabled until the artifacts and redistribution rights are established.

## Operator console (.NET Framework 4.8)

NuGet dependencies are declared in the project: MaterialDesignThemes, OpenCvSharp4, MathNet.Numerics, SharpGL, and SharpGL.WPF. Put licensed Hikvision x64 runtime files in `operator-console/native/`; Git ignores them.

