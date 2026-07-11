# AutoSurgView

[English](README.md) | [简体中文](README_CN.md)

开放手术记录面临一个长期难题：术者及手术团队会频繁遮挡术野。AutoSurgView 通过持续测量术野遮挡并自主调整相机位姿，帮助恢复清晰、连续的手术视野。

本仓库是论文 *“Autonomous occlusion-aware robotic recording enables continuous visualization of the surgical field in open cardiac surgery”* 的配套代码，包含遮挡感知、基于深度强化学习的视点规划和机器人控制模块。

![遮挡感知与自主视点规划流程概览](docs/occlusion-aware-viewpoint-planning.png)

> **科研软件与安全声明：**本仓库包含物理机器人控制代码，不属于医疗器械，不得直接用于临床。启用执行器前，必须先在仿真环境和隔离测试平台中验证运动范围、急停机制、标定结果与碰撞检测功能。

## 仓库结构

- `robot-controller/`：Windows C++ 机器人控制端，包含感知、视点规划、运动控制和硬件通信代码，原目录名为 `下位机`。
- `operator-console/`：Windows WPF 操作界面，原目录名为 `上位机`。
- `docs/DEPENDENCIES.md`：第三方依赖、厂商 SDK 以及仍然缺少的文件清单。

## 当前完整性

仓库现已包含：

- C++ CMake 构建配置；
- WPF `.csproj` 工程文件；
- 机器人控制参数示例；
- 遮挡规避代码实际使用的 ONNX 策略模型；
- 二进制 STL 模型读取代码；
- 相对路径和环境变量配置支持；
- 开源许可证及依赖说明。

完整构建机器人硬件控制端仍需要 `docs/DEPENDENCIES.md` 中列出的厂商 SDK 和项目自有 RRT 头文件。这些文件在现有源码及提供的历史目录中均不存在，因此无法安全重建。

## 操作界面构建

需要安装 Visual Studio 2022、“.NET 桌面开发”工作负载以及 .NET Framework 4.8 Developer Pack。

```powershell
dotnet restore operator-console/AutoSurgView.OperatorConsole.csproj
dotnet build operator-console/AutoSurgView.OperatorConsole.csproj -c Release
```

摄像机相关功能还需要将已获得合法授权的海康威视 x64 SDK 文件放入 `operator-console/native/`。厂商二进制文件不会提交到 Git。

## 机器人控制端构建

1. 安装 Visual Studio 2022 的“使用 C++ 的桌面开发”工作负载、CMake，以及 `docs/DEPENDENCIES.md` 中列出的开源依赖。
2. 按依赖文档说明，将不能公开分发的头文件和库放入 `robot-controller/vendor/`。
3. 将 `robot-controller/config/parameters.example.txt` 复制为 `parameters.txt`，并根据实际测试平台检查所有参数。
4. 在仓库根目录执行：

```powershell
cmake -S robot-controller -B build/robot-controller
cmake --build build/robot-controller --config Release
```

以下环境变量可以覆盖默认相对路径：

- `AUTOSURGVIEW_CONFIG`：控制参数文件路径；
- `AUTOSURGVIEW_MODEL`：遮挡规避 ONNX 模型路径；
- `AUTOSURGVIEW_LOG_DIR`：运行日志目录。

## 发布前检查

- 确认 `LICENSE` 中的版权持有人名称；
- 补充论文 DOI、最终出版信息及规范 BibTeX；
- 补充机器人、摄像机、雷达等硬件型号、固件和 SDK 版本；
- 补充机器人与摄像机标定流程；
- 如果 RRT 规划代码归作者所有且允许开源，请补充相关文件；
- 不要提交患者数据、手术录像、账号密码、设备序列号或专有 SDK 文件。

## 引用

如果本项目对您的研究有帮助，请引用：

```bibtex
@article{yourname2026autosurgview,
  title   = {Autonomous occlusion-aware robotic recording enables continuous visualization of the surgical field in open cardiac surgery},
  author  = {Haohao Xu¹† and Gang Li¹† and Hao Zhang¹ and Jiongdong Yu¹ and Yueri Cai¹* and Yi Wang² and Ming Gong³ and Hongjia Zhang³},
  journal = {Nature Communications},
  year    = {2026}
}
```

## 许可证

本项目采用 MIT 许可证。第三方开源组件和厂商组件仍受其各自许可证约束。
