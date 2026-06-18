# bci2000-grip — 自定义代码清单

本仓库基于 BCI2000，仅备份**自己编写/修改的代码**。
BCI2000 自带的数万个文件已被 `.gitignore` 排除，不在版本控制内。
下面列出 git 实际跟踪的文件；未列出的即为 BCI2000 原生文件。

## 1. GripForceTask —— 应用模块（握力光标任务）
路径：`src/core/Application/GripForceTask/`
- `GripForceTask.cpp` — 任务主体：握力→小球力学模型、单底部目标、NI6501 自动打标
- `GripForceTask.h` — 头文件
- `CMakeLists.txt` — 模块构建定义（含链接 NIDAQ-MX imports）
- `GripForceTask_SignalGenerator.bat` — 用官方 SignalGenerator 仿真的启动脚本

## 2. GripForceSource —— 信号源模块（专用）
路径：`src/core/SignalSource/GripForceSource/`
- `GripForceSourceADC.cpp` — 信号源：鼠标左键=握力(0/1)，预留 Intan 模拟输入接口
- `GripForceSourceADC.h` — 头文件
- `CMakeLists.txt` — 模块构建定义

## 3. NI6501Marker —— 独立打标工具
路径：`src/core/Tools/NI6501Marker/`
- `NI6501Marker.cpp` — 控制台工具：按键在 NI 6501 P0.0/P0.1 输出 marker 脉冲
- `CMakeLists.txt` — 模块构建定义

## 4. 启动脚本 / 参数 / 编辑器配置
- `batch/GripForceTask_GripForceSource.bat` — 主启动脚本（GripForceSource → DummySignalProcessing → GripForceTask）
- `parms/gripforce_test.prm` — 任务参数文件
- `.vscode/settings.json` — VSCode CMake 配置

## 5. 改动过的 BCI2000 自带文件（仅各加 1 行注册）
这些是原生文件，但被加了一行 `add_subdirectory` 以注册上面的模块，故纳入跟踪：
- `src/core/Application/CMakeLists.txt`  → 加 `add_subdirectory( GripForceTask )`
- `src/core/SignalSource/CMakeLists.txt` → 加 `ADD_SUBDIRECTORY( GripForceSource )`
- `src/core/Tools/CMakeLists.txt`        → 加 `ADD_SUBDIRECTORY( NI6501Marker )`

---

## 说明
- 本仓库**不能单独编译**：它只保存自定义代码，需放入完整的 BCI2000 源码树才能构建。
- 编译产物（`build/`、`prog/`）、采集数据（`data/`、`*.dat`）均不跟踪。
