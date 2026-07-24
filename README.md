# bci2000-grip

本仓库基于 BCI2000，版本控制主要覆盖自行开发或修改的握力任务、信号源、工具、参数和启动脚本。BCI2000 原生文件大多由 `.gitignore` 排除，因此本仓库不能脱离完整的 BCI2000 源码树单独编译。

## 自定义模块

### GripForceSource

路径：`src/core/SignalSource/GripForceSource/`

独立的 BCI2000 SignalSource 模块，负责把输入写入标准的 BCI2000 信号通道。

- 鼠标模式：读取鼠标左键；按下时输出 `MouseForce`，松开时输出 0。
- Arduino 模式：从串口读取 `temperature,voltage` 格式的数据，并使用逗号后的电压值。
- `TargetChannel` 指定握力写入哪个 source channel。
- `Gain` 对最终原始输入进行缩放。

### GripForceTask

路径：`src/core/Application/GripForceTask/`

三维握力光标任务，包含 trial/block 编排、目标、握力归一化、运动模型、状态记录以及可选的 NI6501 marker。

### GripFlightTask

路径：`src/core/Application/GripFlightTask/`

二维横向飞行握力任务，包含：

- 横向滚动地图和碰撞检测；
- 独立的飞行物理与游戏状态机；
- 鼠标/Arduino 双输入配置；
- 随窗口宽高比变化的相机画幅；
- 保持原始长宽比的 player 图像；
- 可替换的 SVG/PNG 素材和 CSV 地图；
- 独立素材预览程序 `GripFlightAssetPreview`。

### NI6501Marker

路径：`src/core/Tools/NI6501Marker/`

独立控制台工具，用于通过 NI 6501 P0.0/P0.1 输出 marker 脉冲。

## 运行方法

### GripForceTask

运行：

```text
batch\GripForceTask_GripForceSource.bat
```

默认模块链：

```text
GripForceSource -> DummySignalProcessing -> GripForceTask
```

主要参数文件：

```text
parms\examples\GripForceTask_GripForceSource.prm
```

### GripFlightTask

运行：

```text
batch\GripFlightTask.bat
```

默认模块链：

```text
GripForceSource -> DummySignalProcessing -> GripFlightTask
```

任务参数文件：

```text
src\core\Application\GripFlightTask\GripFlightTask.prm
```

启动并完成配置后，在 BCI2000 Operator 中点击 `Start`。

## 输入模式

输入模式在相应 `.prm` 文件中设置：

```prm
Source:GripForce int InputMode= 0
```

- `InputMode=0`：鼠标模式，默认值。
- `InputMode=1`：Arduino 串口模式。

鼠标模式常用参数：

```prm
Source:GripForce float MouseForce= 5.0
Source:GripForce float Gain= 1.0
```

Arduino 模式常用参数：

```prm
Source:GripForce string SerialPort= COM6
Source:GripForce int SerialBaud= 115200
Source:GripForce float Gain= 1.0
```

Arduino 每行应输出：

```text
temperature,voltage
```

例如：

```text
0.000,2.137
```

切换到 Arduino 模式前，应确认串口名称、波特率和 Arduino 输出格式一致。

## 握力换算

Application 从 `GripForceChannel` 指定的信号通道读取原始值，并通过下列范围归一化：

```prm
GripForceMin= 0
GripForceMax= 5
```

换算关系：

```text
normalizedGrip = clamp(
    (rawGrip - GripForceMin) / (GripForceMax - GripForceMin),
    0,
    1
)
```

`GripForceSmoothing` 控制低通平滑程度。只修改 `.prm` 参数通常不需要重新编译。

## GripFlightTask 主要参数

窗口大小：

```prm
WindowWidth= 1200
WindowHeight= 500
```

飞行物理：

```prm
ForwardSpeed= 30
LiftGain= 80
Gravity= 25
FlightDamping= 0.98
PlayerStart= 2 20 50
PlayerSize= 2 20 20
```

`PlayerSize` 是世界坐标中的碰撞盒尺寸。player 素材保持其原始长宽比，不会被拉伸为碰撞盒比例。

地图与素材：

```prm
WorldHeight= 90
MapFile= ../src/core/Application/GripFlightTask/maps/default.csv
PlayerImage= ../src/core/Application/GripFlightTask/assets/player.svg
BackgroundImage= ../src/core/Application/GripFlightTask/assets/background.svg
```

窗口高度对应固定的世界高度；窗口越宽，相机显示的横向世界范围越大。场景使用统一的横纵缩放比例，不按固定画幅拉伸或留黑边。

## Trial、Run 与数据保存

BCI2000 中一次 `Start` 对应一个 run。只要不在每个 trial 后手动执行 `Stop/Resume`，同一 run 内的多个 trial 会保存在同一个 `.dat` 文件中。

常用时序参数：

```prm
PreRunDuration
PreFeedbackDuration
FeedbackDuration
PostFeedbackDuration
ITIDuration
NumberOfTrials
```

默认数据目录：

```text
data\<SubjectName>\<SubjectName>S001Rxx.dat
```

`.dat` 文件包含信号通道和 Application states，可用于离线切分 trial、分析握力、碰撞和任务结果。

GripFlightTask 记录的主要状态包括：

```text
GamePhase
BallWorldX
BallWorldY
BallVelocityY
GripForceRaw
GripForceNormalized
CameraWorldX
Collision
CollisionObject
FlightTrialResult
```

## 模块独立性

BCI2000 使用下列模块链：

```text
SignalSource -> SignalProcessing -> Application
```

本项目当前使用：

```text
mouse / Arduino
    -> GripForceSource
    -> BCI2000 signal channel
    -> DummySignalProcessing
    -> GripForceTask 或 GripFlightTask
```

三个运行阶段是独立的可执行模块：

```text
GripForceSource.exe          SignalSource
DummySignalProcessing.exe    SignalProcessing
GripForceTask.exe            Application
GripFlightTask.exe           Application
```

### SignalSource 的职责

`GripForceSourceADC` 注册在 filter level 1，负责：

- 调用鼠标或串口 API；
- 解析 Arduino 数据；
- 将握力值写入标准 BCI2000 信号通道。

硬件读取逻辑只存在于 Source 模块。

### Application 的职责

`GripForceTask` 和 `GripFlightTask` 注册在 filter level 3。它们只从传入的 `GenericSignal` 中读取 `GripForceChannel`，负责：

- 将原始值换算为归一化握力；
- 更新任务物理和视觉反馈；
- 管理 trial、结果和任务状态；
- 把 states 与信号一起写入 BCI2000 数据流。

Application 不直接调用鼠标、串口或 Arduino API，因此不知道上游信号来自哪种设备。鼠标和 Arduino 对 Application 来说是同一个接口：

```text
GenericSignal channel -> Application
```

### 参数边界

Source 参数控制采集方式：

```text
InputMode
MouseForce
SerialPort
SerialBaud
Gain
TargetChannel
```

Application 参数控制任务如何解释信号：

```text
GripForceChannel
GripForceMin
GripForceMax
GripForceSmoothing
任务物理、地图、素材和时序参数
```

即使这些参数保存在同一个 `.prm` 文件中，所属模块和职责仍然分离。

### 替换输入源

由于 Application 只依赖标准信号通道，将来可以用 EEG SignalSource 或其他采集模块替换 `GripForceSource`。新 Source 只需提供约定的握力通道，并在 `GripForceChannel` 中指定通道编号。

如果希望一个 `.dat` 同时保存 EEG 和握力，应让同一条 BCI2000 source signal stream 同时包含：

```text
EEG channels + grip-force channel
```

这样 EEG、握力和 Application states 会自然写入同一个 run 文件。

## 构建说明

相关模块通过以下原生 CMake 文件注册：

```text
src/core/Application/CMakeLists.txt
src/core/SignalSource/CMakeLists.txt
src/core/Tools/CMakeLists.txt
```

修改 `.cpp` 或 `.h` 后需要重新编译，并将生成的模块部署到 `prog\`。编译产物、运行数据和 `.dat` 文件不纳入版本控制。
