////////////////////////////////////////////////////////////////////////////////
// GripForceTask.cpp
//
// BCI2000 Application Module: Grip Force Cursor Task
//
// 使用方法：
//   1. 将此文件和 GripForceTask.h 放入
//      src/core/Application/GripForceTask/
//   2. 在同目录创建 CMakeLists.txt（见文末注释）
//   3. 在 src/core/Application/CMakeLists.txt 添加：
//      add_subdirectory( GripForceTask )
//   4. 重新cmake并编译
//   5. 创建对应bat文件启动
//
// 握力传感器接入：
//   - 传感器模拟电压输出 → EEG放大器AUX通道
//   - 在参数 GripForceChannel 填入对应通道号（1-based）
//   - 在参数 GripForceMin/Max 填入传感器的实际电压范围
//   - 仿真测试时设 GripForceChannel=0，使用传入的第1个ControlSignal通道
////////////////////////////////////////////////////////////////////////////////
#include "GripForceTask.h"

#include "FeedbackScene2D.h"
#include "FeedbackScene3D.h"
#include "FileUtils.h"
#include "Localization.h"
#include "buffers.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>

// NI6501 硬件打标：复用 NIDAQ-MX 模块的动态加载封装（运行时加载 nicaiu.dll）
#include "NIDAQmx.imports.h"
namespace Dylib { bool nicaiu_Loaded(); }

// 注册为第3级Filter（Application层标准）
RegisterFilter(GripForceTask, 3);

// State位数：12bit对应0-4095，覆盖0-100%的位置
#define CURSOR_POS_BITS "12"
const int cCursorPosBits = ::atoi(CURSOR_POS_BITS);

////////////////////////////////////////////////////////////////////////////////
// 构造函数：声明所有参数和State
////////////////////////////////////////////////////////////////////////////////
GripForceTask::GripForceTask()
    : mpFeedbackScene(nullptr),
      mRenderingQuality(0),
      mpMessage(nullptr),
      mCursorColorFront(RGBColor::Red),
      mCursorColorBack(RGBColor::Yellow),
      mTargetColorNormal(RGBColor(0x808080)),
      mTargetColorHit(RGBColor::Yellow),
      mRunCount(0),
      mTrialCount(0),
      mCurFeedbackDuration(0),
      mMaxFeedbackDuration(0),
      mGripForceChannel(0),
      mGripForceMin(0.0f),
      mGripForceMax(1.0f),
      mGripForceSmoothing(0.8f),
      mCurrentGripForce(0.0f),
      mCursorSpeedX(0.0f),
      mCursorSpeedY(1.0f),
      mCursorSpeedZ(0.0f),
      mLiftGain(1.0f),
      mGravityForce(0.3f),
      mCursorDamping(0.85f),
      mBallVelocityY(0.0f),
      mParadigmType(0),
      mTrackingIndex(0),
      mHits(0),
      mMisses(0),
      mTimeouts(0),
      mGripVis("GRPF"),
      mMarkerTask(nullptr),
      mMarkerEnabled(false),
      mMarkerPulseMs(200),
      mMarkerCount(0),
      mrWindow(Window())
{
    // ----------------------------------------------------------------
    // 参数定义
    // ----------------------------------------------------------------
    BEGIN_PARAMETER_DEFINITIONS

        // 渲染质量
        "Application:Window int RenderingQuality= 1 0 0 1 "
        " // rendering quality: 0: low(2D), 1: high(3D) (enumeration)",

        // 试次时序
        "Application:Sequencing float MaxFeedbackDuration= 15s % 0 % "
        " // 每个trial最长反馈时间，超时算失败",

        // 握力传感器参数
        "Application:GripForce int GripForceChannel= 0 0 0 % "
        " // 握力信号通道号(1-based)，0=使用第1个ControlSignal通道",

        "Application:GripForce float GripForceMin= 0.0 0.0 % % "
        " // 传感器最小值（对应球最低位置，单位与信号单位一致）",

        "Application:GripForce float GripForceMax= 1.0 1.0 % % "
        " // 传感器最大值（对应球最高位置）",

        "Application:GripForce float GripForceSmoothing= 0.0 0.0 0.0 0.99 "
        " // 握力低通平滑系数(0=无平滑, 0.99=极平滑)",

        "Application:GripForce float MVCThreshold= 0.3 0.3 0.0 1.0 "
        " // 击中目标所需的最小握力（占归一化范围的比例）",

        // NI6501 硬件打标
        "Application:Marker int EnableMarker= 0 0 0 1 "
        " // 小球出现时输出NI6501硬件脉冲(0=关,1=开) (boolean)",

        "Application:Marker int MarkerPulseMs= 200 200 1 % "
        " // 打标脉冲宽度(ms)",

        // 力学模型：握力产生向上的力，重力恒定向下，球有惯性
        "Application:GripPhysics float LiftGain= 1.0 1.0 0.0 % "
        " // 握力→上升力的比值（越大越容易把球托起）",

        "Application:GripPhysics float GravityForce= 0.3 0.3 0.0 % "
        " // 恒定向下力（重力）大小（越大球下落越快）",

        "Application:GripPhysics float CursorDamping= 0.85 0.85 0.0 0.99 "
        " // 速度阻尼(0~1)，越大惯性越强、越接近0越'黏'",

        // 范式类型
        "Application:Paradigm int ParadigmType= 0 0 0 2 "
        " // 范式类型: "
        "0: VoluntaryGrip(自主握力), "
        "1: CuedGrip(提示握力), "
        "2: TrackingGrip(追踪握力) "
        "(enumeration)",

        // 目标参数
        // 握力垂直任务：目标默认固定在顶部（被试需用握力将球托到上方目标）
        "Application:Targets int NumberTargets= 1 1 1 4 "
        " // 目标数量（默认顶部单目标=1）",

        "Application:Targets matrix Targets= "
            " 1 "
            " [pos%20x pos%20y pos%20z width%20x width%20y width%20z] "
            "  50  80  50 20 20 20 "
            " // 目标位置与大小（百分比坐标，单个顶部目标）",

        "Application:Targets int TargetColor= 0x808080 % % % "
        " // 目标颜色 (color)",

        "Application:Targets string TargetTexture= % % % % "
        " // 目标纹理路径 (inputfile)",

        // 光标参数
        "Application:Cursor float CursorWidth= 10 10 1 30 "
        " // 光标（球）大小（占屏幕宽度百分比）",

        "Application:Cursor int CursorColorFront= 0xff0000 % % % "
        " // 光标前景色 (color)",

        "Application:Cursor int CursorColorBack= 0xffff00 % % % "
        " // 光标背景色 (color)",

        "Application:Cursor string CursorTexture= images/marble.bmp % % % "
        " // 光标纹理路径 (inputfile)",

        "Application:Cursor floatlist CursorPos= 3 50 50 50 % % "
        " // 光标（球）起始位置",

        // 3D环境
        "Application:3DEnvironment floatlist CameraPos= 3 50 70 140 % % "
        " // 摄像机位置（保持和CursorTask一致）",

        "Application:3DEnvironment floatlist CameraAim= 3 50 50 50 % % "
        " // 摄像机朝向",

        "Application:3DEnvironment int CameraProjection= 0 0 0 2 "
        " // 投影类型: 0:平行 1:广角透视 2:窄角透视 (enumeration)",

        "Application:3DEnvironment floatlist LightSourcePos= 3 50 50 100 % % "
        " // 光源位置",

        "Application:3DEnvironment int LightSourceColor= 0x808080 % % "
        " // 光源颜色 (color)",

        "Application:3DEnvironment int WorkspaceBoundaryColor= 0x303030 0 % % "
        " // 工作区边界颜色 (color)",

        "Application:3DEnvironment floatlist WorkspaceBoundaryPos= 3 50 50 50 % % "
        " // 工作区中心位置",

        "Application:3DEnvironment floatlist WorkspaceBoundarySize= 3 100 100 100 % % "
        " // 工作区尺寸",

        "Application:3DEnvironment string WorkspaceBoundaryTexture= % % % % "
        " // 工作区纹理（空=不显示网格） (inputfile)",

    END_PARAMETER_DEFINITIONS

    // ----------------------------------------------------------------
    // State定义：这些值自动和每个SampleBlock的神经信号对齐保存
    // ----------------------------------------------------------------
    BEGIN_STATE_DEFINITIONS
        // 光标位置（12bit，0-4095对应0-100%）
        "CursorPosX " CURSOR_POS_BITS " 0 0 0",
        "CursorPosY " CURSOR_POS_BITS " 0 0 0",
        "CursorPosZ " CURSOR_POS_BITS " 0 0 0",

        // 握力相关State
        "GripForceNormalized 16 0 0 0",   // 归一化握力（0-65535对应0-100%）
        "GripForceRaw        16 0 0 0",   // 原始握力值（用于离线分析）

        // 任务事件State（用于ERP分析的trigger）
        "TargetAppear    1 0 0 0",         // 目标出现时=1
        "FeedbackOnset   1 0 0 0",         // 反馈开始时=1
        "TrialResult     3 0 0 0",         // 0:进行中 1:hit 2:miss 3:timeout
        "GripMarker      8 0 0 0",   // marker计数，0=无，1-255=第N次小球出现
    END_STATE_DEFINITIONS

    // ----------------------------------------------------------------
    // 消息文本框（显示"准备"/"超时"等提示）
    // ----------------------------------------------------------------
    GUI::Rect rect = {0.5f, 0.4f, 0.5f, 0.6f};
    mpMessage = new TextField(mrWindow);
    mpMessage->SetTextColor(RGBColor(0x79b88a))
               .SetFontFamily("Bahnschrift SemiBold")
               .SetTextHeight(0.62f)
               .SetColor(RGBColor::NullColor)
               .SetScalingMode(GUI::ScalingMode::AdjustWidth)
               .SetObjectRect(rect);
}

GripForceTask::~GripForceTask()
{
    SaveCameraToPrm();
    CloseMarkerDevice();
    delete mpFeedbackScene;
}

////////////////////////////////////////////////////////////////////////////////
// OnPreflight：参数合法性检查，在Initialize之前调用
////////////////////////////////////////////////////////////////////////////////
void GripForceTask::OnPreflight(const SignalProperties& Input) const
{
    // ---- 握力相关检查 ----
    int gripCh = Parameter("GripForceChannel");
    if (gripCh > 0 && gripCh > Input.Channels())
        bcierr << "GripForceChannel=" << gripCh
               << " 超出信号通道数 " << Input.Channels();

    if (Parameter("GripForceMin") >= Parameter("GripForceMax"))
        bcierr << "GripForceMin 必须小于 GripForceMax";

    if (Parameter("MaxFeedbackDuration").InSampleBlocks() <= 0)
        bcierr << "MaxFeedbackDuration 必须大于0";

    Parameter("GripForceSmoothing");
    Parameter("MVCThreshold");
    Parameter("ParadigmType");
    Parameter("LiftGain");
    if (Parameter("GravityForce") < 0)
        bcierr << "GravityForce must be non-negative; it is treated as a downward force magnitude";
    Parameter("CursorDamping");

    // ---- 场景相关参数检查（必须与 OnInitialize / FeedbackScene 访问的参数一致）----
    const char* vectorParams[] = {
        "CameraPos", "CameraAim", "LightSourcePos", "CursorPos",
        "WorkspaceBoundaryPos", "WorkspaceBoundarySize"};
    for (auto p : vectorParams)
        if (Parameter(p)->NumValues() != 3)
            bcierr << "参数 " << p << " 必须有3个值";

    Parameter("WorkspaceBoundaryColor");
    const char* colorParams[] = {
        "CursorColorBack", "CursorColorFront", "TargetColor", "LightSourceColor"};
    for (auto p : colorParams)
        if (RGBColor(Parameter(p)) == RGBColor(RGBColor::NullColor))
            bcierr << "参数 " << p << " 的RGB值非法";

    bool showTextures = (Parameter("RenderingQuality") > 0);
    const char* texParams[] = {"CursorTexture", "TargetTexture", "WorkspaceBoundaryTexture"};
    for (auto p : texParams) {
        std::string filename = Parameter(p);
        if (showTextures && !filename.empty()) {
            int w, h;
            std::vector<GLubyte> ignored;
            if (!buffers::loadWindowsBitmap(FileUtils::AbsolutePath(filename), w, h, ignored))
                bcierr << "纹理文件无效 \"" << filename << "\"（参数 " << p << "）";
        }
    }

    if (Parameter("NumberTargets") > Parameter("Targets")->NumRows())
        bcierr << "Targets 行数(" << Parameter("Targets")->NumRows()
               << ") 少于 NumberTargets(" << Parameter("NumberTargets") << ")";

    Parameter("CursorWidth");
    Parameter("CameraProjection");
}

////////////////////////////////////////////////////////////////////////////////
// OnInitialize：每次Set Config时调用，初始化场景
////////////////////////////////////////////////////////////////////////////////
void GripForceTask::OnInitialize(const SignalProperties& /*Input*/)
{
    // 读取握力参数
    mGripForceChannel   = Parameter("GripForceChannel");
    mGripForceMin       = Parameter("GripForceMin");
    mGripForceMax       = Parameter("GripForceMax");
    mGripForceSmoothing = Parameter("GripForceSmoothing");
    mParadigmType       = Parameter("ParadigmType");
    mMaxFeedbackDuration = static_cast<int>(
        Parameter("MaxFeedbackDuration").InSampleBlocks());

    // 力学模型参数
    mLiftGain      = Parameter("LiftGain");
    mGravityForce  = Parameter("GravityForce");
    mCursorDamping = Parameter("CursorDamping");

    // 光标速度：握力任务只用Y轴，X/Z固定在中央
    float feedbackDuration = Parameter("MaxFeedbackDuration").InSampleBlocks();
    mCursorSpeedY = 100.0f / feedbackDuration / 2.0f;
    mCursorSpeedX = 0.0f;
    mCursorSpeedZ = 0.0f;

    // 颜色
    mCursorColorFront  = RGBColor(Parameter("CursorColorFront"));
    mCursorColorBack   = RGBColor(Parameter("CursorColorBack"));
    mTargetColorNormal = RGBColor(Parameter("TargetColor"));

    // 创建/重建场景
    int quality = Parameter("RenderingQuality");
    if (quality != mRenderingQuality) {
        mrWindow.Hide();
        mRenderingQuality = quality;
    }
    delete mpFeedbackScene;
    mpFeedbackScene = (quality == 0)
        ? static_cast<FeedbackScene*>(new FeedbackScene2D(mrWindow))
        : static_cast<FeedbackScene*>(new FeedbackScene3D(mrWindow));
    mpFeedbackScene->Initialize();
    mpFeedbackScene->SetCursorColor(mCursorColorFront);
    SetCameraControlsEnabled(true);

    // 追踪任务：预生成目标轨迹（正弦波，周期约为feedbackDuration的一半）
    if (mParadigmType == 2) {
        mTrackingTargetY.resize(mMaxFeedbackDuration);
        for (int i = 0; i < mMaxFeedbackDuration; ++i) {
            float phase = 2.0f * 3.14159f * i / (mMaxFeedbackDuration * 0.5f);
            mTrackingTargetY[i] = 50.0f + 35.0f * std::sin(phase);
        }
    }

    // 握力实时可视化窗口
    mGripVis.Send(CfgID::WindowTitle, "Grip Force");
    SignalProperties gripVisProperties(2, 1, SignalType::float32);
    gripVisProperties.ChannelLabels()[0] = "force_norm";
    gripVisProperties.ChannelLabels()[1] = "raw_voltage_norm";
    gripVisProperties.ValueUnit().SetRawMin(0).SetRawMax(1).SetGain(1).SetOffset(0).SetSymbol("");
    mGripVis.Send(gripVisProperties);
    mGripVis.Send(CfgID::NumSamples,  256);

    // NI6501 硬件打标设备
    mMarkerEnabled = (Parameter("EnableMarker") != 0);
    mMarkerPulseMs = Parameter("MarkerPulseMs");
    InitMarkerDevice();

    mrWindow.Show();
    DisplayMessage("准备开始...");
}

////////////////////////////////////////////////////////////////////////////////
// OnStartRun / OnStopRun
////////////////////////////////////////////////////////////////////////////////
void GripForceTask::OnStartRun()
{
    SetCameraControlsEnabled(false);
    ++mRunCount;
    mTrialCount = 0;
    mHits = mMisses = mTimeouts = 0;
    mMarkerCount = 0;
    AppLog << "GripForceTask Run #" << mRunCount << " 开始\n";
    AppLog << "[Marker] === Run #" << mRunCount
           << " 打标 " << (mMarkerEnabled ? "已启用" : "未启用") << " ===\n";
    AppLog << "范式类型: "
           << (mParadigmType == 0 ? "VoluntaryGrip" :
               mParadigmType == 1 ? "CuedGrip" : "TrackingGrip") << "\n";
    DisplayMessage("准备...");
}

void GripForceTask::OnStopRun()
{
    SaveCameraToPrm();
    AppLog << "Run #" << mRunCount << " 结束: "
           << mTrialCount << " trials | "
           << mHits << " hits | "
           << mMisses << " misses | "
           << mTimeouts << " timeouts\n";
    if (mTrialCount > 0)
        AppLog << "命中率: "
               << (100 * mHits / mTrialCount) << "%\n";
    AppLog << "=====================\n";
    DisplayMessage("结束");
}

////////////////////////////////////////////////////////////////////////////////
// OnTrialBegin：每个trial开始时
////////////////////////////////////////////////////////////////////////////////
void GripForceTask::OnTrialBegin()
{
    ++mTrialCount;
    mCurFeedbackDuration = 0;
    mTrackingIndex = 0;

    // 设置目标可见性
    // TargetCode=1: 上方目标（需要握紧）
    // TargetCode=2: 下方目标（需要放松）
    for (int i = 0; i < mpFeedbackScene->NumTargets(); ++i) {
        mpFeedbackScene->SetTargetColor(mTargetColorNormal, i);
        mpFeedbackScene->SetTargetVisible(State("TargetCode") == i + 1, i);
    }

    // 设置目标出现的State trigger
    State("TargetAppear") = 1;
    State("TrialResult")  = 0;

    AppLog.Screen << "Trial #" << mTrialCount
                  << " Target: " << (State("TargetCode") == 1 ? "上(握紧)" : "下(放松)")
                  << "\n";

    // 提示信息
    if (mParadigmType == 1)
        DisplayMessage(State("TargetCode") == 1 ? "握紧!" : "放松!");
    else
        DisplayMessage("");
}

void GripForceTask::OnTrialEnd()
{
    DisplayMessage("");
    mpFeedbackScene->SetCursorVisible(false);
    for (int i = 0; i < mpFeedbackScene->NumTargets(); ++i)
        mpFeedbackScene->SetTargetVisible(false, i);
    State("TargetAppear") = 0;
}

////////////////////////////////////////////////////////////////////////////////
// OnFeedbackBegin / OnFeedbackEnd
////////////////////////////////////////////////////////////////////////////////
void GripForceTask::OnFeedbackBegin()
{
    mCurFeedbackDuration = 0;

    // 球从中央开始，速度和上一试次的平滑握力清零
    mBallVelocityY = 0.0f;
    mCurrentGripForce = 0.0f;
    MoveCursorTo(50.0f, 50.0f, 50.0f);
    mpFeedbackScene->SetCursorVisible(true);

    // 小球出现的瞬间：触发NI6501硬件打标脉冲
    SendMarkerPulse();

    State("FeedbackOnset") = 1;

    if (mParadigmType != 1)
        DisplayMessage("");
}

void GripForceTask::OnFeedbackEnd()
{
    State("FeedbackOnset") = 0;
    State("GripMarker") = 0;

    int result = State("ResultCode");
    if (result == 0) {
        // 超时
        ++mTimeouts;
        State("TrialResult") = 3;
        AppLog.Screen << "-> 超时\n";
        DisplayMessage("超时");
    } else if (result == State("TargetCode")) {
        // 命中
        ++mHits;
        State("TrialResult") = 1;
        mpFeedbackScene->SetCursorColor(RGBColor::Yellow);
        mpFeedbackScene->SetTargetColor(RGBColor::Yellow, result - 1);
        AppLog.Screen << "-> 命中!\n";
        DisplayMessage("命中!");
    } else {
        // 错误目标
        ++mMisses;
        State("TrialResult") = 2;
        AppLog.Screen << "-> 失误\n";
        DisplayMessage("失误");
    }
}

////////////////////////////////////////////////////////////////////////////////
// DoFeedback：核心循环，每个SampleBlock调用一次
////////////////////////////////////////////////////////////////////////////////
void GripForceTask::DoFeedback(const GenericSignal& ControlSignal, bool& doProgress)
{
    // ------------------------------------------------------------------
    // 1. 获取并平滑握力信号
    // ------------------------------------------------------------------
    float rawGrip = GetGripForceFromSignal(ControlSignal);
    float normGrip = NormalizeGripForce(rawGrip);

    // 等比 不做修改
    mCurrentGripForce = normGrip;

    // 保存到State（16bit: 0-65535）。原始信号可能为负或越界，必须限幅到合法范围，
    // 否则负值会被当成无符号大数(如4294967295)写入16位State而导致崩溃。
    int normVal = static_cast<int>(mCurrentGripForce * 65535);
    State("GripForceNormalized") = std::max(0, std::min(65535, normVal));
    int rawVal = static_cast<int>(rawGrip);
    State("GripForceRaw")        = std::max(0, std::min(65535, rawVal));

    // 实时可视化：标量握力值需包装成单通道单采样的GenericSignal
    GenericSignal gripVisSignal(2, 1);
    gripVisSignal(0, 0) = mCurrentGripForce;
    gripVisSignal(1, 0) = NormalizeGripForce(rawGrip);
    mGripVis.Send(gripVisSignal);

    // ------------------------------------------------------------------
    // 2. 力学模型：握力产生向上的力，重力恒定向下，球有惯性
    //    上升力 = LiftGain × 握力（握力越大越往上托）
    //    净力   = 上升力 − 重力（重力始终存在，与握力无关）
    //    速度  += 净力；速度 *= 阻尼；位置 += 速度
    // ------------------------------------------------------------------
    Vector3D pos = mpFeedbackScene->CursorPosition();

    float upForce  = mLiftGain * mCurrentGripForce;   // 握力≥0 → 只产生向上的力
    float gravity  = std::fabs(mGravityForce);        // 重力参数表示大小，方向固定向下
    float netForce = upForce - gravity;               // 向下的重力恒定存在
    mBallVelocityY += netForce;                       // 力 → 速度增量（积分加速度）
    mBallVelocityY *= mCursorDamping;                 // 阻尼，防止惯性失控
    pos.y += mBallVelocityY;                          // 速度 → 位置

    // X/Z固定在中央
    pos.x = 50.0f;
    pos.z = 50.0f;

    // 限制在窗口边界内；撞到上/下边界时速度清零，避免力持续累积"黏"在边界
    float r = mpFeedbackScene->CursorRadius();
    if (pos.y <= r)            { pos.y = r;            mBallVelocityY = 0.0f; }
    else if (pos.y >= 100.0f - r) { pos.y = 100.0f - r; mBallVelocityY = 0.0f; }

    // 更新场景
    mpFeedbackScene->SetCursorPosition(pos);
    mpFeedbackScene->SetCursorVelocity({0, mBallVelocityY, 0});

    // 保存光标位置到State（自动和神经信号时间对齐）
    const float coordToState = ((1 << cCursorPosBits) - 1) / 100.0f;
    State("CursorPosX") = static_cast<int>(pos.x * coordToState);
    State("CursorPosY") = static_cast<int>(pos.y * coordToState);
    State("CursorPosZ") = static_cast<int>(pos.z * coordToState);

    // ------------------------------------------------------------------
    // 3. 检测是否击中目标
    // ------------------------------------------------------------------
    if (mpFeedbackScene->TargetHit(State("TargetCode") - 1))
        State("ResultCode") = State("TargetCode");

    // ------------------------------------------------------------------
    // 4. 判断trial是否结束（超时或击中）
    // ------------------------------------------------------------------
    doProgress = (++mCurFeedbackDuration >= mMaxFeedbackDuration)
              || (State("ResultCode") != 0);
}

// 其余Do*函数：FeedbackTask基类要求实现但这里不需要额外逻辑
void GripForceTask::DoPreRun(const GenericSignal&, bool&) {}
void GripForceTask::DoPreFeedback(const GenericSignal&, bool&) {}
void GripForceTask::DoPostFeedback(const GenericSignal&, bool&) {}
void GripForceTask::DoITI(const GenericSignal&, bool&) {}

////////////////////////////////////////////////////////////////////////////////
// 辅助函数
////////////////////////////////////////////////////////////////////////////////

// 从信号中读取握力值（取幅度大小，与方向/正负无关）
// GripForceChannel=0 时使用ControlSignal第1通道
float GripForceTask::GetGripForceFromSignal(const GenericSignal& signal) const
{
    int ch = (mGripForceChannel > 0) ? (mGripForceChannel - 1) : 0;
    if (ch < signal.Channels() && signal.Elements() > 0)
        return std::fabs(static_cast<float>(signal(ch, 0))); // 幅度大小，方向无关
    return 0.0f; // 默认无握力
}

// 将原始握力值归一化到[0, 1]
float GripForceTask::NormalizeGripForce(float raw) const
{
    if (mGripForceMax <= mGripForceMin)
        return 0.5f;
    float norm = (raw - mGripForceMin) / (mGripForceMax - mGripForceMin);
    return std::max(0.0f, std::min(1.0f, norm));
}

// 移动球到指定位置并根据Z更新颜色
void GripForceTask::MoveCursorTo(float x, float y, float z)
{
    float t = z / 100.0f;
    RGBColor color = t * mCursorColorFront + (1.0f - t) * mCursorColorBack;
    mpFeedbackScene->SetCursorColor(color);
    mpFeedbackScene->SetCursorPosition({x, y, z});
}

// 生成追踪任务的目标轨迹（已在OnInitialize中调用）
void GripForceTask::GenerateTrackingTarget()
{
    // 预留：可以改成从文件读取或更复杂的生成逻辑
}

// 显示文本提示
void GripForceTask::DisplayMessage(const std::string& msg)
{
    if (msg.empty())
        mpMessage->Hide();
    else {
        mpMessage->SetText(" " + msg + " ");
        mpMessage->Show();
    }
}

void GripForceTask::SetCameraControlsEnabled(bool enabled)
{
    if (auto scene3D = dynamic_cast<FeedbackScene3D*>(mpFeedbackScene))
        scene3D->SetCameraControlsEnabled(enabled);
}

void GripForceTask::SaveCameraToPrm() const
{
    auto scene3D = dynamic_cast<FeedbackScene3D*>(mpFeedbackScene);
    if (!scene3D)
        return;

    std::string path = FileUtils::AbsolutePath("../parms/examples/GripForceTask_GripForceSource.prm");
    if (!FileUtils::IsFile(path))
        path = FileUtils::AbsolutePath("parms/examples/GripForceTask_GripForceSource.prm");
    if (!FileUtils::IsFile(path))
        return;

    std::ifstream in(path);
    if (!in)
        return;

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(in, line))
        lines.push_back(line);
    in.close();

    auto formatVector = [](const char* name, const Vector3D& v, const char* comment) {
        std::ostringstream s;
        s << std::fixed << std::setprecision(3)
          << "Application:3DEnvironment floatlist " << name << "= 3 "
          << v.x << " " << v.y << " " << v.z << " % % // " << comment;
        return s.str();
    };

    const std::string cameraPos = formatVector("CameraPos", scene3D->CameraPosition(), "camera position");
    const std::string cameraAim = formatVector("CameraAim", scene3D->CameraAim(), "camera aim");
    bool wrotePos = false, wroteAim = false;
    for (auto& l : lines)
    {
        if (l.find("Application:3DEnvironment floatlist CameraPos=") != std::string::npos)
        {
            l = cameraPos;
            wrotePos = true;
        }
        else if (l.find("Application:3DEnvironment floatlist CameraAim=") != std::string::npos)
        {
            l = cameraAim;
            wroteAim = true;
        }
    }
    if (!wrotePos)
        lines.push_back(cameraPos);
    if (!wroteAim)
        lines.push_back(cameraAim);

    std::ofstream out(path, std::ios::trunc);
    if (!out)
        return;
    for (const auto& l : lines)
        out << l << "\n";
}

void GripForceTask::UpdateTargetVisibility()
{
    for (int i = 0; i < mpFeedbackScene->NumTargets(); ++i)
        mpFeedbackScene->SetTargetVisible(State("TargetCode") == i + 1, i);
}

////////////////////////////////////////////////////////////////////////////////
// NI6501 硬件打标
//   复用 NIDAQ-MX 的动态加载封装：P0.0 上输出一个短脉冲，
//   在小球出现(OnFeedbackBegin)时调用 SendMarkerPulse()。
//   脉冲的"拉高-等待-拉低"放在独立线程，避免阻塞实时反馈循环。
////////////////////////////////////////////////////////////////////////////////
void GripForceTask::InitMarkerDevice()
{
    CloseMarkerDevice();   // 防止重复Set Config时句柄泄漏
    if (!mMarkerEnabled)
        return;

    if (!Dylib::nicaiu_Loaded()) {
        bciwarn << "EnableMarker=1 但未能加载 NI-DAQmx 运行时(nicaiu.dll)，"
                   "打标已禁用。请确认已安装 NI-DAQmx 驱动。";
        AppLog << "[Marker] 禁用: 未能加载 nicaiu.dll\n";
        mMarkerEnabled = false;
        return;
    }

    TaskHandle task = 0;
    int32 err = DAQmxCreateTask("", &task);
    if (!DAQmxFailed(err))
        err = DAQmxCreateDOChan(task, "Dev1/port0/line0:1", "",
                                DAQmx_Val_ChanForAllLines);
    if (!DAQmxFailed(err))
        err = DAQmxStartTask(task);

    if (DAQmxFailed(err)) {
        char buf[2048] = {0};
        DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
        bciwarn << "NI6501 打标初始化失败，打标已禁用: " << buf;
        AppLog << "[Marker] 初始化失败，已禁用: " << buf << "\n";
        if (task) DAQmxClearTask(task);
        mMarkerEnabled = false;
        mMarkerTask = nullptr;
        return;
    }

    // 初始化两条线为低电平
    uInt8 low[2] = {0, 0};
    DAQmxWriteDigitalLines(task, 1, 1, 10.0, DAQmx_Val_GroupByChannel, low, NULL, NULL);
    mMarkerTask = task;
    AppLog << "[Marker] NI6501 打标已就绪 (Dev1/port0/line0, 脉冲 "
           << mMarkerPulseMs << "ms)\n";
}

void GripForceTask::CloseMarkerDevice()
{
    if (mMarkerTask) {
        TaskHandle task = static_cast<TaskHandle>(mMarkerTask);
        uInt8 low[2] = {0, 0};
        DAQmxWriteDigitalLines(task, 1, 1, 10.0, DAQmx_Val_GroupByChannel, low, NULL, NULL);
        DAQmxStopTask(task);
        DAQmxClearTask(task);
        mMarkerTask = nullptr;
    }
}

void GripForceTask::SendMarkerPulse()
{
    if (!mMarkerEnabled || !mMarkerTask)
        return;

    TaskHandle task = static_cast<TaskHandle>(mMarkerTask);
    int pulseMs = mMarkerPulseMs;

    // 立即拉高 P0.0（主线程，瞬间完成），并检查返回值，结果写入 .applog 文件
    uInt8 high[2] = {0, 1};   // P0.0 0, P0.1 1
    int32 err = DAQmxWriteDigitalLines(task, 1, 1, 10.0,
                                       DAQmx_Val_GroupByChannel, high, NULL, NULL);
    if (DAQmxFailed(err)) {
        char buf[2048] = {0};
        DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
        AppLog << "[Marker] 发送脉冲失败: " << buf << "\n";
        return;
    }

    ++mMarkerCount;
    State("GripMarker") = (mMarkerCount % 255) + 1;  // 写State（主线程，安全）

    AppLog << "[Marker] #" << mMarkerCount
           << " P0.0 脉冲 (" << pulseMs << "ms) @ 小球出现\n";

    // 仅把"等待→拉低"放到后台线程，避免阻塞实时反馈循环
    std::thread([task, pulseMs]() {
        uInt8 low[2] = {0, 0};
        std::this_thread::sleep_for(std::chrono::milliseconds(pulseMs));
        DAQmxWriteDigitalLines(task, 1, 1, 10.0, DAQmx_Val_GroupByChannel, low, NULL, NULL);
    }).detach();
}

////////////////////////////////////////////////////////////////////////////////
// CMakeLists.txt 内容（保存为同目录下的CMakeLists.txt）：
//
// CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )
// SET( EXECUTABLE_NAME GripForceTask )
// SET( PROJECT_SRC_DIR "${BCI2000_ROOT_DIR}/src" )
// INCLUDE( ${PROJECT_SRC_DIR}/cmake/SetupBCI2000Application.cmake )
// BCI2000_ADD_APPLICATION_MODULE(
//   BINARY_NAME GripForceTask
//   SOURCES
//     GripForceTask.cpp
//     ${PROJECT_SRC_DIR}/core/Application/CursorTask/FeedbackScene2D.cpp
//     ${PROJECT_SRC_DIR}/core/Application/CursorTask/FeedbackScene3D.cpp
//     ${PROJECT_SRC_DIR}/core/Application/CursorTask/AudioFeedback.cpp
// )
////////////////////////////////////////////////////////////////////////////////
