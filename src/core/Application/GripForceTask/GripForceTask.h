////////////////////////////////////////////////////////////////////////////////
// GripForceTask.h
// 
// BCI2000 Application Module: Grip Force Cursor Task
// 
// 范式设计：
//   被试通过握力控制3D场景中球的垂直位置
//   目标出现在上方或下方，握紧=球上升，放松=球下降
//   支持三种子范式：
//     0: VoluntaryGrip    自主握力（被试自行决定何时握）
//     1: CuedGrip         提示握力（出现视觉提示后握）
//     2: TrackingGrip     追踪握力（球跟随目标曲线）
//
// 接入方式：
//   握力传感器 → EEG放大器AUX通道 → Channel "GripForce"
//   或通过SignalGenerator/GripForceSource仿真输入
////////////////////////////////////////////////////////////////////////////////
#ifndef GRIP_FORCE_TASK_H
#define GRIP_FORCE_TASK_H

#include "FeedbackTask.h"
#include "FeedbackScene2D.h"
#include "FeedbackScene3D.h"
#include "TextField.h"
#include "Color.h"
#include "GenericVisualization.h"

class GripForceTask : public FeedbackTask
{
public:
    GripForceTask();
    virtual ~GripForceTask();

protected:
    // BCI2000 FeedbackTask 生命周期回调
    void OnPreflight(const SignalProperties& Input) const override;
    void OnInitialize(const SignalProperties& Input) override;
    void OnStartRun() override;
    void OnStopRun() override;
    void OnTrialBegin() override;
    void OnTrialEnd() override;
    void OnFeedbackBegin() override;
    void OnFeedbackEnd() override;

    // 每个SampleBlock调用一次
    void DoPreRun(const GenericSignal&, bool& doProgress) override;
    void DoPreFeedback(const GenericSignal&, bool& doProgress) override;
    void DoFeedback(const GenericSignal& ControlSignal, bool& doProgress) override;
    void DoPostFeedback(const GenericSignal&, bool& doProgress) override;
    void DoITI(const GenericSignal&, bool& doProgress) override;

private:
    // 辅助函数
    void MoveCursorTo(float x, float y, float z);
    void DisplayMessage(const std::string& msg);
    void UpdateTargetVisibility();
    float NormalizeGripForce(float raw) const;
    float GetGripForceFromSignal(const GenericSignal& signal) const;
    void GenerateTrackingTarget();

    // NI6501 打标：在小球出现时输出一个硬件脉冲
    void InitMarkerDevice();
    void CloseMarkerDevice();
    void SendMarkerPulse();

    // 场景对象
    FeedbackScene*  mpFeedbackScene;
    int             mRenderingQuality;
    TextField*      mpMessage;

    // 颜色
    RGBColor        mCursorColorFront;
    RGBColor        mCursorColorBack;
    RGBColor        mTargetColorNormal;
    RGBColor        mTargetColorHit;

    // 运行状态
    int             mRunCount;
    int             mTrialCount;
    int             mCurFeedbackDuration;
    int             mMaxFeedbackDuration;

    // 握力参数
    int             mGripForceChannel;    // 握力信号所在通道（0-based）
    float           mGripForceMin;        // 传感器最小值（对应球最低点）
    float           mGripForceMax;        // 传感器最大值（对应球最高点）
    float           mGripForceSmoothing;  // 低通平滑系数 0-1
    float           mCurrentGripForce;    // 当前平滑后的握力值（0-1归一化）

    // 光标速度
    float           mCursorSpeedX;
    float           mCursorSpeedY;
    float           mCursorSpeedZ;

    // 力学模型参数（握力对抗重力）
    float           mLiftGain;        // 握力→上升力 的比值
    float           mGravityForce;    // 恒定向下力大小
    float           mCursorDamping;   // 速度阻尼(0~1)，防止惯性失控
    float           mBallVelocityY;   // 球当前垂直速度（积分量）

    // 范式类型
    int             mParadigmType;        // 0:Voluntary 1:Cued 2:Tracking

    // 追踪任务的目标轨迹
    std::vector<float> mTrackingTargetY;  // 预生成的目标Y位置序列
    int                mTrackingIndex;    // 当前追踪轨迹索引

    // 试次统计
    int             mHits;
    int             mMisses;
    int             mTimeouts;

    // GUI可视化
    GenericVisualization mGripVis;        // 实时握力显示窗口

    // NI6501 打标设备
    void*           mMarkerTask;          // DAQmx TaskHandle（void*避免在头文件引入NI头）
    bool            mMarkerEnabled;       // 是否启用硬件打标
    int             mMarkerPulseMs;       // 脉冲宽度(ms)
    int             mMarkerCount;         // 已发出的脉冲计数（写入日志便于核对）

    // 窗口引用
    GUI::DisplayWindow& mrWindow;
};

#endif // GRIP_FORCE_TASK_H
