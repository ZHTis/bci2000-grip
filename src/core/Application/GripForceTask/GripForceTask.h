////////////////////////////////////////////////////////////////////////////////
// GripForceTask.h
//
// BCI2000 Application Module: Grip Force Cursor Task
//
// The participant controls the vertical position of a cursor/ball with grip
// force. Higher normalized force produces upward lift; gravity pulls downward.
// Input is supplied by the BCI2000 signal source chain, e.g. SignalGenerator or
// the custom GripForceSource/SerialWidget source.
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
    // BCI2000 FeedbackTask lifecycle callbacks.
    void OnPreflight(const SignalProperties& Input) const override;
    void OnInitialize(const SignalProperties& Input) override;
    void OnStartRun() override;
    void OnStopRun() override;
    void OnTrialBegin() override;
    void OnTrialEnd() override;
    void OnFeedbackBegin() override;
    void OnFeedbackEnd() override;

    // Called once per sample block by FeedbackTask.
    void DoPreRun(const GenericSignal&, bool& doProgress) override;
    void DoPreFeedback(const GenericSignal&, bool& doProgress) override;
    void DoFeedback(const GenericSignal& ControlSignal, bool& doProgress) override;
    void DoPostFeedback(const GenericSignal&, bool& doProgress) override;
    void DoITI(const GenericSignal&, bool& doProgress) override;

private:
    // Helpers.
    void MoveCursorTo(float x, float y, float z);
    void DisplayMessage(const std::string& msg);
    void UpdateTargetVisibility();
    void SetCameraControlsEnabled(bool enabled);
    void SaveCameraToPrm() const;
    void ApplyBlockPhysicsParameters();
    float NormalizeGripForce(float raw) const;
    float GetGripForceFromSignal(const GenericSignal& signal) const;
    void GenerateTrackingTarget();

    // NI6501 marker pulse at cursor onset.
    void InitMarkerDevice();
    void CloseMarkerDevice();
    void SendMarkerPulse();

    // Scene objects.
    FeedbackScene*  mpFeedbackScene;
    int             mRenderingQuality;
    TextField*      mpMessage;

    // Colors.
    RGBColor        mCursorColorFront;
    RGBColor        mCursorColorBack;
    RGBColor        mTargetColorNormal;
    RGBColor        mTargetColorHit;

    // Run state.
    int             mRunCount;
    int             mTrialCount;
    int             mCurFeedbackDuration;
    int             mMaxFeedbackDuration;

    // Block/session design.
    int             mNumberBlocks;
    int             mTrialsPerBlock;
    int             mBlockRestDuration;
    int             mCurrentBlock;
    int             mBlockTrialIndex;
    int             mSessionTrialIndex;
    int             mITIBlocksElapsed;
    bool            mBlockRestActive;

    // Grip input mapping.
    int             mGripForceChannel;    // 1-based source channel; 0 uses first ControlSignal channel.
    float           mGripForceMin;        // Raw value mapped to normalized 0.
    float           mGripForceMax;        // Raw value mapped to normalized 1.
    float           mGripForceSmoothing;  // Reserved low-pass coefficient; current mapping is direct.
    float           mCurrentGripForce;    // Current normalized grip force, 0-1.

    // Cursor speed axes.
    float           mCursorSpeedX;
    float           mCursorSpeedY;
    float           mCursorSpeedZ;

    // Physics model.
    float           mLiftGain;            // Multiplier from normalized grip force to upward lift.
    float           mGravityForce;        // Downward force magnitude.
    float           mCursorDamping;       // Velocity damping, 0-0.99.
    float           mBallVelocityY;       // Integrated vertical velocity.
    std::vector<float> mBlockLiftGains;
    std::vector<float> mBlockGravityForces;

    // Paradigm type: 0 Voluntary, 1 Cued, 2 Tracking.
    int             mParadigmType;

    // Tracking target trajectory.
    std::vector<float> mTrackingTargetY;
    int                mTrackingIndex;

    // Trial statistics.
    int             mHits;
    int             mMisses;
    int             mTimeouts;

    // Grip force visualization.
    GenericVisualization mGripVis;

    // NI6501 marker device.
    void*           mMarkerTask;          // DAQmx TaskHandle kept opaque to avoid NI headers here.
    bool            mMarkerEnabled;
    int             mMarkerPulseMs;
    int             mMarkerCount;

    // Application display window.
    GUI::DisplayWindow& mrWindow;
};

#endif // GRIP_FORCE_TASK_H
