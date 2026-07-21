////////////////////////////////////////////////////////////////////////////////
// GripForceTask.cpp
//
// BCI2000 Application Module: Grip Force Cursor Task
//
// This application maps a BCI2000 control signal to vertical cursor motion.
// Grip input may come from SignalGenerator or a serial/custom source module.
// The application remains source-agnostic and reads only ControlSignal.
////////////////////////////////////////////////////////////////////////////////
//   GripForceChannel selects the incoming BCI2000 signal channel.
//   GripForceMin/GripForceMax map raw values to normalized force.
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

// NI6501 marker support.
#include "NIDAQmx.imports.h"
namespace Dylib { bool nicaiu_Loaded(); }

// Register as an Application-layer filter.
RegisterFilter(GripForceTask, 3);

// State bits: 12 bits map 0-4095 to 0-100%.
#define CURSOR_POS_BITS "12"
const int cCursorPosBits = ::atoi(CURSOR_POS_BITS);

////////////////////////////////////////////////////////////////////////////////
// Constructor: declare all parameters and states.
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
      mNumberBlocks(1),
      mTrialsPerBlock(20),
      mBlockRestDuration(0),
      mCurrentBlock(0),
      mBlockTrialIndex(0),
      mSessionTrialIndex(0),
      mITIBlocksElapsed(0),
      mBlockRestActive(false),
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
    // Parameter definitions.
    // ----------------------------------------------------------------
    BEGIN_PARAMETER_DEFINITIONS

        // Rendering quality
        "Application:Window int RenderingQuality= 1 0 0 1 "
        " // rendering quality: 0: low(2D), 1: high(3D) (enumeration)",

        // Trial timing
        "Application:Sequencing float MaxFeedbackDuration= 15s % 0 % "
        " // maximum feedback duration for one trial",

        "Application:BlockDesign int NumberBlocks= 1 1 1 % "
        " // number of blocks in one run/session",

        "Application:BlockDesign int TrialsPerBlock= 20 20 1 % "
        " // number of trials in each block",

        "Application:BlockDesign float BlockRestDuration= 0s 0s 0 % "
        " // rest duration inserted between blocks",

        "Application:BlockDesign floatlist BlockLiftGains= 1 1.0 % 0.0 % "
        " // per-block upward lift multipliers; last value is reused when the list is shorter than NumberBlocks",

        "Application:BlockDesign floatlist BlockGravityForces= 1 0.3 % 0.0 % "
        " // per-block downward force magnitudes; last value is reused when the list is shorter than NumberBlocks",

        // Grip force input mapping
        "Application:GripForce int GripForceChannel= 0 0 0 % "
        " // grip force signal channel, 1-based; 0 uses first ControlSignal channel",

        "Application:GripForce float GripForceMin= 0.0 0.0 % % "
        " // minimum sensor value",

        "Application:GripForce float GripForceMax= 1.0 1.0 % % "
        " // maximum sensor value",

        "Application:GripForce float GripForceSmoothing= 0.0 0.0 0.0 0.99 "
        " // low-pass smoothing coefficient; 0 disables smoothing",

        "Application:GripForce float MVCThreshold= 0.3 0.3 0.0 1.0 "
        " // minimum normalized grip force required for target hit",

        // NI6501 marker
        "Application:Marker int EnableMarker= 0 0 0 1 "
        " // output NI6501 hardware pulse when the cursor appears (boolean)",

        "Application:Marker int MarkerPulseMs= 200 200 1 % "
        " // marker pulse width in ms",

        "Application:GripPhysics float CursorDamping= 0.85 0.85 0.0 0.99 "
        " // velocity damping, 0-0.99",

        // Paradigm type
        "Application:Paradigm int ParadigmType= 0 0 0 2 "
        " // paradigm type: 0 VoluntaryGrip, 1 CuedGrip, 2 TrackingGrip (enumeration)",

        // Target parameters
        "Application:Targets int NumberTargets= 1 1 1 4 "
        " // number of targets",

        "Application:Targets matrix Targets= "
            " 1 "
            " [pos%20x pos%20y pos%20z width%20x width%20y width%20z] "
            "  50  80  50 20 20 20 "
            " // target position and size in percentage coordinates",

        "Application:Targets int TargetColor= 0x808080 % % % "
        " // target color (color)",

        "Application:Targets string TargetTexture= % % % % "
        " // target texture path (inputfile)",

        // Cursor parameters
        "Application:Cursor float CursorWidth= 10 10 1 30 "
        " // cursor size as percentage of screen width",

        "Application:Cursor int CursorColorFront= 0xff0000 % % % "
        " // cursor front color (color)",

        "Application:Cursor int CursorColorBack= 0xffff00 % % % "
        " // cursor back color (color)",

        "Application:Cursor string CursorTexture= images/marble.bmp % % % "
        " // cursor texture path (inputfile)",

        "Application:Cursor floatlist CursorPos= 3 50 50 50 % % "
        " // cursor start position",

        // 3D environment
        "Application:3DEnvironment floatlist CameraPos= 3 50 70 140 % % "
        " // camera position",

        "Application:3DEnvironment floatlist CameraAim= 3 50 50 50 % % "
        " // camera aim",

        "Application:3DEnvironment int CameraProjection= 0 0 0 2 "
        " // projection type: 0 orthographic, 1 wide perspective, 2 narrow perspective (enumeration)",

        "Application:3DEnvironment floatlist LightSourcePos= 3 50 50 100 % % "
        " // light source position",

        "Application:3DEnvironment int LightSourceColor= 0x808080 % % "
        " // light source color (color)",

        "Application:3DEnvironment int WorkspaceBoundaryColor= 0x303030 0 % % "
        " // workspace boundary color (color)",

        "Application:3DEnvironment floatlist WorkspaceBoundaryPos= 3 50 50 50 % % "
        " // workspace boundary center",

        "Application:3DEnvironment floatlist WorkspaceBoundarySize= 3 100 100 100 % % "
        " // workspace boundary size",

        "Application:3DEnvironment string WorkspaceBoundaryTexture= % % % % "
        " // workspace boundary texture path; empty hides grid texture (inputfile)",

    END_PARAMETER_DEFINITIONS

    // States saved into DAT with each sample block.
    BEGIN_STATE_DEFINITIONS
        "CursorPosX " CURSOR_POS_BITS " 0 0 0",
        "CursorPosY " CURSOR_POS_BITS " 0 0 0",
        "CursorPosZ " CURSOR_POS_BITS " 0 0 0",
        "GripForceNormalized 16 0 0 0",
        "GripForceRaw        16 0 0 0",
        "TargetAppear    1 0 0 0",
        "FeedbackOnset   1 0 0 0",
        "TrialResult     3 0 0 0",
        "GripMarker      8 0 0 0",
        "SessionTrialIndex 16 0 0 0",
        "BlockIndex        8 0 0 0",
        "BlockTrialIndex   16 0 0 0",
        "BlockPhase        3 0 0 0",
        "SessionComplete   1 0 0 0",
    END_STATE_DEFINITIONS

    // ----------------------------------------------------------------
    // Transparent message field for runtime prompts.
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
// OnPreflight: validate parameters before OnInitialize.
////////////////////////////////////////////////////////////////////////////////
void GripForceTask::OnPreflight(const SignalProperties& Input) const
{
    // Grip input checks.
    int gripCh = Parameter("GripForceChannel");
    if (gripCh > 0 && gripCh > Input.Channels())
        bcierr << "GripForceChannel=" << gripCh
               << " exceeds the number of input channels: " << Input.Channels();

    if (Parameter("GripForceMin") >= Parameter("GripForceMax"))
        bcierr << "GripForceMin must be smaller than GripForceMax";

    if (Parameter("MaxFeedbackDuration").InSampleBlocks() <= 0)
        bcierr << "MaxFeedbackDuration must be greater than 0";

    if (Parameter("NumberBlocks") <= 0)
        bcierr << "NumberBlocks must be greater than 0";
    if (Parameter("TrialsPerBlock") <= 0)
        bcierr << "TrialsPerBlock must be greater than 0";
    if (Parameter("BlockRestDuration").InSampleBlocks() < 0)
        bcierr << "BlockRestDuration must be non-negative";
    if (!std::string(Parameter("NumberOfTrials")).empty()) {
        int expectedTrials = Parameter("NumberBlocks") * Parameter("TrialsPerBlock");
        if (Parameter("NumberOfTrials") != expectedTrials)
            bciwarn << "NumberOfTrials should equal NumberBlocks * TrialsPerBlock ("
                    << expectedTrials << ") for block design bookkeeping";
    }
    if (Parameter("BlockLiftGains")->NumValues() < 1)
        bcierr << "BlockLiftGains must contain at least one value";
    if (Parameter("BlockGravityForces")->NumValues() < 1)
        bcierr << "BlockGravityForces must contain at least one value";
    for (int i = 0; i < Parameter("BlockLiftGains")->NumValues(); ++i)
        if (Parameter("BlockLiftGains")(i) < 0)
            bcierr << "BlockLiftGains must be non-negative";
    for (int i = 0; i < Parameter("BlockGravityForces")->NumValues(); ++i)
        if (Parameter("BlockGravityForces")(i) < 0)
            bcierr << "BlockGravityForces must be non-negative";

    Parameter("GripForceSmoothing");
    Parameter("MVCThreshold");
    Parameter("ParadigmType");
    Parameter("CursorDamping");

    // Scene-related parameter checks.
    const char* vectorParams[] = {
        "CameraPos", "CameraAim", "LightSourcePos", "CursorPos",
        "WorkspaceBoundaryPos", "WorkspaceBoundarySize"};
    for (auto p : vectorParams)
        if (Parameter(p)->NumValues() != 3)
            bcierr << "Parameter " << p << " must contain exactly 3 values";

    Parameter("WorkspaceBoundaryColor");
    const char* colorParams[] = {
        "CursorColorBack", "CursorColorFront", "TargetColor", "LightSourceColor"};
    for (auto p : colorParams)
        if (RGBColor(Parameter(p)) == RGBColor(RGBColor::NullColor))
            bcierr << "Parameter " << p << " has an invalid RGB value";

    bool showTextures = (Parameter("RenderingQuality") > 0);
    const char* texParams[] = {"CursorTexture", "TargetTexture", "WorkspaceBoundaryTexture"};
    for (auto p : texParams) {
        std::string filename = Parameter(p);
        if (showTextures && !filename.empty()) {
            int w, h;
            std::vector<GLubyte> ignored;
            if (!buffers::loadWindowsBitmap(FileUtils::AbsolutePath(filename), w, h, ignored))
                bcierr << "Invalid texture file \"" << filename << "\" for parameter " << p;
        }
    }

    if (Parameter("NumberTargets") > Parameter("Targets")->NumRows())
        bcierr << "Targets row count (" << Parameter("Targets")->NumRows()
               << ") is smaller than NumberTargets (" << Parameter("NumberTargets") << ")";

    Parameter("CursorWidth");
    Parameter("CameraProjection");
}

////////////////////////////////////////////////////////////////////////////////
// OnInitialize: called on Set Config; initializes the scene.
////////////////////////////////////////////////////////////////////////////////
void GripForceTask::OnInitialize(const SignalProperties& /*Input*/)
{
    // Read grip force parameters.
    mGripForceChannel   = Parameter("GripForceChannel");
    mGripForceMin       = Parameter("GripForceMin");
    mGripForceMax       = Parameter("GripForceMax");
    mGripForceSmoothing = Parameter("GripForceSmoothing");
    mParadigmType       = Parameter("ParadigmType");
    mMaxFeedbackDuration = static_cast<int>(
        Parameter("MaxFeedbackDuration").InSampleBlocks());
    mNumberBlocks = Parameter("NumberBlocks");
    mTrialsPerBlock = Parameter("TrialsPerBlock");
    mBlockRestDuration = static_cast<int>(Parameter("BlockRestDuration").InSampleBlocks());

    // Physics model parameters.
    mCursorDamping = Parameter("CursorDamping");
    mBlockLiftGains.clear();
    for (int i = 0; i < Parameter("BlockLiftGains")->NumValues(); ++i)
        mBlockLiftGains.push_back(Parameter("BlockLiftGains")(i));
    mBlockGravityForces.clear();
    for (int i = 0; i < Parameter("BlockGravityForces")->NumValues(); ++i)
        mBlockGravityForces.push_back(Parameter("BlockGravityForces")(i));

    // Grip force task uses only the Y axis; X/Z stay centered.
    float feedbackDuration = Parameter("MaxFeedbackDuration").InSampleBlocks();
    mCursorSpeedY = 100.0f / feedbackDuration / 2.0f;
    mCursorSpeedX = 0.0f;
    mCursorSpeedZ = 0.0f;

    // Colors.
    mCursorColorFront  = RGBColor(Parameter("CursorColorFront"));
    mCursorColorBack   = RGBColor(Parameter("CursorColorBack"));
    mTargetColorNormal = RGBColor(Parameter("TargetColor"));

    // Create or recreate the scene.
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

    // Tracking mode: pre-generate a sinusoidal target trajectory.
    if (mParadigmType == 2) {
        mTrackingTargetY.resize(mMaxFeedbackDuration);
        for (int i = 0; i < mMaxFeedbackDuration; ++i) {
            float phase = 2.0f * 3.14159f * i / (mMaxFeedbackDuration * 0.5f);
            mTrackingTargetY[i] = 50.0f + 35.0f * std::sin(phase);
        }
    }

    //  mGripVis.Send(CfgID::WindowTitle, "Grip Force");
    SignalProperties gripVisProperties(2, 1, SignalType::float32);
    gripVisProperties.ChannelLabels()[0] = "force_norm";
    gripVisProperties.ChannelLabels()[1] = "raw_voltage_norm";
    gripVisProperties.ValueUnit().SetRawMin(0).SetRawMax(1).SetGain(1).SetOffset(0).SetSymbol("");
    mGripVis.Send(gripVisProperties);
    mGripVis.Send(CfgID::NumSamples,  256);

    // NI6501 
    mMarkerEnabled = (Parameter("EnableMarker") != 0);
    mMarkerPulseMs = Parameter("MarkerPulseMs");
    InitMarkerDevice();

    mrWindow.Show();
    DisplayMessage("Ready to start...");
}

////////////////////////////////////////////////////////////////////////////////
// OnStartRun / OnStopRun
////////////////////////////////////////////////////////////////////////////////
void GripForceTask::OnStartRun()
{
    SetCameraControlsEnabled(false);
    ++mRunCount;
    mTrialCount = 0;
    mCurrentBlock = 0;
    mBlockTrialIndex = 0;
    mSessionTrialIndex = 0;
    mITIBlocksElapsed = 0;
    mBlockRestActive = false;
    mHits = mMisses = mTimeouts = 0;
    mMarkerCount = 0;
    State("SessionTrialIndex") = 0;
    State("BlockIndex") = 0;
    State("BlockTrialIndex") = 0;
    State("BlockPhase") = 0;
    State("SessionComplete") = 0;
    AppLog << "GripForceTask Run #" << mRunCount << " started\n";
    AppLog << "[Marker] === Run #" << mRunCount
           << " marker " << (mMarkerEnabled ? "enabled" : "disabled") << " ===\n";
    AppLog << "Paradigm type: "
           << (mParadigmType == 0 ? "VoluntaryGrip" :
               mParadigmType == 1 ? "CuedGrip" : "TrackingGrip") << "\n";
    DisplayMessage("Ready...");
}

void GripForceTask::OnStopRun()
{
    SaveCameraToPrm();
    AppLog << "Run #" << mRunCount << " ended: "
           << mTrialCount << " trials | "
           << mHits << " hits | "
           << mMisses << " misses | "
           << mTimeouts << " timeouts\n";
    if (mTrialCount > 0)
        AppLog << "Hit rate: "
               << (100 * mHits / mTrialCount) << "%\n";
    AppLog << "=====================\n";
    DisplayMessage("Finished");
}

////////////////////////////////////////////////////////////////////////////////
// OnTrialBegin
////////////////////////////////////////////////////////////////////////////////
void GripForceTask::OnTrialBegin()
{
    ++mTrialCount;
    ++mSessionTrialIndex;
    mCurrentBlock = ((mSessionTrialIndex - 1) / mTrialsPerBlock) + 1;
    mBlockTrialIndex = ((mSessionTrialIndex - 1) % mTrialsPerBlock) + 1;
    mBlockRestActive = false;
    mITIBlocksElapsed = 0;
    mCurFeedbackDuration = 0;
    mTrackingIndex = 0;
    State("SessionTrialIndex") = mSessionTrialIndex;
    State("BlockIndex") = mCurrentBlock;
    State("BlockTrialIndex") = mBlockTrialIndex;
    State("BlockPhase") = 1;
    ApplyBlockPhysicsParameters();

    // Set target visibility. TargetCode=1 is upper/squeeze; TargetCode=2 is lower/release.
    //
    for (int i = 0; i < mpFeedbackScene->NumTargets(); ++i) {
        mpFeedbackScene->SetTargetColor(mTargetColorNormal, i);
        mpFeedbackScene->SetTargetVisible(State("TargetCode") == i + 1, i);
    }

    // State trigger for target appearance.
    State("TargetAppear") = 1;
    State("TrialResult")  = 0;

    AppLog.Screen << "Trial #" << mTrialCount
                  << " Block " << mCurrentBlock << "/" << mNumberBlocks
                  << " Trial " << mBlockTrialIndex << "/" << mTrialsPerBlock
                  << " LiftGain " << mLiftGain
                  << " GravityForce " << mGravityForce
                  << " Target: " << (State("TargetCode") == 1 ? "Upper (squeeze)" : "Lower (release)")
                  << "\n";

    // Cue text.
    if (mParadigmType == 1)
        DisplayMessage(State("TargetCode") == 1 ? "Squeeze!" : "Release!");
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
    bool lastTrialInBlock = (mBlockTrialIndex >= mTrialsPerBlock);
    bool moreBlocksRemain = (mCurrentBlock < mNumberBlocks);
    mBlockRestActive = lastTrialInBlock && moreBlocksRemain && (mBlockRestDuration > 0);
    mITIBlocksElapsed = 0;
    State("BlockPhase") = mBlockRestActive ? 3 : 0;
    if (mSessionTrialIndex >= mNumberBlocks * mTrialsPerBlock)
        State("SessionComplete") = 1;
}

////////////////////////////////////////////////////////////////////////////////
// OnFeedbackBegin / OnFeedbackEnd
////////////////////////////////////////////////////////////////////////////////
void GripForceTask::OnFeedbackBegin()
{
    mCurFeedbackDuration = 0;

    // Start each trial from center and clear previous velocity/force state.
    mBallVelocityY = 0.0f;
    mCurrentGripForce = 0.0f;
    MoveCursorTo(50.0f, 50.0f, 50.0f);
    // Send a marker pulse at cursor onset.
    SendMarkerPulse();

    State("FeedbackOnset") = 1;
    State("BlockPhase") = 2;

    if (mParadigmType != 1)
        DisplayMessage("");
}

void GripForceTask::OnFeedbackEnd()
{
    State("FeedbackOnset") = 0;
    State("GripMarker") = 0;
    State("BlockPhase") = 1;

    int result = State("ResultCode");
    if (result == 0) {
        // Timeout.
        ++mTimeouts;
        State("TrialResult") = 3;
        AppLog.Screen << "-> Timeout\n";
        DisplayMessage("Timeout");
    } else if (result == State("TargetCode")) {
        // Hit.
        ++mHits;
        State("TrialResult") = 1;
        mpFeedbackScene->SetCursorColor(RGBColor::Yellow);
        mpFeedbackScene->SetTargetColor(RGBColor::Yellow, result - 1);
        AppLog.Screen << "-> Hit!\n";
        DisplayMessage("Hit!");
    } else {
        // Miss or wrong target.
        ++mMisses;
        State("TrialResult") = 2;
        AppLog.Screen << "-> Miss\n";
        DisplayMessage("Miss");
    }
}

////////////////////////////////////////////////////////////////////////////////
// DoFeedback: core loop, called once per sample block.
////////////////////////////////////////////////////////////////////////////////
void GripForceTask::DoFeedback(const GenericSignal& ControlSignal, bool& doProgress)
{
    float rawGrip = GetGripForceFromSignal(ControlSignal);
    float normGrip = NormalizeGripForce(rawGrip);

    // Direct proportional mapping: no smoothing.
    mCurrentGripForce = normGrip;

    int normVal = static_cast<int>(mCurrentGripForce * 65535);
    State("GripForceNormalized") = std::max(0, std::min(65535, normVal));
    int rawVal = static_cast<int>(rawGrip);
    State("GripForceRaw") = std::max(0, std::min(65535, rawVal));

    GenericSignal gripVisSignal(2, 1);
    gripVisSignal(0, 0) = mCurrentGripForce;
    gripVisSignal(1, 0) = NormalizeGripForce(rawGrip);
    mGripVis.Send(gripVisSignal);

    Vector3D pos = mpFeedbackScene->CursorPosition();
    float upForce = mLiftGain * mCurrentGripForce;
    float gravity = std::fabs(mGravityForce);
    float netForce = upForce - gravity;
    mBallVelocityY += netForce;
    mBallVelocityY *= mCursorDamping;
    pos.y += mBallVelocityY;

    pos.x = 50.0f;
    pos.z = 50.0f;

    float r = mpFeedbackScene->CursorRadius();
    if (pos.y <= r)
    {
        pos.y = r;
        mBallVelocityY = 0.0f;
    }
    else if (pos.y >= 100.0f - r)
    {
        pos.y = 100.0f - r;
        mBallVelocityY = 0.0f;
    }

    mpFeedbackScene->SetCursorPosition(pos);
    mpFeedbackScene->SetCursorVelocity({0, mBallVelocityY, 0});

    const float coordToState = ((1 << cCursorPosBits) - 1) / 100.0f;
    State("CursorPosX") = static_cast<int>(pos.x * coordToState);
    State("CursorPosY") = static_cast<int>(pos.y * coordToState);
    State("CursorPosZ") = static_cast<int>(pos.z * coordToState);

    if (mpFeedbackScene->TargetHit(State("TargetCode") - 1))
        State("ResultCode") = State("TargetCode");

    doProgress = (++mCurFeedbackDuration >= mMaxFeedbackDuration)
              || (State("ResultCode") != 0);
}

// Remaining Do* callbacks are required by FeedbackTask but need no extra logic here.
void GripForceTask::DoPreRun(const GenericSignal&, bool&) {}
void GripForceTask::DoPreFeedback(const GenericSignal&, bool&) {}
void GripForceTask::DoPostFeedback(const GenericSignal&, bool&) {}
void GripForceTask::DoITI(const GenericSignal&, bool& doProgress)
{
    if (!mBlockRestActive)
    {
        State("BlockPhase") = 0;
        return;
    }

    State("BlockPhase") = 3;
    DisplayMessage("Rest");
    doProgress = (++mITIBlocksElapsed >= mBlockRestDuration);
    if (doProgress)
    {
        mBlockRestActive = false;
        DisplayMessage("");
        State("BlockPhase") = 0;
    }
}

////////////////////////////////////////////////////////////////////////////////
// Helpers.
////////////////////////////////////////////////////////////////////////////////

// Read grip force magnitude from the configured signal channel.
float GripForceTask::GetGripForceFromSignal(const GenericSignal& signal) const
{
    int ch = (mGripForceChannel > 0) ? (mGripForceChannel - 1) : 0;
    if (ch < signal.Channels() && signal.Elements() > 0)
        return std::fabs(static_cast<float>(signal(ch, 0)));
    return 0.0f;
}

// Normalize raw grip force to [0, 1].
float GripForceTask::NormalizeGripForce(float raw) const
{
    if (mGripForceMax <= mGripForceMin)
        return 0.5f;
    float norm = (raw - mGripForceMin) / (mGripForceMax - mGripForceMin);
    return std::max(0.0f, std::min(1.0f, norm));
}

// Move the cursor and update its color from the Z position.
void GripForceTask::MoveCursorTo(float x, float y, float z)
{
    float t = z / 100.0f;
    RGBColor color = t * mCursorColorFront + (1.0f - t) * mCursorColorBack;
    mpFeedbackScene->SetCursorColor(color);
    mpFeedbackScene->SetCursorPosition({x, y, z});
}

// Hook for more advanced tracking trajectory generation.
void GripForceTask::GenerateTrackingTarget()
{
    // Reserved for file-based or more complex trajectory generation.
}

// Display a centered text prompt.
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

void GripForceTask::ApplyBlockPhysicsParameters()
{
    auto valueForBlock = [](const std::vector<float>& values, int blockIndex) {
        int index = std::max(0, blockIndex - 1);
        index = std::min(index, static_cast<int>(values.size()) - 1);
        return values[index];
    };

    mLiftGain = valueForBlock(mBlockLiftGains, mCurrentBlock);
    mGravityForce = valueForBlock(mBlockGravityForces, mCurrentBlock);
}

void GripForceTask::UpdateTargetVisibility()
{
    for (int i = 0; i < mpFeedbackScene->NumTargets(); ++i)
        mpFeedbackScene->SetTargetVisible(State("TargetCode") == i + 1, i);
}

////////////////////////////////////////////////////////////////////////////////
// NI6501 marker.
////////////////////////////////////////////////////////////////////////////////
void GripForceTask::InitMarkerDevice()
{
    CloseMarkerDevice();
    if (!mMarkerEnabled)
        return;

    if (!Dylib::nicaiu_Loaded()) {
        bciwarn << "EnableMarker=1 but NI-DAQmx runtime nicaiu.dll could not be loaded; marker disabled";
        AppLog << "[Marker] disabled: failed to load nicaiu.dll\n";
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
        bciwarn << "NI6501 marker initialization failed; marker disabled: " << buf;
        AppLog << "[Marker] initialization failed; disabled: " << buf << "\n";
        if (task) DAQmxClearTask(task);
        mMarkerEnabled = false;
        mMarkerTask = nullptr;
        return;
    }

    // Initialize both lines low.
    uInt8 low[2] = {0, 0};
    DAQmxWriteDigitalLines(task, 1, 1, 10.0, DAQmx_Val_GroupByChannel, low, NULL, NULL);
    mMarkerTask = task;
    AppLog << "[Marker] NI6501 ready (Dev1/port0/line0, pulse "
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
    // Raise P0.1 immediately, log the result, then lower it asynchronously.
    uInt8 high[2] = {0, 1};   // P0.0 0, P0.1 1
    int32 err = DAQmxWriteDigitalLines(task, 1, 1, 10.0,
                                       DAQmx_Val_GroupByChannel, high, NULL, NULL);
    if (DAQmxFailed(err)) {
        char buf[2048] = {0};
        DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
        AppLog << "[Marker] pulse failed: " << buf << "\n";
        return;
    }

    ++mMarkerCount;
    State("GripMarker") = (mMarkerCount % 255) + 1;

    AppLog << "[Marker] #" << mMarkerCount
           << " P0.0 pulse (" << pulseMs << "ms) @ cursor onset\n";

    std::thread([task, pulseMs]() {
        uInt8 low[2] = {0, 0};
        std::this_thread::sleep_for(std::chrono::milliseconds(pulseMs));
        DAQmxWriteDigitalLines(task, 1, 1, 10.0, DAQmx_Val_GroupByChannel, low, NULL, NULL);
    }).detach();
}
// Historical CMakeLists.txt example kept for local reference.
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
