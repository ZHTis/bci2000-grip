#include "GripFlightTask.h"

#include "FileUtils.h"

#include <algorithm>
#include <cmath>

RegisterFilter(GripFlightTask, 3);

GripFlightTask::GripFlightTask()
    : mrWindow(Window()), mpMessage(nullptr), mDt(0.02f), mGripMin(0), mGripMax(5), mSmoothedGrip(0),
      mSmoothing(0.2f), mStartX(20), mStartY(50), mGripChannel(1), mTrialResult(0)
{
    BEGIN_PARAMETER_DEFINITIONS
        "Application:GripInput int GripForceChannel= 1 1 1 % // 1-based ControlSignal channel",
        "Application:GripInput float GripForceMin= 0 0 % % // raw force mapped to zero",
        "Application:GripInput float GripForceMax= 5 5 % % // raw force mapped to one",
        "Application:GripInput float GripForceSmoothing= 0.2 0.2 0 1 // low-pass update coefficient",

        "Application:FlightPhysics float ForwardSpeed= 25 25 0 % // world units per second",
        "Application:FlightPhysics float LiftGain= 70 70 0 % // upward acceleration at full grip",
        "Application:FlightPhysics float Gravity= 30 30 0 % // downward acceleration",
        "Application:FlightPhysics float FlightDamping= 0.98 0.98 0 1 // velocity retention at 60Hz",
        "Application:FlightPhysics floatlist PlayerStart= 2 20 50 % % // initial world x y",
        "Application:FlightPhysics floatlist PlayerSize= 2 8 8 % % // collision width height",

        "Application:FlightMap float WorldHeight= 100 100 1 % // vertical world size",
        "Application:FlightMap float CameraViewWidth= 160 160 10 % // fallback view width before window size is available",
        "Application:FlightMap string MapFile= % % % % // optional CSV map; empty uses built-in map",

        "Application:FlightAssets string PlayerImage= % % % % // PNG image for player",
        "Application:FlightAssets string BackgroundImage= % % % % // background image",
        "Application:FlightAssets string TopObstacleImage= % % % % // PNG image for top obstacles",
        "Application:FlightAssets string BottomObstacleImage= % % % % // PNG image for bottom obstacles",
    END_PARAMETER_DEFINITIONS

    BEGIN_STATE_DEFINITIONS
        "GamePhase 4 0 0 0",
        "BallWorldX 24 0 0 0",
        "BallWorldY 16 0 0 0",
        "BallVelocityY 16 32768 0 0",
        "GripForceRaw 16 0 0 0",
        "GripForceNormalized 16 0 0 0",
        "CameraWorldX 24 0 0 0",
        "Collision 1 0 0 0",
        "CollisionObject 16 0 0 0",
        "FlightTrialResult 2 0 0 0",
    END_STATE_DEFINITIONS

    mpMessage = new TextField(mrWindow);
    mpMessage->SetTextColor(RGBColor::White).SetColor(RGBColor::NullColor)
        .SetTextHeight(0.08f).SetText("Ready").SetObjectRect({0.2f, 0.4f, 0.8f, 0.6f});
}

GripFlightTask::~GripFlightTask()
{
    delete mpMessage;
}

void GripFlightTask::OnPreflight(const SignalProperties& input) const
{
    if (Parameter("SamplingRate").InHertz() <= 0)
        bcierr << "SamplingRate must be greater than zero";
    if (Parameter("SampleBlockSize") <= 0)
        bcierr << "SampleBlockSize must be greater than zero";
    if (Parameter("NumberTargets") != 1)
        bcierr << "GripFlightTask requires NumberTargets=1";
    if (Parameter("GripForceChannel") < 1 || Parameter("GripForceChannel") > input.Channels())
        bcierr << "GripForceChannel must identify an input channel (1.." << input.Channels() << ")";
    if (Parameter("GripForceMax") <= Parameter("GripForceMin"))
        bcierr << "GripForceMax must be greater than GripForceMin";
    if (Parameter("WorldHeight") <= 0 || Parameter("CameraViewWidth") <= 0)
        bcierr << "WorldHeight and CameraViewWidth must be greater than zero";
    if (Parameter("ForwardSpeed") <= 0)
        bcierr << "ForwardSpeed must be greater than zero";
    if (Parameter("LiftGain") < 0 || Parameter("Gravity") < 0)
        bcierr << "LiftGain and Gravity must not be negative";
    if (Parameter("FlightDamping") < 0 || Parameter("FlightDamping") > 1)
        bcierr << "FlightDamping must be between zero and one";
    ParamRef playerSize = Parameter("PlayerSize");
    if (playerSize->NumValues() != 2 || playerSize(0) <= 0 || playerSize(1) <= 0)
        bcierr << "PlayerSize must contain two positive values";
    ParamRef playerStart = Parameter("PlayerStart");
    if (playerStart->NumValues() != 2)
        bcierr << "PlayerStart must contain x and y";

    const std::string mapFile = Parameter("MapFile");
    if (!mapFile.empty())
    {
        FlightMap testMap;
        std::string error;
        if (!testMap.LoadCsv(FileUtils::AbsolutePath(mapFile), error))
            bcierr << error;
    }

    // Asset paths are optional, but BCI2000 requires every parameter used in
    // Initialize() to be accessed during Preflight().
    Parameter("PlayerImage");
    Parameter("BackgroundImage");
    Parameter("TopObstacleImage");
    Parameter("BottomObstacleImage");
}

void GripFlightTask::LoadAssets()
{
    mAssets.Clear();
    const struct { const char* id; const char* parameter; } entries[] = {
        {"player", "PlayerImage"}, {"background", "BackgroundImage"},
        {"obstacle_top", "TopObstacleImage"}, {"obstacle_bottom", "BottomObstacleImage"},
    };
    for (const auto& entry : entries)
    {
        const std::string filename = Parameter(entry.parameter);
        if (!filename.empty())
            mAssets.Register(entry.id, FileUtils::AbsolutePath(filename));
    }
}

void GripFlightTask::OnInitialize(const SignalProperties&)
{
    mGripChannel = Parameter("GripForceChannel");
    mGripMin = Parameter("GripForceMin");
    mGripMax = Parameter("GripForceMax");
    mSmoothing = Parameter("GripForceSmoothing");
    mDt = static_cast<float>(Parameter("SampleBlockSize")) /
          static_cast<float>(Parameter("SamplingRate").InHertz());

    ParamRef start = Parameter("PlayerStart");
    ParamRef size = Parameter("PlayerSize");
    mStartX = start(0);
    mStartY = start(1);
    mPhysics.Configure(Parameter("ForwardSpeed"), Parameter("LiftGain"), Parameter("Gravity"),
                       Parameter("FlightDamping"));

    const std::string mapFile = Parameter("MapFile");
    if (mapFile.empty())
        mMap.LoadBuiltIn();
    else
    {
        std::string error;
        if (!mMap.LoadCsv(FileUtils::AbsolutePath(mapFile), error))
            bcierr << error;
    }
    LoadAssets();
    mpScene.reset(new SideScrollScene(mrWindow));
    mpScene->Initialize(mMap, mAssets, Parameter("WorldHeight"), Parameter("CameraViewWidth"), size(0), size(1));
    mpScene->SetVisible(false);
    mrWindow.Show();
}

void GripFlightTask::OnStartRun()
{
    mStateMachine.Reset();
    State("GamePhase") = GameStateMachine::Idle;
    SetMessage("Ready");
}

void GripFlightTask::OnStopRun()
{
    mpScene->SetVisible(false);
    mStateMachine.Enter(GameStateMachine::SessionComplete);
    State("GamePhase") = mStateMachine.Current();
    SetMessage("Run complete");
}

void GripFlightTask::OnTrialBegin()
{
    mStateMachine.Enter(GameStateMachine::Countdown);
    State("GamePhase") = mStateMachine.Current();
    mTrialResult = 0;
    State("FlightTrialResult") = 0;
    State("Collision") = 0;
    SetMessage("Prepare");
}

void GripFlightTask::OnTrialEnd()
{
    mpScene->SetVisible(false);
    mStateMachine.Enter(GameStateMachine::InterTrial);
    State("GamePhase") = mStateMachine.Current();
}

void GripFlightTask::OnFeedbackBegin()
{
    mPhysics.Reset(mStartX, mStartY);
    mSmoothedGrip = 0;
    mTrialResult = 0;
    mStateMachine.Enter(GameStateMachine::Playing);
    mpScene->SetVisible(true);
    mpScene->Update(mStartX, mStartY);
    SetMessage("");
    State("GamePhase") = mStateMachine.Current();
}

void GripFlightTask::OnFeedbackEnd()
{
    if (mTrialResult == 0)
        mTrialResult = 2;
    State("FlightTrialResult") = mTrialResult;
    State("ResultCode") = mTrialResult == 1 ? State("TargetCode") : 0;
    mStateMachine.Enter(mTrialResult == 1 ? GameStateMachine::TrialSuccess : GameStateMachine::TrialFailure);
    State("GamePhase") = mStateMachine.Current();
    SetMessage(mTrialResult == 1 ? "Complete" : "Collision");
}

void GripFlightTask::DoPreRun(const GenericSignal&, bool&) {}
void GripFlightTask::DoPreFeedback(const GenericSignal&, bool&) {}
void GripFlightTask::DoPostFeedback(const GenericSignal&, bool&) {}
void GripFlightTask::DoITI(const GenericSignal&, bool&) {}

float GripFlightTask::RawGrip(const GenericSignal& signal) const
{
    return std::fabs(static_cast<float>(signal(mGripChannel - 1, 0)));
}

float GripFlightTask::NormalizeGrip(float raw) const
{
    return std::max(0.0f, std::min(1.0f, (raw - mGripMin) / (mGripMax - mGripMin)));
}

void GripFlightTask::DoFeedback(const GenericSignal& signal, bool& doProgress)
{
    const float raw = RawGrip(signal);
    const float normalized = NormalizeGrip(raw);
    mSmoothedGrip += mSmoothing * (normalized - mSmoothedGrip);
    mPhysics.Step(mSmoothedGrip, mDt);
    const FlightBody& body = mPhysics.Body();
    mpScene->Update(body.x, body.y);

    int collisionObject = -1;
    const bool collision = mpScene->OutsideWorld(body.y) || mpScene->Collides(body.x, body.y, collisionObject);
    if (collision)
    {
        mTrialResult = 2;
        mStateMachine.Enter(GameStateMachine::Hit);
        doProgress = true;
    }
    else if (body.x >= mMap.FinishX())
    {
        mTrialResult = 1;
        doProgress = true;
    }
    mStateMachine.AdvanceBlock();
    WriteStates(raw, mSmoothedGrip, collisionObject);
}

void GripFlightTask::WriteStates(float raw, float normalized, int collisionObject)
{
    const FlightBody& body = mPhysics.Body();
    const int x = std::max(0, std::min(16777215, static_cast<int>(body.x * 100)));
    const int y = std::max(0, std::min(65535, static_cast<int>(body.y * 100)));
    const int velocity = std::max(0, std::min(65535, static_cast<int>(body.velocityY * 100 + 32768)));
    State("GamePhase") = mStateMachine.Current();
    State("BallWorldX") = x;
    State("BallWorldY") = y;
    State("BallVelocityY") = velocity;
    State("GripForceRaw") = std::max(0, std::min(65535, static_cast<int>(raw * 10000)));
    State("GripForceNormalized") = std::max(0, std::min(65535, static_cast<int>(normalized * 65535)));
    State("CameraWorldX") = std::max(0, std::min(16777215, static_cast<int>(mpScene->CameraX() * 100)));
    State("Collision") = collisionObject >= 0 || mpScene->OutsideWorld(body.y);
    State("CollisionObject") = collisionObject + 1;
}

void GripFlightTask::SetMessage(const std::string& message)
{
    mpMessage->SetText(message);
    mpMessage->SetVisible(!message.empty());
}
