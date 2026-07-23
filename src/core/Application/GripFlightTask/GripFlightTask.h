#ifndef GRIP_FLIGHT_TASK_H
#define GRIP_FLIGHT_TASK_H

#include "AssetManager.h"
#include "FeedbackTask.h"
#include "FlightPhysics.h"
#include "GameStateMachine.h"
#include "MapLoader.h"
#include "SideScrollScene.h"
#include "TextField.h"

#include <memory>

class GripFlightTask : public FeedbackTask
{
  public:
    GripFlightTask();
    ~GripFlightTask() override;

  private:
    void OnPreflight(const SignalProperties&) const override;
    void OnInitialize(const SignalProperties&) override;
    void OnStartRun() override;
    void OnStopRun() override;
    void OnTrialBegin() override;
    void OnTrialEnd() override;
    void OnFeedbackBegin() override;
    void OnFeedbackEnd() override;
    void DoPreRun(const GenericSignal&, bool&) override;
    void DoPreFeedback(const GenericSignal&, bool&) override;
    void DoFeedback(const GenericSignal&, bool&) override;
    void DoPostFeedback(const GenericSignal&, bool&) override;
    void DoITI(const GenericSignal&, bool&) override;

    float RawGrip(const GenericSignal&) const;
    float NormalizeGrip(float) const;
    void SetMessage(const std::string&);
    void WriteStates(float raw, float normalized, int collisionObject);
    void LoadAssets();

    GUI::DisplayWindow& mrWindow;
    std::unique_ptr<SideScrollScene> mpScene;
    TextField* mpMessage;
    FlightMap mMap;
    AssetManager mAssets;
    FlightPhysics mPhysics;
    GameStateMachine mStateMachine;
    float mDt;
    float mGripMin;
    float mGripMax;
    float mSmoothedGrip;
    float mSmoothing;
    float mStartX;
    float mStartY;
    int mGripChannel;
    int mTrialResult;
};

#endif
