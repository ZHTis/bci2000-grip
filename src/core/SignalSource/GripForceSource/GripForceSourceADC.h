////////////////////////////////////////////////////////////////////////////////
// GripForceSourceADC.h
//
// BCI2000 Signal Source Module: Grip Force Source
//
// A dedicated signal source for GripForceTask, fully isolated from
// SignalGenerator.
//
// Concept: a "joystick / lever" model.
//   - Record a baseline at StartRun, then output the displacement magnitude
//     relative to that baseline as the upward force applied to the ball.
//   - InputMode=0 (Mouse): displacement magnitude of the mouse relative to its
//     initial position = force. Back to the initial position -> 0 -> the ball
//     falls under gravity.
//   - InputMode=1 (External): reserved for an external analog sensor (Intan).
//     A larger sensor reading is equivalent to a larger mouse displacement.
//
// Data chain:
//   GripForceSource -> DummySignalProcessing (pass-through) -> GripForceTask
////////////////////////////////////////////////////////////////////////////////
#ifndef GRIP_FORCE_SOURCE_ADC_H
#define GRIP_FORCE_SOURCE_ADC_H

#include "Clock.h"
#include "GenericADC.h"

class GripForceSourceADC : public GenericADC
{
  public:
    GripForceSourceADC();
    ~GripForceSourceADC();

    void Publish() override;
    void AutoConfig(const SignalProperties &) override;
    void Preflight(const SignalProperties &, SignalProperties &) const override;
    void Initialize(const SignalProperties &, const SignalProperties &) override;
    void StartRun() override;
    void Process(const GenericSignal &, GenericSignal &) override;
    void Halt() override;

    // Non-realtime source: allows launching without hardware via --EvaluateTiming=0
    bool IsRealTimeSource() const override
    {
        return false;
    }

  private:
    // Read the current raw lever value (mouse coordinate or external sensor)
    double ReadRawInput() const;

    // Configuration
    int    mInputMode;       // 0 = Mouse, 1 = External (Intan)
    int    mMouseAxis;       // 0 = Y (default), 1 = X
    double mGain;            // displacement -> force scaling
    int    mSourceCh;        // number of channels
    int    mTargetChannel;   // channel (0-based) the force is written to; others output 0

    // Runtime state
    double mBaseline;        // raw reading captured at StartRun
    bool   mHaveBaseline;

    Clock mClock;
};

#endif // GRIP_FORCE_SOURCE_ADC_H
