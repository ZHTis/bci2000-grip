////////////////////////////////////////////////////////////////////////////////
// GripForceSourceADC.cpp
//
// BCI2000 Signal Source Module: Grip Force Source
// See GripForceSourceADC.h for the design notes.
////////////////////////////////////////////////////////////////////////////////
#include "GripForceSourceADC.h"

#include "BCIStream.h"
#include "GenericSignal.h"
#include "MeasurementUnits.h"

#if _WIN32
#include <Windows.h>
#elif USE_QT
#include <QApplication>
#include <QCursor>
#include <QDesktopWidget>
#endif

#include <algorithm>
#include <cmath>

// Register as a level-1 filter (signal source layer)
RegisterFilter(GripForceSourceADC, 1);

GripForceSourceADC::GripForceSourceADC()
    : mInputMode(0), mMouseAxis(0), mGain(1.0), mSourceCh(1),
      mTargetChannel(0), mBaseline(0.0), mHaveBaseline(false)
{
}

GripForceSourceADC::~GripForceSourceADC()
{
}

void GripForceSourceADC::Publish()
{
    BEGIN_PARAMETER_DEFINITIONS
        "Source:Signal%20Properties int SourceCh= 1 "
            "1 1 % // number of output channels (GripForceTask needs only 1)",
        "Source:Signal%20Properties int SampleBlockSize= 32 "
            "32 1 % // number of samples per block",
        "Source:Signal%20Properties int SamplingRate= 256Hz "
            "256Hz 1 % // sample rate",

        "Source list SourceChOffset= 1 auto % % % "
            "// per-channel offset",
        "Source list SourceChGain= 1 auto % % % "
            "// per-channel gain",
        "Source list ChannelNames= 1 auto % % % "
            "// per-channel names",

        // ---- GripForceSource specific parameters ----
        "Source:GripForce int InputMode= 0 0 0 1 "
            "// input source: 0: mouse displacement, 1: external sensor (Intan) (enumeration)",
        "Source:GripForce int MouseAxis= 0 0 0 1 "
            "// mouse axis: 0: Y (vertical), 1: X (horizontal) (enumeration)",
        "Source:GripForce float Gain= 1.0 1.0 0.0 % "
            "// displacement -> force scaling factor",
        "Source:GripForce int TargetChannel= 1 1 1 % "
            "// channel the force is written to (1-based); other channels output 0",
    END_PARAMETER_DEFINITIONS
}

void GripForceSourceADC::AutoConfig(const SignalProperties &)
{
    int numChannels = ActualParameter("SourceCh");
    Parameter("SourceChOffset")->SetNumValues(numChannels);
    Parameter("SourceChGain")->SetNumValues(numChannels);
    for (int i = 0; i < numChannels; ++i)
    {
        Parameter("SourceChOffset")(i) = 0;
        Parameter("SourceChGain")(i) = 1;   // force is output in physical units, gain 1
    }
}

void GripForceSourceADC::Preflight(const SignalProperties &, SignalProperties &Output) const
{
    Parameter("SamplingRate").InHertz();

    int targetCh = Parameter("TargetChannel");
    if (targetCh < 1 || targetCh > Parameter("SourceCh"))
        bcierr << "TargetChannel=" << targetCh
               << " out of channel range [1, " << Parameter("SourceCh") << "]";

    int inputMode = Parameter("InputMode");
    if (inputMode == 1)
    {
        // External sensor (Intan) mode: remove this notice once hardware read is implemented
        bciwarn << "InputMode=1 (External/Intan) hardware read is not implemented yet; "
                   "the source will output 0. Implement it in ReadRawInput().";
    }

    // Single-channel float signal to preserve displacement-magnitude precision
    Output = SignalProperties(Parameter("SourceCh"),
                              Parameter("SampleBlockSize"),
                              SignalType::float32);
}

void GripForceSourceADC::Initialize(const SignalProperties &, const SignalProperties &)
{
    mInputMode     = Parameter("InputMode");
    mMouseAxis     = Parameter("MouseAxis");
    mGain          = Parameter("Gain");
    mSourceCh      = Parameter("SourceCh");
    mTargetChannel = Parameter("TargetChannel") - 1;   // to 0-based
    mHaveBaseline  = false;

    mClock.SetInterval(Time::Seconds(MeasurementUnits::SampleBlockDuration()));
    mClock.Start();
}

void GripForceSourceADC::StartRun()
{
    // Record the baseline: the zero point of the "lever"
    mBaseline = ReadRawInput();
    mHaveBaseline = true;
}

// Read the current raw lever value
double GripForceSourceADC::ReadRawInput() const
{
    if (mInputMode == 0)
    {
        // ---- Mouse mode: left button = grip ----
        // Hold left button -> full force (1); release -> 0.
        // Direct on/off grip; no position/baseline needed so it never gets
        // stuck at a screen edge.
#if _WIN32
        enum { isPressed = 0x8000 };
        bool leftButton = (::GetAsyncKeyState(VK_LBUTTON) & isPressed) != 0;
        return leftButton ? 1.0 : 0.0;
#elif USE_QT
        bool leftButton =
            (QApplication::mouseButtons() & Qt::LeftButton) != 0;
        return leftButton ? 1.0 : 0.0;
#else
        return 0.0;
#endif
    }
    else
    {
        // ---- External sensor (Intan) mode ----
        // TODO: read the Intan analog channel here and return the raw sensor value.
        //       Baseline and gain logic is reused automatically; nothing else changes.
        return 0.0;
    }
}

void GripForceSourceADC::Process(const GenericSignal &, GenericSignal &Output)
{
    if (!mHaveBaseline)
    {
        mBaseline = ReadRawInput();
        mHaveBaseline = true;
    }

    double raw = ReadRawInput();

    double force;
    if (mInputMode == 0)
    {
        // Mouse mode: raw is already 0/1 (button state); baseline not used.
        // Released -> 0 -> ball falls under gravity. Held -> Gain (default 1).
        force = raw * mGain;
    }
    else
    {
        // External sensor: force = displacement magnitude relative to baseline.
        force = std::fabs(raw - mBaseline) * mGain;
    }

    // Write to the target channel; other channels output 0
    for (int ch = 0; ch < Output.Channels(); ++ch)
    {
        double v = (ch == mTargetChannel) ? force : 0.0;
        for (int sample = 0; sample < Output.Elements(); ++sample)
            Output(ch, sample) = v;
    }

    mClock.Wait();
}

void GripForceSourceADC::Halt()
{
    mClock.Stop();
}
