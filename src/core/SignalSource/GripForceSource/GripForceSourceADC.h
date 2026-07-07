////////////////////////////////////////////////////////////////////////////////
// GripForceSourceADC.h
//
// BCI2000 Signal Source Module: Grip Force Source
//
// InputMode:
//   0 - Mouse left button, binary 0/1 for simulation.
//   1 - Arduino serial CSV line: temperature,voltage
////////////////////////////////////////////////////////////////////////////////
#ifndef GRIP_FORCE_SOURCE_ADC_H
#define GRIP_FORCE_SOURCE_ADC_H

#include "Clock.h"
#include "GenericADC.h"

#include <string>

#if _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <Windows.h>
#else
#include <termios.h>
#endif

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

    bool IsRealTimeSource() const override
    {
        return false;
    }

  private:
    double ReadRawInput() const;
    double ReadMouseInput() const;
    double ReadArduinoVoltage() const;
    bool ParseArduinoLine(const std::string &, double &) const;

    void OpenSerialPort(const std::string &, int);
    void CloseSerialPort();

    int mInputMode;
    double mGain;
    int mSourceCh;
    int mTargetChannel;

    mutable double mLastSerialValue;
    mutable std::string mSerialLineBuffer;

#if _WIN32
    mutable HANDLE mSerialHandle;
#else
    mutable int mSerialFd;
#endif

    Clock mClock;
};

#endif // GRIP_FORCE_SOURCE_ADC_H
