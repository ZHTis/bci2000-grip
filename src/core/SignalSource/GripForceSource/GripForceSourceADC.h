////////////////////////////////////////////////////////////////////////////////
// GripForceSourceADC.h
//
// BCI2000 Signal Source Module: Grip Force Source
//
// InputMode:
//   0 - Mouse left button (simulation, 0/1 binary)
//   1 - Intan Recording Controller TCP (AUX analog channel)
//   2 - Arduino serial port, ASCII line e.g. "0.532\n" (MPX5050GP)
////////////////////////////////////////////////////////////////////////////////
#ifndef GRIP_FORCE_SOURCE_ADC_H
#define GRIP_FORCE_SOURCE_ADC_H

#include "ADCFilter.h"        // BCI2000 Source base class (wraps GenericADC)
#include "Clock.h"

#include <string>

// Platform socket type
#if _WIN32
  #ifndef WIN32_LEAN_AND_MEAN
  #define WIN32_LEAN_AND_MEAN
  #endif
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #pragma comment(lib, "Ws2_32.lib")
  // INVALID_SOCKET is already defined by winsock2.h
#else
  #include <sys/socket.h>
  #include <arpa/inet.h>
  #include <netinet/in.h>
  // Mirror the Windows constant for cross-platform code
  using SOCKET = int;
  static constexpr SOCKET INVALID_SOCKET = -1;
#endif

class GripForceSourceADC : public ADCFilter
{
public:
    GripForceSourceADC();
    ~GripForceSourceADC() override;

    // BCI2000 ADC interface
    void Publish()   override;
    void AutoConfig(const SignalProperties&) override;
    void Preflight (const SignalProperties&, SignalProperties&) const override;
    void Initialize(const SignalProperties&, const SignalProperties&) override;
    void StartRun() override;
    void Process   (const GenericSignal&, GenericSignal&) override;
    void Halt()     override;

private:
    // ---- Core parameters ----
    int    mInputMode;
    int    mMouseAxis;
    double mGain;
    int    mSourceCh;
    int    mTargetChannel;     // 0-based
    double mBaseline;
    bool   mHaveBaseline;

    Clock  mClock;

    // ---- Mode 1: Intan TCP ----
    mutable SOCKET mIntanSocket;
    int            mIntanAuxChannel;   // 0-based
    mutable double mLastIntanValue;

    void   OpenIntanSocket(const std::string& address, int port);
    void   CloseIntanSocket();
    double ParseIntanResponse(const std::string& resp) const;

    // ---- Mode 2: Arduino serial ----
#if _WIN32
    mutable HANDLE mSerialHandle;
#else
    mutable int    mSerialFd;
#endif
    mutable double      mLastSerialValue;
    mutable std::string mSerialLineBuffer;  // accumulates partial lines between blocks

    void OpenSerialPort (const std::string& portName, int baud);
    void CloseSerialPort();

    // ---- Unified input read ----
    double ReadRawInput() const;
};

#endif // GRIP_FORCE_SOURCE_ADC_H
