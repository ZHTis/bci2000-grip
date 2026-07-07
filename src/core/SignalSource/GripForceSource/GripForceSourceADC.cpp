////////////////////////////////////////////////////////////////////////////////
// GripForceSourceADC.cpp
//
// BCI2000 Signal Source Module: Grip Force Source
//
// InputMode:
//   0 - Mouse left button, binary 0/1 for simulation.
//   1 - Arduino serial CSV line: temperature,voltage
////////////////////////////////////////////////////////////////////////////////
#include "GripForceSourceADC.h"

#include "BCIStream.h"
#include "GenericSignal.h"
#include "MeasurementUnits.h"

#if _WIN32
#include <Windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <sstream>

RegisterFilter(GripForceSourceADC, 1);

GripForceSourceADC::GripForceSourceADC()
    : mInputMode(0),
      mGain(1.0),
      mSourceCh(1),
      mTargetChannel(0),
      mLastSerialValue(0.0)
#if _WIN32
      ,
      mSerialHandle(INVALID_HANDLE_VALUE)
#else
      ,
      mSerialFd(-1)
#endif
{
}

GripForceSourceADC::~GripForceSourceADC()
{
    CloseSerialPort();
}

void GripForceSourceADC::Publish()
{
    BEGIN_PARAMETER_DEFINITIONS
        "Source:Signal%20Properties int SourceCh= 1 "
            "1 1 % // number of output channels",
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

        "Source:GripForce int InputMode= 0 0 0 1 "
            "// input source: 0 mouse left button, 1 Arduino serial voltage (enumeration)",
        "Source:GripForce float Gain= 1.0 1.0 0.0 % "
            "// raw input scaling factor",
        "Source:GripForce int TargetChannel= 1 1 1 % "
            "// channel the force is written to (1-based); other channels output 0",

        "Source:GripForce string SerialPort= COM6 % % % "
            "// Arduino serial port, e.g. COM6",
        "Source:GripForce int SerialBaud= 115200 115200 9600 % "
            "// Arduino baud rate",
    END_PARAMETER_DEFINITIONS
}

void GripForceSourceADC::AutoConfig(const SignalProperties &)
{
    int numChannels = ActualParameter("SourceCh");
    Parameter("SourceChOffset")->SetNumValues(numChannels);
    Parameter("SourceChGain")->SetNumValues(numChannels);
    Parameter("ChannelNames")->SetNumValues(numChannels);
    for (int i = 0; i < numChannels; ++i)
    {
        Parameter("SourceChOffset")(i) = 0;
        Parameter("SourceChGain")(i) = 1;
        std::ostringstream name;
        name << "GripForce" << (i + 1);
        Parameter("ChannelNames")(i) = name.str();
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
        std::string port = Parameter("SerialPort");
        if (port.empty())
            bcierr << "SerialPort must not be empty when InputMode=1";
        Parameter("SerialBaud");
    }

    Output = SignalProperties(Parameter("SourceCh"),
                              Parameter("SampleBlockSize"),
                              SignalType::float32);
}

void GripForceSourceADC::Initialize(const SignalProperties &, const SignalProperties &)
{
    mInputMode = Parameter("InputMode");
    mGain = Parameter("Gain");
    mSourceCh = Parameter("SourceCh");
    mTargetChannel = Parameter("TargetChannel") - 1;
    mLastSerialValue = 0.0;
    mSerialLineBuffer.clear();

    CloseSerialPort();
    if (mInputMode == 1)
        OpenSerialPort((std::string)Parameter("SerialPort"), Parameter("SerialBaud"));

    mClock.SetInterval(Time::Seconds(MeasurementUnits::SampleBlockDuration()));
    mClock.Start();
}

void GripForceSourceADC::StartRun()
{
}

double GripForceSourceADC::ReadRawInput() const
{
    return mInputMode == 0 ? ReadMouseInput() : ReadArduinoVoltage();
}

double GripForceSourceADC::ReadMouseInput() const
{
#if _WIN32
    enum { isPressed = 0x8000 };
    bool leftButton = (::GetAsyncKeyState(VK_LBUTTON) & isPressed) != 0;
    return leftButton ? 1.0 : 0.0;
#else
    return 0.0;
#endif
}

bool GripForceSourceADC::ParseArduinoLine(const std::string &line, double &voltage) const
{
    size_t comma = line.find(',');
    if (comma == std::string::npos)
        return false;
    std::string voltageText = line.substr(comma + 1);
    try
    {
        size_t parsed = 0;
        voltage = std::stod(voltageText, &parsed);
        return parsed > 0;
    }
    catch (...)
    {
        return false;
    }
}

double GripForceSourceADC::ReadArduinoVoltage() const
{
#if _WIN32
    if (mSerialHandle == INVALID_HANDLE_VALUE)
        return mLastSerialValue;

    COMSTAT status = {};
    DWORD errors = 0;
    if (!::ClearCommError(mSerialHandle, &errors, &status) || status.cbInQue == 0)
        return mLastSerialValue;

    DWORD toRead = std::min<DWORD>(status.cbInQue, 256);
    std::string incoming(toRead, '\0');
    DWORD bytesRead = 0;
    if (!::ReadFile(mSerialHandle, &incoming[0], toRead, &bytesRead, NULL))
        return mLastSerialValue;
    incoming.resize(bytesRead);
#else
    if (mSerialFd < 0)
        return mLastSerialValue;

    char buffer[256];
    ssize_t bytesRead = ::read(mSerialFd, buffer, sizeof(buffer));
    if (bytesRead <= 0)
        return mLastSerialValue;
    std::string incoming(buffer, static_cast<size_t>(bytesRead));
#endif

    mSerialLineBuffer += incoming;
    double parsedVoltage = mLastSerialValue;
    bool gotNew = false;

    size_t pos = std::string::npos;
    while ((pos = mSerialLineBuffer.find('\n')) != std::string::npos)
    {
        std::string line = mSerialLineBuffer.substr(0, pos);
        mSerialLineBuffer.erase(0, pos + 1);
        if (!line.empty() && line.back() == '\r')
            line.pop_back();

        double voltage = 0.0;
        if (ParseArduinoLine(line, voltage))
        {
            parsedVoltage = voltage;
            gotNew = true;
        }
    }

    if (gotNew)
        mLastSerialValue = parsedVoltage;
    return mLastSerialValue;
}

void GripForceSourceADC::OpenSerialPort(const std::string &portName, int baud)
{
#if _WIN32
    std::string fullName = portName;
    if (portName.size() > 4 && portName.substr(0, 3) == "COM")
        fullName = "\\\\.\\" + portName;

    mSerialHandle = ::CreateFileA(
        fullName.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        NULL);

    if (mSerialHandle == INVALID_HANDLE_VALUE)
    {
        bcierr << "[ArduinoSerial] Cannot open " << portName
               << " (Windows error " << ::GetLastError() << ")";
        return;
    }

    DCB dcb = {};
    dcb.DCBlength = sizeof(dcb);
    if (!::GetCommState(mSerialHandle, &dcb))
    {
        bcierr << "[ArduinoSerial] GetCommState failed for " << portName;
        CloseSerialPort();
        return;
    }
    dcb.BaudRate = static_cast<DWORD>(baud);
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;
    if (!::SetCommState(mSerialHandle, &dcb))
    {
        bcierr << "[ArduinoSerial] SetCommState failed for " << portName;
        CloseSerialPort();
        return;
    }

    COMMTIMEOUTS timeouts = {};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 0;
    ::SetCommTimeouts(mSerialHandle, &timeouts);
    ::PurgeComm(mSerialHandle, PURGE_RXCLEAR | PURGE_TXCLEAR);
#else
    mSerialFd = ::open(portName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (mSerialFd < 0)
    {
        bcierr << "[ArduinoSerial] Cannot open " << portName;
        return;
    }

    termios tty = {};
    if (::tcgetattr(mSerialFd, &tty) != 0)
    {
        bcierr << "[ArduinoSerial] tcgetattr failed for " << portName;
        CloseSerialPort();
        return;
    }
    speed_t speed = B115200;
    if (baud == 9600)
        speed = B9600;
    else if (baud == 57600)
        speed = B57600;
    ::cfsetispeed(&tty, speed);
    ::cfsetospeed(&tty, speed);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag &= ~IGNBRK;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    ::tcsetattr(mSerialFd, TCSANOW, &tty);
    ::tcflush(mSerialFd, TCIFLUSH);
#endif

    bciout << "[ArduinoSerial] Opened " << portName << " at " << baud
           << " baud, expecting CSV temperature,voltage";
}

void GripForceSourceADC::CloseSerialPort()
{
#if _WIN32
    if (mSerialHandle != INVALID_HANDLE_VALUE)
    {
        ::CloseHandle(mSerialHandle);
        mSerialHandle = INVALID_HANDLE_VALUE;
    }
#else
    if (mSerialFd >= 0)
    {
        ::close(mSerialFd);
        mSerialFd = -1;
    }
#endif
    mSerialLineBuffer.clear();
}

void GripForceSourceADC::Process(const GenericSignal &, GenericSignal &Output)
{
    double force = ReadRawInput() * mGain;

    for (int ch = 0; ch < Output.Channels(); ++ch)
    {
        double value = (ch == mTargetChannel) ? force : 0.0;
        for (int sample = 0; sample < Output.Elements(); ++sample)
            Output(ch, sample) = value;
    }

    mClock.Wait();
}

void GripForceSourceADC::Halt()
{
    mClock.Stop();
    CloseSerialPort();
}
