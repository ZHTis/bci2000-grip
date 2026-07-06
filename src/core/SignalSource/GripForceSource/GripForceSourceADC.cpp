////////////////////////////////////////////////////////////////////////////////
// GripForceSourceADC.cpp
//
// BCI2000 Signal Source Module: Grip Force Source
// See GripForceSourceADC.h for the design notes.
//
// InputMode:
//   0 - Mouse left button (0/1 binary, for simulation)
//   1 - Intan AUX analog channel (via shared memory or TCP from IntanRecordingController)
//   2 - Arduino serial port (ASCII line, e.g. "0.532\n", from MPX5050GP)
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
#include <string>
#include <sstream>
#include <stdexcept>

// Register as a level-1 filter (signal source layer)
RegisterFilter(GripForceSourceADC, 1);

GripForceSourceADC::GripForceSourceADC()
    : mInputMode(0), mMouseAxis(0), mGain(1.0), mSourceCh(1),
      mTargetChannel(0), mBaseline(0.0), mHaveBaseline(false),
      // Intan TCP
      mIntanSocket(INVALID_SOCKET), mIntanAuxChannel(0),
      mLastIntanValue(0.0),
      // Arduino serial
#if _WIN32
      mSerialHandle(INVALID_HANDLE_VALUE),
#else
      mSerialFd(-1),
#endif
      mLastSerialValue(0.0)
{
#if _WIN32
    // Initialize Winsock once per process (safe to call multiple times)
    WSADATA wsaData;
    ::WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
}

GripForceSourceADC::~GripForceSourceADC()
{
    CloseIntanSocket();
    CloseSerialPort();
#if _WIN32
    ::WSACleanup();
#endif
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
        "Source:GripForce int InputMode= 0 0 0 2 "
            "// input source: "
            "0: mouse left button (simulation), "
            "1: Intan AUX analog channel (TCP), "
            "2: Arduino serial port (MPX5050GP ASCII) "
            "(enumeration)",
        "Source:GripForce int MouseAxis= 0 0 0 1 "
            "// reserved; mouse button mode does not use an axis (enumeration)",
        "Source:GripForce float Gain= 1.0 1.0 0.0 % "
            "// raw input -> force scaling factor",
        "Source:GripForce int TargetChannel= 1 1 1 % "
            "// channel the force is written to (1-based); other channels output 0",

        // ---- Intan TCP mode (InputMode=1) ----
        "Source:GripForce string IntanAddress= localhost % % % "
            "// Intan Recording Controller hostname or IP (InputMode=1)",
        "Source:GripForce int IntanPort= 36000 36000 1024 65535 "
            "// Intan TCP command/data port (InputMode=1)",
        "Source:GripForce int IntanAuxChannel= 1 1 1 3 "
            "// Intan AUX channel index 1-3 (InputMode=1)",

        // ---- Arduino serial mode (InputMode=2) ----
        "Source:GripForce string SerialPort= COM6 % % % "
            "// Serial port for Arduino (e.g. COM6 on Windows, /dev/ttyUSB0 on Linux) (InputMode=2)",
        "Source:GripForce int SerialBaud= 115200 115200 9600 % "
            "// Serial baud rate matching Arduino sketch (InputMode=2)",
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
        // Intan TCP mode: validate address and port are non-empty
        std::string addr = (std::string)Parameter("IntanAddress");
        if (addr.empty())
            bcierr << "IntanAddress must not be empty when InputMode=1";
        int port = Parameter("IntanPort");
        if (port < 1024 || port > 65535)
            bcierr << "IntanPort=" << port << " is out of valid range [1024, 65535]";
        int auxCh = Parameter("IntanAuxChannel");
        if (auxCh < 1 || auxCh > 3)
            bcierr << "IntanAuxChannel=" << auxCh << " must be 1, 2, or 3";
    }
    if (inputMode == 2)
    {
        // Serial mode: just check port name is non-empty; actual open is in Initialize
        std::string port = (std::string)Parameter("SerialPort");
        if (port.empty())
            bcierr << "SerialPort must not be empty when InputMode=2";
    }

    // Single-channel float signal to preserve force precision
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

    // Close any previously open connections before re-initializing
    CloseIntanSocket();
    CloseSerialPort();

    if (mInputMode == 1)
    {
        // ---- Intan TCP mode ----
        mIntanAuxChannel = (int)Parameter("IntanAuxChannel") - 1;  // 0-based
        std::string addr = (std::string)Parameter("IntanAddress");
        int port         = Parameter("IntanPort");
        OpenIntanSocket(addr, port);
    }
    else if (mInputMode == 2)
    {
        // ---- Arduino serial mode ----
        std::string portName = (std::string)Parameter("SerialPort");
        int baud             = Parameter("SerialBaud");
        OpenSerialPort(portName, baud);
    }

    mClock.SetInterval(Time::Seconds(MeasurementUnits::SampleBlockDuration()));
    mClock.Start();
}

void GripForceSourceADC::StartRun()
{
    // Record the baseline: the zero point of the "lever"
    mBaseline = ReadRawInput();
    mHaveBaseline = true;
}

// ============================================================
// ReadRawInput：根据 InputMode 从对应硬件读取一次原始值
//   Mode 0: 鼠标左键 → 0.0 / 1.0
//   Mode 1: Intan TCP → AUX 通道电压 (V)
//   Mode 2: Arduino 串口 → ASCII 浮点数
// ============================================================
double GripForceSourceADC::ReadRawInput() const
{
    switch (mInputMode)
    {
    // ----------------------------------------------------------
    // Mode 0: 鼠标左键（仿真用）
    // ----------------------------------------------------------
    case 0:
    {
#if _WIN32
        enum { isPressed = 0x8000 };
        bool leftButton = (::GetAsyncKeyState(VK_LBUTTON) & isPressed) != 0;
        return leftButton ? 1.0 : 0.0;
#elif USE_QT
        bool leftButton = (QApplication::mouseButtons() & Qt::LeftButton) != 0;
        return leftButton ? 1.0 : 0.0;
#else
        return 0.0;
#endif
    }

    // ----------------------------------------------------------
    // Mode 1: Intan Recording Controller TCP
    //
    // Intan RHD USB 控制器支持通过 TCP 端口（默认 36000）查询
    // 实时 AUX 通道电压，命令格式为 ASCII 行：
    //   发送: "get aux 0\n"   (0-based AUX 索引)
    //   接收: "aux 0 0.532\n" 或 "0.532\n"
    //
    // 注意：这要求 Intan 软件已在运行且"TCP Data Output"已开启。
    // 若 Intan 固件版本不同，返回格式可能有差异，此时修改
    // ParseIntanResponse() 即可，其余逻辑不变。
    // ----------------------------------------------------------
    case 1:
    {
        if (mIntanSocket == INVALID_SOCKET)
            return mLastIntanValue;   // 连接未建立，返回上次值

        // 发送查询命令
        char cmd[32];
        int cmdLen = ::snprintf(cmd, sizeof(cmd), "get aux %d\n", mIntanAuxChannel);
#if _WIN32
        int sent = ::send(mIntanSocket, cmd, cmdLen, 0);
#else
        int sent = static_cast<int>(::send(mIntanSocket, cmd, cmdLen, 0));
#endif
        if (sent <= 0)
        {
            bciwarn << "[Intan] send() failed, retaining last value=" << mLastIntanValue;
            return mLastIntanValue;
        }

        // 接收响应（非阻塞，已在 OpenIntanSocket 中设置超时）
        char buf[128] = {};
#if _WIN32
        int received = ::recv(mIntanSocket, buf, sizeof(buf) - 1, 0);
#else
        int received = static_cast<int>(::recv(mIntanSocket, buf, sizeof(buf) - 1, 0));
#endif
        if (received <= 0)
        {
            bciwarn << "[Intan] recv() failed, retaining last value=" << mLastIntanValue;
            return mLastIntanValue;
        }

        double val = ParseIntanResponse(std::string(buf, received));
        mLastIntanValue = val;
        return val;
    }

    // ----------------------------------------------------------
    // Mode 2: Arduino 串口 ASCII
    //
    // Arduino 端每隔约 1 ms 输出一行：
    //   Serial.println(sensorValue);   // e.g. "0.532\n"
    //
    // 这里读取串口缓冲区中所有已到达的行，取最新的一行解析。
    // 若本次 Process() 调用间隔内没有新数据，返回上次缓存值，
    // 避免阻塞实时循环。
    // ----------------------------------------------------------
    case 2:
    {
#if _WIN32
        if (mSerialHandle == INVALID_HANDLE_VALUE)
            return mLastSerialValue;

        // 将串口缓冲区中现有的字节全部读入（非阻塞，Timeout=0）
        COMSTAT cs = {};
        DWORD errors = 0;
        ::ClearCommError(mSerialHandle, &errors, &cs);
        DWORD available = cs.cbInQue;

        if (available == 0)
            return mLastSerialValue;    // 没有新数据，用上次值

        // 最多读 256 字节，保证不阻塞
        DWORD toRead = std::min<DWORD>(available, 256);
        std::string incoming(toRead, '\0');
        DWORD bytesRead = 0;
        ::ReadFile(mSerialHandle, &incoming[0], toRead, &bytesRead, NULL);
        incoming.resize(bytesRead);

        // 追加到行缓冲区，解析所有完整行，保留最后一个有效值
        mSerialLineBuffer += incoming;
        double parsed = mLastSerialValue;
        bool   gotNew = false;

        std::string::size_type pos;
        while ((pos = mSerialLineBuffer.find('\n')) != std::string::npos)
        {
            std::string line = mSerialLineBuffer.substr(0, pos);
            mSerialLineBuffer.erase(0, pos + 1);
            // 去除 Windows 换行符 \r
            if (!line.empty() && line.back() == '\r')
                line.pop_back();
            if (line.empty())
                continue;
            try {
                double v = std::stod(line);
                parsed = v;
                gotNew = true;
            } catch (...) {
                // 非数字行（如调试输出），忽略
            }
        }

        if (gotNew)
            mLastSerialValue = parsed;
        return mLastSerialValue;

#else
        // Linux/macOS: 同样使用非阻塞 read()
        if (mSerialFd < 0)
            return mLastSerialValue;

        char buf[256];
        ssize_t n = ::read(mSerialFd, buf, sizeof(buf) - 1);
        if (n <= 0)
            return mLastSerialValue;

        buf[n] = '\0';
        mSerialLineBuffer += std::string(buf, n);

        double parsed = mLastSerialValue;
        bool   gotNew = false;

        std::string::size_type pos;
        while ((pos = mSerialLineBuffer.find('\n')) != std::string::npos)
        {
            std::string line = mSerialLineBuffer.substr(0, pos);
            mSerialLineBuffer.erase(0, pos + 1);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (line.empty()) continue;
            try {
                double v = std::stod(line);
                parsed = v;
                gotNew = true;
            } catch (...) {}
        }

        if (gotNew)
            mLastSerialValue = parsed;
        return mLastSerialValue;
#endif
    }

    default:
        return 0.0;
    }
}

// ============================================================
// OpenIntanSocket / CloseIntanSocket
// ============================================================
void GripForceSourceADC::OpenIntanSocket(const std::string& address, int port)
{
#if _WIN32
    mIntanSocket = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (mIntanSocket == INVALID_SOCKET)
    {
        bciwarn << "[Intan] socket() failed (WSA error " << ::WSAGetLastError() << "), Intan mode disabled.";
        return;
    }

    // 设置接收超时 50 ms，避免阻塞实时循环
    DWORD timeout = 50;
    ::setsockopt(mIntanSocket, SOL_SOCKET, SO_RCVTIMEO,
                 reinterpret_cast<const char*>(&timeout), sizeof(timeout));

    struct sockaddr_in srv = {};
    srv.sin_family = AF_INET;
    srv.sin_port   = htons(static_cast<u_short>(port));
    ::inet_pton(AF_INET, address.c_str(), &srv.sin_addr);

    if (::connect(mIntanSocket, reinterpret_cast<sockaddr*>(&srv), sizeof(srv)) != 0)
    {
        bciwarn << "[Intan] connect() to " << address << ":" << port
                << " failed (WSA error " << ::WSAGetLastError()
                << "). Check that Intan software is running and TCP output is enabled.";
        ::closesocket(mIntanSocket);
        mIntanSocket = INVALID_SOCKET;
        return;
    }
    AppLog << "[Intan] TCP connected to " << address << ":" << port << "\n";
#else
    mIntanSocket = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (mIntanSocket < 0) { mIntanSocket = INVALID_SOCKET; return; }

    struct timeval tv = {0, 50000};  // 50 ms
    ::setsockopt(mIntanSocket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in srv = {};
    srv.sin_family = AF_INET;
    srv.sin_port   = htons(static_cast<uint16_t>(port));
    ::inet_pton(AF_INET, address.c_str(), &srv.sin_addr);

    if (::connect(mIntanSocket, reinterpret_cast<sockaddr*>(&srv), sizeof(srv)) != 0)
    {
        bciwarn << "[Intan] connect() failed. Check Intan TCP output is enabled.";
        ::close(mIntanSocket);
        mIntanSocket = INVALID_SOCKET;
        return;
    }
    AppLog << "[Intan] TCP connected to " << address << ":" << port << "\n";
#endif
}

void GripForceSourceADC::CloseIntanSocket()
{
#if _WIN32
    if (mIntanSocket != INVALID_SOCKET)
    {
        ::closesocket(mIntanSocket);
        mIntanSocket = INVALID_SOCKET;
    }
#else
    if (mIntanSocket != INVALID_SOCKET)
    {
        ::close(mIntanSocket);
        mIntanSocket = INVALID_SOCKET;
    }
#endif
}

// Intan 响应解析：取最后一个空格之后的浮点数
// "aux 0 0.532\n" → 0.532
// "0.532\n"       → 0.532
double GripForceSourceADC::ParseIntanResponse(const std::string& resp) const
{
    std::string s = resp;
    // 去除末尾空白/换行
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' '))
        s.pop_back();
    // 找最后一个空格，取其后部分
    auto spacePos = s.rfind(' ');
    std::string numStr = (spacePos == std::string::npos) ? s : s.substr(spacePos + 1);
    try {
        return std::stod(numStr);
    } catch (...) {
        bciwarn << "[Intan] Failed to parse response: \"" << resp << "\"";
        return mLastIntanValue;
    }
}

// ============================================================
// OpenSerialPort / CloseSerialPort
// ============================================================
void GripForceSourceADC::OpenSerialPort(const std::string& portName, int baud)
{
#if _WIN32
    // Windows 需要把 "COM10" 以上的端口名加上 "\\.\" 前缀
    std::string fullName = portName;
    if (portName.size() > 4 &&
        portName.substr(0, 3) == "COM" &&
        std::stoi(portName.substr(3)) >= 10)
        fullName = "\\\\.\\" + portName;

    mSerialHandle = ::CreateFileA(
        fullName.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0, NULL, OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL, NULL);

    if (mSerialHandle == INVALID_HANDLE_VALUE)
    {
        bcierr << "[Serial] Cannot open " << portName
               << " (error " << ::GetLastError()
               << "). Check device manager for the correct COM port.";
        return;
    }

    // 配置波特率、数据位、停止位、校验
    DCB dcb = {};
    dcb.DCBlength = sizeof(dcb);
    ::GetCommState(mSerialHandle, &dcb);
    dcb.BaudRate = static_cast<DWORD>(baud);
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity   = NOPARITY;
    ::SetCommState(mSerialHandle, &dcb);

    // 超时设为0：ReadFile 立即返回现有数据，不等待（非阻塞）
    COMMTIMEOUTS timeouts = {};
    timeouts.ReadIntervalTimeout         = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier  = 0;
    timeouts.ReadTotalTimeoutConstant    = 0;
    ::SetCommTimeouts(mSerialHandle, &timeouts);

    // 清空缓冲区中实验前的残留数据
    ::PurgeComm(mSerialHandle, PURGE_RXCLEAR | PURGE_TXCLEAR);

    AppLog << "[Serial] Opened " << portName << " at " << baud << " baud\n";

#else
    // Linux/macOS
    #include <fcntl.h>
    #include <termios.h>
    #include <unistd.h>

    mSerialFd = ::open(portName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (mSerialFd < 0)
    {
        bcierr << "[Serial] Cannot open " << portName << ". Check /dev/ permissions.";
        return;
    }

    struct termios tty = {};
    ::tcgetattr(mSerialFd, &tty);

    // 设置波特率（仅支持标准值）
    speed_t speed = B115200;
    if (baud == 9600)   speed = B9600;
    if (baud == 57600)  speed = B57600;
    if (baud == 115200) speed = B115200;
    ::cfsetispeed(&tty, speed);
    ::cfsetospeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit chars
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag  = 0;         // 原始模式（不缓冲行）
    tty.c_oflag  = 0;
    tty.c_cc[VMIN]  = 0;     // 非阻塞
    tty.c_cc[VTIME] = 0;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    ::tcsetattr(mSerialFd, TCSANOW, &tty);

    // 清空缓冲区
    ::tcflush(mSerialFd, TCIFLUSH);

    AppLog << "[Serial] Opened " << portName << " at " << baud << " baud\n";
#endif
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
    CloseIntanSocket();
    CloseSerialPort();
}
