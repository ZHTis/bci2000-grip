###########################################################################
## Description: Build information for NI6501Marker
##   一个独立的控制台小工具：按键盘 K/L 在 NI 6501 的 P0.0/P0.1 上输出
##   marker 脉冲。复用 NIDAQ-MX 模块的 imports 封装(运行时动态加载
##   nicaiu.dll)，因此不需要链接 NI 的 .lib，也不需要 NI 头文件路径。
##
##   构建产物输出到 bci2000/prog/NI6501Marker.exe (BCI2000 统一输出目录)。

SET( NAME NI6501Marker )
SET( DIR_NAME "${DIR_NAME}NI6501Marker/" )

# NIDAQ-MX 模块里的动态加载封装目录(含 NIDAQmx.imports.cpp / .h 和 NIDAQmx.h)
SET( NIDAQ_DIR ${PROJECT_SRC_DIR}/contrib/SignalSource/NIDAQ-MX/nidaq )

SET( SRC_PROJECT
  NI6501Marker.cpp
  ${NIDAQ_DIR}/NIDAQmx.imports.cpp
)

# 让 #include "NIDAQmx.imports.h" / "NIDAQmx.h" 以及 imports 封装依赖的
# 框架头文件 (DylibImports.h 等) 都能被找到。
INCLUDE_DIRECTORIES(
  ${NIDAQ_DIR}
  ${PROJECT_SRC_DIR}/shared/utils/Lib
)

IF( WIN32 )
  # Pull in the BCI2000 framework libraries (LibTiny etc.) so that the
  # dynamic-loader symbols used by NIDAQmx.imports.cpp (Dylib::StartupLoader,
  # Dylib::Library) resolve at link time.
  UTILS_INCLUDE( frameworks/Core )
  BCI2000_ADD_TARGET( CONSOLEAPP ${NAME} ${SRC_PROJECT} )
  SET_OUTPUT_DIRECTORY( "${PROJECT_BUILD_ROOT}/prog" ${NAME} )
ELSE()
  MESSAGE( "NI6501Marker: NI-DAQmx is Windows-only; skipping this tool." )
ENDIF()