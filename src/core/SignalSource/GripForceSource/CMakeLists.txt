###########################################################################
## Description: Build information for GripForceSource
##   专为 GripForceTask 设计的独立信号源模块

BCI2000_ADD_SIGNAL_SOURCE_MODULE(
  GripForceSource
  GripForceSourceADC.cpp
  SUPPORT /batch/*_GripForceSource.bat
)
