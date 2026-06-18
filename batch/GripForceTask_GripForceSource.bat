#! ../prog/BCI2000Shell
@cls & ..\prog\BCI2000Shell %0 %* #! && exit /b 0 || exit /b 1
#######################################################################################
## Description: Launch GripForceTask with the dedicated GripForceSource
##   Chain: GripForceSource -> DummySignalProcessing (pass-through) -> GripForceTask
#######################################################################################
Change directory $BCI2000LAUNCHDIR
Show window; Set title ${Extract file base $0}
Reset system
Startup system localhost
Start executable GripForceSource --local
Start executable DummySignalProcessing --local
Start executable GripForceTask --local
Wait for Connected
