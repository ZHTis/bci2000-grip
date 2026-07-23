#! ../prog/BCI2000Shell
@cls & ..\prog\BCI2000Shell %0 %* #! && exit /b 0 || exit /b 1
#######################################################################################
## Description: Launch GripFlightTask with GripForceSource.
##   Chain: GripForceSource -> DummySignalProcessing -> GripFlightTask
#######################################################################################
Change directory $BCI2000LAUNCHDIR
Show window; Set title ${Extract file base $0}
Reset system
Startup system localhost
Start executable GripForceSource --local
Start executable DummySignalProcessing --local
Start executable GripFlightTask --local
Wait for Connected
Load parameterfile "../parms/examples/GripForceTask_GripForceSource.prm"
Load parameterfile "../src/core/Application/GripFlightTask/GripFlightTask.prm"
# Set Config manually in Operator while GripFlightTask is under development.
