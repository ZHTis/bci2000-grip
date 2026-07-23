#! ../prog/BCI2000Shell
@cls & ..\prog\BCI2000Shell %0 %* #! && exit /b 0 || exit /b 1
#######################################################################################
## Preview GripFlight image assets without running game physics or map logic.
#######################################################################################
Change directory $BCI2000LAUNCHDIR
Show window; Set title ${Extract file base $0}
Reset system
Startup system localhost
Start executable SignalGenerator --local
Start executable DummySignalProcessing --local
Start executable GripFlightAssetPreview --local
Wait for Connected
Load parameterfile "../src/core/Application/GripFlightTask/GripFlightAssetPreview.prm"
# Set Config manually to keep image-loading errors visible in Operator.
