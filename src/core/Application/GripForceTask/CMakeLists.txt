# GripForceTask reuses the feedback-scene support files from CursorTask.
# They live in ../CursorTask, so reference them by relative path and add
# that directory to the include search path for the headers.
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/../CursorTask )

# NI6501 hardware marker: reuse the NIDAQ-MX dynamic-loading wrapper
# (loads nicaiu.dll at runtime; no NI .lib / include paths needed at build time).
set( NIDAQ_DIR ${PROJECT_SRC_DIR}/contrib/SignalSource/NIDAQ-MX/nidaq )
include_directories(
    ${NIDAQ_DIR}
    ${PROJECT_SRC_DIR}/shared/utils/Lib
)

bci2000_add_application_module(
    GripForceTask
    GripForceTask.cpp
    ../CursorTask/FeedbackScene2D.cpp
    ../CursorTask/FeedbackScene3D.cpp
    ../CursorTask/FeedbackScene.h
    ../CursorTask/AudioFeedback.cpp
    ../CursorTask/SimpleSynth.cpp
    ${NIDAQ_DIR}/NIDAQmx.imports.cpp
)
