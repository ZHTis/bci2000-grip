bci2000_add_application_module(
    GripFlightTask
    GripFlightTask.cpp
    GameStateMachine.cpp
    FlightPhysics.cpp
    MapLoader.cpp
    AssetManager.cpp
    SideScrollScene.cpp
)

# ImageStimulus discovers the SVG image plugin at runtime. The plugin itself is
# deployed by BCI2000, but its Qt6Svg dependency is not part of the default
# application-module runtime set.
find_package(Qt6 REQUIRED COMPONENTS Svg)
add_custom_command(
    TARGET GripFlightTask POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:Qt6::Svg>" "${PROJECT_SRC_DIR}/../prog/"
    COMMENT "Deploying Qt SVG runtime for GripFlightTask assets"
)

bci2000_add_application_module(
    GripFlightAssetPreview
    GripFlightAssetPreview.cpp
)
add_custom_command(
    TARGET GripFlightAssetPreview POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:Qt6::Svg>" "${PROJECT_SRC_DIR}/../prog/"
    COMMENT "Deploying Qt SVG runtime for asset preview"
)
