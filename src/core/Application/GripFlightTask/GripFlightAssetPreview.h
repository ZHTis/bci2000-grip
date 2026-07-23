#ifndef GRIP_FLIGHT_ASSET_PREVIEW_H
#define GRIP_FLIGHT_ASSET_PREVIEW_H

#include "ApplicationBase.h"
#include "ImageStimulus.h"
#include "Runnable.h"
#include "Shapes.h"

#include <vector>

class GripFlightAssetPreview : public ApplicationBase, private Runnable
{
  public:
    GripFlightAssetPreview();
    ~GripFlightAssetPreview() override;

  private:
    void Preflight(const SignalProperties&, SignalProperties&) const override;
    void Initialize(const SignalProperties&, const SignalProperties&) override;
    void Process(const GenericSignal&, GenericSignal&) override;
    void Halt() override;
    void OnRun() override;
    void Clear();
    void UpdateImageRect();
    void RegisterControls();
    void UnregisterControls();

    GUI::DisplayWindow& mrWindow;
    ImageStimulus* mpImage;
    std::vector<RectangularShape*> mCheckerTiles;
    float mCenterX;
    float mCenterY;
    float mWidth;
    float mHeight;
    bool mControlsRegistered;
};

#endif
