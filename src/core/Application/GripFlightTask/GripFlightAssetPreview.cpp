#include "GripFlightAssetPreview.h"

#include "Broadcaster.h"
#include "FileUtils.h"

#include <algorithm>
#include <cmath>

RegisterFilter(GripFlightAssetPreview, 3);

GripFlightAssetPreview::GripFlightAssetPreview()
    : mrWindow(Window()), mpImage(nullptr), mCenterX(0.5f), mCenterY(0.5f),
      mWidth(0.35f), mHeight(0.35f), mControlsRegistered(false)
{
    BEGIN_PARAMETER_DEFINITIONS
        "Application:AssetPreview string PreviewAsset= % % % % // image file to preview",
        "Application:AssetPreview floatlist PreviewCenter= 2 50 50 % % // image center x y in percent",
        "Application:AssetPreview floatlist PreviewSize= 2 35 35 % % // image width height in percent",
        "Application:AssetPreview int PreviewCheckerboard= 1 1 0 1 // show transparency checkerboard (boolean)",
        "Application:AssetPreview int PreviewBackgroundColor= 0x30343b % % % // background RGB color",
        "Application:AssetPreview int CheckerLightColor= 0xd8dbe0 % % % // light checker RGB color",
        "Application:AssetPreview int CheckerDarkColor= 0xaeb4bd % % % // dark checker RGB color",
    END_PARAMETER_DEFINITIONS
}

GripFlightAssetPreview::~GripFlightAssetPreview()
{
    UnregisterControls();
    Clear();
}

void GripFlightAssetPreview::Preflight(const SignalProperties& input, SignalProperties& output) const
{
    output = input;
    const std::string filename = Parameter("PreviewAsset");
    if (filename.empty())
        bcierr << "PreviewAsset must specify an image file";
    else if (!FileUtils::IsFile(FileUtils::AbsolutePath(filename)))
        bcierr << "PreviewAsset does not exist: " << filename;

    ParamRef center = Parameter("PreviewCenter");
    ParamRef size = Parameter("PreviewSize");
    if (center->NumValues() != 2)
        bcierr << "PreviewCenter must contain x and y";
    if (size->NumValues() != 2 || size(0) <= 0 || size(1) <= 0)
        bcierr << "PreviewSize must contain two positive values";

    Parameter("PreviewCheckerboard");
    Parameter("PreviewBackgroundColor");
    Parameter("CheckerLightColor");
    Parameter("CheckerDarkColor");
}

void GripFlightAssetPreview::Initialize(const SignalProperties& input, const SignalProperties& output)
{
    ApplicationBase::Initialize(input, output);
    Clear();
    mrWindow.SetColor(RGBColor(Parameter("PreviewBackgroundColor")));

    if (Parameter("PreviewCheckerboard"))
    {
        const int columns = 10;
        const int rows = 8;
        const RGBColor light(Parameter("CheckerLightColor"));
        const RGBColor dark(Parameter("CheckerDarkColor"));
        for (int row = 0; row < rows; ++row)
        {
            for (int column = 0; column < columns; ++column)
            {
                auto* tile = new RectangularShape(mrWindow);
                const RGBColor color = ((row + column) % 2) ? light : dark;
                tile->SetColor(color).SetFillColor(color).SetZOrder(5);
                tile->SetObjectRect({column / float(columns), row / float(rows),
                                     (column + 1) / float(columns), (row + 1) / float(rows)});
                mCheckerTiles.push_back(tile);
            }
        }
    }

    ParamRef center = Parameter("PreviewCenter");
    ParamRef size = Parameter("PreviewSize");
    mCenterX = center(0) / 100.0f;
    mCenterY = center(1) / 100.0f;
    mWidth = size(0) / 100.0f;
    mHeight = size(1) / 100.0f;
    mpImage = new ImageStimulus(mrWindow);
    mpImage->SetRenderingMode(GUI::RenderingMode::Transparent)
        .SetFile(FileUtils::AbsolutePath(static_cast<std::string>(Parameter("PreviewAsset"))))
        .SetScalingMode(GUI::ScalingMode::AdjustNone)
        .SetZOrder(-2);
    UpdateImageRect();
    mpImage->Present();
    RegisterControls();
    mrWindow.Show();
}

void GripFlightAssetPreview::Process(const GenericSignal& input, GenericSignal& output)
{
    output = input;
}

void GripFlightAssetPreview::Halt()
{
    UnregisterControls();
    Clear();
}

void GripFlightAssetPreview::OnRun()
{
    if (!mpImage)
        return;
    const auto& message = Broadcaster::Message();
    if (message.Id() == GUI::DisplayWindow::OnMouseWheel)
    {
        const auto* wheel = static_cast<GUI::DisplayWindow::MouseWheelEvent*>(message.Argument());
        const float scale = std::pow(1.12f, wheel->delta / 120.0f);
        mWidth = std::max(0.02f, std::min(2.0f, mWidth * scale));
        mHeight = std::max(0.02f, std::min(2.0f, mHeight * scale));
        UpdateImageRect();
    }
}

void GripFlightAssetPreview::UpdateImageRect()
{
    if (mpImage)
        mpImage->SetObjectRect({mCenterX - mWidth / 2, mCenterY - mHeight / 2,
                                mCenterX + mWidth / 2, mCenterY + mHeight / 2});
}

void GripFlightAssetPreview::RegisterControls()
{
    if (!mControlsRegistered)
    {
        mrWindow.AddListener(GUI::DisplayWindow::OnMouseWheel, this);
        mControlsRegistered = true;
    }
}

void GripFlightAssetPreview::UnregisterControls()
{
    if (mControlsRegistered)
    {
        mrWindow.RemoveListener(GUI::DisplayWindow::OnMouseWheel, this);
        mControlsRegistered = false;
    }
}

void GripFlightAssetPreview::Clear()
{
    delete mpImage;
    mpImage = nullptr;
    for (auto* tile : mCheckerTiles)
        delete tile;
    mCheckerTiles.clear();
}
