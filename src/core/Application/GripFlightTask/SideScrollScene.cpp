#include "SideScrollScene.h"

#include "GraphDisplay.h"

#include <algorithm>

SideScrollScene::SideScrollScene(GUI::GraphDisplay& display)
    : mDisplay(display), mpMap(nullptr), mpPlayerShape(nullptr), mpPlayerImage(nullptr), mpBackground(nullptr),
      mWorldHeight(100), mViewWidth(160), mPlayerWidth(8), mPlayerHeight(8), mCameraX(0), mVisible(false)
{
}

SideScrollScene::~SideScrollScene()
{
    Clear();
}

void SideScrollScene::Clear()
{
    delete mpBackground;
    delete mpPlayerImage;
    delete mpPlayerShape;
    mpBackground = nullptr;
    mpPlayerImage = nullptr;
    mpPlayerShape = nullptr;
    for (auto& visual : mVisuals)
    {
        delete visual.image;
        delete visual.shape;
    }
    mVisuals.clear();
}

void SideScrollScene::Initialize(const FlightMap& map, const AssetManager& assets, float worldHeight,
                                 float viewWidth, float playerWidth, float playerHeight)
{
    Clear();
    mpMap = &map;
    mWorldHeight = worldHeight;
    mViewWidth = viewWidth;
    mPlayerWidth = playerWidth;
    mPlayerHeight = playerHeight;
    mCameraX = 0;

    if (assets.Has("background"))
    {
        mpBackground = new ImageStimulus(mDisplay);
        mpBackground->SetFile(assets.Path("background")).SetObjectRect({0, 0, 1, 1}).SetZOrder(10);
        mpBackground->Present();
    }

    mpPlayerShape = new EllipticShape(mDisplay);
    mpPlayerShape->SetColor(RGBColor::Yellow).SetFillColor(RGBColor::Yellow).SetZOrder(-2);
    if (assets.Has("player"))
    {
        mpPlayerImage = new ImageStimulus(mDisplay);
        mpPlayerImage->SetFile(assets.Path("player"))
            .SetScalingMode(GUI::ScalingMode::AdjustWidth)
            .SetZOrder(-3);
        mpPlayerImage->Present();
        mpPlayerShape->Hide();
    }

    for (const auto& object : map.Objects())
    {
        Visual visual{object, new RectangularShape(mDisplay), nullptr};
        visual.shape->SetColor(RGBColor::Gray).SetFillColor(RGBColor::Gray).SetZOrder(-1);
        if (assets.Has(object.assetId))
        {
            visual.image = new ImageStimulus(mDisplay);
            visual.image->SetFile(assets.Path(object.assetId)).SetZOrder(-2);
            visual.image->Present();
            visual.shape->Hide();
        }
        mVisuals.push_back(visual);
    }
    SetVisible(false);
}

float SideScrollScene::ViewWidth() const
{
    const GUI::Rect viewport = mDisplay.Context().rect;
    const float pixelWidth = viewport.right - viewport.left;
    const float pixelHeight = viewport.bottom - viewport.top;
    if (pixelWidth <= 0.0f || pixelHeight <= 0.0f)
        return mViewWidth;
    return mWorldHeight * pixelWidth / pixelHeight;
}

GUI::Rect SideScrollScene::ScreenRect(float x, float y, float width, float height) const
{
    const float viewWidth = ViewWidth();
    const float left = (x - width * 0.5f - mCameraX) / viewWidth;
    const float right = (x + width * 0.5f - mCameraX) / viewWidth;
    const float top = 1.0f - (y + height * 0.5f) / mWorldHeight;
    const float bottom = 1.0f - (y - height * 0.5f) / mWorldHeight;
    return {left, top, right, bottom};
}

void SideScrollScene::SetVisible(bool visible)
{
    mVisible = visible;
    if (mpBackground)
        mpBackground->SetVisible(visible);
    if (mpPlayerImage)
        mpPlayerImage->SetVisible(visible);
    if (mpPlayerShape && !mpPlayerImage)
        mpPlayerShape->SetVisible(visible);
    for (auto& visual : mVisuals)
    {
        if (visual.image)
            visual.image->SetVisible(visible);
        else
            visual.shape->SetVisible(visible);
    }
}

void SideScrollScene::Update(float playerX, float playerY)
{
    const float viewWidth = ViewWidth();
    mCameraX = std::max(0.0f, playerX - viewWidth * 0.30f);
    const GUI::Rect playerRect = ScreenRect(playerX, playerY, mPlayerWidth, mPlayerHeight);
    if (mpPlayerImage)
        mpPlayerImage->SetObjectRect(playerRect);
    else if (mpPlayerShape)
        mpPlayerShape->SetObjectRect(playerRect);

    for (auto& visual : mVisuals)
    {
        const GUI::Rect rect = ScreenRect(visual.object.x, visual.object.y, visual.object.width, visual.object.height);
        const bool onScreen = rect.right >= 0 && rect.left <= 1;
        if (visual.image)
        {
            visual.image->SetObjectRect(rect);
            visual.image->SetVisible(mVisible && onScreen);
        }
        else
        {
            visual.shape->SetObjectRect(rect);
            visual.shape->SetVisible(mVisible && onScreen);
        }
    }
}

bool SideScrollScene::Collides(float playerX, float playerY, int& objectIndex) const
{
    const float playerLeft = playerX - mPlayerWidth * 0.5f;
    const float playerRight = playerX + mPlayerWidth * 0.5f;
    const float playerBottom = playerY - mPlayerHeight * 0.5f;
    const float playerTop = playerY + mPlayerHeight * 0.5f;
    for (size_t i = 0; i < mpMap->Objects().size(); ++i)
    {
        const MapObject& object = mpMap->Objects()[i];
        if (playerRight >= object.x - object.width * 0.5f && playerLeft <= object.x + object.width * 0.5f &&
            playerTop >= object.y - object.height * 0.5f && playerBottom <= object.y + object.height * 0.5f)
        {
            objectIndex = static_cast<int>(i);
            return true;
        }
    }
    objectIndex = -1;
    return false;
}

bool SideScrollScene::OutsideWorld(float playerY) const
{
    return playerY - mPlayerHeight * 0.5f <= 0 || playerY + mPlayerHeight * 0.5f >= mWorldHeight;
}

float SideScrollScene::CameraX() const
{
    return mCameraX;
}
