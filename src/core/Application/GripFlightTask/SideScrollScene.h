#ifndef GRIP_FLIGHT_SIDE_SCROLL_SCENE_H
#define GRIP_FLIGHT_SIDE_SCROLL_SCENE_H

#include "AssetManager.h"
#include "MapLoader.h"

#include "ImageStimulus.h"
#include "Shapes.h"

#include <vector>

namespace GUI { class GraphDisplay; }

class SideScrollScene
{
  public:
    SideScrollScene(GUI::GraphDisplay& display);
    ~SideScrollScene();

    void Initialize(const FlightMap&, const AssetManager&, float worldHeight, float viewWidth,
                    float playerWidth, float playerHeight);
    void SetVisible(bool);
    void Update(float playerX, float playerY);
    bool Collides(float playerX, float playerY, int& objectIndex) const;
    bool OutsideWorld(float playerY) const;
    float CameraX() const;

  private:
    struct Visual
    {
        MapObject object;
        RectangularShape* shape;
        ImageStimulus* image;
    };

    float ViewWidth() const;
    GUI::Rect ScreenRect(float x, float y, float width, float height) const;
    void Clear();

    GUI::GraphDisplay& mDisplay;
    const FlightMap* mpMap;
    std::vector<Visual> mVisuals;
    EllipticShape* mpPlayerShape;
    ImageStimulus* mpPlayerImage;
    ImageStimulus* mpBackground;
    float mWorldHeight;
    float mViewWidth;
    float mPlayerWidth;
    float mPlayerHeight;
    float mCameraX;
    bool mVisible;
};

#endif
