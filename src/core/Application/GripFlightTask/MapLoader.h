#ifndef GRIP_FLIGHT_MAP_LOADER_H
#define GRIP_FLIGHT_MAP_LOADER_H

#include <string>
#include <vector>

struct MapObject
{
    std::string type;
    float x;
    float y;
    float width;
    float height;
    std::string assetId;
};

class FlightMap
{
  public:
    bool LoadCsv(const std::string& filename, std::string& error);
    void LoadBuiltIn();
    const std::vector<MapObject>& Objects() const;
    float FinishX() const;

  private:
    std::vector<MapObject> mObjects;
    float mFinishX = 1000;
};

#endif
