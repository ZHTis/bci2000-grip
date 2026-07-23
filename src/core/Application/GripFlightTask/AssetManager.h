#ifndef GRIP_FLIGHT_ASSET_MANAGER_H
#define GRIP_FLIGHT_ASSET_MANAGER_H

#include <map>
#include <string>

class AssetManager
{
  public:
    void Clear();
    void Register(const std::string& id, const std::string& filename);
    bool Has(const std::string& id) const;
    const std::string& Path(const std::string& id) const;

  private:
    std::map<std::string, std::string> mPaths;
};

#endif
