#include "AssetManager.h"

void AssetManager::Clear()
{
    mPaths.clear();
}

void AssetManager::Register(const std::string& id, const std::string& filename)
{
    if (!id.empty() && !filename.empty())
        mPaths[id] = filename;
}

bool AssetManager::Has(const std::string& id) const
{
    return mPaths.find(id) != mPaths.end();
}

const std::string& AssetManager::Path(const std::string& id) const
{
    return mPaths.at(id);
}
