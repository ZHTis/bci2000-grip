#include "MapLoader.h"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace
{
std::string Trim(const std::string& value)
{
    const std::string whitespace = " \t\r\n";
    const size_t first = value.find_first_not_of(whitespace);
    if (first == std::string::npos)
        return "";
    return value.substr(first, value.find_last_not_of(whitespace) - first + 1);
}
}

bool FlightMap::LoadCsv(const std::string& filename, std::string& error)
{
    std::ifstream input(filename.c_str());
    if (!input)
    {
        error = "Could not open map file: " + filename;
        return false;
    }

    std::vector<MapObject> objects;
    std::string line;
    int lineNumber = 0;
    float finishX = 0;
    while (std::getline(input, line))
    {
        ++lineNumber;
        line = Trim(line);
        if (line.empty() || line[0] == '#')
            continue;

        std::vector<std::string> fields;
        std::stringstream row(line);
        std::string field;
        while (std::getline(row, field, ','))
            fields.push_back(Trim(field));
        if (fields.size() != 6)
        {
            error = "Map line " + std::to_string(lineNumber) + " must have 6 comma-separated fields";
            return false;
        }
        if (fields[0] == "type")
            continue;

        MapObject object{fields[0], static_cast<float>(std::atof(fields[1].c_str())),
                         static_cast<float>(std::atof(fields[2].c_str())),
                         static_cast<float>(std::atof(fields[3].c_str())),
                         static_cast<float>(std::atof(fields[4].c_str())), fields[5]};
        objects.push_back(object);
        finishX = std::max(finishX, object.x + object.width * 0.5f);
    }
    if (objects.empty())
    {
        error = "Map contains no objects";
        return false;
    }
    mObjects.swap(objects);
    mFinishX = finishX + 50;
    return true;
}

void FlightMap::LoadBuiltIn()
{
    mObjects = {
        {"bottom", 180, 15, 18, 30, "obstacle_bottom"}, {"top", 180, 87.5f, 18, 25, "obstacle_top"},
        {"bottom", 340, 24, 18, 48, "obstacle_bottom"}, {"top", 340, 94, 18, 12, "obstacle_top"},
        {"bottom", 500, 10, 18, 20, "obstacle_bottom"}, {"top", 500, 81, 18, 38, "obstacle_top"},
        {"bottom", 660, 20, 18, 40, "obstacle_bottom"}, {"top", 660, 91, 18, 18, "obstacle_top"},
        {"bottom", 820, 12.5f, 18, 25, "obstacle_bottom"}, {"top", 820, 84, 18, 32, "obstacle_top"},
    };
    mFinishX = 930;
}

const std::vector<MapObject>& FlightMap::Objects() const
{
    return mObjects;
}

float FlightMap::FinishX() const
{
    return mFinishX;
}
