//
// Created by zjiefee on 2021/12/22.
//

#ifndef LUBANENGINE_SRC_LUBAN_SUPPORT_SUPPORTDATASTORAGE_H_
#define LUBANENGINE_SRC_LUBAN_SUPPORT_SUPPORTDATASTORAGE_H_

#include "mesh.h"

namespace cura {

class SupportArea {
public:
    std::vector<int> faces_index;

    std::vector<std::pair<int, int>> outlines_edge_vertices_index;
};

class SupportMeshStorage {
public:
    Settings& settings;

    std::vector<SupportArea> support_areas;

    Mesh support_mesh;

    SupportMeshStorage(Settings &settings) : settings(settings) {}
};

class SupportDataStorage {

public:
    Mesh support_mesh;

    std::vector<SupportMeshStorage> support_meshs;
};

}

#endif //LUBANENGINE_SRC_LUBAN_SUPPORT_SUPPORTDATASTORAGE_H_
