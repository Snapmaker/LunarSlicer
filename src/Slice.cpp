//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ExtruderTrain.h"
#include "Slice.h"

#include <utility>
#include "utils/logoutput.h"

namespace cura
{

Slice::Slice(const size_t num_mesh_groups)
: scene(num_mesh_groups)
{}

Slice::Slice(std::string cmd, const size_t num_mesh_groups)
    : cmd(std::move(cmd)), scene(num_mesh_groups)
{}

void Slice::compute()
{
    logWarning("%s", scene.getAllSettingsString().c_str());
    if (this->cmd == "slice")
    {
        for (std::vector<MeshGroup>::iterator mesh_group = scene.mesh_groups.begin(); mesh_group != scene.mesh_groups.end(); mesh_group++)
        {
          scene.current_mesh_group = mesh_group;
          for (ExtruderTrain& extruder : scene.extruders)
          {
            extruder.settings.setParent(&scene.current_mesh_group->settings);
          }
          scene.processMeshGroup(*mesh_group);
        }
    } else if (this->cmd == "modelsupport")
    {
        for (std::vector<MeshGroup>::iterator mesh_group = scene.mesh_groups.begin(); mesh_group != scene.mesh_groups.end(); mesh_group++)
        {
          scene.current_mesh_group = mesh_group;
          scene.processMeshGroupSupport(*mesh_group);
        }
    }

}

void Slice::reset()
{
    scene.extruders.clear();
    scene.mesh_groups.clear();
    scene.settings = Settings();
}

}