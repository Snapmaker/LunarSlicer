#ifndef LUNARSLICER_SRC_FFFSUPPORTGENERATOR_H_
#define LUNARSLICER_SRC_FFFSUPPORTGENERATOR_H_

#include "MeshGroup.h"
#include "SupportDataStorage.h"
#include "utils/gettime.h"

namespace cura {

class FffSupportGenerator {
public:
  void generateSupport(SupportDataStorage& storage, MeshGroup* mesh_group, TimeKeeper& timeKeeper);

  void generateSupportMesh(SupportDataStorage &storage, MeshGroup* mesh_group, int index);

private:
  void connectSupportArea(SupportMeshStorage &support_mesh_storage, Mesh &mesh);

  void calculateOutline(SupportMeshStorage &support_mesh_storage, Mesh &mesh);

  void generate(SupportMeshStorage &support_mesh_storage, Mesh &generate_mesh, Mesh& support_mesh);
};

} // namespace cura

#endif // LUNARSLICER_SRC_FFFSUPPORTGENERATOR_H_
