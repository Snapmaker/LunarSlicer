#include "FffSupportGenerator.h"
#include <algorithm>

namespace cura {

long edgeHash(int idx1, int idx2) {
    return idx1 < idx2 ? (long) idx2 * 10000000 + idx1 : (long) idx1 * 1000000 + idx2;
}

void FffSupportGenerator::generateSupport(SupportDataStorage& storage, MeshGroup* meshgroup, TimeKeeper& timeKeeper)
{
    for (int i = 0; i < meshgroup->meshes.size(); ++i)
    {
        storage.support_meshs.emplace_back(SupportMeshStorage(meshgroup->meshes[i].settings));
        generateSupportMesh(storage, meshgroup, i);
    }
}

void FffSupportGenerator::generateSupportMesh(SupportDataStorage &storage, MeshGroup *mesh_group, int index)
{
    auto& mesh = mesh_group->meshes[index];

    auto& support_mesh_storage = storage.support_meshs[index];

    connectSupportArea(support_mesh_storage, mesh);

    calculateOutline(support_mesh_storage, mesh);

    generate(support_mesh_storage, mesh, storage.support_mesh);
}

void FffSupportGenerator::connectSupportArea(SupportMeshStorage &support_mesh_storage, Mesh &mesh) {
    auto &support_areas = support_mesh_storage.support_areas;
    std::vector<bool> is_face_add(mesh.faces.size());
    std::fill(is_face_add.begin(), is_face_add.end(), false);

    for (int i = 0; i < mesh.faces.size(); ++i)
    {
        auto &face = mesh.faces[i];

        if (is_face_add[i] || face.support_flag <= 0)
        {
            continue;
        }

        std::deque<int> face_ids_deque;

        face_ids_deque.emplace_back(i);
        is_face_add[i] = true;
        support_areas.emplace_back();
        support_areas.back().faces_index.emplace_back(i);

        while (!face_ids_deque.empty())
        {
            int face_id = face_ids_deque.front();
            face_ids_deque.pop_front();

            auto &connected_face_index = mesh.faces[face_id].connected_face_index;
            for (int j = 0; j < 3; ++j)
            {
                int connected_face_id = connected_face_index[j];
                if (connected_face_id == -1 || is_face_add[connected_face_id] || mesh.faces[connected_face_id].support_flag == 0)
                {
                    continue;
                }

                is_face_add[connected_face_id] = true;
                support_areas.back().faces_index.emplace_back(connected_face_id);
                face_ids_deque.emplace_back(connected_face_id);
            }
        }
    }
}

void FffSupportGenerator::calculateOutline(SupportMeshStorage &support_mesh_storage, Mesh &mesh) {
    for (int i = 0; i < support_mesh_storage.support_areas.size(); ++i)
    {
        auto &support_area = support_mesh_storage.support_areas[i];
        auto &outlines_edge_vertices_index = support_area.outlines_edge_vertices_index;

        std::vector<std::pair<long, std::pair<int, int>>> edge_index;
        std::map<int, std::vector<int>> edge_map;

        for (int j = 0; j < support_area.faces_index.size(); ++j)
        {
            auto &face = mesh.faces[support_area.faces_index[j]];
            for (int k = 0; k < 3; ++k)
            {
                int v1 = face.vertex_index[k];
                int v2 = face.vertex_index[(k + 1) % 3];
                edge_index.emplace_back();
                edge_index.back().first = v1 < v2 ? (long) v2 * 1000000 + v1 : (long) v1 * 1000000 + v2;
                edge_index.back().second.first = v1;
                edge_index.back().second.second = v2;
            }
        }

        std::sort(edge_index.begin(), edge_index.end(), [](std::pair<long, std::pair<int, int>> &a, std::pair<long, std::pair<int, int>> &b) {
            return a.first < b.first;
        });

        for (int j = 0; j < (int) edge_index.size(); ++j)
        {
            if (j < (int) edge_index.size() - 1 && edge_index[j].first == edge_index[j + 1].first)
            {
                j++;
                continue;
            }
            outlines_edge_vertices_index.emplace_back(edge_index[j].second);
        }
    }
}

void FffSupportGenerator::generate(SupportMeshStorage &support_mesh_storage, Mesh &generate_mesh,  Mesh& support_mesh)
{
    for (int i = 0; i < (int) support_mesh_storage.support_areas.size(); ++i)
    {
        auto &support_area = support_mesh_storage.support_areas[i];
        for (int j = 0; j < (int) support_area.faces_index.size(); ++j)
        {
            auto &face = generate_mesh.faces[support_area.faces_index[j]];

            auto& v0 = generate_mesh.vertices[face.vertex_index[2]].p;
            auto& v1 = generate_mesh.vertices[face.vertex_index[1]].p;
            auto& v2 = generate_mesh.vertices[face.vertex_index[0]].p;
            support_mesh.addFace(v0, v1, v2);
        }

        for (int j = 0; j < (int) support_area.faces_index.size(); ++j)
        {
            auto &face = generate_mesh.faces[support_area.faces_index[j]];

            auto& v0 = generate_mesh.vertices[face.vertex_index[0]].p;
            auto& v1 = generate_mesh.vertices[face.vertex_index[1]].p;
            auto& v2 = generate_mesh.vertices[face.vertex_index[2]].p;
            support_mesh.addFace(v0, v1, v2);
        }

        for (int j = 0; j < (int) support_area.outlines_edge_vertices_index.size(); ++j)
        {
            auto &cur_point = generate_mesh.vertices[support_area.outlines_edge_vertices_index[j].first].p;
            auto &next_point = generate_mesh.vertices[support_area.outlines_edge_vertices_index[j].second].p;

            auto v1 = Point3(cur_point.x, cur_point.y, 0);
            auto v2 = Point3(next_point.x, next_point.y, 0);

            support_mesh.addFace(cur_point, v1, v2);

            auto v3 = Point3(next_point.x, next_point.y, 0);

            support_mesh.addFace(cur_point, v3, next_point);
        }
    }
}

}