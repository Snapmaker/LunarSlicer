//Copyright (C) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <iterator>
#include <algorithm>
#include <cmath>
#include <limits>

#include "AdaptiveLayerHeights.h"
#include "EnumSettings.h"
#include "types/AngleRadians.h"
#include "../Application.h"
#include "../Slice.h"
#include "../utils/logoutput.h"

namespace cura
{

double getArea(std::vector<FPoint3> points)
{
    if (points.size() < 3)
    {
        std::cout << "Error: less points" << std:: endl;
        return 0;
    }
    double area = 0;
    FPoint3 p1 = points[0];
    FPoint3 p2 = points[1];
    FPoint3 p3 = points[2];

    if (points.size() < 3)
    {
        double i = (p1.y - p2.y) * (p1.z - p3.z) - (p1.y - p3.y) * (p1.z - p2.z);
        double j = (p1.x - p3.x) * (p1.z - p2.z) - (p1.x - p2.x) * (p1.z - p3.z);
        double k = (p1.x - p2.x) * (p1.y - p3.y) - (p1.x - p3.x) * (p1.y - p2.y);
        area = std::pow((i + j + k), 1/2);
    }
    else
    {
        FPoint3 p0 = points[points.size() - 1];

        double a = std::pow(((p2.y-p1.y)*(p3.z-p1.z)-(p3.y-p1.y)*(p2.z-p1.z)),2)
            + std::pow(((p3.x-p1.x)*(p2.z-p1.z)-(p2.x-p1.x)*(p3.z-p1.z)),2)
            + std::pow(((p2.x-p1.x)*(p3.y-p1.y)-(p3.x-p1.x)*(p2.y-p1.y)),2);

        double cosnx = ((p2.y-p1.y)*(p3.z-p1.z)-(p3.y-p1.y)*(p2.z-p1.z)) / (std::pow(a,1/2));
        double cosny = ((p3.x-p1.x)*(p2.z-p1.z)-(p2.x-p1.x)*(p3.z-p1.z)) / (std::pow(a,1/2));
        double cosnz = ((p2.x-p1.x)*(p3.y-p1.y)-(p3.x-p1.x)*(p2.y-p1.y)) / (std::pow(a,1/2));

        area = cosnz*(p0.x*p1.y-p1.x*p0.y) + cosnx*(p0.y*p1.z-p1.y*p0.z) + cosny*(p2.z*p1.x-p1.z*p0.x);

        for (int j = 0; j < points.size()-1; j++)
        {
            FPoint3 pj1 = points[j];
            FPoint3 pj2 = points[j+1];
            area += cosnz *((pj1.x)*(pj2.y)-(pj2.x)*(pj1.y))
                + cosnx*((pj1.y)*(pj2.z)-(pj2.y)*(pj1.z))
                + cosny*((pj1.z)*(pj2.x)-(pj2.z)*(pj1.x));
        }
    }
    return std::abs(area) * 0.5;
}

FPoint3 getPoint(float z, FPoint3 p1, FPoint3 p2)
{
    FPoint3 k;
    k.z = z;
    k.x = (z - p1.z) * (p2.x - p1.x) / (p2.z - p1.z) + p1.x;
    k.y = (z - p1.z) * (p2.y - p1.y) / (p2.z - p1.z) + p1.y;
    return k;
}

void getPoints(float z1, float z2, std::vector<FPoint3> points, std::vector<FPoint3>& res)
{
    for (int i = 0; i < 3; i++)
    {
        FPoint3 p0 = points[i];
        FPoint3 p1 = points[(i+1) % 3];

        if (p0.z == p1.z) {
            if (p0.z == z1 || p0.z == z2) {
                res.push_back(p0);
                if (p1 != p0) {
                    res.push_back(p1);
                }
            }
        }
        else if (p0.z < p1.z) {
            FPoint3 q1 = getPoint(z1, p0, p1);
            if (res.empty()) {
                res.push_back(q1);
            }
            else if (q1 != res[res.size() - 1]) {
                res.push_back(q1);
            }

            FPoint3 q2 = getPoint(z2, p0, p1);
            if (q1 != q2) {
                res.push_back(q2);
            }
        }
        else {
            FPoint3 q2  = getPoint(z2, p0, p1);
            if (res.empty()) {
                res.push_back(q2);
            }
            else if (q2 != res[res.size() - 1]) {
                res.push_back(q2);
            }

            FPoint3 q1 = getPoint(z1, p0, p1);
            if (q1 != q2) {
                res.push_back(q1);
            }
        }
    }

}

double getSingleArea(size_t triangle_index, float z1, float z2)
{
    const Mesh& mesh = Application::getInstance().current_slice->scene.current_mesh_group->meshes[0];
    MeshFace face = mesh.faces[triangle_index];

    const MeshVertex &v0 = mesh.vertices[face.vertex_index[0]];
    const MeshVertex &v1 = mesh.vertices[face.vertex_index[1]];
    const MeshVertex &v2 = mesh.vertices[face.vertex_index[2]];

    if (v0.p.z == v1.p.z && v0.p.z == v2.p.z)
    {
        return 0;
    }

    std::vector<FPoint3> points;
    points.emplace_back(FPoint3(v0.p));
    points.emplace_back(FPoint3(v1.p));
    points.emplace_back(FPoint3(v2.p));

    std::vector<FPoint3> res;
    getPoints(z1, z2, points, res);
    if (res.size() < 3) {
        int j = 0;
    }
    double area = getArea(res);
    return area;
}

double roundM(double x) {
    return  ((int)( x * 1000 + 0.5 ) );
}

AdaptiveLayer::AdaptiveLayer(const coord_t layer_height) : layer_height(layer_height) { }

AdaptiveLayerHeights::AdaptiveLayerHeights(const coord_t base_layer_height, const coord_t variation,
                                           const coord_t step_size, const coord_t threshold)
    : base_layer_height(base_layer_height)
    , max_variation(variation)
    , step_size(step_size)
    , threshold(threshold)
{
    layers = {};

    calculateAllowedLayerHeights();
    calculateMeshTriangleSlopes();
    calculateLayers();
}

int AdaptiveLayerHeights::getLayerCount()
{
    return layers.size();
}

std::vector<AdaptiveLayer>* AdaptiveLayerHeights::getLayers()
{
    return &layers;
}

void AdaptiveLayerHeights::calculateAllowedLayerHeights()
{
    // calculate the allowed layer heights from variation and step size
    // note: the order is from thickest to thinnest height!
    for (int allowed_layer_height = base_layer_height + max_variation; allowed_layer_height >= base_layer_height - max_variation; allowed_layer_height -= step_size)
    {
        // we should only consider using layer_heights that are > 0
        if (allowed_layer_height <= 0)
        {
            break;
        }
        allowed_layer_heights.push_back(allowed_layer_height);
    }
}

void AdaptiveLayerHeights::calculateLayers()
{
    const coord_t minimum_layer_height = *std::min_element(allowed_layer_heights.begin(), allowed_layer_heights.end());
    Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    SlicingTolerance slicing_tolerance = mesh_group_settings.get<SlicingTolerance>("slicing_tolerance");
    std::vector<size_t> triangles_of_interest;
    coord_t z_level = 0;
    coord_t previous_layer_height = 0;

    // the first layer has it's own independent height set, so we always add that
    const coord_t initial_layer_height = mesh_group_settings.get<coord_t>("layer_height_0");
    z_level += initial_layer_height;

    AdaptiveLayer adaptive_layer(initial_layer_height);
    adaptive_layer.z_position = z_level;
    previous_layer_height = adaptive_layer.layer_height;
    layers.push_back(adaptive_layer);

    // loop while triangles are found
    while (!triangles_of_interest.empty() || layers.size() < 2)
    {
        double global_min_slope = std::numeric_limits<double>::max();
        int layer_height_for_global_min_slope = 0;
        // loop over all allowed layer heights starting with the largest
        bool has_added_layer = false;
        for (auto & layer_height : allowed_layer_heights)
        {
            // use lower and upper bounds to filter on triangles that are interesting for this potential layer
            const coord_t lower_bound = z_level;
            // if slicing tolerance "middle" is used, a layer is interpreted as the middle of the upper and lower bounds.
            const coord_t upper_bound = z_level + ((slicing_tolerance == SlicingTolerance::MIDDLE) ? (layer_height / 2) : layer_height);

            if (layer_height == allowed_layer_heights[0])
            {
                // this is the max layer thickness, search through all of the triangles in the mesh to find those
                // that intersect with a layer this thick
                triangles_of_interest.clear();

                for (size_t i = 0; i < face_min_z_values.size(); ++i)
                {
                    if (face_min_z_values[i] <= upper_bound && face_max_z_values[i] >= lower_bound)
                    {
                        triangles_of_interest.push_back(i);
                    }
                }
            }
            else
            {
                // this is a reduced thickness layer, just search those triangles that intersected with the layer
                // in the previous iteration
                std::vector<size_t> last_triangles_of_interest = triangles_of_interest;

                triangles_of_interest.clear();

                for (size_t i : last_triangles_of_interest)
                {
                    if (face_min_z_values[i] <= upper_bound)
                    {
                        triangles_of_interest.push_back(i);
                    }
                }
            }

            // when there not interesting triangles in this potential layer go to the next one
            if (triangles_of_interest.empty())
            {
                break;
            }

            // find the minimum slope of all the interesting triangles
            double minimum_slope = std::numeric_limits<double>::max();
//            double max_area_loss = std::numeric_limits<double>::min();
//            double avg_area_loss = 0;
            for (const size_t& triangle_index : triangles_of_interest)
            {
                const double slope = face_slopes.at(triangle_index);
                if (minimum_slope > slope)
                {
                    minimum_slope = slope;
                }
//                double area = getSingleArea(triangle_index, z_level* 0.001, (z_level+layer_height)* 0.001);
//                if (max_area_loss < area)
//                {
//                    max_area_loss = area;
//                }
//                avg_area_loss += area;
            }
//            avg_area_loss /= triangles_of_interest.size();
            if (global_min_slope > minimum_slope)
            {
                global_min_slope = minimum_slope;
                layer_height_for_global_min_slope = layer_height;
            }

            // check if the maximum step size has been exceeded depending on layer height direction
            bool has_exceeded_step_size = false;
            if (previous_layer_height > layer_height && previous_layer_height - layer_height > step_size)
            {
                has_exceeded_step_size = true;
            }
            else if (layer_height - previous_layer_height > step_size && layer_height > minimum_layer_height)
            {
                continue;
            }

            // we add the layer in the following cases:
            // 1) the layer angle is below the threshold and the layer height difference with the previous layer is the maximum allowed step size
            // 2) the layer height is the smallest it is allowed
            // 3) the layer is a flat surface (we can't divide by 0)
            const double minimum_slope_tan = std::tan(minimum_slope);
            const double minimum_slope_cos = roundM((minimum_slope_cos > 0 && minimum_slope_cos <= 1) ? std::cos(minimum_slope) : 0);
            if (minimum_slope_tan == 0.0
//                || (layer_height / minimum_slope_tan) <= threshold
//                || avg_area_loss <= 10
//                || (layer_height / minimum_slope_tan) <= 200
//                || max_area_loss <= 100
                || layer_height*1e-3 * 70.82 / std::sin(minimum_slope) <= 20
                || layer_height * minimum_slope_cos * 1e-3 * 0.5 + 0.18403 * layer_height<= 0.1 * 1e3
                || layer_height == minimum_layer_height
                || has_exceeded_step_size)
            {
                z_level += layer_height;
                AdaptiveLayer adaptive_layer(layer_height);
                adaptive_layer.z_position = z_level;

                const double minimum_slope_sin = std::sin(minimum_slope);
                const double minimum_slope_cos = std::cos(minimum_slope);
                adaptive_layer.minimum_slope_tan = roundM(minimum_slope_tan > 0 ? minimum_slope_tan : 0);
                adaptive_layer.minimum_slope_sin = roundM((minimum_slope_sin > 0 && minimum_slope_sin <= 1) ? std::sin(minimum_slope) : 0);
                adaptive_layer.minimum_slope_cos = roundM((minimum_slope_cos > 0 && minimum_slope_cos <= 1) ? std::cos(minimum_slope) : 0);

                previous_layer_height = adaptive_layer.layer_height;
                layers.push_back(adaptive_layer);
                has_added_layer = true;
                break;
            }
        }

        // stop calculating when we're out of triangles (e.g. above the mesh)
        if (triangles_of_interest.empty())
        {
            break;
        }
        // this means we cannot find a layer height that has an angle lower than the threshold.
        // in this case, we use the layer height with the lowest
        if (!has_added_layer)
        {
            z_level += layer_height_for_global_min_slope;
            AdaptiveLayer adaptive_layer(layer_height_for_global_min_slope);
            adaptive_layer.z_position = z_level;
            previous_layer_height = adaptive_layer.layer_height;
            layers.push_back(adaptive_layer);
        }
    }

    double midline = 0;
    double cusp_height = 0;
    double Ra = 0;
    double delta_R = 0;
    double delta_S = 0;
    for (AdaptiveLayer layer : layers) {

        midline += layer.minimum_slope_tan == 0 ? 0 : layer.layer_height / layer.minimum_slope_tan * 0.5;

        cusp_height += (layer.layer_height * 1e-3) * (layer.minimum_slope_cos * 1e-3);
        Ra += layer.minimum_slope_sin == 0 ? 0 : 70.82 * layer.layer_height / layer.minimum_slope_sin;
        delta_R += (layer.layer_height * 1e-3) * (layer.minimum_slope_cos * 1e-3) * 0.5;
        delta_S += 0.18403 * layer.layer_height * 1e-3;
    }
    int L = layers.size();
    log("mindline: %.3f\n", midline / L);
    log("Cusp height: %.3f\n", cusp_height / L);
    log("Ra: %.3f\n", Ra / L);
    log("delta: %.3f\n", (delta_R+delta_S) / L);
}

void AdaptiveLayerHeights::calculateMeshTriangleSlopes()
{
    // loop over all mesh faces (triangles) and find their slopes
    for (const Mesh& mesh : Application::getInstance().current_slice->scene.current_mesh_group->meshes)
    {
        // Skip meshes that are not printable
        if (mesh.settings.get<bool>("infill_mesh") || mesh.settings.get<bool>("cutting_mesh") || mesh.settings.get<bool>("anti_overhang_mesh"))
        {
            continue;
        }

        for (const MeshFace& face : mesh.faces)
        {
            const MeshVertex& v0 = mesh.vertices[face.vertex_index[0]];
            const MeshVertex& v1 = mesh.vertices[face.vertex_index[1]];
            const MeshVertex& v2 = mesh.vertices[face.vertex_index[2]];

            const FPoint3 p0 = v0.p;
            const FPoint3 p1 = v1.p;
            const FPoint3 p2 = v2.p;

            float min_z = p0.z;
            min_z = std::min(min_z, p1.z);
            min_z = std::min(min_z, p2.z);
            float max_z = p0.z;
            max_z = std::max(max_z, p1.z);
            max_z = std::max(max_z, p2.z);

            // calculate the angle of this triangle in the z direction
            const FPoint3 n = FPoint3(p1 - p0).cross(p2 - p0);
            const FPoint3 normal = n.normalized();
            AngleRadians z_angle = std::acos(std::abs(normal.z));

            // prevent flat surfaces from influencing the algorithm
            if (z_angle == 0)
            {
                z_angle = M_PI;
            }

            face_min_z_values.push_back(min_z * 1000);
            face_max_z_values.push_back(max_z * 1000);
            face_slopes.push_back(z_angle);
        }
    }
}
}
