#include "MultiMaterialSegmentation.h"
#include "Application.h"
#include "Scene.h"
#include "Slice.h"
#include "boost/polygon/voronoi.hpp"
#include "utils/Constant.h"
#include "utils/Simplify.h"
#include "utils/ThreadPool.h"
#include "utils/gettime.h"
#include "utils/linearAlg2D.h"
#include "utils/polygonUtils.h"

namespace cura
{

class Slice;

void MultiMaterialSegmentation::paintingSlicer(Slicer* slicer, Slicer* color_slicer)
{
    MultiMaterialSegmentation mms(slicer->layers.size());
    mms.paintingSlicerLayers(slicer, color_slicer);
}

void MultiMaterialSegmentation::paintingSlicerLayers(Slicer* slicer, Slicer* color_slicer)
{
    auto& layers = slicer->layers;
    auto* p_mesh = slicer->mesh;

    for (int i = 0; i < layers.size(); ++i)
    {
        layers[i].polygons = layers[i].polygons.unionPolygons();
        layers[i].polygons = Simplify(100, 10, 10000).polygon(layers[i].polygons);
        color_slicer->layers.emplace_back();
        color_slicer->layers[i].z = layers[i].z;
    }

    int top_layers = std::max(slicer->mesh->settings.get<int>("top_layers"), 1);
    int bottom_layers = std::max(slicer->mesh->settings.get<int>("bottom_layers"), 1);
    coord_t wall_outer_line_width = slicer->mesh->settings.get<coord_t>("wall_line_width_0");
    coord_t wall_inner_line_width = slicer->mesh->settings.get<coord_t>("wall_line_width_x");

    coord_t z_h = MM2INT(1);
    coord_t min_offset_len = MM2INT(0.04);

    std::vector<Polygons> no_colored_top_faces_polys_list(layers.size());
    std::vector<Polygons> no_colored_bottom_faces_polys_list(layers.size());

    cura::parallel_for<int>(0,
                            (int)layers.size(),
                            [&](int layer_nr)
                            {
                                m_colored_lines_polys_list[layer_nr] = paintingSlicerLayerColoredLines(slicer->layers[layer_nr]);

                                Polygons top_diff_polys = layer_nr < (layers.size() - 1) ? layers[layer_nr].polygons.difference(layers[layer_nr + 1].polygons) : layers[layer_nr].polygons;
                                Polygons bottom_diff_polys = layer_nr > 0 ? layers[layer_nr].polygons.difference(layers[layer_nr - 1].polygons) : layers[layer_nr].polygons;

                                coord_t top_z = layer_nr < (layers.size() - 1) ? layers[layer_nr + 1].z : layers[layer_nr].z + z_h;
                                coord_t bottom_z = layer_nr > 0 ? layers[layer_nr - 1].z : layers[layer_nr].z - z_h;

                                // Top
                                Polygons colored_top_faces_polys = paintingSlicerLayerColoredFaces(layers[layer_nr], p_mesh, layers[layer_nr].z, top_z);
                                m_colored_top_faces_polys_list[layer_nr] = colored_top_faces_polys.intersection(top_diff_polys);
                                no_colored_top_faces_polys_list[layer_nr] = top_diff_polys.difference(colored_top_faces_polys);

                                // Bottom
                                Polygons colored_bottom_faces_polys = paintingSlicerLayerColoredFaces(layers[layer_nr], p_mesh, bottom_z, layers[layer_nr].z);
                                m_colored_bottom_faces_polys_list[layer_nr] = colored_bottom_faces_polys.intersection(bottom_diff_polys);
                                no_colored_bottom_faces_polys_list[layer_nr] = bottom_diff_polys.difference(colored_bottom_faces_polys);
                            });

    int top_parallel_size = std::ceil(static_cast<double>(layers.size()) / top_layers);
    int bottom_parallel_size = std::ceil(static_cast<double>(layers.size()) / bottom_layers);

    std::vector<Polygons> colored_skin_faces_polys_list(layers.size());
    std::vector<Polygons> no_colored_skin_faces_polys_list(layers.size());

    for (int i = 0; i < top_layers; ++i)
    {
        cura::parallel_for<int>(0,
                                top_parallel_size,
                                [&](int j)
                                {
                                    int layer_nr = j * top_layers + i;
                                    if (layer_nr >= layers.size())
                                    {
                                        return;
                                    }

                                    m_colored_faces_polys_list[layer_nr].add(m_colored_top_faces_polys_list[layer_nr]);

                                    Polygons intersection_outline_polys = layers[layer_nr].polygons;
                                    coord_t width = -wall_outer_line_width;
                                    for (int k = layer_nr - 1; k >= std::max(0, layer_nr - top_layers + 1); --k)
                                    {
                                        intersection_outline_polys = intersection_outline_polys.intersection(layers[k].polygons);

                                        Polygons offset_polys = intersection_outline_polys.offset(width);

                                        Polygons skin_polys = m_colored_top_faces_polys_list[layer_nr].intersection(offset_polys);
                                        skin_polys = PolygonUtils::simplifyByScale(skin_polys, wall_outer_line_width);
                                        colored_skin_faces_polys_list[k].add(skin_polys);

                                        Polygons no_skin_polys = no_colored_top_faces_polys_list[layer_nr].intersection(offset_polys);
                                        no_skin_polys = PolygonUtils::simplifyByScale(no_skin_polys, wall_outer_line_width);
                                        no_colored_skin_faces_polys_list[k].add(no_skin_polys);

                                        width -= wall_inner_line_width;
                                    }
                                });
    }

    for (int i = 0; i < top_layers; ++i)
    {
        cura::parallel_for<int>(0,
                                bottom_parallel_size,
                                [&](int j)
                                {
                                    int layer_nr = j * top_layers + i;
                                    if (layer_nr >= layers.size())
                                    {
                                        return;
                                    }
                                    m_colored_faces_polys_list[layer_nr].add(m_colored_bottom_faces_polys_list[layer_nr]);

                                    Polygons intersection_outline_polys = layers[layer_nr].polygons;
                                    coord_t width = -wall_outer_line_width;
                                    for (int k = layer_nr + 1; k <= std::min((int)layers.size() - 1, layer_nr + top_layers - 1); ++k)
                                    {
                                        intersection_outline_polys = intersection_outline_polys.intersection(layers[k].polygons);

                                        Polygons offset_polys = intersection_outline_polys.offset(width);

                                        Polygons skin_polys = m_colored_bottom_faces_polys_list[layer_nr].intersection(offset_polys);
                                        skin_polys = PolygonUtils::simplifyByScale(skin_polys, wall_outer_line_width);
                                        colored_skin_faces_polys_list[k].add(skin_polys);

                                        Polygons no_skin_polys = no_colored_bottom_faces_polys_list[layer_nr].intersection(offset_polys);
                                        no_skin_polys = PolygonUtils::simplifyByScale(no_skin_polys, wall_outer_line_width);
                                        no_colored_skin_faces_polys_list[k].add(no_skin_polys);

                                        width -= wall_inner_line_width;
                                    }
                                });
    }

    cura::parallel_for<int>(0,
                            (int)layers.size(),
                            [&](int layer_nr) {
                                Polygons offset_polys = layers[layer_nr].polygons.offset(-wall_outer_line_width);

                                colored_skin_faces_polys_list[layer_nr] = colored_skin_faces_polys_list[layer_nr].unionPolygons();
                                no_colored_skin_faces_polys_list[layer_nr] = no_colored_skin_faces_polys_list[layer_nr].unionPolygons();

                                colored_skin_faces_polys_list[layer_nr] = colored_skin_faces_polys_list[layer_nr].intersection(offset_polys);
                                no_colored_skin_faces_polys_list[layer_nr] = no_colored_skin_faces_polys_list[layer_nr].intersection(offset_polys);

                                m_colored_faces_polys_list[layer_nr] = m_colored_faces_polys_list[layer_nr].unionPolygons(colored_skin_faces_polys_list[layer_nr]);
                            });

    cura::parallel_for<int>(0,
                            (int)layers.size(),
                            [&](int layer_nr)
                            {
                                Polygons offset_polys = layers[layer_nr].polygons.offset(-wall_outer_line_width);

                                m_colored_lines_polys_list[layer_nr] = m_colored_lines_polys_list[layer_nr].difference(no_colored_skin_faces_polys_list[layer_nr]);

                                Polygons top_diff_polys = layer_nr < (layers.size() - 1) ? layers[layer_nr].polygons.difference(layers[layer_nr + 1].polygons) : layers[layer_nr].polygons;
                                Polygons no_colored_top_skin_polys = top_diff_polys.difference(m_colored_top_faces_polys_list[layer_nr].offset(min_offset_len));
                                no_colored_top_skin_polys = no_colored_top_skin_polys.difference(m_colored_faces_polys_list[layer_nr]);

                                m_colored_lines_polys_list[layer_nr] = m_colored_lines_polys_list[layer_nr].difference(no_colored_top_skin_polys.intersection(offset_polys));
                                m_colored_lines_polys_list[layer_nr] = m_colored_lines_polys_list[layer_nr].difference(PolygonUtils::simplifyByScale(no_colored_top_skin_polys, wall_outer_line_width));


                                Polygons bottom_diff_polys = layer_nr > 0 ? layers[layer_nr].polygons.difference(layers[layer_nr - 1].polygons) : layers[layer_nr].polygons;
                                Polygons no_colored_bottom_skin_polys = bottom_diff_polys.difference(m_colored_bottom_faces_polys_list[layer_nr].offset(min_offset_len));
                                no_colored_bottom_skin_polys = no_colored_bottom_skin_polys.difference(m_colored_faces_polys_list[layer_nr]);

                                m_colored_lines_polys_list[layer_nr] = m_colored_lines_polys_list[layer_nr].difference(no_colored_bottom_skin_polys.intersection(offset_polys));
                                m_colored_lines_polys_list[layer_nr] = m_colored_lines_polys_list[layer_nr].difference(PolygonUtils::simplifyByScale(no_colored_bottom_skin_polys, wall_outer_line_width));


                                if (m_colored_lines_polys_list[layer_nr].size() > 1)
                                {
                                    std::vector<Polygons> split_polys;
                                    Polygons res;
                                    PolygonUtils::splitToSimplePolygons(m_colored_lines_polys_list[layer_nr], split_polys);

                                    for (int j = 0; j < split_polys.size(); ++j)
                                    {
                                        if (split_polys[j].difference(offset_polys).empty())
                                        {
                                            continue;
                                        }
                                        res.add(split_polys[j]);
                                    }
                                    m_colored_lines_polys_list[layer_nr] = res;
                                }
                            });

    cura::parallel_for<int>(0,
                            (int)layers.size(),
                            [&](int layer_nr)
                            {
                                Polygons colored_polys = m_colored_faces_polys_list[layer_nr].unionPolygons(m_colored_lines_polys_list[layer_nr]);
                                Polygons no_colored_polys = layers[layer_nr].polygons.difference(colored_polys.offset(20));
                                color_slicer->layers[layer_nr].polygons = colored_polys;
                                layers[layer_nr].polygons = no_colored_polys;
                            });
}

Polygons MultiMaterialSegmentation::paintingSlicerLayerColoredLines(SlicerLayer& slicer_layer)
{
    if (! slicer_layer.hasColoredSegments())
    {
        return Polygons();
    }
    Polygons old_polygons = slicer_layer.polygons;
    if (slicer_layer.color_segments_set.size() == 1)
    {
        return old_polygons;
    }

    std::vector<SlicerSegment> slicer_segments;
    std::copy_if(slicer_layer.segments.begin(), slicer_layer.segments.end(), std::back_inserter(slicer_segments), [](SlicerSegment& slicer_segment) { return slicer_segment.color == MESH_PAINTING_COLOR; });

    if (slicer_segments.size() == 0)
    {
        return Polygons();
    }

    Polygons color_line_polys;
    for (int i = 0; i < slicer_segments.size(); ++i)
    {
        Polygon poly;
        auto slicer_segment = slicer_segments[i];
        poly.add(slicer_segment.start);
        poly.add(slicer_segment.end);
        color_line_polys.add(poly);
    }

    coord_t min_offset_len = MM2INT(0.04);
    color_line_polys = color_line_polys.offsetPolyLine(min_offset_len);

    Polygons all_voronoi_color_polys;
    std::vector<Polygons> simple_polygons;

    PolygonUtils::splitToSimplePolygons(old_polygons, simple_polygons);
    for (int i = 0; i < simple_polygons.size(); ++i)
    {
        Polygons polys = simple_polygons[i];

        Polygons color_polys;
        std::vector<Segment> color_segments;
        coloredLineSegmentMatching2(polys, color_line_polys, color_polys, color_segments);
//        coloredLineSegmentMatching(polys, color_line_polys, color_polys, color_segments);

        std::set<int> colors;
        for (int j = 0; j < color_segments.size(); ++j)
        {
            colors.insert(color_segments[j].color);
        }

        if (colors.size() == 1)
        {
            if (colors.find(MESH_PAINTING_COLOR) != colors.end())
            {
                all_voronoi_color_polys.add(polys);
            }
            continue;
        }

        Polygons voronoi_color_polys = toVoronoiColorPolygons(color_segments);
        voronoi_color_polys = voronoi_color_polys.intersection(polys);
        all_voronoi_color_polys.add(voronoi_color_polys);
    }

    all_voronoi_color_polys = all_voronoi_color_polys.unionPolygons();
    all_voronoi_color_polys = all_voronoi_color_polys.intersection(old_polygons);

    return all_voronoi_color_polys;
}

Polygons MultiMaterialSegmentation::toVoronoiColorPolygons(std::vector<Segment>& colored_segments)
{
    vd_t vonoroi_diagram;
    construct_voronoi(colored_segments.begin(), colored_segments.end(), &vonoroi_diagram);

    Polygons voronoi_color_polys;

    for (int i = 0; i < vonoroi_diagram.cells().size(); ++i)
    {
        auto& cell = vonoroi_diagram.cells()[i];
        if (! cell.incident_edge() && ! cell.contains_segment())
        {
            continue;
        }
        auto& segment = colored_segments[cell.source_index()];
        if (segment.color <= MESH_NO_PAINTING_COLOR)
        {
            continue;
        }

        Polygon poly;
        auto from = segment.from();
        auto to = segment.to();

        auto* start = cell.incident_edge();
        bool is_circle = false;
        do
        {
            if (start->vertex0() == nullptr && start->vertex1())
            {
                break;
            }
            start = start->prev();

            if (start == cell.incident_edge())
            {
                is_circle = true;
            }
        } while (start != cell.incident_edge());

        if (! is_circle)
        {
            poly.add(to);
        }
        auto* next = start;
        do
        {
            if (next->vertex0() && next->vertex1() == nullptr)
            {
                break;
            }
            poly.add(Point(next->vertex1()->x(), next->vertex1()->y()));
            next = next->next();
        } while (next != start);
        if (! is_circle)
        {
            poly.add(from);
        }
        poly.sortArea();
        voronoi_color_polys.add(poly);
    }

    voronoi_color_polys = voronoi_color_polys.unionPolygons();

    return voronoi_color_polys;
}


void MultiMaterialSegmentation::coloredLineSegmentMatching(Polygons& polys, Polygons& color_line_polys, Polygons& out_color_polys, std::vector<Segment>& out_color_segments)
{
    AABB poly_lines_aabb;
    for (int i = 0; i < color_line_polys.size(); ++i)
    {
        for (int j = 0; j < color_line_polys[i].size(); ++j)
        {
            poly_lines_aabb.include(color_line_polys[i][j]);
        }
    }

    for (int i = 0; i < polys.size(); ++i)
    {
        Polygon color_poly;
        for (int j = 0; j < polys[i].size(); ++j)
        {
            Point& p1 = polys[i][j];
            Point& p2 = polys[i][(j + 1) % polys[i].size()];

            AABB aabb;
            aabb.include(p1);
            aabb.include(p2);

            if (! poly_lines_aabb.hit(aabb))
            {
                out_color_segments.emplace_back(&out_color_polys, out_color_polys.size(), color_poly.size(), MESH_NO_PAINTING_COLOR);
                color_poly.emplace_back(p1);
                continue;
            }

            Line res = linePolygonsIntersection(p1, p2, color_line_polys);

            for (int k = 0; k < res.colors.size(); ++k)
            {
                out_color_segments.emplace_back(&out_color_polys, out_color_polys.size(), color_poly.size(), res.colors[k]);
                color_poly.emplace_back(res.points[k]);
            }
        }
        out_color_polys.add(color_poly);
    }
}


void MultiMaterialSegmentation::coloredLineSegmentMatching2(Polygons& polys, Polygons& color_line_polys, Polygons& out_color_polys, std::vector<Segment>& out_color_segments)
{
    for (int i = 0; i < polys.size(); ++i)
    {
        if (polys[i].front() != polys[i].back()) {
            polys[i].add(polys[i].front());
        }
    }

    auto copyPolygons = [&polys, &out_color_polys, &out_color_segments](int color){
        for (int i = 0; i < polys.size(); ++i)
        {
            Polygon color_poly;
            for (int j = 0; j < polys[i].size(); ++j)
            {
                color_poly.add(polys[i][j]);
                out_color_segments.emplace_back(&out_color_polys, i, j, color);
            }
            out_color_polys.add(color_poly);
        }
    };

    Polygons diffLines = color_line_polys.differenceOpenPolygons(polys);


    if (diffLines.empty()) {
        copyPolygons(MESH_PAINTING_COLOR);
        return;
    }

    Polygons interLines = color_line_polys.intersectionOpenPolygons(polys);

    if (interLines.empty()) {
        copyPolygons(MESH_NO_PAINTING_COLOR);
        return;
    }

    int color_size = diffLines.size();

//    diffLines.print();
//    interLines.print();

    diffLines.add(interLines);

    auto getColor = [&color_size](int size){
        return size < color_size ? MESH_NO_PAINTING_COLOR : MESH_PAINTING_COLOR;
    };

    int use_count = 0;
    std::vector<bool> used(diffLines.size(), false);

    // The lines front in the first and back in the second;
    std::unordered_map<Point, std::vector<int>> point_lines_map;
    std::vector<std::vector<int>> color_segments_tmp;

    for (int i = 0; i < diffLines.size(); ++i)
    {
        Point& front = diffLines[i].front();
        Point& back = diffLines[i].back();
        if (front == back) {
            use_count++;
            used[i] = true;
            out_color_polys.add(diffLines[i]);
            color_segments_tmp.emplace_back();
            color_segments_tmp.back().resize(diffLines[i].size(), getColor(i));
            continue;
        }
        if (point_lines_map.find(front) == point_lines_map.end()) {
            point_lines_map[front] = std::vector<int>();
        }
        if (point_lines_map.find(back) == point_lines_map.end()) {
            point_lines_map[back] = std::vector<int>();
        }

        point_lines_map[front].push_back(i);
        point_lines_map[back].push_back(i);
    }

    while (use_count < diffLines.size()) {
        Polygon poly;
        color_segments_tmp.emplace_back();
        for (int i = 0; i < diffLines.size(); ++i)
        {
            if (used[i]) {
                continue;
            }
            used[i] = true;
            use_count++;
            for (int j = 0; j < diffLines[i].size(); ++j)
            {
                poly.add(diffLines[i][j]);
                color_segments_tmp.back().emplace_back(getColor(i));
            }
            break;
        }

        while (use_count < diffLines.size()) {
            auto next = point_lines_map.find(poly.back());
            assert(next != point_lines_map.end());
            if (next == point_lines_map.end()) {
                spdlog::warn("Abnormal color matching of line segments, next is null");
                break;
            }
            int next_idx = -1;
            for (int i = 0; i < next->second.size(); ++i)
            {
                if (used[next->second[i]]) {
                    continue;
                }
                next_idx = next->second[i];
            }
            if (next_idx == -1) {
                spdlog::warn("Abnormal color matching of line segments, next_idx is -1");
                break;
            }
            use_count++;
            used[next_idx] = true;
            auto next_poly = diffLines[next_idx];
            bool is_first = poly.back() == next_poly.front();
            for (int j = is_first ? 0 : next_poly.size() - 1; is_first ? j <= next_poly.size() - 1: j >= 0;is_first ? ++j:--j)
            {
                poly.add(next_poly[j]);
                color_segments_tmp.back().emplace_back(getColor(next_idx));
            }
            if (poly.front() == poly.back()) {
                break;
            }
        }
        out_color_polys.add(poly);
    }

    assert(polys.size() == out_color_polys.size());
    assert(out_color_polys.size() == color_segments_tmp.size());

    // Check the direction of polygons after splicing
    std::vector<int> areas;
    for (int i = 0; i < polys.size(); ++i)
    {
        areas.push_back(polys[i].area());
    }
    for (int i = 0; i < out_color_polys.size(); ++i)
    {
        int area = out_color_polys[i].area();
        int abs_area = std::abs(area);
        int min_area = std::abs(areas[0]) - abs_area;
        int min_area_idx = 0;
        for (int j = 0; j < areas.size(); ++j)
        {
            int area_tmp = std::abs(areas[j]) - std::abs(area);
            if (area_tmp < min_area) {
                min_area = area_tmp;
                min_area_idx = j;
            }
        }
        if ((bool)(area > 0) != (bool)(areas[min_area_idx] > 0)) {
            out_color_polys[i].reverse();
            std::reverse(color_segments_tmp[i].begin(), color_segments_tmp[i].end());
        }
    }

    // Check edge redundant small line segments
    coord_t min_offset_s_len2 = MM2INT(0.06) * MM2INT(0.06);
    for (int i = 0; i < out_color_polys.size(); ++i)
    {
        for (int j = 0; j < out_color_polys[i].size(); ++j)
        {
            if (color_segments_tmp[i][j] != MESH_PAINTING_COLOR) {
                continue ;
            }
            int prev_color = color_segments_tmp[i][(j - 1) % out_color_polys[i].size()];
            int next_color = color_segments_tmp[i][(j + 1) % out_color_polys[i].size()];
            if (prev_color == next_color) {
                continue ;
            }
            coord_t len2 = vSize2((out_color_polys[i][(j + 1) % out_color_polys[i].size()] - out_color_polys[i][j]));
            if (len2 > min_offset_s_len2) {
                continue ;
            }
            color_segments_tmp[i][j] = MESH_NO_PAINTING_COLOR;
        }
    }

    for (int i = 0; i < color_segments_tmp.size(); ++i)
    {
        for (int j = 0; j < color_segments_tmp[i].size(); ++j)
        {
            out_color_segments.emplace_back(&out_color_polys, i, j, color_segments_tmp[i][j]);
        }
    }

}

MultiMaterialSegmentation::Line MultiMaterialSegmentation::linePolygonsIntersection(Point& p1, Point& p2, Polygons& line_polys)
{
    Polygons polys;
    Polygon poly;
    poly.add(p1);
    poly.add(p2);
    polys.add(poly);
    Polygons res = line_polys.intersectionPolyLines(polys, false);

    Line line;
    if (res.size() == 0)
    {
        line.points.emplace_back(p1);
        line.points.emplace_back(p2);
        line.colors.emplace_back(MESH_NO_PAINTING_COLOR);
        return line;
    }
    coord_t len2 = MM2INT(0.05) * MM2INT(0.05);
    if (res.size() == 1)
    {
        Point& res_p1 = res[0][0];
        Point& res_p2 = res[0][1];
        if ((p1 == res_p1 && p2 == res_p2) || (p1 == res_p2 && p2 == res_p1))
        {
            line.points.emplace_back(p1);
            line.points.emplace_back(p2);
            line.colors.emplace_back(MESH_PAINTING_COLOR);
            return line;
        }
        if (p1 == res_p1 || p1 == res_p2)
        {
            Point middle = p1 == res_p1 ? res_p2 : res_p1;
            if (vSize2(middle - p1) < len2)
            {
                line.points.emplace_back(p1);
                line.points.emplace_back(p2);
                line.colors.emplace_back(MESH_NO_PAINTING_COLOR);
            }
            else
            {
                line.points.emplace_back(p1);
                line.points.emplace_back(middle);
                line.points.emplace_back(p2);
                line.colors.emplace_back(MESH_PAINTING_COLOR);
                line.colors.emplace_back(MESH_NO_PAINTING_COLOR);
            }
            return line;
        }
        if (p2 == res_p1 || p2 == res_p2)
        {
            Point middle = p2 == res_p1 ? res_p2 : res_p1;
            if (vSize2(p2 - middle) < len2)
            {
                line.points.emplace_back(p1);
                line.points.emplace_back(p2);
                line.colors.emplace_back(MESH_NO_PAINTING_COLOR);
            }
            else
            {
                line.points.emplace_back(p1);
                line.points.emplace_back(middle);
                line.points.emplace_back(p2);
                line.colors.emplace_back(MESH_NO_PAINTING_COLOR);
                line.colors.emplace_back(MESH_PAINTING_COLOR);
            }
            return line;
        }
    }

    line.points.emplace_back(p1);
    line.points.emplace_back(p2);

    for (int i = 0; i < res.size(); ++i)
    {
        assert(res[i].size() == 2);
        for (int j = 0; j < res[i].size(); ++j)
        {
            line.points.emplace_back(res[i][j]);
        }
    }

    bool sort_key = std::abs(p1.X - p2.X) > std::abs(p1.Y - p2.Y);

    std::sort(line.points.begin(),
              line.points.end(),
              [&sort_key](Point& a, Point& b)
              {
                  if (sort_key)
                  {
                      return a.X < b.X;
                  }
                  else
                  {
                      return a.Y < b.Y;
                  }
              });
    if (line.points.front() != p1)
    {
        std::reverse(line.points.begin(), line.points.end());
        assert(line.points.front() == p1);
        assert(line.points.back() == p2);
    }

    int color = MESH_NO_PAINTING_COLOR;
    for (int i = 0; i < line.points.size() - 1; ++i)
    {
        line.colors.emplace_back(color);
        color = color == MESH_NO_PAINTING_COLOR ? MESH_PAINTING_COLOR : MESH_NO_PAINTING_COLOR;
    }
    if (line.points[0] == line.points[1])
    {
        line.points.erase(line.points.begin());
        line.colors.erase(line.colors.begin());
    }
    if (line.points[line.points.size() - 2] == line.points.back())
    {
        line.points.pop_back();
        line.colors.pop_back();
    }
    return line;
}

Polygons MultiMaterialSegmentation::paintingSlicerLayerColoredFaces(SlicerLayer& layer, const Mesh* p_mesh, coord_t min_z, coord_t max_z)
{
    if (layer.color_faces_idx.empty())
    {
        return Polygons();
    }
    Polygons polygons;
    Point3 z_p(0, 0, 1);

    for (int i = 0; i < layer.color_faces_idx.size(); ++i)
    {
        int face_idx = layer.color_faces_idx[i];
        auto& face = p_mesh->faces[face_idx];
        Point3 vs[3];
        vs[0] = p_mesh->vertices[face.vertex_index[0]].p;
        vs[1] = p_mesh->vertices[face.vertex_index[1]].p;
        vs[2] = p_mesh->vertices[face.vertex_index[2]].p;

        auto cb = vs[2] - vs[1];
        auto ab = vs[0] - vs[1];
        auto normal = cb.cross(ab);

        double cos = normal.dot(z_p) / std::sqrt(normal.vSize2());
        double angle = std::acos(cos);
        if (angle > M_PI / 3 && angle < M_PI / 3 * 2)
        {
            continue;
        }

        std::sort(vs, vs + 3, [](Point3& p1, Point3& p2) { return p1.z < p2.z; });
        if (vs[0].z > max_z || vs[2].z < min_z)
        {
            continue;
        }

        Polygon poly;
        if (vs[0].z >= min_z && vs[2].z <= max_z)
        {
            poly.add(Point(vs[0].x, vs[0].y));
            poly.add(Point(vs[1].x, vs[1].y));
            poly.add(Point(vs[2].x, vs[2].y));
        }
        else if (vs[0].z < min_z && vs[2].z <= max_z)
        {
            poly.add(Point(vs[2].x, vs[2].y));
            if (vs[1].z >= min_z)
            {
                poly.add(Point(vs[1].x, vs[1].y));
                Point3 p1 = getPoint3ByZ(vs[1], vs[0], min_z);
                Point3 p2 = getPoint3ByZ(vs[0], vs[2], min_z);
                poly.add(Point(p1.x, p1.y));
                poly.add(Point(p2.x, p2.y));
            }
            else
            {
                Point3 p1 = getPoint3ByZ(vs[2], vs[1], min_z);
                Point3 p2 = getPoint3ByZ(vs[0], vs[2], min_z);
                poly.add(Point(p1.x, p1.y));
                poly.add(Point(p2.x, p2.y));
            }
        }
        else if (vs[0].z >= min_z && vs[2].z > max_z)
        {
            poly.add(Point(vs[0].x, vs[0].y));
            if (vs[1].z <= max_z)
            {
                poly.add(Point(vs[1].x, vs[1].y));
                Point3 p1 = getPoint3ByZ(vs[1], vs[2], max_z);
                Point3 p2 = getPoint3ByZ(vs[2], vs[0], max_z);
                poly.add(Point(p1.x, p1.y));
                poly.add(Point(p2.x, p2.y));
            }
            else
            {
                Point3 p1 = getPoint3ByZ(vs[0], vs[1], max_z);
                Point3 p2 = getPoint3ByZ(vs[2], vs[0], max_z);
                poly.add(Point(p1.x, p1.y));
                poly.add(Point(p2.x, p2.y));
            }
        }
        else
        {
            if (vs[1].z > max_z)
            {
                Point3 p1 = getPoint3ByZ(vs[0], vs[2], max_z);
                Point3 p2 = getPoint3ByZ(vs[1], vs[0], max_z);
                Point3 p3 = getPoint3ByZ(vs[1], vs[0], min_z);
                Point3 p4 = getPoint3ByZ(vs[0], vs[2], min_z);
                poly.add(Point(p1.x, p1.y));
                poly.add(Point(p2.x, p2.y));
                poly.add(Point(p3.x, p3.y));
                poly.add(Point(p4.x, p4.y));
            }
            else if (vs[1].z < min_z)
            {
                Point3 p1 = getPoint3ByZ(vs[0], vs[2], max_z);
                Point3 p2 = getPoint3ByZ(vs[2], vs[1], max_z);
                Point3 p3 = getPoint3ByZ(vs[2], vs[1], min_z);
                Point3 p4 = getPoint3ByZ(vs[0], vs[2], min_z);
                poly.add(Point(p1.x, p1.y));
                poly.add(Point(p2.x, p2.y));
                poly.add(Point(p3.x, p3.y));
                poly.add(Point(p4.x, p4.y));
            }
            else
            {
                Point3 p1 = getPoint3ByZ(vs[0], vs[2], max_z);
                Point3 p2 = getPoint3ByZ(vs[2], vs[1], max_z);
                Point3 p3 = vs[1];
                Point3 p4 = getPoint3ByZ(vs[1], vs[0], min_z);
                Point3 p5 = getPoint3ByZ(vs[0], vs[2], min_z);
                poly.add(Point(p1.x, p1.y));
                poly.add(Point(p2.x, p2.y));
                poly.add(Point(p3.x, p3.y));
                poly.add(Point(p4.x, p4.y));
                poly.add(Point(p5.x, p5.y));
            }
        }

        if (poly.empty())
        {
            continue;
        }

        poly.sortArea();
        polygons.add(poly);
    }

    polygons = polygons.unionPolygons();
    polygons = polygons.intersection(layer.polygons);
    return polygons;
}

Point3 MultiMaterialSegmentation::getPoint3ByZ(Point3& p1, Point3& p2, int z)
{
    if (p1.z == z)
    {
        return p1;
    }
    if (p2.z == z)
    {
        return p2;
    }
    double k = (double)(z - p1.z) / (p2.z - p1.z);
    return p1 + (p2 - p1) * k;
}

} // namespace cura