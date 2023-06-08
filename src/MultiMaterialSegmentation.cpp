#include "MultiMaterialSegmentation.h"
#include "Application.h"
#include "Scene.h"
#include "Slice.h"
#include "boost/polygon/voronoi.hpp"
#include "utils/Constant.h"
#include "utils/Simplify.h"
#include "utils/ThreadPool.h"
#include "utils/linearAlg2D.h"
#include "utils/polygonUtils.h"

namespace cura
{

class Slice;

void MultiMaterialSegmentation::multiMaterialSegmentationByPainting(Slicer* slicer, Slicer* color_slicer)
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

                                        Polygons skin_polys = m_colored_top_faces_polys_list[layer_nr].intersection(intersection_outline_polys.offset(width));
                                        skin_polys = PolygonUtils::simplifyByScale(skin_polys, wall_outer_line_width);
                                        colored_skin_faces_polys_list[k].add(skin_polys);

                                        Polygons no_skin_polys = no_colored_top_faces_polys_list[layer_nr].intersection(intersection_outline_polys.offset(width));
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

                                        Polygons skin_polys = m_colored_bottom_faces_polys_list[layer_nr].intersection(intersection_outline_polys.offset(width));
                                        skin_polys = PolygonUtils::simplifyByScale(skin_polys, wall_outer_line_width);
                                        colored_skin_faces_polys_list[k].add(skin_polys);

                                        Polygons no_skin_polys = no_colored_bottom_faces_polys_list[layer_nr].intersection(intersection_outline_polys.offset(width));
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
                                Polygons no_colored_top_skin_polys = top_diff_polys.difference(m_colored_top_faces_polys_list[layer_nr].offset(40));
                                no_colored_top_skin_polys = no_colored_top_skin_polys.difference(m_colored_faces_polys_list[layer_nr]);

                                m_colored_lines_polys_list[layer_nr] = m_colored_lines_polys_list[layer_nr].difference(no_colored_top_skin_polys.intersection(offset_polys));
                                m_colored_lines_polys_list[layer_nr] = m_colored_lines_polys_list[layer_nr].difference(PolygonUtils::simplifyByScale(no_colored_top_skin_polys, wall_outer_line_width));


                                Polygons bottom_diff_polys = layer_nr > 0 ? layers[layer_nr].polygons.difference(layers[layer_nr - 1].polygons) : layers[layer_nr].polygons;
                                Polygons no_colored_bottom_skin_polys = bottom_diff_polys.difference(m_colored_bottom_faces_polys_list[layer_nr].offset(40));
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

    color_line_polys = color_line_polys.offsetPolyLine(40);

    Polygons all_voronoi_color_polys;
    std::vector<Polygons> simple_polygons;

    PolygonUtils::splitToSimplePolygons(old_polygons, simple_polygons);
    for (int i = 0; i < simple_polygons.size(); ++i)
    {
        Polygons polys = simple_polygons[i];

        Polygons color_polys;
        std::vector<Segment> color_segments;
        paintingPolygonByColorLinePolygons(polys, color_line_polys, color_polys, color_segments);

        std::set<int> colors;
        for (int j = 0; j < color_segments.size(); ++j)
        {
            colors.insert(color_segments[j].color);
        }

        if (colors.size() == 1)
        {
            if (colors.find(MESH_PAINTING_COLOR) == colors.end())
            {
                continue;
            }
            all_voronoi_color_polys.add(polys);
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


void MultiMaterialSegmentation::paintingPolygonByColorLinePolygons(Polygons& polys, Polygons& color_line_polys, Polygons& color_polys, std::vector<Segment>& color_segments)
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
                color_segments.emplace_back(&color_polys, color_polys.size(), color_poly.size(), MESH_NO_PAINTING_COLOR);
                color_poly.emplace_back(p1);
                continue;
            }

            Line res = linePolygonsIntersection(p1, p2, color_line_polys);

            for (int k = 0; k < res.colors.size(); ++k)
            {
                color_segments.emplace_back(&color_polys, color_polys.size(), color_poly.size(), res.colors[k]);
                color_poly.emplace_back(res.points[k]);
            }
        }
        color_polys.add(color_poly);
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