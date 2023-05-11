#ifndef MULTI_MATERIAL_SEGMENTATION_H
#define MULTI_MATERIAL_SEGMENTATION_H

#include <boost/geometry.hpp>
#include <boost/polygon/voronoi.hpp>
#include <boost/polygon/voronoi_diagram.hpp>
#include <vector>

#include "BoostInterface.hpp"
#include "SkeletalTrapezoidationGraph.h"
#include "slicer.h"
#include "utils/PolygonsSegmentIndex.h"
#include "utils/polygonUtils.h"
#include <spdlog/spdlog.h>

namespace cura
{
class MultiMaterialSegmentation
{
    using pos_t = double;
    using vd_t = boost::polygon::voronoi_diagram<pos_t>;
    using Segment = PolygonsSegmentIndex;

    class Line
    {
    public:
        std::vector<Point> points;
        std::vector<int> colors;
    };

    int scale = 1;

    std::vector<Polygons> m_colored_lines_polys_list;
    std::vector<Polygons> m_colored_top_faces_polys_list;
    std::vector<Polygons> m_colored_bottom_faces_polys_list;
    std::vector<Polygons> m_colored_faces_polys_list;

public:
    MultiMaterialSegmentation(size_t size)
    {
        m_colored_lines_polys_list.resize(size);
        m_colored_top_faces_polys_list.resize(size);
        m_colored_bottom_faces_polys_list.resize(size);
        m_colored_faces_polys_list.resize(size);
    };

    static void multiMaterialSegmentationByPainting(Slicer* slicer, Slicer* color_slicer);

private:
    void paintingSlicerLayers(Slicer* slicer, Slicer* color_slicer);

    Polygons paintingSlicerLayerColoredLines(SlicerLayer& slicer_layer);

    void paintingPolygonByColorLinePolygons(Polygons& polys, Polygons& color_line_polys, Polygons& color_polys, std::vector<Segment>& color_segments);

    Polygons toVoronoiColorPolygons(std::vector<Segment>& colored_segments);

    Line linePolygonsIntersection(Point& p1, Point& p2, Polygons& line_polys);

    Polygons paintingSlicerLayerColoredFaces(SlicerLayer& layer, const Mesh* p_mesh, coord_t min_z, coord_t max_z);

    Point3 getPoint3ByZ(Point3& p1, Point3& p2, int z);
};

} // namespace cura

#endif // MULTI_MATERIAL_SEGMENTATION_H
