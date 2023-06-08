// Copyright (c) 2020 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_POLYGONS_SEGMENT_INDEX_H
#define UTILS_POLYGONS_SEGMENT_INDEX_H

#include <vector>

#include <boost/polygon/polygon.hpp>
#include <boost/polygon/voronoi.hpp>

#include "PolygonsPointIndex.h"

namespace cura
{

/*!
 * A class for iterating over the points in one of the polygons in a \ref Polygons object
 */
class PolygonsSegmentIndex : public PolygonsPointIndex
{
public:
    int color;

    PolygonsSegmentIndex();

    PolygonsSegmentIndex(const Polygons* polygons, unsigned int poly_idx, unsigned int point_idx);

    PolygonsSegmentIndex(const Polygons* polygons, unsigned int poly_idx, unsigned int point_idx, int color);

    Point from() const;

    Point to() const;
};

using Segment = PolygonsSegmentIndex;
using Segments = std::vector<Segment>;

static INLINE void polygonsToSegments(const Polygons& polygons, Segments& segments) {
    for (int i = 0; i < polygons.size(); ++i)
    {
        for (int j = 0; j < polygons[i].size(); ++j)
        {
            segments.emplace_back(&polygons, i, j);
        }
    }
}

} // namespace cura

namespace boost
{
namespace polygon
{


template<>
struct geometry_concept<cura::Point>
{
    typedef point_concept type;
};

template<>
struct point_traits<cura::Point>
{
    typedef cura::coord_t coordinate_type;

    static inline coordinate_type get(const cura::Point& point, orientation_2d orient)
    {
        return (orient == HORIZONTAL) ? point.X : point.Y;
    }
};

template<>
struct geometry_concept<cura::Segment>
{
    typedef segment_concept type;
};

template<>
struct segment_traits<cura::Segment>
{
    typedef cura::coord_t coordinate_type;
    typedef cura::Point point_type;
    static inline point_type get(const cura::Segment& CSegment, direction_1d dir)
    {
        return dir.to_int() ? CSegment.p() : CSegment.next().p();
    }
};


} // namespace polygon
} // namespace boost


#endif//UTILS_POLYGONS_SEGMENT_INDEX_H
