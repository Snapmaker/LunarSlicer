#include "utils/PolygonsVoronoi.h"
#include "utils/VoronoiUtils.h"

namespace cura
{
void PolygonsVoronoi::constructVoronoi(Segments& segments, const Polygons& polygons)
{
    m_cells.clear();
    m_edges.clear();
    m_vertices.clear();

    vd_t vonoroi_diagram;
    boost::polygon::construct_voronoi(segments.begin(), segments.end(), &vonoroi_diagram);

    for (vd_t::cell_type cell : vonoroi_diagram.cells())
    {
        if (!cell.incident_edge()) {
            continue;
        }
        if (cell.contains_point() && cell.incident_edge() && cell.incident_edge()->is_infinite()) {
            continue;
        }

        convertToPolygonsCell(cell, segments, polygons);
    }

    removeSmallEdge();

    //    std::cout << std::endl;
    m_edges_idx_map.clear();
    m_vertices_idx_map.clear();
}

void PolygonsVoronoi::convertToPolygonsCell(vd_t::cell_type& vd_cell, Segments& segments,const Polygons& polygons)
{
    vd_t::edge_type* p_start_vd_edge = nullptr;
    vd_t::edge_type* p_end_vd_edge = nullptr;

    vd_t::vertex_type source_start_p(0, 0);
    vd_t::vertex_type source_end_p(0, 0);

    if (vd_cell.contains_point()) {
        Point point = VoronoiUtils::getSourcePoint(vd_cell, std::vector<Point>(), segments);
        source_start_p = vd_t::vertex_type(point.X, point.Y);
        source_end_p = vd_t::vertex_type(point.X, point.Y);
    } else {
        Segment segment = VoronoiUtils::getSourceSegment(vd_cell, std::vector<Point>(), segments);
        source_start_p = vd_t::vertex_type(segment.to().X, segment.to().Y);
        source_end_p =  vd_t::vertex_type(segment.from().X, segment.from().Y);
    }

    if (!searchStartAndEnd(vd_cell, p_start_vd_edge, p_end_vd_edge, &source_start_p, &source_end_p)) {
        return;
    }

    if(!checkInsidePolygons(p_start_vd_edge, p_end_vd_edge, polygons)) {
        return;
    }

    int cell_idx = getCell(this, &vd_cell);

    vd_t::vertex_type last_point(0, 0);

    int first_edge_idx = getEdge(this, cell_idx, &source_end_p, &source_start_p, true);
    m_cells[cell_idx].incident_edge_idx(first_edge_idx);

    int last_edge_idx = first_edge_idx;
    last_point = source_start_p;

    while (p_start_vd_edge != p_end_vd_edge) {
        int edge_idx = getEdge(this, cell_idx, p_start_vd_edge, &last_point, p_start_vd_edge->vertex1());
        m_edges[last_edge_idx].nextIdx(edge_idx);
        m_edges[edge_idx].prevIdx(last_edge_idx);

        last_edge_idx = edge_idx;
        last_point = vd_t::vertex_type(m_edges[edge_idx].vertex1()->x(), m_edges[edge_idx].vertex1()->y());
        p_start_vd_edge = p_start_vd_edge->next();
    }

    int edge_idx = getEdge(this, cell_idx, p_end_vd_edge, p_end_vd_edge->vertex0(), &source_end_p);
    m_edges[edge_idx].prevIdx(last_edge_idx);
    m_edges[last_edge_idx].nextIdx(edge_idx);

    m_edges[edge_idx].nextIdx(first_edge_idx);
    m_edges[first_edge_idx].prevIdx(edge_idx);

//    cell->print();
//    std::cout << ",";
}

bool PolygonsVoronoi::searchStartAndEnd(vd_t::cell_type& vd_cell, vd_t::edge_type*& p_start_vd_edge, vd_t::edge_type*& p_end_vd_edge, vd_t::vertex_type* source_start_p, vd_t::vertex_type* source_end_p)
{
    vd_t::edge_type* p_vd_edge = vd_cell.incident_edge();

    do
    {
        if (p_vd_edge->is_infinite()) {
            if (p_vd_edge->is_secondary()) {
                if (!p_vd_edge->vertex0() && ! vdVertexEqual(p_vd_edge->vertex1(), source_start_p)) {
                    p_start_vd_edge = p_vd_edge;
                }
                if (!p_vd_edge->vertex1() && ! vdVertexEqual(p_vd_edge->vertex0(), source_end_p)) {
                    p_end_vd_edge = p_vd_edge;
                }
            }
        } else {
            if (vdVertexEqual(p_vd_edge->vertex0(), source_start_p)) {
                p_start_vd_edge = p_vd_edge;
            } else if (vdVertexEqual(p_vd_edge->vertex1(), source_end_p)) {
                p_end_vd_edge = p_vd_edge;
            } else if (p_vd_edge->is_secondary() && !vdVertexEqual(p_vd_edge->vertex1(), source_start_p) && !vdVertexEqual(p_vd_edge->vertex0(), source_end_p)) {
                pos_t area1 = computeArea(source_start_p, p_vd_edge->vertex0(), p_vd_edge->vertex1());
                pos_t area2 = computeArea(source_end_p, p_vd_edge->vertex0(), p_vd_edge->vertex1());
                if (std::abs(area1) < std::abs(area2)) {
                    p_start_vd_edge = p_vd_edge;
                } else {
                    p_end_vd_edge = p_vd_edge;
                }
            }
        }
        p_vd_edge = p_vd_edge->next();
    } while (p_vd_edge != vd_cell.incident_edge());

    if (p_start_vd_edge != nullptr && p_end_vd_edge != nullptr) {
        return true;
    }

    return false;
}

bool PolygonsVoronoi::vdVertexEqual(const vd_t::vertex_type* p_vertex1, const vd_t::vertex_type* p_vertex2)
{
    return p_vertex1->x() == p_vertex2->x() && p_vertex1->y() == p_vertex2->y();
}

bool PolygonsVoronoi::checkInsidePolygons(vd_t::edge_type* p_start_vd_edge, vd_t::edge_type* p_end_vd_edge, const Polygons& polygons)
{
    if (polygons.inside(VoronoiUtils::p(p_start_vd_edge->vertex1()), true)
        && polygons.inside(VoronoiUtils::p(p_end_vd_edge->vertex0()), true)) {
        return true;
    }

    return false;
}

double PolygonsVoronoi::computeArea(vd_t::vertex_type* p0, vd_t::vertex_type* p1, vd_t::vertex_type* p2)
{
    return p0->x() * p1->y() - p0->y() * p1->x() + p1->x() * p2->y() - p1->y() * p2->x() + p2->x() * p0->y() - p2->y() * p0->x();
}

void PolygonsVoronoi::removeSmallEdge()
{
    std::unordered_map<int, std::set<int>> vertex_edges_map;

    std::unordered_map<Edge*, bool> small_edges;

    auto add_to_map = [&vertex_edges_map](int vertex_idx, int edge_idx){
        if (vertex_edges_map.find(vertex_idx) == vertex_edges_map.end()) {
            vertex_edges_map[vertex_idx] = std::set<int>();
        }
        vertex_edges_map[vertex_idx].insert(edge_idx);
    };

    for (Cell cell : m_cells)
    {
        Edge* begin = cell.incident_edge();
        for (auto *edge = begin->next(); edge != begin; edge = edge->next())
        {
            Point p0 = VoronoiUtils::p(edge->vertex0());
            Point p1 = VoronoiUtils::p(edge->vertex1());
            if (p0 == p1) {
                small_edges[edge] = false;
            }
            add_to_map(edge->vertex0()->idx, edge->idx);
            add_to_map(edge->vertex1()->idx, edge->idx);
        }
    }

    if (!small_edges.empty()) {
        for (auto& item : small_edges)
        {
            if (item.second) {
                continue ;
            }
            auto *small_edge = item.first;
            if (small_edge->isSource()) {
                continue ;
            }
            small_edges[small_edge] = true;
            assert(small_edges.find(small_edge->twin()) != small_edges.end());
            small_edges[small_edge->twin()] = true;

            bool is_v0 = small_edge->vertex1()->isSource();
            auto remove_vertex_idx = is_v0 ? small_edge->vertex0()->idx : small_edge->vertex1()->idx;
            auto replace_vertex_idx = is_v0 ? small_edge->vertex1()->idx : small_edge->vertex0()->idx;
            std::set<int> edges_ids = vertex_edges_map[remove_vertex_idx];

            for (auto edge_idx : edges_ids)
            {
                if (m_edges[edge_idx].vertex0()->idx == remove_vertex_idx) {
                    m_edges[edge_idx].vertex0Idx(replace_vertex_idx);
                    add_to_map(replace_vertex_idx, edge_idx);
                } else {
                    m_edges[edge_idx].vertex1Idx(replace_vertex_idx);
                    add_to_map(replace_vertex_idx, edge_idx);
                }
            }

            int prev_edge_idx = small_edge->prev()->idx;
            int next_edge_idx = small_edge->next()->idx;
            m_edges[prev_edge_idx].nextIdx(next_edge_idx);
            m_edges[next_edge_idx].prevIdx(prev_edge_idx);

            int twin_prev_edge_idx = small_edge->twin()->prev()->idx;
            int twin_next_edge_idx = small_edge->twin()->next()->idx;
            m_edges[twin_prev_edge_idx].nextIdx(twin_next_edge_idx);
            m_edges[twin_next_edge_idx].prevIdx(twin_prev_edge_idx);
        }
    }
}

void PolygonsVoronoi::print()
{
    std::cout << "[";
    for (const auto& cell : m_cells)
    {
        std::cout << "[";
        auto e = cell.incident_edge();
        do
        {
            std::cout << e->vertex0()->x() << "," << e->vertex0()->y();
            e = e->next();
            if (e != cell.incident_edge()) {
                std::cout << ",";
            }
        } while (e != cell.incident_edge());
        std::cout << "],";
    }
    std::cout << "[]]" << std::endl;
}

} // namespace cura