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
    m_edges_map.clear();
    m_vertices_map.clear();
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

    if (!checkIsInfinite(p_start_vd_edge, p_end_vd_edge))
    {
        return;
    }

    if(!checkInsidePolygons(p_start_vd_edge, p_end_vd_edge, polygons)) {
        return;
    }
    
    Cell* cell = getCell(&vd_cell);

    vd_t::vertex_type* last_point;

//    if (vd_cell.contains_point()) {
//        cell->incident_edge(getEdge(cell, p_start_vd_edge, &source_start_p, p_start_vd_edge->vertex1(), false));
//        last_point = p_start_vd_edge->vertex1();
//        p_start_vd_edge = p_start_vd_edge->next();
//    } else {
    cell->incident_edge(getEdge(cell, &source_end_p, &source_start_p, true));
    last_point = &source_start_p;
//    }

    Edge* last_edge = cell->incident_edge();

    while (p_start_vd_edge != p_end_vd_edge) {
        Edge* edge = getEdge(cell, p_start_vd_edge, last_point, p_start_vd_edge->vertex1());
        last_edge->next(edge);
        edge->prev(last_edge);

        last_edge = edge;
        last_point = last_edge->vertex1();
        p_start_vd_edge = p_start_vd_edge->next();
    }

    Edge* edge = getEdge(cell, p_end_vd_edge, p_end_vd_edge->vertex0(), &source_end_p);
    edge->prev(last_edge);
    last_edge->next(edge);
    edge->next(cell->incident_edge());
    cell->incident_edge()->prev(edge);

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
                if (!p_vd_edge->vertex0() && p_vd_edge->vertex1() && ! vdVertexEqual(p_vd_edge->vertex1(), source_start_p)) {
                    p_start_vd_edge = p_vd_edge;
                }
                if (!p_vd_edge->vertex1() && p_vd_edge->vertex0() && ! vdVertexEqual(p_vd_edge->vertex0(), source_end_p)) {
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
    return esEqual(p_vertex1->x(), p_vertex2->x()) && esEqual(p_vertex1->y(), p_vertex2->y());
}

bool PolygonsVoronoi::checkInsidePolygons(vd_t::edge_type* p_start_vd_edge, vd_t::edge_type* p_end_vd_edge, const Polygons& polygons)
{
    if (polygons.inside(VoronoiUtils::p(p_start_vd_edge->vertex1()), true)
        && polygons.inside(VoronoiUtils::p(p_end_vd_edge->vertex0()), true)) {
        return true;
    }

    return false;
}

bool PolygonsVoronoi::checkIsInfinite(vd_t::edge_type* p_start_vd_edge, vd_t::edge_type* p_end_vd_edge)
{
    if (!p_start_vd_edge->vertex1() || !p_end_vd_edge->vertex0())
    {
        return false;
    }
    
    vd_t::edge_type* e = p_start_vd_edge->next();
    while (e != p_end_vd_edge)
    {
        if (e->is_infinite())
        {
            return false;
        }
        e = e->next();
    }
    return true;
}

double PolygonsVoronoi::computeArea(vd_t::vertex_type* p0, vd_t::vertex_type* p1, vd_t::vertex_type* p2)
{
    return p0->x() * p1->y() - p0->y() * p1->x() + p1->x() * p2->y() - p1->y() * p2->x() + p2->x() * p0->y() - p2->y() * p0->x();
}

void PolygonsVoronoi::removeSmallEdge()
{
    std::unordered_map<Vertex*, std::set<Edge*>> vertex_edges_map;

    std::unordered_map<Edge*, bool> small_edges;

    auto add_to_map = [&vertex_edges_map](Vertex* vertex, Edge* edge){
        if (vertex_edges_map.find(vertex) == vertex_edges_map.end()) {
            vertex_edges_map[vertex] = std::set<Edge*>();
        }
        vertex_edges_map[vertex].insert(edge);
    };

    for (Cell cell : m_cells)
    {
        Edge* begin = cell.incident_edge();
        for (auto *edge = begin->next(); edge != begin; edge = edge->next())
        {
            if (computeLength2(edge->vertex0(), edge->vertex1()) < 0.000001) {
                small_edges[edge] = false;
                small_edges[edge->twin()] = false;
            }
            add_to_map(edge->vertex0(), edge);
            add_to_map(edge->vertex1(), edge);
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
            auto *remove_vertex = is_v0 ? small_edge->vertex0() : small_edge->vertex1();
            auto *replace_vertex = is_v0 ? small_edge->vertex1() : small_edge->vertex0();
            std::set<Edge*> edges = vertex_edges_map[remove_vertex];

            for (auto *edge : edges)
            {
                if (edge->vertex0() == remove_vertex) {
                    edge->vertex0(replace_vertex);
                } else {
                    edge->vertex1(replace_vertex);
                }
                add_to_map(replace_vertex, edge);
            }

            auto *prev_edge = small_edge->prev();
            auto *next_edge = small_edge->next();
            prev_edge->next(next_edge);
            next_edge->prev(prev_edge);
            auto *twin_prev_edge = small_edge->twin()->prev();
            auto *twin_next_edge = small_edge->twin()->next();
            twin_prev_edge->next(twin_next_edge);
            twin_next_edge->prev(twin_prev_edge);
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

double PolygonsVoronoi::computeLength2(boost::polygon::voronoi_vertex<double>* p0, boost::polygon::voronoi_vertex<double>* p1)
{
    return (p0->x() - p1->x()) * (p0->x() - p1->x()) + (p0->y() - p1->y()) * (p0->y() - p1->y());
}

} // namespace cura