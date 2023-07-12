#ifndef CURAENGINE_INCLUDE_UTILS_POLYGONSVORONOI_H_
#define CURAENGINE_INCLUDE_UTILS_POLYGONSVORONOI_H_


#include "PolygonsSegmentIndex.h"
#include "polygon.h"
#include <boost/polygon/voronoi_diagram.hpp>

namespace cura
{

const int vertex_meld_distance = MM2INT(0.03);

class PolygonsVoronoi
{
    using pos_t = double;
    using vd_t = boost::polygon::voronoi_diagram<pos_t>;

public:
    class Cell;
    class Edge;
    class Vertex;

    class Cell : public vd_t::cell_type
    {
    private:
        Edge* m_incident_edge = nullptr;

    public:
        Cell(const vd_t::cell_type* vd_cell) : voronoi_cell(vd_cell->source_index(), vd_cell->source_category())
        {
            this->color(vd_cell->color());
        };

        Edge* incident_edge()
        {
            return m_incident_edge;
        }

        Edge* incident_edge() const
        {
            return m_incident_edge;
        }

        void incident_edge(Edge* e)
        {
            m_incident_edge = e;
        }

        void print()
        {
            Edge* edge = incident_edge();
            std::cout << "[";
            do
            {
                std::cout << edge->vertex0()->x() << "," << edge->vertex0()->y();
                edge = edge->next();
                if (edge != incident_edge()) {
                    std::cout << ",";
                }
            } while (edge != incident_edge());
            std::cout << "]" << std::endl;
        }
    };

    class Edge : public vd_t::edge_type
    {
    private:
        Cell* m_cell = nullptr;
        Vertex* m_vertex_0 = nullptr;
        Vertex* m_vertex_1 = nullptr;
        Edge* m_twin = nullptr;
        Edge* m_next = nullptr;
        Edge* m_prev = nullptr;
        bool source = false;

    public:
        Edge(Cell* cell, vd_t::edge_type* vd_edge) : m_cell(cell), vd_t::edge_type(vd_edge->is_linear(), vd_edge->is_primary())
        {
            this->color(vd_edge->color());
        }

        Edge(Cell* cell, vd_t::edge_type* vd_edge, Vertex* v0, Vertex* v1, bool source) :m_cell(cell), m_vertex_0(v0), m_vertex_1(v1), vd_t::edge_type(vd_edge->is_linear(), vd_edge->is_primary())
        {
            this->color(vd_edge->color());
            this->source = source;
        }

        Cell* cell() { return m_cell; }
        const Cell* cell() const { return m_cell; }
        void cell(Cell* c) { m_cell = c; }

        Vertex* vertex0() { return m_vertex_0; }
        const Vertex* vertex0() const { return m_vertex_0; }
        void vertex0(Vertex* v) { m_vertex_0 = v; }

        Vertex* vertex1() { return m_vertex_1; }
        const Vertex* vertex1() const { return m_vertex_1; }
        void vertex1(Vertex* v) { m_vertex_1 = v; }

        Edge* twin() { return m_twin; }
        const Edge* twin() const { return m_twin; };
        void twin(Edge* e) { m_twin = e; }

        Edge* next() { return m_next; }
        const Edge* next() const { return m_next; }
        void next(Edge* e) { m_next = e; }

        Edge* prev() { return m_prev; }
        const Edge* prev() const { return m_prev; }
        void prev(Edge* e) { m_prev = e; }

        bool is_finite() const { return vertex0() && vertex1(); }
        bool is_infinite() const { return !vertex0() || !vertex1(); }

        bool isSource() { return source; }
        bool isSource() const { return source; }
    };

    class Vertex : public vd_t::vertex_type
    {
    private:
        bool source = false;

    public:
        Vertex(pos_t x, pos_t y, bool source) : voronoi_vertex(x, y), source(source) {};

        explicit Vertex(vd_t::vertex_type* vd_vertex, bool source) : voronoi_vertex(vd_vertex->x(), vd_vertex->y()), source(source)
        {
            this->color(vd_vertex->color());
        };

        bool operator==(const Vertex& vertex) const
        {
            return this->x() == vertex.x() && this->y() == vertex.y();
        }

        bool isSource() { return source; }
        bool isSource() const { return source; }
    };

    struct VertexHash
    {
        size_t operator()(const Vertex& vertex) const
        {
            size_t hash_x = std::hash<pos_t>()(vertex.x());
            size_t hash_y = std::hash<pos_t>()(vertex.y());

            // from boost
            hash_x ^= hash_y + 0x9e3779b9 + (hash_x << 6) + (hash_x >> 2);

            return hash_x;
        }
    };

    struct EdgeHash
    {
        size_t operator()(const std::pair<Vertex, Vertex> pair) const
        {
            size_t hash_x0 = std::hash<pos_t>()(pair.first.x());
            size_t hash_y0 = std::hash<pos_t>()(pair.first.y());
            size_t hash_x1 = std::hash<pos_t>()(pair.second.x());
            size_t hash_y1 = std::hash<pos_t>()(pair.second.y());

            hash_x0 ^= hash_y0 + 0x9e3779b9 + (hash_x0 << 6) + (hash_x0 >> 2);
            hash_x1 ^= hash_y1 + 0x9e3779b9 + (hash_x1 << 6) + (hash_x1 >> 2);
            hash_x0 ^= hash_x1 + 0x9e3779b9 + (hash_x0 << 6) + (hash_x0 >> 2);
            return hash_x0;
        }
    };

    std::list<Cell> m_cells;
    std::list<Edge> m_edges;
    std::list<Vertex> m_vertices;

    std::unordered_map<std::pair<Vertex, Vertex>, Edge*, EdgeHash> m_edges_map;
    std::unordered_map<Vertex, Vertex*, VertexHash> m_vertices_map;

public:
    PolygonsVoronoi() {};

    const std::list<Cell>& cells() const
    {
        return m_cells;
    }

    const std::list<Edge>& edges() const
    {
        return m_edges;
    }

    const std::list<Vertex>& vertices() const
    {
        return m_vertices;
    }

    void constructVoronoi(Segments& segments, const Polygons& polygons);

    void print();

private:
    void convertToPolygonsCell(vd_t::cell_type& vd_cell, Segments& segments, const Polygons& polygons);

    Cell* getCell(const vd_t::cell_type* vd_cell)
    {
        Cell cell(vd_cell);
        this->m_cells.push_back(cell);
        return &this->m_cells.back();
    };

    Edge* getEdge(Cell* cell, vd_t::vertex_type* v0, vd_t::vertex_type* v1, bool source = false)
    {
        vd_t::edge_type vd_edge(true, true);
        return getEdge(cell, &vd_edge, getVertex(v0, source), getVertex(v1, source), source);
    }

    Edge* getEdge(Cell* cell, vd_t::edge_type* vd_edge, vd_t::vertex_type* v0, vd_t::vertex_type* v1, bool source = false)
    {
        return getEdge(cell, vd_edge, getVertex(v0, source), getVertex(v1, source), source);
    }

    Edge* getEdge(Cell* cell, vd_t::edge_type* vd_edge, Vertex* v0, Vertex* v1, bool source = false)
    {
        if (m_edges_map.find({*v0, *v1}) != m_edges_map.end())
        {
            return m_edges_map[{ *v0, *v1 }];
        }
        Edge edge(cell, vd_edge, v0, v1, source);
        this->m_edges.emplace_back(edge);
        Edge* p_edge = &this->m_edges.back();

        m_edges_map[{*v0, *v1}] = p_edge;

        if (!source && m_edges_map.find({*v1, *v0}) != m_edges_map.end())
        {
            Edge* p_edge_twin = m_edges_map[{*v1, *v0}];
            p_edge->twin(p_edge_twin);
            p_edge_twin->twin(p_edge);
        }

        return p_edge;
    }

    Vertex* getVertex(pos_t x, pos_t y, bool source)
    {
        Vertex vertex(x, y, source);
        return getVertex(&vertex);
    }

    Vertex* getVertex(vd_t::vertex_type* vd_vertex, bool source = false)
    {
        Vertex vertex(vd_vertex, source);
        return getVertex(&vertex);
    }

    Vertex* getVertex(Vertex* vertex)
    {
        if (m_vertices_map.find(*vertex) != m_vertices_map.end())
        {
            return m_vertices_map[*vertex];
        }
        this->m_vertices.emplace_back(vertex->x(), vertex->y(), vertex->isSource());
        m_vertices_map[*vertex] = &this->m_vertices.back();
        return &this->m_vertices.back();
    }
    bool searchStartAndEnd(vd_t::cell_type& vd_cell, vd_t::edge_type*& p_start_edge, vd_t::edge_type*& p_end_edge, vd_t::vertex_type* source_start_point, vd_t::vertex_type* source_end_point);

    bool vdVertexEqual(const vd_t::vertex_type* p_vertex1, const vd_t::vertex_type* p_vertex2);

    bool checkInsidePolygons(vd_t::edge_type* p_start_vd_edge, vd_t::edge_type* p_end_vd_edge, const Polygons& polygons);

    double computeArea(vd_t::vertex_type* p0, vd_t::vertex_type* p1, vd_t::vertex_type* p2);
    void removeSmallEdge();
};


} // namespace cura

#endif // CURAENGINE_INCLUDE_UTILS_POLYGONSVORONOI_H_
