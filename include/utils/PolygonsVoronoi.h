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
        PolygonsVoronoi* m_polygons_voronoi;
        int m_incident_edge_idx = -1;

    public:
        int idx = -1;

        Cell(PolygonsVoronoi* polygons_voronoi, const vd_t::cell_type* vd_cell) : m_polygons_voronoi(polygons_voronoi), voronoi_cell(vd_cell->source_index(), vd_cell->source_category())
        {
            this->color(vd_cell->color());
        };

        Edge* incident_edge()
        {
            return &m_polygons_voronoi->m_edges[m_incident_edge_idx];
        }

        Edge* incident_edge() const
        {
            return &m_polygons_voronoi->m_edges[m_incident_edge_idx];
        }

        void incident_edge_idx(int e_idx)
        {
            m_incident_edge_idx = e_idx;
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
        PolygonsVoronoi* m_polygons_voronoi;
        int m_cell_idx = -1;

        int m_twin_idx = -1;
        int m_next_idx = -1;
        int m_prev_idx = -1;

        int m_vertex_0_idx = -1;
        int m_vertex_1_idx = -1;
        bool source = false;

    public:
        int idx = -1;

        Edge(PolygonsVoronoi* polygons_voronoi, Cell* cell, vd_t::edge_type* vd_edge) :m_polygons_voronoi(polygons_voronoi), m_cell_idx(cell->idx), vd_t::edge_type(vd_edge->is_linear(), vd_edge->is_primary())
        {
            this->color(vd_edge->color());
        }

        Edge(PolygonsVoronoi* polygons_voronoi, int cell_idx, vd_t::edge_type* vd_edge, int v0_idx, int v1_idx, bool source) :m_polygons_voronoi(polygons_voronoi), m_cell_idx(cell_idx), m_vertex_0_idx(v0_idx), m_vertex_1_idx(v1_idx), vd_t::edge_type(vd_edge->is_linear(), vd_edge->is_primary())
        {
            this->color(vd_edge->color());
            this->source = source;
        }

        Cell* cell() { return &m_polygons_voronoi->m_cells[m_cell_idx]; }
        const Cell* cell() const { return &m_polygons_voronoi->m_cells[m_cell_idx]; }
        void cell(Cell* c) { m_cell_idx = c->idx; }

        Vertex* vertex0() { return &m_polygons_voronoi->m_vertices[m_vertex_0_idx]; }
        const Vertex* vertex0() const { return &m_polygons_voronoi->m_vertices[m_vertex_0_idx]; }
        void vertex0Idx(int v_idx) { m_vertex_0_idx = v_idx; }

        Vertex* vertex1() { return &m_polygons_voronoi->m_vertices[m_vertex_1_idx]; }
        const Vertex* vertex1() const { return &m_polygons_voronoi->m_vertices[m_vertex_1_idx]; }
        void vertex1Idx(int v_idx) { m_vertex_1_idx = v_idx; }

        Edge* twin() { return &m_polygons_voronoi->m_edges[m_twin_idx]; }
        const Edge* twin() const { return &m_polygons_voronoi->m_edges[m_twin_idx]; };
        void twinIdx(int idx) { m_twin_idx = idx; }

        Edge* next() { return &m_polygons_voronoi->m_edges[m_next_idx]; }
        const Edge* next() const { return &m_polygons_voronoi->m_edges[m_next_idx]; }
        void nextIdx(int e_idx) { m_next_idx = e_idx; }

        Edge* prev() { return &m_polygons_voronoi->m_edges[m_prev_idx]; }
        const Edge* prev() const { return &m_polygons_voronoi->m_edges[m_prev_idx]; }
        void prevIdx(int e_idx) { m_prev_idx = e_idx; }

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
        int idx = -1;

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

    struct EdgeIdxHash
    {
        size_t operator()(const std::pair<int, int> pair) const
        {
            size_t hash_x0 = std::hash<pos_t>()(pair.first);
            size_t hash_y0 = std::hash<pos_t>()(pair.second);

            hash_x0 ^= hash_y0 + 0x9e3779b9 + (hash_x0 << 6) + (hash_x0 >> 2);
            return hash_x0;
        }
    };

    std::vector<Cell> m_cells;
    std::vector<Edge> m_edges;
    std::vector<Vertex> m_vertices;

    std::unordered_map<std::pair<int, int>, int, EdgeIdxHash> m_edges_idx_map;
    std::unordered_map<Vertex, int, VertexHash> m_vertices_idx_map;

public:
    PolygonsVoronoi() {};

    const std::vector<Cell>& cells() const
    {
        return m_cells;
    }

    const std::vector<Edge>& edges() const
    {
        return m_edges;
    }

    const std::vector<Vertex>& vertices() const
    {
        return m_vertices;
    }

    void constructVoronoi(Segments& segments, const Polygons& polygons);

    void print();

private:
    void convertToPolygonsCell(vd_t::cell_type& vd_cell, Segments& segments, const Polygons& polygons);

    int getCell(PolygonsVoronoi* polygons_voronoi, const vd_t::cell_type* vd_cell)
    {
        Cell cell(polygons_voronoi, vd_cell);
        this->m_cells.push_back(cell);
        this->m_cells.back().idx = this->m_cells.size() - 1;
        return this->m_cells.back().idx;
    };

    int getEdge(PolygonsVoronoi* polygons_voronoi, int cell_idx, vd_t::vertex_type* v0, vd_t::vertex_type* v1, bool source = false)
    {
        vd_t::edge_type vd_edge(true, true);
        return getEdge(polygons_voronoi, cell_idx, &vd_edge, getVertex(v0, source), getVertex(v1, source), source);
    }

    int getEdge(PolygonsVoronoi* polygons_voronoi, int cell_idx, vd_t::edge_type* vd_edge, vd_t::vertex_type* v0, vd_t::vertex_type* v1, bool source = false)
    {
        return getEdge(polygons_voronoi, cell_idx, vd_edge, getVertex(v0, source), getVertex(v1, source), source);
    }

    int getEdge(PolygonsVoronoi* polygons_voronoi, int cell_idx, vd_t::edge_type* vd_edge, int v0_idx, int v1_idx, bool source = false)
    {
        if (m_edges_idx_map.find({v0_idx, v1_idx}) != m_edges_idx_map.end())
        {
            return m_edges_idx_map[{v0_idx, v1_idx}];
        }
        Edge edge(polygons_voronoi, cell_idx, vd_edge, v0_idx, v1_idx, source);
        this->m_edges.emplace_back(edge);
        int idx = this->m_edges.size() - 1;
        this->m_edges.back().idx = idx;

        m_edges_idx_map[{v0_idx, v1_idx}] = idx;

        if (!source && m_edges_idx_map.find({v1_idx, v0_idx}) != m_edges_idx_map.end())
        {
            int p_edge_twin_idx = m_edges_idx_map[{v1_idx, v0_idx}];
            this->m_edges[idx].twinIdx(p_edge_twin_idx);
            this->m_edges[p_edge_twin_idx].twinIdx(idx);
        }

        return idx;
    }

    int getVertex(vd_t::vertex_type* vd_vertex, bool source = false)
    {
        Vertex vertex(vd_vertex, source);
        return getVertex(&vertex);
    }

    int getVertex(Vertex* vertex)
    {
        if (m_vertices_idx_map.find(*vertex) != m_vertices_idx_map.end())
        {
            return m_vertices_idx_map[*vertex];
        }
        this->m_vertices.emplace_back(vertex->x(), vertex->y(), vertex->isSource());
        int idx = this->m_vertices.size() - 1;
        this->m_vertices.back().idx = idx;
        m_vertices_idx_map[*vertex] = idx;

        return idx;
    }
    bool searchStartAndEnd(vd_t::cell_type& vd_cell, vd_t::edge_type*& p_start_edge, vd_t::edge_type*& p_end_edge, vd_t::vertex_type* source_start_point, vd_t::vertex_type* source_end_point);

    bool vdVertexEqual(const vd_t::vertex_type* p_vertex1, const vd_t::vertex_type* p_vertex2);

    bool checkInsidePolygons(vd_t::edge_type* p_start_vd_edge, vd_t::edge_type* p_end_vd_edge, const Polygons& polygons);

    double computeArea(vd_t::vertex_type* p0, vd_t::vertex_type* p1, vd_t::vertex_type* p2);

    void removeSmallEdge();
};


} // namespace cura

#endif // CURAENGINE_INCLUDE_UTILS_POLYGONSVORONOI_H_
