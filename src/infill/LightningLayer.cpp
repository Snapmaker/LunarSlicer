//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "LightningLayer.h"

#include <iterator> // advance

#include "LightningTree.h"

#include "../sliceDataStorage.h"
#include "../utils/linearAlg2D.h"
#include "../utils/SVG.h"
#include "../utils/SparsePointGridInclusive.h"

using namespace cura;

coord_t LightningLayer::getWeightedDistance(const Point& boundary_loc, const Point& unsupported_loc)
{
    return vSize(boundary_loc - unsupported_loc);
}

LightningDistanceField::LightningDistanceField
(
    const coord_t& radius,
 const Polygons& current_outline,
 const Polygons& current_overhang,
 const std::vector<std::shared_ptr<LightningTreeNode>>& initial_trees
)
: cell_size(radius / 6) // TODO: make configurable!
, grid(cell_size)
, supporting_radius(radius)
, current_outline(current_outline)
, current_overhang(current_overhang)
{
    std::vector<Point> regular_dots = PolygonUtils::spreadDotsArea(current_overhang, cell_size);
    for (Point p : regular_dots)
    {
        const ClosestPolygonPoint cpp = PolygonUtils::findClosest(p, current_outline);
        const coord_t dist_to_boundary = vSize(p - cpp.p());
        unsupported_points.emplace_back(p, dist_to_boundary);
    }
    unsupported_points.sort(
        [](const UnsupCell& a, const UnsupCell& b)
        {
            coord_t da = a.dist_to_boundary + std::hash<Point>{}(a.loc) % 191;
            coord_t db = b.dist_to_boundary + std::hash<Point>{}(b.loc) % 191;
            return da < db;
        });
    for (auto it = unsupported_points.begin(); it != unsupported_points.end(); ++it)
    {
        UnsupCell& cell = *it;
        unsupported_points_grid.emplace(grid.toGridPoint(cell.loc), it);
    }
}

bool LightningDistanceField::tryGetNextPoint(Point* p) const
{
    if (unsupported_points.empty()) return false;
    *p = unsupported_points.front().loc;
    return true;
}

Point LightningDistanceField::getNearbyUnsupportedPoint(const Point p, const Point fall_back, coord_t supporting_radius, coord_t total_radius) const
{
    struct NearbyUnsup
    {
        GridPoint p;
        coord_t dist_to_leaf;
        NearbyUnsup(GridPoint p, coord_t dist_to_leaf)
        : p(p)
        , dist_to_leaf(dist_to_leaf)
        {}
    };
    std::vector<NearbyUnsup> nearby_unsupported_points;
    coord_t shortest_dist_to_leaf = supporting_radius * 4; // more than any of the points to be considered
    grid.processNearby(fall_back, supporting_radius,
        [supporting_radius, p, fall_back, &nearby_unsupported_points, &shortest_dist_to_leaf, this](const GridPoint& grid_loc)
        {
            auto cell_it = unsupported_points_grid.find(grid_loc);
            if (cell_it != unsupported_points_grid.end())
            {
                coord_t dist_to_leaf = vSize(cell_it->second->loc - p);
                if (dist_to_leaf < shortest_dist_to_leaf + cell_size * 2
                    && shorterThen(cell_it->second->loc - fall_back, supporting_radius)
                )
                {
                    shortest_dist_to_leaf = std::min(shortest_dist_to_leaf, dist_to_leaf);
                    nearby_unsupported_points.emplace_back(grid_loc, dist_to_leaf);
                }
            }
            return true;
        }
        );
    for (auto it = nearby_unsupported_points.begin(); it != nearby_unsupported_points.end(); )
    {
        NearbyUnsup& nearby_point = *it;
        if (nearby_point.dist_to_leaf >= shortest_dist_to_leaf + 2 * cell_size)
        {
            nearby_point = nearby_unsupported_points.back();
            nearby_unsupported_points.pop_back();
        }
        else
        {
            ++it;
        }
    }
    if (nearby_unsupported_points.empty())
    {
        std::cerr << "Couldn't find unsupported location close to leaf?!\n";
        nearby_unsupported_points.emplace_back(grid.toGridPoint(fall_back), 0); // actual dist_to_leaf doesn't matter here
    }
    GridPoint reference_grid_loc = nearby_unsupported_points[rand() % nearby_unsupported_points.size()].p;
    Point reference_point = unsupported_points_grid.find(reference_grid_loc)->second->loc;
    Point farther_away = p + normal(reference_point - p, total_radius);

    GridPoint farthest_unsupported_grid_loc = reference_grid_loc;
    const GridPoint grid_p = grid.toGridPoint(p);
    coord_t farthest_dist2 = vSize2(grid_p - farthest_unsupported_grid_loc);
    grid.processLineCells(std::make_pair(farther_away, reference_point), 
        [grid_p, fall_back, supporting_radius, &farthest_dist2, &farthest_unsupported_grid_loc, this](const GridPoint& grid_loc)
        {
            coord_t dist2 = vSize2(grid_p - grid_loc);
            if (dist2 < farthest_dist2)
            {
                auto it = unsupported_points_grid.find(grid_loc);
                if (it != unsupported_points_grid.end())
                {
                    if (shorterThen(it->second->loc - fall_back, supporting_radius))
                    {
                        farthest_dist2 = dist2;
                        farthest_unsupported_grid_loc = grid_loc;
                    }
                }
            }
            return true;
        }
    );
    return unsupported_points_grid.find(farthest_unsupported_grid_loc)->second->loc;
}

void LightningDistanceField::update(const Point& to_node, const Point& added_leaf)
{
    coord_t r = supporting_radius + cell_size / 2; // offset, because the boundary cells are not taken into account in spreadDotsArea
    const Point a = to_node;
    const Point b = added_leaf;
    Point ab = b - a;
    Point beyond = normal(ab, r);
    Point extent = turn90CCW(beyond);
    Point diag_f = normal(ab, r * .5 * std::sqrt(2.0));
    Point diag_p = normal(extent, r * .5 * std::sqrt(2.0));
    Polygons supported_areas;
    PolygonRef supported_area = supported_areas.newPoly();
    /* order in which the polygon is built:
     *    6_________5
     * 7.-'         '-.4
     *8/   a.......b   \3
     * \               /
     * 9'-._________.-'2
     *   10         1
     */
    supported_area.emplace_back(b + extent);
    supported_area.emplace_back(b + diag_f + diag_p);
    supported_area.emplace_back(b + beyond);
    supported_area.emplace_back(b + diag_f - diag_p);
    supported_area.emplace_back(b - extent);
    supported_area.emplace_back(a - extent);
    supported_area.emplace_back(a - diag_f - diag_p);
    supported_area.emplace_back(a - beyond);
    supported_area.emplace_back(a - diag_f + diag_p);
    supported_area.emplace_back(a + extent);

    std::vector<Point> supported_points = PolygonUtils::spreadDotsArea(supported_areas, cell_size);
    for (Point grid_loc : supported_points)
    {
        auto it = unsupported_points_grid.find(grid_loc);
        if (it != unsupported_points_grid.end())
        {
            std::list<UnsupCell>::iterator& list_it = it->second;
            UnsupCell& cell = *list_it;
            if (shorterThen(cell.loc - added_leaf, supporting_radius))
            {
                unsupported_points.erase(list_it);
                unsupported_points_grid.erase(it);
            }
        }
    }
}

// -- -- -- -- -- --
// -- -- -- -- -- --

Point GroundingLocation::p() const
{
    if (tree_node != nullptr)
    {
        return tree_node->getLocation();
    }
    else
    {
        assert(boundary_location);
        return boundary_location->p();
    }
}

void LightningLayer::fillLocator(SparsePointGridInclusive<std::weak_ptr<LightningTreeNode>>& tree_node_locator)
{
    std::function<void(std::shared_ptr<LightningTreeNode>)> add_node_to_locator_func =
        [&tree_node_locator](std::shared_ptr<LightningTreeNode> node)
        {
            tree_node_locator.insert(node->getLocation(), node);
        };
    for (auto& tree : tree_roots)
    {
        tree->visitNodes(add_node_to_locator_func);
    }
}

void LightningLayer::generateNewTrees(const Polygons& current_overhang, Polygons& current_outlines, coord_t supporting_radius)
{
    LightningDistanceField distance_field(supporting_radius, current_outlines, current_overhang, tree_roots);

    constexpr coord_t locator_cell_size = 2000;
    SparsePointGridInclusive<std::weak_ptr<LightningTreeNode>> tree_node_locator(locator_cell_size);
    fillLocator(tree_node_locator);

    constexpr size_t debug_max_iterations = 9999999; // TODO: remove
    size_t i_debug = 0;

    // Until no more points need to be added to support all:
    // Determine next point from tree/outline areas via distance-field
    Point unsupported_location;
    while (distance_field.tryGetNextPoint(&unsupported_location) && i_debug < debug_max_iterations)
    {
        ++i_debug;

        GroundingLocation grounding_loc = getBestGroundingLocation(unsupported_location, current_outlines, supporting_radius, tree_node_locator);

        if (grounding_loc.tree_node)
        {
            unsupported_location = distance_field.getNearbyUnsupportedPoint(grounding_loc.p(), unsupported_location, supporting_radius, supporting_radius * 2);
        }

        std::shared_ptr<LightningTreeNode> new_parent;
        std::shared_ptr<LightningTreeNode> new_child;
        attach(unsupported_location, grounding_loc, new_child, new_parent);
        tree_node_locator.insert(new_child->getLocation(), new_child);
        if (new_parent)
        {
            tree_node_locator.insert(new_parent->getLocation(), new_parent);
        }

        // update distance field
        distance_field.update(grounding_loc.p(), unsupported_location);
    }
}

GroundingLocation LightningLayer::getBestGroundingLocation(const Point& unsupported_location, const Polygons& current_outlines, const coord_t supporting_radius, const SparsePointGridInclusive<std::weak_ptr<LightningTreeNode>>& tree_node_locator, const std::shared_ptr<LightningTreeNode>& exclude_tree)
{
    ClosestPolygonPoint cpp = PolygonUtils::findClosest(unsupported_location, current_outlines);
    Point node_location = cpp.p();

    std::shared_ptr<LightningTreeNode> sub_tree(nullptr);
    coord_t current_dist = getWeightedDistance(node_location, unsupported_location);
    if (current_dist >= supporting_radius) // don't reconnect tree roots to other trees if they are already at/near the boundary
    { // TODO: make boundary size in which we ignore the valence rule configurable
        auto candidate_trees = tree_node_locator.getNearbyVals(unsupported_location, std::min(current_dist, supporting_radius));
        for (auto& candidate_wptr : candidate_trees)
        {
            auto candidate_sub_tree = candidate_wptr.lock();
            if (candidate_sub_tree && candidate_sub_tree != exclude_tree && !(exclude_tree && exclude_tree->hasOffspring(candidate_sub_tree)))
            {
                const coord_t candidate_dist = candidate_sub_tree->getWeightedDistance(unsupported_location, supporting_radius);
                if (candidate_dist < current_dist)
                {
                    current_dist = candidate_dist;
                    sub_tree = candidate_sub_tree;
                }
            }
        }
    }

    if (!sub_tree)
    {
        return GroundingLocation{ nullptr, cpp };
    }
    else
    {
        return GroundingLocation{ sub_tree, std::optional<ClosestPolygonPoint>() };
    }
}

bool LightningLayer::attach(const Point& unsupported_location, const GroundingLocation& grounding_loc, std::shared_ptr<LightningTreeNode>& new_child, std::shared_ptr<LightningTreeNode>& new_root)
{
    // Update trees & distance fields.
    if (grounding_loc.boundary_location)
    {
        new_root = LightningTreeNode::create(grounding_loc.p());
        new_child = new_root->addChild(unsupported_location);
        tree_roots.push_back(new_root);
        return true;
    }
    else
    {
        new_child = grounding_loc.tree_node->addChild(unsupported_location);
        return false;
    }
}

void LightningLayer::reconnectRoots(std::vector<std::shared_ptr<LightningTreeNode>>& to_be_reconnected_tree_roots, const Polygons& current_outlines, const coord_t supporting_radius)
{
    constexpr coord_t locator_cell_size = 2000;
    SparsePointGridInclusive<std::weak_ptr<LightningTreeNode>> tree_node_locator(locator_cell_size);
    fillLocator(tree_node_locator);

    for (auto root_ptr : to_be_reconnected_tree_roots)
    {
        auto old_root_it = std::find(tree_roots.begin(), tree_roots.end(), root_ptr);
        GroundingLocation ground = getBestGroundingLocation(root_ptr->getLocation(), current_outlines, supporting_radius, tree_node_locator, root_ptr);
        if (ground.boundary_location)
        {
            if (ground.boundary_location.value().p() == root_ptr->getLocation())
            {
                continue; // Already on the boundary.
            }

            auto new_root = LightningTreeNode::create(ground.p());
            new_root->addChild(root_ptr);
            tree_node_locator.insert(new_root->getLocation(), new_root);

            *old_root_it = std::move(new_root); // replace old root with new root
        }
        else
        {
            assert(ground.tree_node);
            assert(ground.tree_node != root_ptr);
            assert(!root_ptr->hasOffspring(ground.tree_node));
            assert(!ground.tree_node->hasOffspring(root_ptr));

            ground.tree_node->addChild(root_ptr);

            // remove old root
            *old_root_it = std::move(tree_roots.back());
            tree_roots.pop_back();
        }
    }
}

// Returns 'added someting'.
Polygons LightningLayer::convertToLines() const
{
    Polygons result_lines;
    if (tree_roots.empty())
    {
        return result_lines;
    }

    for (const auto& tree : tree_roots)
    {
        tree->convertToPolylines(result_lines);
    }

    // TODO: allow for polylines!
    Polygons split_lines;
    for (PolygonRef line : result_lines)
    {
        if (line.size() <= 1) continue;
        Point last = line[0];
        for (size_t point_idx = 1; point_idx < line.size(); point_idx++)
        {
            Point here = line[point_idx];
            split_lines.addLine(last, here);
            last = here;
        }
    }

    return split_lines;
}
