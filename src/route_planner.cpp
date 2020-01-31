#include <algorithm>

#include "route_planner.h"

RoutePlanner::RoutePlanner(RouteModel& model, float start_x, float start_y,
    float end_x, float end_y)
    : m_Model(model)
{
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    this->start_node = &model.FindClosestNode(start_x, start_y);
    this->end_node = &model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const* node)
{
    return node->distance(*(this->end_node));
}

void RoutePlanner::AddNeighbors(RouteModel::Node* current)
{
    current->FindNeighbors();
    for (auto* neighbor : current->neighbors) {
        if (neighbor->visited == false) {
            neighbor->parent = current;
            neighbor->h_value = CalculateHValue(neighbor);
            neighbor->g_value = current->g_value + current->distance(*neighbor);
            this->open_list.emplace_back(neighbor);
            neighbor->visited = true;
        }
    }
}

void RoutePlanner::PopulatePath(RouteModel::Node* current,
    std::vector<RouteModel::Node>* path)
{
    this->distance += current->distance(*(current->parent));
    RouteModel::Node* parent = current->parent;
    path->insert(path->begin(), *parent);
    if (this->isAtStart(parent))
        return;
    this->PopulatePath(parent, path);
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(
    RouteModel::Node* ending_node)
{
    std::vector<RouteModel::Node> path;
    path.insert(path.begin(), *ending_node);
    this->PopulatePath(ending_node, &path);
    this->distance *= m_Model.MetricScale();
    return path;
}

void RoutePlanner::sortOpenList()
{
    std::sort(this->open_list.begin(), this->open_list.end(), [](auto a, auto b) {
        return a->g_value + a->h_value > b->g_value + b->h_value;
    });
}

RouteModel::Node* RoutePlanner::NextNode()
{
    this->sortOpenList();
    RouteModel::Node* lowest_node = this->open_list.back();
    this->open_list.pop_back();
    return lowest_node;
}

void RoutePlanner::AStarSearch()
{
    this->start_node->visited = true;
    this->open_list.emplace_back(this->start_node);

    while (this->open_list.size() > 0) {
        RouteModel::Node* current = this->NextNode();
        if (this->isAtEnd(current))
            this->m_Model.path = ConstructFinalPath(this->end_node);
        this->AddNeighbors(current);
    }
}