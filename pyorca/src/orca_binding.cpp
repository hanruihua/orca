#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include "RVOSimulator.h"
#include "Vector2.h"

namespace py = pybind11;

// Convert Vector2 to Python tuple
py::tuple vector2_to_tuple(const RVO::Vector2& v) {
    return py::make_tuple(v.x(), v.y());
}

// Convert Python tuple to Vector2
RVO::Vector2 tuple_to_vector2(const py::tuple& t) {
    if (t.size() != 2) {
        throw py::value_error("Tuple must have exactly 2 elements");
    }
    return RVO::Vector2(t[0].cast<float>(), t[1].cast<float>());
}

PYBIND11_MODULE(orca, m) {
    m.doc() = "ORCA Python bindings"; // Module documentation

    // Bind Vector2 class
    py::class_<RVO::Vector2>(m, "Vector2")
        .def(py::init<float, float>())
        .def("x", &RVO::Vector2::x)
        .def("y", &RVO::Vector2::y)
        .def("__repr__", [](const RVO::Vector2& v) {
            return "Vector2(" + std::to_string(v.x()) + ", " + std::to_string(v.y()) + ")";
        });

    // Bind RVOSimulator class
    py::class_<RVO::RVOSimulator>(m, "RVOSimulator")
        .def(py::init<>())
        .def(py::init<float, float, size_t, float, float, float, float>())
        .def(py::init<float, float, size_t, float, float, float, float, RVO::Vector2>())
        
        // Basic operations
        .def("do_step", &RVO::RVOSimulator::doStep)
        .def("process_obstacles", &RVO::RVOSimulator::processObstacles)
        .def("get_global_time", &RVO::RVOSimulator::getGlobalTime)
        .def("get_time_step", &RVO::RVOSimulator::getTimeStep)
        .def("set_time_step", &RVO::RVOSimulator::setTimeStep)
        
        // Agent-related operations
        .def("add_agent", [](RVO::RVOSimulator& sim, const py::tuple& pos) {
            return sim.addAgent(tuple_to_vector2(pos));
        })
        .def("add_agent", [](RVO::RVOSimulator& sim, const py::tuple& pos, 
            float neighbor_dist, size_t max_neighbors, float time_horizon,
            float time_horizon_obst, float radius, float max_speed) {
            return sim.addAgent(tuple_to_vector2(pos), neighbor_dist, max_neighbors,
                              time_horizon, time_horizon_obst, radius, max_speed);
        })
        .def("add_agent", [](RVO::RVOSimulator& sim, const py::tuple& pos,
            float neighbor_dist, size_t max_neighbors, float time_horizon,
            float time_horizon_obst, float radius, float max_speed,
            const py::tuple& velocity) {
            return sim.addAgent(tuple_to_vector2(pos), neighbor_dist, max_neighbors,
                              time_horizon, time_horizon_obst, radius, max_speed,
                              tuple_to_vector2(velocity));
        })
        
        // Get agent properties
        .def("get_agent_position", [](const RVO::RVOSimulator& sim, size_t agent_no) {
            return vector2_to_tuple(sim.getAgentPosition(agent_no));
        })
        .def("get_agent_velocity", [](const RVO::RVOSimulator& sim, size_t agent_no) {
            return vector2_to_tuple(sim.getAgentVelocity(agent_no));
        })
        .def("get_agent_pref_velocity", [](const RVO::RVOSimulator& sim, size_t agent_no) {
            return vector2_to_tuple(sim.getAgentPrefVelocity(agent_no));
        })
        .def("get_agent_radius", &RVO::RVOSimulator::getAgentRadius)
        .def("get_agent_max_speed", &RVO::RVOSimulator::getAgentMaxSpeed)
        .def("get_agent_neighbor_dist", &RVO::RVOSimulator::getAgentNeighborDist)
        .def("get_agent_time_horizon", &RVO::RVOSimulator::getAgentTimeHorizon)
        .def("get_agent_time_horizon_obst", &RVO::RVOSimulator::getAgentTimeHorizonObst)
        .def("get_agent_max_neighbors", &RVO::RVOSimulator::getAgentMaxNeighbors)
        
        // Set agent properties
        .def("set_agent_position", [](RVO::RVOSimulator& sim, size_t agent_no, const py::tuple& pos) {
            sim.setAgentPosition(agent_no, tuple_to_vector2(pos));
        })
        .def("set_agent_velocity", [](RVO::RVOSimulator& sim, size_t agent_no, const py::tuple& vel) {
            sim.setAgentVelocity(agent_no, tuple_to_vector2(vel));
        })
        .def("set_agent_pref_velocity", [](RVO::RVOSimulator& sim, size_t agent_no, const py::tuple& vel) {
            sim.setAgentPrefVelocity(agent_no, tuple_to_vector2(vel));
        })
        .def("set_agent_radius", &RVO::RVOSimulator::setAgentRadius)
        .def("set_agent_max_speed", &RVO::RVOSimulator::setAgentMaxSpeed)
        .def("set_agent_neighbor_dist", &RVO::RVOSimulator::setAgentNeighborDist)
        .def("set_agent_time_horizon", &RVO::RVOSimulator::setAgentTimeHorizon)
        .def("set_agent_time_horizon_obst", &RVO::RVOSimulator::setAgentTimeHorizonObst)
        .def("set_agent_max_neighbors", &RVO::RVOSimulator::setAgentMaxNeighbors)
        
        // Obstacle-related operations
        .def("add_obstacle", [](RVO::RVOSimulator& sim, const std::vector<py::tuple>& vertices) {
            std::vector<RVO::Vector2> vec_vertices;
            for (const auto& v : vertices) {
                vec_vertices.push_back(tuple_to_vector2(v));
            }
            return sim.addObstacle(vec_vertices);
        })
        .def("get_obstacle_vertex", [](const RVO::RVOSimulator& sim, size_t vertex_no) {
            return vector2_to_tuple(sim.getObstacleVertex(vertex_no));
        })
        
        // Other functionality
        .def("get_num_agents", &RVO::RVOSimulator::getNumAgents)
        .def("get_num_obstacle_vertices", &RVO::RVOSimulator::getNumObstacleVertices)
        .def("query_visibility", [](const RVO::RVOSimulator& sim, const py::tuple& point1, const py::tuple& point2) {
            return sim.queryVisibility(tuple_to_vector2(point1), tuple_to_vector2(point2));
        })
        .def("query_visibility", [](const RVO::RVOSimulator& sim, const py::tuple& point1, 
            const py::tuple& point2, float radius) {
            return sim.queryVisibility(tuple_to_vector2(point1), tuple_to_vector2(point2), radius);
        });
} 