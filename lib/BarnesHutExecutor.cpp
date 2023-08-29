//
// Created by gomkyung2 on 2023/08/26.
//

#include "BarnesHutExecutor.hpp"

#include <numeric>
#include <glm/common.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/gtx/norm.hpp>

NBodyExecutor::Cube NBodyExecutor::BarnesHutExecutor::getBodyBound(std::span<Body> bodies) {
    constexpr glm::vec3 max { std::numeric_limits<float>::max() };
    constexpr glm::vec3 min { std::numeric_limits<float>::lowest() };

    // Get bound of body positions.
    auto positions = bodies | std::views::transform(&Body::position);
    const glm::vec3 min_bound = std::reduce(
        positions.begin(), positions.end(),
        max,
        [](const glm::vec3 &lhs, const glm::vec3 &rhs) noexcept { return glm::min(lhs, rhs); }
    );
    const glm::vec3 max_bound = std::reduce(
        positions.begin(), positions.end(),
        min,
        [](const glm::vec3 &lhs, const glm::vec3 &rhs) noexcept { return glm::max(lhs, rhs); }
    );

    const float cube_size = glm::compMax(max_bound - min_bound);
    const glm::vec3 cube_center = glm::mix(min_bound, max_bound, 0.5f);
    return { cube_center - glm::vec3(cube_size / 2.f), cube_size };
}

void NBodyExecutor::BarnesHutExecutor::applyGravityField(Body &body, const OctTree::Node *node) const{
    // Traverse OctTree nodes in preorder manner.

    const glm::vec3 displacement = node->data.center_of_mass - body.position;
    const float distance = glm::max(glm::length(displacement), min_distance);

    const float criteria = node->data.bound.size / distance;
    if (criteria < threshold) { // Body is sufficiently far from the node.
        // In Barnes-Hut algorithm, the force affects to the body can be approximated as the force from the virtual body,
        // whose mass is total mass of the node and located the center of mass of the current node.
        body.acceleration += gravity_constant * node->data.total_mass * displacement / glm::pow(distance, 3.f); // G*m2 / r^2
    }
    else if (node->isLeaf()){
        // It is unnecessary to consider self-interacting, because displacement from self to self is zero, so it will
        // append the zero acceleration.
        body.acceleration += std::transform_reduce(
            node->bodies.cbegin(), node->bodies.cend(), glm::zero<glm::vec3>(), std::plus<>{},
            [&](const Body *other_body){
                const glm::vec3 displacement = other_body->position - body.position;
                const float distance = glm::max(glm::length(displacement), min_distance);
                return gravity_constant * other_body->mass * displacement / glm::pow(distance, 3.f); // G*m2 / r^2
            }
        );
    }
    else{
        for (const OctTree::Node *child_node : node->nonEmptyChildNodes()){
            applyGravityField(body, child_node);
        }
    }
}

NBodyExecutor::BarnesHutExecutor::BarnesHutExecutor(std::unique_ptr<BS::thread_pool> thread_pool) : Executor { std::move(thread_pool) } {

}

void NBodyExecutor::BarnesHutExecutor::execute(std::span<Body> bodies, float time_delta) {
    // OctTree accepts the const pointer of bodies.
    auto body_addresses = bodies | std::views::transform([](Body &body) { return &body; });
    const OctTree tree { getBodyBound(bodies), body_addresses };

    const auto partial_task = [&](std::size_t start_idx, std::size_t end_idx){
        for (std::size_t idx = start_idx; idx < end_idx; ++idx){
            applyGravityField(bodies[idx], tree.getRoot());
        }
    };
    if (thread_pool){
        thread_pool->push_loop(bodies.size(), partial_task);
        thread_pool->wait_for_tasks();
    }
    else{
        partial_task(0, bodies.size());
    }

    for (Body &body : bodies){
        body.velocity += body.acceleration * time_delta;
        body.position += body.velocity * time_delta;
    }
}