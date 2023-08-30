//
// Created by gomkyung2 on 2023/08/26.
//

#include "NBodyExecutor/OctTree.hpp"

#include <numeric>
#include "glm/gtc/constants.hpp"
#include "glm/vector_relational.hpp"

std::size_t NBodyExecutor::OctTree::Node::getChildNodeIndex(const glm::vec3 &position) const NBODY_EXECUTOR_NOEXCEPT_IF_RELEASE {
    // Check if position is inside the data.bound.
    // TODO: this assertion may not pass due to floating point error.
//    assert(glm::all(glm::greaterThanEqual(position, data.bound.position)));
//    assert(glm::all(glm::lessThanEqual(position, data.bound.position + glm::vec3(data.bound.size))));

    const glm::vec3 mid = data.bound.position + 0.5f * data.bound.size;
    return 4 * (position.x >= mid.x) + 2 * (position.y >= mid.y) + (position.z >= mid.z);
}

void NBodyExecutor::OctTree::Node::addBodyToNode(const Body *body) NBODY_EXECUTOR_NOEXCEPT_IF_RELEASE {
    assert(isLeaf());
    assert(bodies.size() < max_bodies_in_node);

    bodies.push_back(body);
    ++data.num_bodies;
    data.total_mass += body->mass;
}

NBodyExecutor::OctTree::Node::Node(const Cube &bound) NBODY_EXECUTOR_NOEXCEPT_IF_RELEASE : data { .bound = bound } {
    assert(bound.size > 0.f);
}

bool NBodyExecutor::OctTree::Node::isLeaf() const noexcept{
    return std::get<0>(child_nodes) == nullptr;
}

bool NBodyExecutor::OctTree::Node::empty() const noexcept{
    return data.num_bodies == 0;
}

void NBodyExecutor::OctTree::Node::updateCenterOfMass() NBODY_EXECUTOR_NOEXCEPT_IF_RELEASE{
    if (isLeaf()){ // node is leaf.
        assert(data.total_mass != 0.f);
        data.center_of_mass = std::transform_reduce(bodies.cbegin(), bodies.cend(), glm::zero<glm::vec3>(), std::plus<>{}, [](const Body *body){
            return body->mass * body->position;
        }) / data.total_mass;
    }
    else{
        auto nonempty_nodes = nonEmptyChildNodes();
        data.center_of_mass = std::transform_reduce(nonempty_nodes.begin(), nonempty_nodes.end(), glm::zero<glm::vec3>(), std::plus<>{}, [](auto *child_node){
            child_node->updateCenterOfMass();
            return child_node->data.total_mass * child_node->data.center_of_mass;
        }) / data.total_mass;
    }
}

const NBodyExecutor::OctTree::Node *NBodyExecutor::OctTree::getRoot() const noexcept{
    return root;
}

std::vector<NBodyExecutor::Cube> NBodyExecutor::OctTree::getNonEmptyBoxes() const{
    std::vector<Cube> result;
    fetchNonEmptyBoxes(root, result);
    return result;
}

NBodyExecutor::OctTree::Node *NBodyExecutor::OctTree::getNodeFromPool(const Cube &bound){
    return &node_pool.emplace_back(bound);
}

void NBodyExecutor::OctTree::fetchNonEmptyBoxes(const Node *node, std::vector<Cube> &boxes) const{
    if (node->isLeaf()){
        boxes.push_back(node->data.bound);
    }
    else{
        for (const NBodyExecutor::OctTree::Node *child_node : node->nonEmptyChildNodes()){
            fetchNonEmptyBoxes(child_node, boxes);
        }
    }
}