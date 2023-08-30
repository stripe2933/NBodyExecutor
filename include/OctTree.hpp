//
// Created by gomkyung2 on 2023/08/26.
//

#pragma once

#include <deque>
#include "Body.hpp"
#include "Cube.hpp"

namespace NBodyExecutor{
    class OctTree{
    public:
        class Node{
        private:
            [[nodiscard]] std::size_t getChildNodeIndex(const glm::vec3 &position) const NBODY_EXECUTOR_NOEXCEPT_IF_RELEASE;
            void addBodyToNode(const Body *body) NBODY_EXECUTOR_NOEXCEPT_IF_RELEASE;

        public:
            struct {
                Cube bound;
                std::size_t num_bodies = 0;
                float total_mass = 0.f;
                glm::vec3 center_of_mass { 0.f };
            } data;
            std::vector<const Body*> bodies;
            std::array<Node*, 8> child_nodes { nullptr };

            explicit Node(const Cube &bound) NBODY_EXECUTOR_NOEXCEPT_IF_RELEASE;

            [[nodiscard]] bool isLeaf() const noexcept;
            [[nodiscard]] bool empty() const noexcept;
            [[nodiscard]] inline auto nonEmptyChildNodes() const noexcept;
            void addBody(const Body *body, auto &&node_gen);
            void updateCenterOfMass() NBODY_EXECUTOR_NOEXCEPT_IF_RELEASE;
        };

        static constexpr std::size_t max_bodies_in_node = 32;

        OctTree(const Cube &bound, std::ranges::input_range auto &&bodies);

        [[nodiscard]] const Node *getRoot() const noexcept;
        [[nodiscard]] std::vector<Cube> getNonEmptyBoxes() const;

    private:
        std::deque<Node> node_pool;
        Node *root;

        Node *getNodeFromPool(const Cube &bound);
        void fetchNonEmptyBoxes(const Node *node, std::vector<Cube> &boxes) const;
    };
};

#include <ranges>

auto NBodyExecutor::OctTree::Node::nonEmptyChildNodes() const noexcept{
    return child_nodes | std::views::filter([](Node *child_node) { return !child_node->empty(); });
}

void NBodyExecutor::OctTree::Node::addBody(const Body *body, auto &&node_gen){
    if (isLeaf()){
        if (bodies.size() < max_bodies_in_node){
            bodies.push_back(body);
        }
        else{
            const auto &oct_regions = data.bound.subdivideIntoEqualCubes();
            child_nodes = [&]<std::size_t... I>(std::index_sequence<I...>) -> std::array<Node*, 8>{
                return { node_gen(oct_regions[I])... };
            }(std::make_index_sequence<8>{});

            // Move node's bodies to corresponding child nodes.
            for (const Body *previous_body : bodies){
                const std::size_t child_index = getChildNodeIndex(previous_body->position);
                child_nodes[child_index]->addBodyToNode(previous_body);
            }
            bodies.clear();

            addBody(body, NBODY_EXECUTOR_FWD(node_gen));
            return; // num_bodies and total_mass should not be updated in this scope.
        }
    }
    else{
        const std::size_t child_index = getChildNodeIndex(body->position);
        child_nodes[child_index]->addBody(body, NBODY_EXECUTOR_FWD(node_gen));
    }

    ++data.num_bodies;
    data.total_mass += body->mass;
}

NBodyExecutor::OctTree::OctTree(const Cube &bound, std::ranges::input_range auto &&bodies){
    static_assert(std::is_convertible_v<std::ranges::range_value_t<decltype(bodies)>, const Body*>);

    root = getNodeFromPool(bound);
    for (const Body *body : bodies){
        root->addBody(body, [this](const Cube &bound){
            return getNodeFromPool(bound);
        });
    }

    assert(!root->empty());
    root->updateCenterOfMass();
}