//
// Created by gomkyung2 on 2023/08/26.
//

#pragma once

#include "Executor.hpp"
#include "OctTree.hpp"

namespace NBodyExecutor{
    class BarnesHutExecutor : public Executor{
    private:
        std::vector<Cube> node_boxes; // will be recorded for every execution, when record_node_boxes is true.

        /**
         * @brief Get the bound of given bodies.
         * @param bodies Bodies.
         * @return The smallest cube that contains all bodies.
         */
        static Cube getBodyBound(std::span<Body> bodies);
        void applyGravityField(Body &body, const OctTree::Node *node) const;

    public:
        // If true, node boxes of built octtree is recorded for every execution. You can get recorded node boxes by
        // getNodeBoxes() method.
        bool record_node_boxes = false;
        // If (node size) / (body distance) is less than this value, regards the node as a single body.
        float threshold = 1.f;

        explicit BarnesHutExecutor(std::unique_ptr<BS::thread_pool> thread_pool = nullptr);

        void execute(std::span<Body> bodies, float time_delta) override;
        [[nodiscard]] std::span<const Cube> getNodeBoxes() const noexcept;
    };
};