//
// Created by gomkyung2 on 2023/08/26.
//

#pragma once

#include "Executor.hpp"
#include "OctTree.hpp"

namespace NBodyExecutor{
    class BarnesHutExecutor : public Executor{
    private:
        /**
         * @brief Get the bound of given bodies.
         * @param bodies Bodies.
         * @return The smallest cube that contains all bodies.
         */
        static Cube getBodyBound(std::span<Body> bodies);
        void applyGravityField(Body &body, const OctTree::Node *node) const;

    public:
        float threshold = 1.f; // If (node size) / (body distance) is less than this value, regards the node as a single body.

        explicit BarnesHutExecutor(std::unique_ptr<BS::thread_pool> thread_pool = nullptr);

        void execute(std::span<Body> bodies, float time_delta) override;
    };
};