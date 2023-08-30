//
// Created by gomkyung2 on 2023/08/26.
//

#pragma once

#include <span>
#include <BS_thread_pool.hpp>
#include "Body.hpp"

namespace NBodyExecutor{
    struct Executor{
        float gravity_constant = 1e-6;
        float min_distance = 1e-2;
        std::unique_ptr<BS::thread_pool> thread_pool;

        explicit Executor(std::unique_ptr<BS::thread_pool> thread_pool = nullptr) : thread_pool { std::move(thread_pool) }{

        }

        virtual ~Executor() = default;

        /**
         * @brief Execute the N-body simulation.
         * @param bodies Bodies interacting with each other.
         * @param time_delta Elapsed seconds since the last execution.
         * @note The acceleration of bodies must be initialized to zero before calling this function.
         */
        virtual void execute(std::span<Body> bodies, float time_delta) = 0;
    };
};