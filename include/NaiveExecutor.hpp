//
// Created by gomkyung2 on 2023/08/26.
//

#pragma once

#include "Executor.hpp"

namespace NBodyExecutor{
    struct NaiveExecutor : public Executor{
        std::vector<std::size_t> work_partition;

        /**
         * @brief N-Body executor with naive O(n^2) algorithm.
         * @param thread_pool Set thread-pool to parallelize the execution or \p nullptr to execute in serial.
         * @param size_hint If number of bodies and thread count are fixed for every execution, setting this values at
         * parallel execution preallocate the required partial interaction vector and can improve performance.
         */
        NaiveExecutor(std::unique_ptr<BS::thread_pool> thread_pool = nullptr, std::size_t size_hint = 0);

        void execute(std::span<Body> bodies, float time_delta) override;
    };
};