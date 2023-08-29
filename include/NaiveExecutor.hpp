//
// Created by gomkyung2 on 2023/08/26.
//

#pragma once

#include "Executor.hpp"

namespace NBodyExecutor{
    class NaiveExecutor : public Executor{
    private:
        static std::vector<std::size_t> generateWorkPartition(std::size_t num_bodies, std::size_t num_threads) noexcept;

    public:
        std::vector<std::size_t> work_partition;

        NaiveExecutor(std::unique_ptr<BS::thread_pool> thread_pool = nullptr);

        void execute(std::span<Body> bodies, float time_delta) override;
    };
};