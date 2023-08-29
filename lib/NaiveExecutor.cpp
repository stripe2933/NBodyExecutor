//
// Created by gomkyung2 on 2023/08/26.
//

#include "NaiveExecutor.hpp"

#include <glm/gtx/common.hpp>
#include <glm/gtx/norm.hpp>

NBodyExecutor::NaiveExecutor::NaiveExecutor(std::unique_ptr<BS::thread_pool> thread_pool, std::size_t size_hint)
        : Executor { std::move(thread_pool) }
{
    if (this->thread_pool && size_hint){
        const auto num_threads = this->thread_pool->get_thread_count();
        work_partition.reserve(num_threads + 1);

        const auto next_partition =
                [n=static_cast<double>(size_hint),
                 td=static_cast<double>(size_hint)*(static_cast<double>(size_hint)-1.0) / (2.0*this->thread_pool->get_thread_count())]
                (std::size_t previous_partition) -> std::size_t {
            double m2a1 = 2.0 * static_cast<double>(previous_partition) + 1.0;
            return std::floor(n - 0.5 * (1.0 + std::sqrt(4.0 * n * n - 8 * td - 4 * n * m2a1 + m2a1 * m2a1)));
        };

        work_partition.push_back(0);
        for (std::size_t i = 0; i < num_threads - 1; ++i){
            work_partition.push_back(next_partition(work_partition.back()));
        }
        work_partition.push_back(size_hint);
    }
}

void NBodyExecutor::NaiveExecutor::execute(std::span<Body> bodies, float time_delta){
    if (thread_pool){
        const auto interaction_partial_task = [&](std::size_t start_idx, std::size_t end_idx) -> std::vector<glm::vec3>{
            std::vector partial_acceleration { bodies.size(), glm::vec3(0.f) };
            for (std::size_t idx1 = start_idx; idx1 < end_idx; ++idx1){
                for (std::size_t idx2 = idx1 + 1; idx2 < bodies.size(); ++idx2){
                    const glm::vec3 displacement = bodies[idx2].position - bodies[idx1].position;
                    const float distance = glm::max(glm::length(displacement), min_distance);
                    const glm::vec3 coefficient = gravity_constant * displacement / glm::pow(distance, 3.f); // G / r^2

                    partial_acceleration[idx1] += coefficient * bodies[idx2].mass;
                    partial_acceleration[idx2] -= coefficient * bodies[idx1].mass;
                }
            }

            return partial_acceleration;
        };
        const auto partial_accelerations = thread_pool->parallelize_loop(thread_pool->get_thread_count(), [&](std::size_t thread_idx, auto){
            return interaction_partial_task(work_partition[thread_idx], work_partition[thread_idx + 1]);
        }).get();

        const auto merge_partial_task = [&](std::size_t start_idx, std::size_t end_idx){
            for (std::size_t idx = start_idx; idx < end_idx; ++idx){
                for (std::span partial_acceleration : partial_accelerations){
                    bodies[idx].acceleration += partial_acceleration[idx];
                }
            }
        };
        thread_pool->push_loop(bodies.size(), merge_partial_task);
        thread_pool->wait_for_tasks();
    }
    else{
        for (std::size_t idx1 = 0; idx1 < bodies.size() - 1; ++idx1){
            for (std::size_t idx2 = idx1 + 1; idx2 < bodies.size(); ++idx2){
                const glm::vec3 displacement = bodies[idx2].position - bodies[idx1].position;
                const float distance = glm::max(glm::length(displacement), min_distance);
                const glm::vec3 coefficient = gravity_constant * displacement / glm::pow(distance, 3.f); // G / r^2

                bodies[idx1].acceleration += coefficient * bodies[idx2].mass;
                bodies[idx2].acceleration -= coefficient * bodies[idx1].mass;
            }
        }
    }

    for (auto &body : bodies){
        body.velocity += body.acceleration * time_delta;
        body.position += body.velocity * time_delta;
    }
}
