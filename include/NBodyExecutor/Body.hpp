//
// Created by gomkyung2 on 2023/08/26.
//

#pragma once

#include <glm/ext/vector_float3.hpp>

namespace NBodyExecutor{
    struct Body{
        const float mass;
        glm::vec3 position;
        glm::vec3 velocity;
        glm::vec3 acceleration;
    };
};