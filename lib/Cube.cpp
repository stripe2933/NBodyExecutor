//
// Created by gomkyung2 on 2023/08/26.
//

#include "Cube.hpp"

NBodyExecutor::Cube::Cube(const glm::vec3 &position, float size) NBODY_EXECUTOR_NOEXCEPT_IF_RELEASE : position { position }, size { size } {
    assert(size > 0.f);
}

std::array<NBodyExecutor::Cube, 8> NBodyExecutor::Cube::subdivideIntoEqualCubes() const noexcept {
    const float half_size = size / 2;
    const glm::vec3 mid = position + half_size;

    return {
        Cube { position, half_size },
        Cube { { position.x, position.y, mid.z }, half_size },
        Cube { { position.x, mid.y, position.z }, half_size },
        Cube { { position.x, mid.y, mid.z }, half_size },
        Cube { { mid.x, position.y, position.z }, half_size },
        Cube { { mid.x, position.y, mid.z }, half_size },
        Cube { { mid.x, mid.y, position.z }, half_size },
        Cube { mid, half_size }
    };
}