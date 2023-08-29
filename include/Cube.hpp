//
// Created by gomkyung2 on 2023/08/26.
//

#pragma once

#include <array>
#include <glm/ext/vector_float3.hpp>
#include "macros.hpp"

namespace NBodyExecutor{
/**
 * @brief A cube that is aligned with the axes and at the arbitrary position.
 * @note It follows the standard layout.
 * @note In debug mode, positive size assertion is performed.
 */
    struct Cube{
        const glm::vec3 position;
        const float size;

        /**
         * @brief Construct a new Cube object.
         * @param position Cube position.
         * @param size Cube side length, must be positive.
         * @note In debug mode, positive size assertion is performed.
         */
        Cube(const glm::vec3 &position, float size) NBODY_EXECUTOR_NOEXCEPT_IF_RELEASE;

        /**
         * @brief Subdivide the cube into eight equally-sized smaller cubes.
         *
         * A cube is subdivided into 8 regions by 3 planes that are parallel to the axes and pass through the center of the cube.
         * The 8 regions are numbered from 0 to 7 in the following order: ---, --+, -+-, -++, +--, +-+, ++-, +++.
         * where the first, second, and third characters represent the x, y, and z coordinates of the region, respectively.
         * (+) sign means the coordinate is greater than the center coordinate, and (-) sign means the coordinate is less than the center coordinate.
         *
         * @return 8 equally subdivided cubes in explained ordering manner.
         */
        [[nodiscard]] std::array<Cube, 8> subdivideIntoEqualCubes() const noexcept;
    };

    static_assert(std::is_standard_layout_v<Cube>);
};