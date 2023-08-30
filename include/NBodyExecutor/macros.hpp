//
// Created by gomkyung2 on 2023/08/26.
//

#pragma once

#ifndef NDEBUG
#define NBODY_EXECUTOR_NOEXCEPT_IF_RELEASE
#else
#define NBODY_EXECUTOR_NOEXCEPT_IF_RELEASE noexcept
#endif

#define NBODY_EXECUTOR_FWD(x) std::forward<decltype(x)>(x)