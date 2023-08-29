include(FetchContent)
FetchContent_Declare(
        OpenGLApp
        GIT_REPOSITORY https://github.com/stripe2933/OpenGLApp.git
        GIT_TAG main
)
FetchContent_MakeAvailable(OpenGLApp)

add_executable(NBodyExecutor_galaxy galaxy.cpp)
target_compile_features(NBodyExecutor_galaxy PRIVATE cxx_std_20)
target_compile_options(NBodyExecutor_galaxy PRIVATE -ffast-math)
target_link_libraries(NBodyExecutor_galaxy PRIVATE NBodyExecutor OpenGLApp)

# Copy shader files to executable folder.
add_custom_target(copy_shaders COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_LIST_DIR}/copy_shaders.cmake)
add_dependencies(${PROJECT_NAME} copy_shaders)