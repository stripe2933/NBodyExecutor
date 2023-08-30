//
// Created by gomkyung2 on 2023/08/26.
//

#include <random>
#include <algorithm>
#include <OpenGLApp/Window.hpp>
#include <OpenGLApp/Program.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtc/matrix_transform.hpp>

#define EXECUTE_METHOD 1 // 0 -> NaiveExecutor, 1 -> BarnesHutExecutor
#if EXECUTE_METHOD == 0
#include <NBodyExecutor/NaiveExecutor.hpp>
#define EXECUTOR NaiveExecutor
#elif EXECUTE_METHOD == 1
#include <NBodyExecutor/BarnesHutExecutor.hpp>
#define EXECUTOR BarnesHutExecutor
#endif

using namespace NBodyExecutor;

class App : public OpenGL::Window{
private:
    std::vector<Body> bodies;
    EXECUTOR executor;

    OpenGL::Program program;
    GLuint vao, vbo;

    void update(float time_delta) override {
        // Reset each body's acceleration to zero before n-body execution.
        std::ranges::for_each(bodies, [](Body &body) { body.acceleration = glm::zero<glm::vec3>(); });
        executor.execute(bodies, time_delta);

        // Copy data in bodies to GPU. Since allocated data size in GPU is equal to bodies, use glBufferSubData rather
        // than glBufferData.
        glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<GLsizeiptr>(bodies.size() * sizeof(Body)), bodies.data());

    }

    void draw() const override {
        // Draw bodies.
        glClear(GL_COLOR_BUFFER_BIT);
        glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(bodies.size()));
    }

    std::vector<Body> generateBodies(std::size_t num_bodies){
        static std::random_device rd;
        static std::mt19937 gen { rd() };
        std::uniform_real_distribution<float> uniform_dis { 0.f, 1.f };
        std::normal_distribution<float> normal_dis { 0.f, 1e-2f };

        std::vector<Body> result;
        result.reserve(num_bodies);
        std::generate_n(std::back_inserter(result), num_bodies, [&]{
            const float radius = uniform_dis(gen);
            const float longitude = glm::two_pi<float>() * uniform_dis(gen);
            const glm::vec3 noise { normal_dis(gen), 4.f * normal_dis(gen), normal_dis(gen) };
            const glm::vec3 position = radius * glm::vec3(std::cos(longitude), 0.f, std::sin(longitude)) + noise;

            const glm::vec3 velocity = 0.2f * glm::vec3(position.z, normal_dis(gen), -position.x);

            return Body {
                .mass = std::lerp(0.5f, 2.f, uniform_dis(gen)),
                .position = position,
                .velocity = velocity,
                /* .acceleration = glm::zero<glm::vec3>() */ // will be initialized as zero later.
            };
        });

        return result;
    }

public:
    App(std::size_t num_bodies) : OpenGL::Window { 640, 480, "N-Body Simulation" },
                                  program { "shaders/galaxy.vert", "shaders/galaxy.frag" },
                                  bodies { generateBodies(num_bodies) },
                                  executor { std::make_unique<BS::thread_pool>() }
    {
        const glm::mat4 view = glm::lookAt(glm::vec3(-1.f, 1.5f, 5.f), glm::vec3(0.f), glm::vec3(0.f, 1.f, 0.f));
        const glm::mat4 projection = glm::perspective(glm::radians(45.f), getAspectRatio(), 0.1f, 100.f);

        program.use();
        program.setUniform("view", view);
        program.setUniform("projection", projection);

        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);

        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(bodies.size() * sizeof(Body)), bodies.data(), GL_STREAM_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 1, GL_FLOAT, GL_FALSE, sizeof(Body), reinterpret_cast<GLint*>(offsetof(Body, mass)));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Body), reinterpret_cast<GLint*>(offsetof(Body, position)));

        glEnable(GL_PROGRAM_POINT_SIZE);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_BLEND);
    }

    ~App() noexcept override {
        glDeleteBuffers(1, &vbo);
        glDeleteVertexArrays(1, &vao);
    }
};

int main(){
    App { 16384 }.run();
}