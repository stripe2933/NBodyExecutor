#version 330 core

layout(location = 0) in float mass;
layout(location = 1) in vec3 position;

uniform mat4 view;
uniform mat4 projection;

void main(){
    gl_Position = projection * view * vec4(position, 1.0);
    gl_PointSize = 2.0 * mass;
}