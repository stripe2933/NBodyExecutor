#version 330 core

out vec4 color;

void main(){
    vec2 circCoord = 2.0 * gl_PointCoord - 1.0;
    float transparency = mix(1.0, 0.0, dot(circCoord, circCoord));
    color = vec4(0.2, 0.5, 1.0, transparency);
}