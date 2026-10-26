#version 330

/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 *
 * Full-screen sky gradient (one triangle covering the viewport), drawn at the
 * far plane after the scene so only uncovered pixels are shaded
 */

uniform mat4 inverseViewProjection;     // inverse(projection * view without translation)

out vec3 vDirection;

void main() {
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2) * 2.0 - 1.0;
    vec4 world = inverseViewProjection * vec4(p, 1.0, 1.0);
    vDirection = world.xyz / world.w;
    gl_Position = vec4(p, 1.0, 1.0);
}
