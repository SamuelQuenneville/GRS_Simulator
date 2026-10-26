#version 330

/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 *
 * Close-up x-ray view: brighter where the surface is seen edge-on (Fresnel-like rim)
 */

uniform vec3 ghostColor;
uniform float opacity;

in vec3 vPosCameraSpace;
in vec3 vNormalCameraSpace;

out vec4 color;

void main() {
    vec3 N = normalize(vNormalCameraSpace);
    vec3 V = normalize(-vPosCameraSpace);
    float rim = pow(1.0 - abs(dot(N, V)), 2.0);
    color = vec4(ghostColor, opacity * (0.12 + 0.88 * rim));
}
