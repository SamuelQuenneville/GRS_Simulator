#version 330

/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 *
 * Sky gradient with a soft sun glow
 */

uniform vec3 zenithColor;
uniform vec3 horizonColor;
uniform vec3 sunDirWorld;
uniform vec3 sunColor;
uniform float brightness;               // 1 in the main view, dimmed in the close-up

in vec3 vDirection;

out vec4 color;

void main() {
    vec3 d = normalize(vDirection);
    float h = clamp(d.y, 0.0, 1.0);

    vec3 sky = mix(horizonColor, zenithColor, pow(h, 0.5));

    float sunAmount = max(dot(d, sunDirWorld), 0.0);
    sky += sunColor * (pow(sunAmount, 800.0) * 1.2 + pow(sunAmount, 8.0) * 0.12);

    color = vec4(sky * brightness, 1.0);
}
