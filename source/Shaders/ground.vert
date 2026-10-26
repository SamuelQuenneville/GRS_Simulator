#version 330

/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

uniform mat4 localToWorldMatrix;
uniform mat4 worldToCameraMatrix;
uniform mat4 projectionMatrix;
uniform mat4 lightSpaceMatrix;

layout(location = 0) in vec4 vertexPosition;

out vec3 vPosWorld;
out vec3 vPosCameraSpace;
out vec4 vShadowCoords;

void main() {
    vec4 positionWorld  = localToWorldMatrix * vertexPosition;
    vec4 positionCamera = worldToCameraMatrix * positionWorld;

    vPosWorld       = positionWorld.xyz;
    vPosCameraSpace = positionCamera.xyz;
    vShadowCoords   = lightSpaceMatrix * positionWorld;

    gl_Position = projectionMatrix * positionCamera;
}
