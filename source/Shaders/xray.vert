#version 330

/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 *
 * Close-up x-ray view: see-through airframe
 */

uniform mat4 localToWorldMatrix;
uniform mat4 worldToCameraMatrix;
uniform mat4 projectionMatrix;
uniform mat3 normalMatrix;

layout(location = 0) in vec4 vertexPosition;
layout(location = 1) in vec3 vertexNormal;

out vec3 vPosCameraSpace;
out vec3 vNormalCameraSpace;

void main() {
    vec4 positionCamera = worldToCameraMatrix * localToWorldMatrix * vertexPosition;
    vPosCameraSpace    = positionCamera.xyz;
    vNormalCameraSpace = normalMatrix * vertexNormal;
    gl_Position = projectionMatrix * positionCamera;
}
