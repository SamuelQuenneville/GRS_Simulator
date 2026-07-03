#version 330

// Uniform variables
uniform mat4 localToWorldMatrix;        // Local-space to world-space matrix
uniform mat4 worldToCameraMatrix;       // World-space to camera-space matrix
uniform mat4 projectionMatrix;          // Projection matrix

// In variables
layout(location = 0) in vec4 vertexPosition;

void main() {

    // Compute the clip-space vertex coordinates
    gl_Position = projectionMatrix * worldToCameraMatrix * localToWorldMatrix * vertexPosition;
}
