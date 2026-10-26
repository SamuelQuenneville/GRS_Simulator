#version 330

/********************************************************************************
* OpenGL-Framework                                                              *
* Copyright (c) 2015 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.                                                         *
*                                                                                *
* Permission is granted to anyone to use this software for any purpose,         *
* including commercial applications, and to alter it and redistribute it        *
* freely, subject to the following restrictions:                                *
*                                                                               *
* 1. The origin of this software must not be misrepresented; you must not claim *
*    that you wrote the original software. If you use this software in a        *
*    product, an acknowledgment in the product documentation would be           *
*    appreciated but is not required.                                           *
*                                                                               *
* 2. Altered source versions must be plainly marked as such, and must not be    *
*    misrepresented as being the original software.                             *
*                                                                               *
* 3. This notice may not be removed or altered from any source distribution.    *
*                                                                               *
********************************************************************************/

/*
 * Altered source version: lighting rewritten for the GRS Simulation Environment
 * Lit mesh shader (aircraft, payload, tethers, launcher)
 */

uniform mat4 localToWorldMatrix;        // Local-space to world-space matrix
uniform mat4 worldToCameraMatrix;       // World-space to camera-space matrix
uniform mat4 projectionMatrix;          // Projection matrix
uniform mat3 normalMatrix;              // Normal matrix (local to camera space)
uniform mat4 lightSpaceMatrix;          // World-space to shadow map texture space (bias * proj * view)

layout(location = 0) in vec4 vertexPosition;
layout(location = 1) in vec3 vertexNormal;

out vec3 vPosCameraSpace;
out vec3 vNormalCameraSpace;
out vec4 vShadowCoords;

void main() {
    vec4 positionWorld  = localToWorldMatrix * vertexPosition;
    vec4 positionCamera = worldToCameraMatrix * positionWorld;

    vPosCameraSpace    = positionCamera.xyz;
    vNormalCameraSpace = normalMatrix * vertexNormal;
    vShadowCoords      = lightSpaceMatrix * positionWorld;

    gl_Position = projectionMatrix * positionCamera;
}
