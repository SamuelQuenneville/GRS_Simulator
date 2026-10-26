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
 * Lit mesh shader: hemispheric ambient, sun with soft shadows, fill light,
 * Blinn-Phong specular and distance fog.
 */

uniform vec4 globalVertexColor;         // Object base color (alpha used for blending)

uniform vec3 sunDirCameraSpace;         // Direction towards the sun (normalized)
uniform vec3 sunColor;
uniform vec3 fillDirCameraSpace;        // Direction towards the fill light (normalized)
uniform vec3 fillColor;
uniform vec3 upCameraSpace;             // World up in camera space
uniform vec3 skyAmbientColor;
uniform vec3 groundAmbientColor;

uniform sampler2DShadow shadowMapSampler;
uniform vec2 shadowMapTexelSize;
uniform bool isShadowEnabled;

uniform vec3 fogColor;
uniform float fogStart;
uniform float fogEnd;

in vec3 vPosCameraSpace;
in vec3 vNormalCameraSpace;
in vec4 vShadowCoords;

out vec4 color;

float sunVisibility(vec4 shadowCoords) {
    vec3 p = shadowCoords.xyz / shadowCoords.w;
    if (p.x <= 0.0 || p.x >= 1.0 || p.y <= 0.0 || p.y >= 1.0 || p.z >= 1.0) return 1.0;

    // 4 taps half a texel apart, each one bilinearly filtered by the hardware comparison
    vec2 o = 0.75 * shadowMapTexelSize;
    float sum = texture(shadowMapSampler, vec3(p.xy + vec2(-o.x, -o.y), p.z))
              + texture(shadowMapSampler, vec3(p.xy + vec2( o.x, -o.y), p.z))
              + texture(shadowMapSampler, vec3(p.xy + vec2(-o.x,  o.y), p.z))
              + texture(shadowMapSampler, vec3(p.xy + vec2( o.x,  o.y), p.z));
    return sum * 0.25;
}

void main() {
    vec3 base = globalVertexColor.rgb;
    vec3 N = normalize(vNormalCameraSpace);
    if (!gl_FrontFacing) N = -N;
    vec3 V = normalize(-vPosCameraSpace);

    // Ambient: sky from above, ground bounce from below
    vec3 ambient = mix(groundAmbientColor, skyAmbientColor, dot(N, upCameraSpace) * 0.5 + 0.5);

    float NdotL = max(dot(N, sunDirCameraSpace), 0.0);
    float visibility = isShadowEnabled ? sunVisibility(vShadowCoords) : 1.0;
    visibility = mix(0.35, 1.0, visibility); // shadowed areas keep some direct light

    vec3 H = normalize(sunDirCameraSpace + V);
    float specular = NdotL > 0.0 ? pow(max(dot(N, H), 0.0), 64.0) * 0.4 : 0.0;

    float fill = max(dot(N, fillDirCameraSpace), 0.0);

    vec3 result = base * (ambient + sunColor * NdotL * visibility + fillColor * fill)
                + sunColor * specular * visibility;

    float fogFactor = clamp((length(vPosCameraSpace) - fogStart) / (fogEnd - fogStart), 0.0, 1.0);
    result = mix(result, fogColor, fogFactor * fogFactor);

    color = vec4(result, globalVertexColor.a);
}
