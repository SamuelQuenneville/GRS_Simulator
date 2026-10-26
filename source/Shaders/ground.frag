#version 330

/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 *
 * Ground plane: grass with low-frequency variation, anti-aliased grid
 * (10 m minor / 50 m major, X and Z axes), sun shadows and distance fog.
 */

uniform vec3 groundColor;
uniform vec3 gridColor;
uniform vec3 axisXColor;                // Line z = 0
uniform vec3 axisZColor;                // Line x = 0
uniform float gridMinorSpacing;
uniform float gridMajorSpacing;
uniform float gridExtent;               // Grid drawn for |x|,|z| < extent

uniform vec3 sunDirWorld;               // Direction towards the sun (normalized)
uniform vec3 sunColor;
uniform vec3 skyAmbientColor;

uniform sampler2D noiseSampler;          // Tileable smooth noise (R channel)
uniform sampler2DShadow shadowMapSampler;
uniform vec2 shadowMapTexelSize;
uniform bool isShadowEnabled;

uniform vec3 fogColor;
uniform float fogStart;
uniform float fogEnd;

in vec3 vPosWorld;
in vec3 vPosCameraSpace;
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

// Anti-aliased grid lines of roughly widthPx pixels, faded out where they would alias
float gridLines(vec2 p, float spacing, float widthPx) {
    vec2 coord = p / spacing;
    vec2 derivative = fwidth(coord);
    vec2 g = abs(fract(coord - 0.5) - 0.5) / derivative;
    float line = 1.0 - clamp(min(g.x, g.y) - (widthPx - 1.0) * 0.5, 0.0, 1.0);
    float fade = 1.0 - smoothstep(0.15, 0.5, max(derivative.x, derivative.y));
    return line * fade;
}

float axisLine(float d, float widthPx) {
    float g = abs(d) / fwidth(d);
    return 1.0 - clamp(g - (widthPx - 1.0) * 0.5, 0.0, 1.0);
}

void main() {
    vec2 p = vPosWorld.xz;

    float variation = texture(noiseSampler, p / 250.0).r;
    vec3 albedo = groundColor * mix(0.88, 1.12, variation);

    float insideGrid = step(abs(p.x), gridExtent) * step(abs(p.y), gridExtent);
    float minor = gridLines(p, gridMinorSpacing, 1.0) * 0.18;
    float major = gridLines(p, gridMajorSpacing, 1.5) * 0.45;
    albedo = mix(albedo, gridColor, max(minor, major) * insideGrid);
    albedo = mix(albedo, axisXColor, axisLine(p.y, 2.0) * insideGrid);
    albedo = mix(albedo, axisZColor, axisLine(p.x, 2.0) * insideGrid);

    float NdotL = max(sunDirWorld.y, 0.0);
    float visibility = isShadowEnabled ? sunVisibility(vShadowCoords) : 1.0;
    visibility = mix(0.45, 1.0, visibility);

    vec3 result = albedo * (skyAmbientColor + sunColor * NdotL * visibility);

    float fogFactor = clamp((length(vPosCameraSpace) - fogStart) / (fogEnd - fogStart), 0.0, 1.0);
    result = mix(result, fogColor, fogFactor * fogFactor);

    color = vec4(result, 1.0);
}
