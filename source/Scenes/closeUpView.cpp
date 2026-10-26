/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "closeUpView.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>

namespace {

constexpr double kFieldOfView = 30.0 * M_PI / 180.0;
constexpr double kFeatureAngle = 28.0 * M_PI / 180.0;

grs::Matrix4d lookAt(const grs::Vec3d& eye, const grs::Vec3d& target, const grs::Vec3d& up) {
    const grs::Vec3d f = grs::normalize(target - eye);
    const grs::Vec3d s = grs::normalize(grs::cross(f, up));
    const grs::Vec3d u = grs::cross(s, f);
    return { s[0],  s[1],  s[2], -grs::dot(s, eye),
             u[0],  u[1],  u[2], -grs::dot(u, eye),
            -f[0], -f[1], -f[2],  grs::dot(f, eye),
             0.0,   0.0,   0.0,   1.0};
}

grs::Matrix4d perspective(const double fovY, const double aspect, const double nearPlane, const double farPlane) {
    const double t = 1.0 / std::tan(fovY / 2.0);
    return {t / aspect, 0.0, 0.0, 0.0,
            0.0, t, 0.0, 0.0,
            0.0, 0.0, -(farPlane + nearPlane) / (farPlane - nearPlane), -2.0 * farPlane * nearPlane / (farPlane - nearPlane),
            0.0, 0.0, -1.0, 0.0};
}

enum class FrameKind { CenterOfMass, Aero, Propulsion, Tether, Other };

struct FrameStyle {
    FrameKind kind;
    std::string label;
    NVGcolor color;
};

std::string suffixOf(const std::string& name) {
    const auto pos = name.find('_');
    return pos == std::string::npos ? name : name.substr(pos + 1);
}

std::string prefixOf(const std::string& name) {
    const auto pos = name.find('_');
    return pos == std::string::npos ? name : name.substr(0, pos);
}

FrameStyle styleFor(const frameRenderData& frame) {
    const std::string suffix = suffixOf(frame.name);
    if (frame.isCenterOfMass || suffix == "centerOfMass") return {.kind = FrameKind::CenterOfMass, .label = "CG", .color = nvgRGB(255, 255, 255)};
    if (suffix == "aeroRef")       return {.kind = FrameKind::Aero, .label = "aero ref", .color = nvgRGB(56, 214, 232)};
    if (suffix == "propulsionRef") return {.kind = FrameKind::Propulsion, .label = "propulsion", .color = nvgRGB(255, 159, 67)};
    if (suffix == "tetherAttach")  return {.kind = FrameKind::Tether, .label = "tether attach", .color = nvgRGB(255, 210, 63)};
    return {.kind = FrameKind::Other, .label = suffix, .color = nvgRGB(190, 200, 210)};
}

std::string forceName(const FrameKind kind) {
    switch (kind) {
        case FrameKind::Propulsion:   return "thrust";
        case FrameKind::Tether:       return "tether";
        case FrameKind::Aero:         return "aero";
        case FrameKind::CenterOfMass: return "CG";
        default:                      return "force";
    }
}

// Direction of a vector expressed in the frame's own axes (NED), in render world coordinates
grs::Vec3d frameAxisToWorld(const frameRenderData& frame, const grs::Vec3d& vectorNed) {
    return frame.transform.toMatrix4().upperLeft3x3() * vectorNed.toOpenglRef();
}

} // namespace

/*
 *   WIREFRAME EDGES
 */

WireframeEdges::WireframeEdges(const Mesh& mesh) {
    const auto& vertices = mesh.getVertices();
    const auto& indices = mesh.getIndices(0);

    // Weld vertices duplicated by the mesh reader (per-face normals or UVs)
    std::map<std::array<long long, 3>, uint32_t> welded;
    std::vector<uint32_t> remap(vertices.size());
    for (size_t i = 0; i < vertices.size(); ++i) {
        const std::array<long long, 3> key{std::llround(vertices[i][0] * 1e5), std::llround(vertices[i][1] * 1e5), std::llround(vertices[i][2] * 1e5)};
        const auto [it, inserted] = welded.emplace(key, static_cast<uint32_t>(m_vertices.size()));
        if (inserted) m_vertices.push_back(vertices[i]);
        remap[i] = it->second;
    }

    grs::Vec3f minCorner = m_vertices.front();
    grs::Vec3f maxCorner = m_vertices.front();
    for (const auto& v : m_vertices) {
        for (int k = 0; k < 3; ++k) {
            minCorner[k] = std::min(minCorner[k], v[k]);
            maxCorner[k] = std::max(maxCorner[k], v[k]);
        }
    }
    m_boundingRadius = 0.5 * static_cast<double>((maxCorner - minCorner).norm());

    // Faces and edge adjacency
    // Faces of each edge, and whether the face uses the edge from low to high index
    // (two consistently wound neighbours use it in opposite directions)
    std::unordered_map<uint64_t, std::vector<std::pair<uint32_t, bool>>> edgeFaces;
    for (size_t t = 0; t + 2 < indices.size(); t += 3) {
        const uint32_t a = remap[indices[t]], b = remap[indices[t + 1]], c = remap[indices[t + 2]];
        if (a == b || b == c || a == c) continue;

        const grs::Vec3f n = grs::cross(m_vertices[b] - m_vertices[a], m_vertices[c] - m_vertices[a]);
        if (n.norm() < 1e-12f) continue;

        const auto face = static_cast<uint32_t>(m_faceNormals.size());
        m_faceNormals.push_back(grs::normalize(n));
        m_faceCenters.push_back((m_vertices[a] + m_vertices[b] + m_vertices[c]) * (1.0f / 3.0f));

        for (const auto& [p, q] : {std::pair{a, b}, std::pair{b, c}, std::pair{c, a}}) {
            const uint64_t key = (static_cast<uint64_t>(std::min(p, q)) << 32) | std::max(p, q);
            edgeFaces[key].emplace_back(face, p < q);
        }
    }
    m_frontFacing.resize(m_faceNormals.size());

    std::vector<grs::Vec3f> featureLines;
    const auto cosFeature = static_cast<float>(std::cos(kFeatureAngle));
    for (const auto& [key, faces] : edgeFaces) {
        const auto v0 = static_cast<uint32_t>(key >> 32);
        const auto v1 = static_cast<uint32_t>(key & 0xffffffffu);
        bool sharp = faces.size() < 2;
        bool flipped = false;
        if (faces.size() > 2) {
            // Non-manifold (e.g. double-sided thin plates): sharp only if the faces really bend
            for (size_t i = 0; i < faces.size() && !sharp; ++i) {
                for (size_t j = i + 1; j < faces.size() && !sharp; ++j) {
                    sharp = std::abs(grs::dot(m_faceNormals[faces[i].first], m_faceNormals[faces[j].first])) < cosFeature;
                }
            }
            if (!sharp) continue;
        } else if (!sharp) {
            flipped = faces[0].second == faces[1].second;
            const float sign = flipped ? -1.0f : 1.0f;
            sharp = sign * grs::dot(m_faceNormals[faces[0].first], m_faceNormals[faces[1].first]) < cosFeature;
        }
        if (sharp) {
            featureLines.push_back(m_vertices[v0]);
            featureLines.push_back(m_vertices[v1]);
        } else {
            m_smoothEdges.push_back({.v0 = v0, .v1 = v1, .f0 = faces[0].first, .f1 = faces[1].first, .flipped = flipped});
        }
    }

    m_createLineBuffer(m_featureVao, m_featureVbo);
    glBindBuffer(GL_ARRAY_BUFFER, m_featureVbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(featureLines.size() * sizeof(grs::Vec3f)), featureLines.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    m_featureVertexCount = static_cast<GLsizei>(featureLines.size());

    m_createLineBuffer(m_silhouetteVao, m_silhouetteVbo);
}

WireframeEdges::~WireframeEdges() {
    glDeleteBuffers(1, &m_featureVbo);
    glDeleteVertexArrays(1, &m_featureVao);
    glDeleteBuffers(1, &m_silhouetteVbo);
    glDeleteVertexArrays(1, &m_silhouetteVao);
}

void WireframeEdges::m_createLineBuffer(GLuint& vao, GLuint& vbo) {
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(grs::Vec3f), nullptr);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void WireframeEdges::updateSilhouettes(const grs::Vec3d& cameraPositionLocal) {
    const grs::Vec3f camera{static_cast<float>(cameraPositionLocal[0]), static_cast<float>(cameraPositionLocal[1]), static_cast<float>(cameraPositionLocal[2])};

    for (size_t f = 0; f < m_faceNormals.size(); ++f) {
        m_frontFacing[f] = grs::dot(m_faceNormals[f], camera - m_faceCenters[f]) > 0.0f ? 1 : 0;
    }

    m_silhouetteLines.clear();
    for (const auto& e : m_smoothEdges) {
        if ((m_frontFacing[e.f0] != m_frontFacing[e.f1]) != e.flipped) {
            m_silhouetteLines.push_back(m_vertices[e.v0]);
            m_silhouetteLines.push_back(m_vertices[e.v1]);
        }
    }

    glBindBuffer(GL_ARRAY_BUFFER, m_silhouetteVbo);
    const auto bytes = static_cast<GLsizeiptr>(m_silhouetteLines.size() * sizeof(grs::Vec3f));
    if (m_silhouetteLines.size() > m_silhouetteCapacity) {
        m_silhouetteCapacity = m_silhouetteLines.size() * 2;
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(m_silhouetteCapacity * sizeof(grs::Vec3f)), nullptr, GL_DYNAMIC_DRAW);
    }
    if (bytes > 0) {
        glBufferSubData(GL_ARRAY_BUFFER, 0, bytes, m_silhouetteLines.data());
    }
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    m_silhouetteVertexCount = static_cast<GLsizei>(m_silhouetteLines.size());
}

void WireframeEdges::drawFeatureEdges() const {
    glBindVertexArray(m_featureVao);
    glDrawArrays(GL_LINES, 0, m_featureVertexCount);
    glBindVertexArray(0);
}

void WireframeEdges::drawSilhouetteEdges() const {
    if (m_silhouetteVertexCount == 0) return;
    glBindVertexArray(m_silhouetteVao);
    glDrawArrays(GL_LINES, 0, m_silhouetteVertexCount);
    glBindVertexArray(0);
}

/*
 *   CLOSE-UP VIEW
 */

CloseUpView::CloseUpView(const std::string& executablePath)
    : m_xrayShader(executablePath + R"(/source/Shaders/xray.vert)", executablePath + R"(/source/Shaders/xray.frag)")
{
}

CloseUpView::~CloseUpView() {
    m_wireframes.clear();
    m_xrayShader.destroy();
}

WireframeEdges& CloseUpView::m_wireframeFor(const PhysicsObject& object) {
    auto& entry = m_wireframes[&object];
    if (!entry) {
        entry = std::make_unique<WireframeEdges>(object);
    }
    return *entry;
}

void CloseUpView::render(PhysicsObject& object, const int selectedFrame, const int framebufferWidth, const int framebufferHeight,
                         const float pixelRatio, const Shader& colorShader, Shader& depthShader, const SkyRenderer& drawSky) {
    m_body = object.getBodyRenderData();
    m_selectedFrame = selectedFrame;
    m_framebufferHeight = framebufferHeight;
    m_pixelRatio = pixelRatio;

    WireframeEdges& wire = m_wireframeFor(object);

    // True-scale model matrix (the main view may draw aircraft larger)
    const grs::Matrix4d scaling = object.trueScaleMatrix();
    const grs::Matrix4d bodyMatrix = m_body.transform.toMatrix4();
    const grs::Matrix4d modelMatrix = bodyMatrix * scaling;
    const double scale = std::max({scaling(0, 0), scaling(1, 1), scaling(2, 2)});
    m_bodyRadius = std::max(0.05, wire.boundingRadius() * scale);

    // Orbit target: the CG
    grs::Vec3d target = m_body.transform.position;
    for (const auto& frame : m_body.frames) {
        if (frame.isCenterOfMass) target = frame.transform.position;
    }

    // Body-fixed orbit camera
    const grs::Matrix3d bodyRotation = bodyMatrix.upperLeft3x3();
    const grs::Vec3d directionLocal(std::cos(m_elevation) * std::cos(m_azimuth), std::sin(m_elevation), std::cos(m_elevation) * std::sin(m_azimuth));
    const grs::Vec3d direction = grs::normalize(bodyRotation * directionLocal);
    const grs::Vec3d up = grs::normalize(bodyRotation * grs::Vec3d(0.0, 1.0, 0.0));
    const double distance = m_zoom * 0.72 * m_bodyRadius / std::sin(kFieldOfView / 2.0);
    const grs::Vec3d eye = target + direction * distance;

    // Inset rectangle, bottom-right
    const int margin = static_cast<int>(std::lround(16.0f * pixelRatio));
    m_rectWidth = std::clamp(static_cast<int>(0.36 * framebufferWidth), static_cast<int>(300 * pixelRatio), std::max(1, framebufferWidth - 2 * margin));
    m_rectHeight = std::min(static_cast<int>(m_rectWidth * 0.62), std::max(1, framebufferHeight - 2 * margin));
    m_rectX = framebufferWidth - m_rectWidth - margin;
    m_rectY = margin;

    m_viewMatrix = lookAt(eye, target, up);
    m_projectionMatrix = perspective(kFieldOfView, static_cast<double>(m_rectWidth) / m_rectHeight, distance * 0.02, distance * 20.0);

    wire.updateSilhouettes(modelMatrix.inverse() * eye);

    // ---------- GL ---------- //
    glEnable(GL_SCISSOR_TEST);
    glScissor(m_rectX, m_rectY, m_rectWidth, m_rectHeight);
    glViewport(m_rectX, m_rectY, m_rectWidth, m_rectHeight);
    glDisable(GL_CULL_FACE);

    colorShader.bind();
    colorShader.setMatrix4x4Uniform("localToWorldMatrix", modelMatrix);
    colorShader.setMatrix4x4Uniform("worldToCameraMatrix", m_viewMatrix);
    colorShader.setMatrix4x4Uniform("projectionMatrix", m_projectionMatrix);
    colorShader.setIntUniform("isGlobalVertexColorEnabled", 1, false);
    colorShader.unbind();

    auto drawEdges = [&](const grs::Vec4f& featureColor, const grs::Vec4f& silhouetteColor) {
        colorShader.bind();
        colorShader.setVector4Uniform("globalVertexColor", featureColor, false);
        wire.drawFeatureEdges();
        colorShader.setVector4Uniform("globalVertexColor", silhouetteColor, false);
        wire.drawSilhouetteEdges();
        colorShader.unbind();
    };

    if (m_mode == Mode::Wireframe) {
        glClearColor(0.06f, 0.08f, 0.11f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Depth of the surface only, pushed back so edges lying on it pass the test
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.5f, 2.0f);
        depthShader.bind();
        depthShader.setMatrix4x4Uniform("projectionMatrix", m_projectionMatrix);
        depthShader.unbind();
        object.renderWithModel(depthShader, modelMatrix, m_viewMatrix);
        glDisable(GL_POLYGON_OFFSET_FILL);
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        // Hidden edges faint, visible edges bright
        glDepthFunc(GL_GREATER);
        drawEdges({0.56f, 0.64f, 0.73f, 0.16f}, {0.84f, 0.89f, 0.94f, 0.22f});
        glDepthFunc(GL_LEQUAL);
        drawEdges({0.56f, 0.64f, 0.73f, 0.60f}, {0.84f, 0.89f, 0.94f, 0.95f});
    } else {
        glClear(GL_DEPTH_BUFFER_BIT);
        drawSky(m_viewMatrix, m_projectionMatrix, 0.30f);

        // See-through airframe: additive, brighter at grazing angles
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        m_xrayShader.bind();
        m_xrayShader.setMatrix4x4Uniform("projectionMatrix", m_projectionMatrix);
        m_xrayShader.setVector3Uniform("ghostColor", {0.55f, 0.85f, 1.0f});
        m_xrayShader.setFloatUniform("opacity", 0.40f);
        m_xrayShader.unbind();
        object.renderWithModel(m_xrayShader, modelMatrix, m_viewMatrix);

        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        drawEdges({0.75f, 0.92f, 1.0f, 0.25f}, {0.75f, 0.92f, 1.0f, 0.90f});
    }

    // Restore the state expected by the rest of the frame
    glDisable(GL_BLEND);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_SCISSOR_TEST);
    glViewport(0, 0, framebufferWidth, framebufferHeight);

    m_hasFrame = true;
}

bool CloseUpView::m_project(const grs::Vec3d& world, float& x, float& y) const {
    const grs::Matrix4d m = m_projectionMatrix * m_viewMatrix;
    const double cx = m(0,0)*world[0] + m(0,1)*world[1] + m(0,2)*world[2] + m(0,3);
    const double cy = m(1,0)*world[0] + m(1,1)*world[1] + m(1,2)*world[2] + m(1,3);
    const double cw = m(3,0)*world[0] + m(3,1)*world[1] + m(3,2)*world[2] + m(3,3);
    if (cw <= 1e-6) return false;

    const double fbX = m_rectX + (cx / cw * 0.5 + 0.5) * m_rectWidth;
    const double fbY = m_rectY + (cy / cw * 0.5 + 0.5) * m_rectHeight;
    x = static_cast<float>(fbX / m_pixelRatio);
    y = static_cast<float>((m_framebufferHeight - fbY) / m_pixelRatio);
    return true;
}

bool CloseUpView::m_contains(const double x, const double y) const {
    const double left = m_rectX / m_pixelRatio;
    const double top = (m_framebufferHeight - m_rectY - m_rectHeight) / m_pixelRatio;
    return x >= left && x <= left + m_rectWidth / m_pixelRatio && y >= top && y <= top + m_rectHeight / m_pixelRatio;
}

void CloseUpView::drawOverlay(NVGcontext* vg) const {
    if (!m_hasFrame || m_body.frames.empty()) return;

    const float rx = static_cast<float>(m_rectX) / m_pixelRatio;
    const float ry = static_cast<float>(m_framebufferHeight - m_rectY - m_rectHeight) / m_pixelRatio;
    const float rw = static_cast<float>(m_rectWidth) / m_pixelRatio;
    const float rh = static_cast<float>(m_rectHeight) / m_pixelRatio;

    nvgSave(vg);
    nvgScissor(vg, rx, ry, rw, rh);

    auto line = [&](float x0, float y0, float x1, float y1, NVGcolor c, float w) {
        nvgBeginPath(vg); nvgMoveTo(vg, x0, y0); nvgLineTo(vg, x1, y1);
        nvgStrokeColor(vg, c); nvgStrokeWidth(vg, w); nvgStroke(vg);
    };
    auto dashed = [&](float x0, float y0, float x1, float y1, NVGcolor c, float w) {
        const float len = std::hypot(x1 - x0, y1 - y0);
        const int n = std::max(1, static_cast<int>(len / 7.0f));
        for (int i = 0; i < n; i += 2) {
            const float a = static_cast<float>(i) / n, b = std::min(1.0f, static_cast<float>(i + 1) / n);
            line(x0 + (x1 - x0) * a, y0 + (y1 - y0) * a, x0 + (x1 - x0) * b, y0 + (y1 - y0) * b, c, w);
        }
    };
    auto arrow = [&](float x0, float y0, float x1, float y1, NVGcolor c, float w) {
        line(x0, y0, x1, y1, c, w);
        const float len = std::hypot(x1 - x0, y1 - y0);
        if (len < 4.0f) return;
        const float ux = (x1 - x0) / len, uy = (y1 - y0) / len;
        const float head = std::min(10.0f, 0.4f * len);
        nvgBeginPath(vg);
        nvgMoveTo(vg, x1, y1);
        nvgLineTo(vg, x1 - ux * head - uy * head * 0.45f, y1 - uy * head + ux * head * 0.45f);
        nvgLineTo(vg, x1 - ux * head + uy * head * 0.45f, y1 - uy * head - ux * head * 0.45f);
        nvgClosePath(vg);
        nvgFillColor(vg, c); nvgFill(vg);
    };
    auto text = [&](float x, float y, const std::string& s, NVGcolor c, float size, int align, const char* face = "sans") {
        nvgFontFace(vg, face);
        nvgFontSize(vg, size);
        nvgTextAlign(vg, align);
        nvgFontBlur(vg, 2.5f);
        nvgFillColor(vg, nvgRGBA(5, 8, 12, 230));
        nvgText(vg, x, y, s.c_str(), nullptr);
        nvgFontBlur(vg, 0.0f);
        nvgFillColor(vg, c);
        nvgText(vg, x, y, s.c_str(), nullptr);
    };
    auto project = [&](const grs::Vec3d& p, float& x, float& y) { return m_project(p, x, y); };

    // CG frame (origin of the lever arms)
    size_t cgIndex = 0;
    for (size_t i = 0; i < m_body.frames.size(); ++i) {
        if (m_body.frames[i].isCenterOfMass) cgIndex = i;
    }
    const frameRenderData& cg = m_body.frames[cgIndex];
    const double L = m_bodyRadius;

    float cgx = 0, cgy = 0;
    const bool cgVisible = project(cg.transform.position, cgx, cgy);

    // ---------- Geometry overlays ---------- //
    if (m_mode == Mode::Wireframe) {
        // Body axes at the CG
        const std::array<std::pair<grs::Vec3d, NVGcolor>, 3> axes{{
            {{1, 0, 0}, nvgRGB(255, 92, 92)}, {{0, 1, 0}, nvgRGB(92, 214, 92)}, {{0, 0, 1}, nvgRGB(92, 157, 255)}}};
        const char* axisNames[3] = {"x", "y", "z"};
        for (int a = 0; a < 3; ++a) {
            float ex, ey;
            if (cgVisible && project(cg.transform.position + frameAxisToWorld(cg, axes[a].first * (0.30 * L)), ex, ey)) {
                arrow(cgx, cgy, ex, ey, axes[a].second, 1.8f);
                text(ex + (ex - cgx) * 0.12f, ey + (ey - cgy) * 0.12f, axisNames[a], axes[a].second, 13.0f, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, "sans-bold");
            }
        }

        for (size_t i = 0; i < m_body.frames.size(); ++i) {
            if (i == cgIndex) continue;
            const frameRenderData& f = m_body.frames[i];
            const FrameStyle style = styleFor(f);
            float px, py;
            if (!project(f.transform.position, px, py)) continue;

            // Lever arm from the CG
            if (cgVisible && (f.transform.position - cg.transform.position).norm() > 1e-4) {
                dashed(cgx, cgy, px, py, style.color, 1.4f);
            }

            float ex, ey;
            if (style.kind == FrameKind::Propulsion && project(f.transform.position + frameAxisToWorld(f, {0.45 * L, 0, 0}), ex, ey)) {
                arrow(px, py, ex, ey, style.color, 2.2f);   // thrust axis
            }
            if (style.kind == FrameKind::Tether && f.forceAtOrigin.norm() > 1e-6 &&
                project(f.transform.position + grs::normalize(frameAxisToWorld(f, f.forceAtOrigin)) * (0.7 * L), ex, ey)) {
                line(px, py, ex, ey, nvgTransRGBA(style.color, 200), 1.6f);   // tether direction
            }
        }
    } else {
        for (const auto& f : m_body.frames) {
            const FrameStyle style = styleFor(f);
            float px, py;
            if (!project(f.transform.position, px, py)) continue;

            // Frame triad, as drawn in the main view
            const std::array<std::pair<grs::Vec3d, NVGcolor>, 3> axes{{
                {{1, 0, 0}, nvgRGB(255, 92, 92)}, {{0, 1, 0}, nvgRGB(92, 214, 92)}, {{0, 0, 1}, nvgRGB(92, 157, 255)}}};
            for (const auto& [axis, color] : axes) {
                float ex, ey;
                if (project(f.transform.position + frameAxisToWorld(f, axis * (0.14 * L)), ex, ey)) {
                    line(px, py, ex, ey, color, 1.5f);
                }
            }

            // Live force at the frame origin
            const double magnitude = f.forceAtOrigin.norm();
            if (magnitude > 1e-3) {
                const double length = L * (0.20 + 0.40 * std::min(1.0, magnitude / 60.0));
                float ex, ey;
                if (project(f.transform.position + grs::normalize(frameAxisToWorld(f, f.forceAtOrigin)) * length, ex, ey)) {
                    arrow(px, py, ex, ey, style.color, 2.4f);
                    char buffer[64];
                    std::snprintf(buffer, sizeof(buffer), "%s  %.1f N", forceName(style.kind).c_str(), magnitude);
                    text(ex + 6.0f, ey, buffer, style.color, 13.0f, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);
                }
            }
        }
    }

    // ---------- Markers ---------- //
    struct Label { float x, y; std::string text; NVGcolor color; bool selected; };
    std::vector<Label> labels;

    for (size_t i = 0; i < m_body.frames.size(); ++i) {
        const frameRenderData& f = m_body.frames[i];
        const FrameStyle style = styleFor(f);
        float px, py;
        if (!project(f.transform.position, px, py)) continue;

        const bool selected = static_cast<int>(i) == m_selectedFrame;
        if (selected) {
            nvgBeginPath(vg); nvgCircle(vg, px, py, 11.0f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 200)); nvgStrokeWidth(vg, 1.5f); nvgStroke(vg);
        }

        if (style.kind == FrameKind::CenterOfMass) {
            const float r = 7.0f;
            for (int q = 0; q < 4; ++q) {
                nvgBeginPath(vg);
                nvgMoveTo(vg, px, py);
                nvgArc(vg, px, py, r, static_cast<float>(q * NVG_PI / 2), static_cast<float>((q + 1) * NVG_PI / 2), NVG_CW);
                nvgClosePath(vg);
                nvgFillColor(vg, q % 2 == 0 ? nvgRGB(20, 20, 20) : nvgRGB(240, 240, 240));
                nvgFill(vg);
            }
            nvgBeginPath(vg); nvgCircle(vg, px, py, r);
            nvgStrokeColor(vg, nvgRGB(240, 240, 240)); nvgStrokeWidth(vg, 1.0f); nvgStroke(vg);
        } else {
            nvgBeginPath(vg); nvgCircle(vg, px, py, 5.0f);
            nvgFillColor(vg, nvgRGBA(12, 16, 22, 220)); nvgFill(vg);
            nvgStrokeColor(vg, style.color); nvgStrokeWidth(vg, 2.0f); nvgStroke(vg);
        }

        std::string labelText = style.label;
        if (m_mode == Mode::Wireframe && style.kind != FrameKind::CenterOfMass) {
            const grs::Vec3d r = f.localPosition - cg.localPosition;
            char buffer[96];
            std::snprintf(buffer, sizeof(buffer), "   r = (%+.3f, %+.3f, %+.3f) m", r[0], r[1], r[2]);
            labelText += buffer;
        }
        labels.push_back({px, py, labelText, style.color, selected});
    }

    // ---------- Labels: column on the right, spread vertically ---------- //
    std::sort(labels.begin(), labels.end(), [](const Label& a, const Label& b) { return a.y < b.y; });
    const float columnX = rx + rw - 12.0f;
    const float top = ry + 36.0f, bottom = ry + rh - 12.0f, spacing = 19.0f;
    std::vector<float> ys(labels.size());
    float previous = top - spacing;
    for (size_t i = 0; i < labels.size(); ++i) {
        ys[i] = std::max(std::clamp(labels[i].y, top, bottom), previous + spacing);
        previous = ys[i];
    }
    if (!ys.empty() && ys.back() > bottom) {
        const float shift = ys.back() - bottom;
        for (auto& y : ys) y -= shift;
    }
    for (size_t i = 0; i < labels.size(); ++i) {
        const Label& l = labels[i];
        const char* face = l.selected ? "sans-bold" : "sans";
        nvgFontFace(vg, face);
        nvgFontSize(vg, 14.0f);
        float bounds[4];
        const float textStart = columnX - nvgTextBounds(vg, 0.0f, 0.0f, l.text.c_str(), nullptr, bounds);
        line(l.x, l.y, textStart - 6.0f, ys[i], nvgTransRGBA(l.color, 150), 1.0f);
        text(columnX, ys[i], l.text, l.selected ? nvgRGB(255, 255, 255) : l.color, 14.0f, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, face);
    }

    nvgResetScissor(vg);

    // ---------- Title bar and border ---------- //
    nvgBeginPath(vg); nvgRect(vg, rx, ry, rw, 24.0f);
    nvgFillColor(vg, nvgRGBA(24, 28, 34, 225)); nvgFill(vg);
    const std::string title = "Close-up : " + prefixOf(cg.name) + "     " + (m_mode == Mode::Wireframe ? "Wireframe" : "X-ray");
    text(rx + 10.0f, ry + 12.0f, title, nvgRGB(230, 236, 244), 14.0f, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, "sans-bold");
    text(rx + rw - 10.0f, ry + 12.0f, "drag : orbit    scroll : zoom", nvgRGB(150, 165, 185), 13.0f, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE);

    nvgBeginPath(vg); nvgRect(vg, rx + 0.5f, ry + 0.5f, rw - 1.0f, rh - 1.0f);
    nvgStrokeColor(vg, nvgRGB(58, 70, 88)); nvgStrokeWidth(vg, 1.5f); nvgStroke(vg);

    nvgRestore(vg);
}

bool CloseUpView::mouseButton(const bool pressed, const double x, const double y) {
    if (pressed && m_hasFrame && m_contains(x, y)) {
        m_dragging = true;
        m_lastX = x;
        m_lastY = y;
        return true;
    }
    if (!pressed && m_dragging) {
        m_dragging = false;
        return true;
    }
    return false;
}

bool CloseUpView::mouseMotion(const double x, const double y) {
    if (!m_dragging) return false;
    m_azimuth += (x - m_lastX) * 0.008;
    m_elevation = std::clamp(m_elevation + (y - m_lastY) * 0.008, -1.45, 1.45);
    m_lastX = x;
    m_lastY = y;
    return true;
}

bool CloseUpView::scroll(const double dy, const double x, const double y) {
    if (!m_hasFrame || !m_contains(x, y)) return false;
    // Same direction as the main view: scrolling down moves closer
    m_zoom = std::clamp(m_zoom * std::pow(0.88, -dy), 0.25, 5.0);
    return true;
}
