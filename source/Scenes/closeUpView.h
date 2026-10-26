/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_CLOSEUPVIEW_H
#define GRS_SIMULATOR_CLOSEUPVIEW_H

#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

#include <nanogui/opengl.h>

#include "OpenglFramework/mesh.h"
#include "OpenglFramework/shader.h"
#include "RigidBodyAndLink/SceneObjects/physicsObject.h"
#include "Definitions/renderObjectDefinitions.h"

/*
 * Feature and silhouette edges of a mesh, drawn as lines.
 * Feature edges (sharp creases and open borders) are computed once,
 * silhouette edges are recomputed on the CPU for each camera position.
 */
class WireframeEdges {
public:
    explicit WireframeEdges(const Mesh& mesh);
    ~WireframeEdges();

    WireframeEdges(const WireframeEdges&) = delete;
    WireframeEdges& operator=(const WireframeEdges&) = delete;

    // Camera position in mesh coordinates
    void updateSilhouettes(const grs::Vec3d& cameraPositionLocal);

    void drawFeatureEdges() const;
    void drawSilhouetteEdges() const;

    [[nodiscard]] double boundingRadius() const {return m_boundingRadius;};

private:
    struct SmoothEdge {
        uint32_t v0, v1;
        uint32_t f0, f1;
        bool flipped;   // the two faces are wound inconsistently (normals point to opposite sides)
    };

    std::vector<grs::Vec3f> m_vertices;
    std::vector<grs::Vec3f> m_faceNormals;
    std::vector<grs::Vec3f> m_faceCenters;
    std::vector<SmoothEdge> m_smoothEdges;
    std::vector<uint8_t> m_frontFacing;
    std::vector<grs::Vec3f> m_silhouetteLines;

    double m_boundingRadius = 1.0;

    GLuint m_featureVao = 0;
    GLuint m_featureVbo = 0;
    GLsizei m_featureVertexCount = 0;

    GLuint m_silhouetteVao = 0;
    GLuint m_silhouetteVbo = 0;
    GLsizei m_silhouetteVertexCount = 0;
    size_t m_silhouetteCapacity = 0;

    static void m_createLineBuffer(GLuint& vao, GLuint& vbo);
};

/*
 * Close-up inset: renders one body at true scale from a body-fixed orbit camera,
 * with its frames, lever arms from the CG (wireframe mode) or frame triads and
 * force vectors (x-ray mode). Labels and markers are drawn with NanoVG.
 */
class CloseUpView {
public:
    enum class Mode { Wireframe, XRay };

    using SkyRenderer = std::function<void(const grs::Matrix4d& worldToCamera, const grs::Matrix4d& projection, float brightness)>;

    explicit CloseUpView(const std::string& executablePath);
    ~CloseUpView();

    void setMode(Mode mode) {m_mode = mode;};
    [[nodiscard]] Mode mode() const {return m_mode;};

    // GL pass, called after the main scene. Framebuffer size in pixels.
    void render(PhysicsObject& object, int selectedFrame, int framebufferWidth, int framebufferHeight, float pixelRatio,
                const Shader& colorShader, Shader& depthShader, const SkyRenderer& drawSky);

    // NanoVG pass (window coordinates), between nvgBeginFrame and nvgEndFrame
    void drawOverlay(NVGcontext* vg) const;

    // Mouse in window coordinates; return true when the event was used by the inset
    bool mouseButton(bool pressed, double x, double y);
    bool mouseMotion(double x, double y);
    bool scroll(double dy, double x, double y);

    // Forget cached edges (scene objects were recreated) / stop drawing and capturing the mouse
    void forgetObjects() {m_wireframes.clear();};
    void hide() {m_hasFrame = false; m_dragging = false;};

private:
    Mode m_mode = Mode::Wireframe;
    Shader m_xrayShader;

    std::unordered_map<const PhysicsObject*, std::unique_ptr<WireframeEdges>> m_wireframes;

    // Orbit camera, in body (mesh) axes
    double m_azimuth = 38.0 * M_PI / 180.0;
    double m_elevation = 22.0 * M_PI / 180.0;
    double m_zoom = 1.0;
    bool m_dragging = false;
    double m_lastX = 0.0;
    double m_lastY = 0.0;

    // State of the last render, used by the overlay and the mouse handling
    bool m_hasFrame = false;
    bodyRenderData m_body{};
    int m_selectedFrame = -1;
    double m_bodyRadius = 1.0;
    grs::Matrix4d m_viewMatrix = grs::Matrix4d::identity();
    grs::Matrix4d m_projectionMatrix = grs::Matrix4d::identity();
    int m_framebufferHeight = 1;
    float m_pixelRatio = 1.0f;
    int m_rectX = 0, m_rectY = 0, m_rectWidth = 1, m_rectHeight = 1; // framebuffer pixels, origin bottom-left

    [[nodiscard]] bool m_contains(double x, double y) const;
    [[nodiscard]] bool m_project(const grs::Vec3d& world, float& x, float& y) const;
    WireframeEdges& m_wireframeFor(const PhysicsObject& object);
};

#endif //GRS_SIMULATOR_CLOSEUPVIEW_H
