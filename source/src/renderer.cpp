#define GL_SILENCE_DEPRECATION
#include "scene.h"
#include "bvh.h"
#include "timer.h"

#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLFW/glfw3.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <set>
#include <vector>

// ── camera ──────────────────────────────────────────────────────────────────
static float cam_dist   = 200.0f;
static float cam_yaw    = 30.0f;   // degrees
static float cam_pitch  = 25.0f;
static float cam_cx, cam_cy, cam_cz;  // look-at target (scene center)
static bool  mouse_dragging = false;
static double last_mx, last_my;

// ── viz toggles ─────────────────────────────────────────────────────────────
static bool show_bvh       = false;
static int  bvh_max_depth  = 3;
static bool paused         = false;

// ── scene state ─────────────────────────────────────────────────────────────
static Scene scene;
static std::vector<CollisionPair> collisions;
static std::set<int> colliding_set;
static Scenario cur_scenario = Scenario::TWO_CLUSTER;
static int num_objects = 500;

// ─────────────────────────────────────────────────────────────────────────────
static void reset_scene() {
    scene.init(num_objects, cur_scenario);
    collisions.clear();
    colliding_set.clear();
    float ws = 100.0f;
    cam_cx = ws * 0.5f; cam_cy = ws * 0.5f; cam_cz = ws * 0.5f;
}

// ── draw helpers ─────────────────────────────────────────────────────────────
static void draw_cube_at(const Vec3& pos, float half, float r, float g, float b, float a) {
    glPushMatrix();
    glTranslatef(pos.x, pos.y, pos.z);

    glColor4f(r, g, b, a);

    float h = half;
    // 6 faces
    float faces[6][4][3] = {
        {{-h,-h, h},{ h,-h, h},{ h, h, h},{-h, h, h}},  // front
        {{-h,-h,-h},{-h, h,-h},{ h, h,-h},{ h,-h,-h}},  // back
        {{-h, h,-h},{-h, h, h},{ h, h, h},{ h, h,-h}},  // top
        {{-h,-h,-h},{ h,-h,-h},{ h,-h, h},{-h,-h, h}},  // bottom
        {{ h,-h,-h},{ h, h,-h},{ h, h, h},{ h,-h, h}},  // right
        {{-h,-h,-h},{-h,-h, h},{-h, h, h},{-h, h,-h}},  // left
    };
    float normals[6][3] = {
        {0,0,1},{0,0,-1},{0,1,0},{0,-1,0},{1,0,0},{-1,0,0}
    };
    glBegin(GL_QUADS);
    for (int f = 0; f < 6; f++) {
        glNormal3fv(normals[f]);
        for (int v = 0; v < 4; v++)
            glVertex3fv(faces[f][v]);
    }
    glEnd();
    glPopMatrix();
}

static void draw_wire_box(const AABB& box, float r, float g, float b, float a) {
    glColor4f(r, g, b, a);
    Vec3 lo = box.lo, hi = box.hi;
    glBegin(GL_LINES);
    // bottom
    glVertex3f(lo.x,lo.y,lo.z); glVertex3f(hi.x,lo.y,lo.z);
    glVertex3f(hi.x,lo.y,lo.z); glVertex3f(hi.x,lo.y,hi.z);
    glVertex3f(hi.x,lo.y,hi.z); glVertex3f(lo.x,lo.y,hi.z);
    glVertex3f(lo.x,lo.y,hi.z); glVertex3f(lo.x,lo.y,lo.z);
    // top
    glVertex3f(lo.x,hi.y,lo.z); glVertex3f(hi.x,hi.y,lo.z);
    glVertex3f(hi.x,hi.y,lo.z); glVertex3f(hi.x,hi.y,hi.z);
    glVertex3f(hi.x,hi.y,hi.z); glVertex3f(lo.x,hi.y,hi.z);
    glVertex3f(lo.x,hi.y,hi.z); glVertex3f(lo.x,hi.y,lo.z);
    // verticals
    glVertex3f(lo.x,lo.y,lo.z); glVertex3f(lo.x,hi.y,lo.z);
    glVertex3f(hi.x,lo.y,lo.z); glVertex3f(hi.x,hi.y,lo.z);
    glVertex3f(hi.x,lo.y,hi.z); glVertex3f(hi.x,hi.y,hi.z);
    glVertex3f(lo.x,lo.y,hi.z); glVertex3f(lo.x,hi.y,hi.z);
    glEnd();
}

static void draw_tetrahedron_at(const Body& body, float r, float g, float b, float a) {
    // 4 vertices, 4 triangular faces
    static const int faces[4][3] = {{0,2,1},{0,1,3},{0,3,2},{1,2,3}};
    const auto& verts = body.shape.vertices;
    glColor4f(r, g, b, a);
    glBegin(GL_TRIANGLES);
    for (auto& f : faces) {
        Vec3 v0 = verts[f[0]] + body.position;
        Vec3 v1 = verts[f[1]] + body.position;
        Vec3 v2 = verts[f[2]] + body.position;
        Vec3 n = (v1 - v0).cross(v2 - v0).normalized();
        glNormal3f(n.x, n.y, n.z);
        glVertex3f(v0.x, v0.y, v0.z);
        glVertex3f(v1.x, v1.y, v1.z);
        glVertex3f(v2.x, v2.y, v2.z);
    }
    glEnd();
}

static void draw_octahedron_at(const Body& body, float r, float g, float b, float a) {
    // 6 vertices layout: 0=(+s,0,0) 1=(-s,0,0) 2=(0,+s,0) 3=(0,-s,0) 4=(0,0,+s) 5=(0,0,-s)
    // 8 triangular faces
    static const int faces[8][3] = {
        {2,0,4},{2,4,1},{2,1,5},{2,5,0},  // y>0 hemisphere
        {3,4,0},{3,1,4},{3,5,1},{3,0,5},  // y<0 hemisphere
    };
    const auto& verts = body.shape.vertices;
    glColor4f(r, g, b, a);
    glBegin(GL_TRIANGLES);
    for (auto& f : faces) {
        Vec3 v0 = verts[f[0]] + body.position;
        Vec3 v1 = verts[f[1]] + body.position;
        Vec3 v2 = verts[f[2]] + body.position;
        Vec3 n = (v1 - v0).cross(v2 - v0).normalized();
        glNormal3f(n.x, n.y, n.z);
        glVertex3f(v0.x, v0.y, v0.z);
        glVertex3f(v1.x, v1.y, v1.z);
        glVertex3f(v2.x, v2.y, v2.z);
    }
    glEnd();
}

static void draw_icosphere_at(const Body& body, float r, float g, float b, float a) {
    static const int faces[20][3] = {
        {0,1,8}, {0,8,4}, {0,4,5}, {0,5,9}, {0,9,1},
        {1,6,8}, {8,6,10}, {8,10,4}, {4,10,2}, {4,2,5},
        {5,2,11}, {5,11,9}, {9,11,7}, {9,7,1}, {1,7,6},
        {3,6,7}, {3,10,6}, {3,2,10}, {3,11,2}, {3,7,11},
    };
    const auto& verts = body.shape.vertices;
    glColor4f(r, g, b, a);
    glBegin(GL_TRIANGLES);
    for (auto& f : faces) {
        Vec3 v0 = verts[f[0]] + body.position;
        Vec3 v1 = verts[f[1]] + body.position;
        Vec3 v2 = verts[f[2]] + body.position;
        Vec3 n = (v1 - v0).cross(v2 - v0).normalized();
        glNormal3f(n.x, n.y, n.z);
        glVertex3f(v0.x, v0.y, v0.z);
        glVertex3f(v1.x, v1.y, v1.z);
        glVertex3f(v2.x, v2.y, v2.z);
    }
    glEnd();
}

static void draw_body_at(const Body& body, float r, float g, float b, float a) {
    int nv = (int)body.shape.vertices.size();
    if (nv == 4)
        draw_tetrahedron_at(body, r, g, b, a);
    else if (nv == 6)
        draw_octahedron_at(body, r, g, b, a);
    else if (nv == 12)
        draw_icosphere_at(body, r, g, b, a);
    else
        draw_cube_at(body.position, 0.5f, r, g, b, a);
}

static void draw_bvh_recursive(const BVH& bvh, int idx, int depth) {
    if (idx < 0 || idx >= (int)bvh.nodes.size()) return;
    if (depth > bvh_max_depth) return;

    const BVHNode& node = bvh.nodes[idx];

    // Color by depth: green→cyan→blue
    float t = (float)depth / (float)std::max(bvh_max_depth, 1);
    draw_wire_box(node.box, 0.0f, 1.0f - t * 0.5f, t, 0.25f + 0.15f * t);

    if (!node.is_leaf()) {
        draw_bvh_recursive(bvh, node.left, depth + 1);
        draw_bvh_recursive(bvh, node.right, depth + 1);
    }
}

// ── callbacks ────────────────────────────────────────────────────────────────
static void key_callback(GLFWwindow* w, int key, int /*scancode*/, int action, int /*mods*/) {
    if (action != GLFW_PRESS) return;
    switch (key) {
        case GLFW_KEY_ESCAPE: case GLFW_KEY_Q:
            glfwSetWindowShouldClose(w, GLFW_TRUE); break;
        case GLFW_KEY_SPACE:
            paused = !paused; break;
        case GLFW_KEY_B:
            show_bvh = !show_bvh; break;
        case GLFW_KEY_UP:
            bvh_max_depth = std::min(bvh_max_depth + 1, 20); break;
        case GLFW_KEY_DOWN:
            bvh_max_depth = std::max(bvh_max_depth - 1, 0); break;
        case GLFW_KEY_1:
            cur_scenario = Scenario::RANDOM_WALK; reset_scene(); break;
        case GLFW_KEY_2:
            cur_scenario = Scenario::TWO_CLUSTER; reset_scene(); break;
        case GLFW_KEY_3:
            cur_scenario = Scenario::AVALANCHE; reset_scene(); break;
        case GLFW_KEY_R:
            reset_scene(); break;
        default: break;
    }
}

static void scroll_callback(GLFWwindow*, double /*xoff*/, double yoff) {
    cam_dist *= (yoff > 0) ? 0.9f : 1.1f;
    cam_dist = std::max(5.0f, std::min(cam_dist, 1000.0f));
}

static void mouse_button_callback(GLFWwindow* w, int button, int action, int /*mods*/) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        mouse_dragging = (action == GLFW_PRESS);
        if (mouse_dragging) glfwGetCursorPos(w, &last_mx, &last_my);
    }
}

static void cursor_pos_callback(GLFWwindow*, double mx, double my) {
    if (!mouse_dragging) return;
    float dx = (float)(mx - last_mx);
    float dy = (float)(my - last_my);
    cam_yaw   += dx * 0.3f;
    cam_pitch += dy * 0.3f;
    cam_pitch  = std::max(-89.0f, std::min(89.0f, cam_pitch));
    last_mx = mx; last_my = my;
}

static void framebuffer_size_callback(GLFWwindow*, int w, int h) {
    glViewport(0, 0, w, h);
}

// ── HUD ──────────────────────────────────────────────────────────────────────
static void draw_hud(GLFWwindow* window, double detect_ms) {
    int w, h;
    glfwGetFramebufferSize(window, &w, &h);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix(); glLoadIdentity();
    glOrtho(0, w, 0, h, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix(); glLoadIdentity();

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);

    // Semi-transparent background
    glColor4f(0, 0, 0, 0.6f);
    glBegin(GL_QUADS);
    glVertex2f(0, h); glVertex2f(320, h);
    glVertex2f(320, h - 130); glVertex2f(0, h - 130);
    glEnd();

    // We can't easily render text in legacy OpenGL without a font library,
    // so we'll print to the terminal instead and use the title bar.
    const char* sname = "???";
    switch (cur_scenario) {
        case Scenario::RANDOM_WALK: sname = "Random Walk"; break;
        case Scenario::TWO_CLUSTER: sname = "Two Cluster"; break;
        case Scenario::AVALANCHE:   sname = "Avalanche"; break;
    }
    char title[256];
    snprintf(title, sizeof(title),
             "DCD Engine | %s | N=%d | collisions=%zu | detect=%.2fms | BVH:%s(d=%d) | %s",
             sname, num_objects, collisions.size(), detect_ms,
             show_bvh ? "ON" : "OFF", bvh_max_depth,
             paused ? "PAUSED" : "RUNNING");
    glfwSetWindowTitle(window, title);

    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION); glPopMatrix();
    glMatrixMode(GL_MODELVIEW); glPopMatrix();
}

// ── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) num_objects = atoi(argv[++i]);
        else if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
            int s = atoi(argv[++i]);
            if (s == 1) cur_scenario = Scenario::RANDOM_WALK;
            else if (s == 2) cur_scenario = Scenario::TWO_CLUSTER;
            else if (s == 3) cur_scenario = Scenario::AVALANCHE;
        }
    }

    if (!glfwInit()) { fprintf(stderr, "Failed to init GLFW\n"); return 1; }

    glfwWindowHint(GLFW_SAMPLES, 4);
    GLFWwindow* window = glfwCreateWindow(1280, 800, "DCD Engine", nullptr, nullptr);
    if (!window) { glfwTerminate(); return 1; }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);  // vsync

    glfwSetKeyCallback(window, key_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_pos_callback);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // OpenGL setup
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_MULTISAMPLE);
    glClearColor(0.08f, 0.08f, 0.12f, 1.0f);

    // Lighting
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    float light_pos[]   = {1.0f, 2.0f, 1.5f, 0.0f};
    float light_amb[]   = {0.25f, 0.25f, 0.28f, 1.0f};
    float light_diff[]  = {0.85f, 0.85f, 0.80f, 1.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
    glLightfv(GL_LIGHT0, GL_AMBIENT,  light_amb);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  light_diff);

    reset_scene();

    printf("\n── Controls ────────────────────────────\n");
    printf("  Mouse drag   Orbit camera\n");
    printf("  Scroll       Zoom in/out\n");
    printf("  Space        Pause / resume\n");
    printf("  B            Toggle BVH wireframes\n");
    printf("  Up/Down      BVH depth +/-\n");
    printf("  1/2/3        Switch scenario\n");
    printf("  R            Reset scene\n");
    printf("  Q/Esc        Quit\n");
    printf("────────────────────────────────────────\n\n");

    Timer t_detect;
    double detect_ms = 0;

    while (!glfwWindowShouldClose(window)) {
        // ── simulate ──
        if (!paused) {
            scene.step();
            t_detect.start("detect");
            collisions = scene.detect_collisions_seq();
            detect_ms = t_detect.stop();

            colliding_set.clear();
            for (auto& c : collisions) {
                colliding_set.insert(c.a);
                colliding_set.insert(c.b);
            }
        }

        // ── render ──
        int fbw, fbh;
        glfwGetFramebufferSize(window, &fbw, &fbh);
        glViewport(0, 0, fbw, fbh);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Projection
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        float aspect = (float)fbw / std::max(fbh, 1);
        gluPerspective(50.0, aspect, 1.0, 2000.0);

        // Camera
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        float yaw_r = cam_yaw * (float)M_PI / 180.0f;
        float pit_r = cam_pitch * (float)M_PI / 180.0f;
        float eye_x = cam_cx + cam_dist * cosf(pit_r) * sinf(yaw_r);
        float eye_y = cam_cy + cam_dist * sinf(pit_r);
        float eye_z = cam_cz + cam_dist * cosf(pit_r) * cosf(yaw_r);
        gluLookAt(eye_x, eye_y, eye_z, cam_cx, cam_cy, cam_cz, 0, 1, 0);

        // Re-apply light in view space
        glLightfv(GL_LIGHT0, GL_POSITION, light_pos);

        // Draw world boundary
        glDisable(GL_LIGHTING);
        draw_wire_box(scene.bounds, 0.3f, 0.3f, 0.35f, 0.5f);

        // Draw BVH wireframes
        if (show_bvh && !scene.last_bvh.nodes.empty()) {
            draw_bvh_recursive(scene.last_bvh, scene.last_bvh.root, 0);
        }

        // Draw bodies
        glEnable(GL_LIGHTING);
        for (int i = 0; i < (int)scene.bodies.size(); i++) {
            bool hit = colliding_set.count(i) > 0;
            if (hit)
                draw_body_at(scene.bodies[i], 1.0f, 0.2f, 0.15f, 0.95f);
            else
                draw_body_at(scene.bodies[i], 0.35f, 0.65f, 0.9f, 0.85f);
        }

        // HUD (title bar)
        draw_hud(window, detect_ms);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
