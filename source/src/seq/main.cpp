#include "scene.h"
#include "timer.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

<<<<<<< Updated upstream:source/src/seq/main.cpp
static void run_scenario(Scenario scenario, int n, int frames, bool validate) {
    printf("\n=== Scenario: %s | N=%d | Frames=%d ===\n",
           scenario_name(scenario), n, frames);
=======
enum class Method { SEQ, OMP };

static const char* scenario_name(Scenario s) {
    switch (s) {
        case Scenario::RANDOM_WALK: return "random_walk";
        case Scenario::TWO_CLUSTER: return "two_cluster";
        case Scenario::AVALANCHE:   return "avalanche";
    }
    return "unknown";
}

static const char* method_name(Method m) {
    switch (m) {
        case Method::SEQ: return "seq";
        case Method::OMP: return "omp";
    }
    return "unknown";
}

static void run_scenario(Scenario scenario, Method method, int n, int frames,
                         bool validate) {
    printf("\n=== Scenario: %s | Method: %s | N=%d | Frames=%d ===\n",
           scenario_name(scenario), method_name(method), n, frames);
>>>>>>> Stashed changes:source/src/main.cpp

    Scene scene;
    scene.init(n, scenario);

    Timer t_total, t_step, t_detect;
    double total_step_ms = 0, total_detect_ms = 0;
    int total_collisions = 0;
<<<<<<< Updated upstream:source/src/seq/main.cpp
    StageTimes acc_stages;
=======
    Scene::StageTimes acc_stages;
>>>>>>> Stashed changes:source/src/main.cpp

    t_total.start("Total");
    for (int f = 0; f < frames; f++) {
        t_step.start("Step");
        if (method == Method::OMP)
            scene.step_omp();
        else
            scene.step();
        total_step_ms += t_step.stop();

        t_detect.start("Detect");
        std::vector<CollisionPair> collisions;
        if (method == Method::OMP)
            collisions = scene.detect_collisions_omp();
        else
            collisions = scene.detect_collisions_seq();
        total_detect_ms += t_detect.stop();

        acc_stages.aabb_ms     += scene.stage_times.aabb_ms;
        acc_stages.morton_ms   += scene.stage_times.morton_ms;
        acc_stages.sort_ms     += scene.stage_times.sort_ms;
        acc_stages.build_ms    += scene.stage_times.build_ms;
        acc_stages.traverse_ms += scene.stage_times.traverse_ms;
        acc_stages.gjk_ms      += scene.stage_times.gjk_ms;

        total_collisions += static_cast<int>(collisions.size());

        if (validate && f == 0) {
            auto bf = scene.detect_collisions_bruteforce();
            std::sort(collisions.begin(), collisions.end());
            std::sort(bf.begin(), bf.end());
            if (collisions == bf) {
                printf("  [PASS] Frame %d: matches brute-force (%zu collisions)\n",
                       f, collisions.size());
            } else {
                printf("  [FAIL] Frame %d: detected=%zu vs brute-force=%zu collisions\n",
                       f, collisions.size(), bf.size());
            }
        }

        if (f < 3 || f == frames - 1) {
            printf("  Frame %3d: %zu collisions | detect: %.3f ms"
                   " [aabb=%.2f mort=%.2f sort=%.2f build=%.2f trav=%.2f gjk=%.2f]\n",
                   f, collisions.size(), t_detect.elapsed_ms,
                   scene.stage_times.aabb_ms, scene.stage_times.morton_ms,
                   scene.stage_times.sort_ms, scene.stage_times.build_ms,
                   scene.stage_times.traverse_ms, scene.stage_times.gjk_ms);
        }
    }
    double total_ms = t_total.stop();

    printf("  ----- Summary -----\n");
    printf("  Total time:        %8.1f ms\n", total_ms);
    printf("  Avg step time:     %8.3f ms\n", total_step_ms / frames);
    printf("  Avg detect time:   %8.3f ms\n", total_detect_ms / frames);
    printf("  Total collisions:  %d (across %d frames)\n", total_collisions, frames);
    printf("  ----- Avg stage breakdown (%d frames) -----\n", frames);
    printf("    AABB update:     %8.3f ms\n", acc_stages.aabb_ms     / frames);
    printf("    Morton codes:    %8.3f ms\n", acc_stages.morton_ms   / frames);
    printf("    Sort:            %8.3f ms\n", acc_stages.sort_ms     / frames);
    printf("    BVH build:       %8.3f ms\n", acc_stages.build_ms    / frames);
    printf("    BVH traverse:    %8.3f ms\n", acc_stages.traverse_ms / frames);
    printf("    GJK narrow:      %8.3f ms\n", acc_stages.gjk_ms      / frames);
}

static void print_usage(const char* prog) {
    printf("Usage: %s [-n N] [-f frames] [-m seq|omp] [--no-validate]\n", prog);
}

int main(int argc, char** argv) {
    int n = 1000;
    int frames = 10;
    bool validate = true;
    Method method = Method::SEQ;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) {
            n = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-f") == 0 && i + 1 < argc) {
            frames = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-m") == 0 && i + 1 < argc) {
            i++;
            if (strcmp(argv[i], "seq") == 0) method = Method::SEQ;
            else if (strcmp(argv[i], "omp") == 0) method = Method::OMP;
            else { print_usage(argv[0]); return 1; }
        } else if (strcmp(argv[i], "--no-validate") == 0) {
            validate = false;
        } else {
            print_usage(argv[0]);
            return 1;
        }
    }

    printf("Parallel DCD Engine\n");
    printf("Method: %s | Objects: %d | Frames: %d | Validate: %s\n",
           method_name(method), n, frames, validate ? "yes" : "no");

    run_scenario(Scenario::RANDOM_WALK, method, n, frames, validate);
    run_scenario(Scenario::TWO_CLUSTER, method, n, frames, validate);
    run_scenario(Scenario::AVALANCHE,   method, n, frames, validate);

    return 0;
}
