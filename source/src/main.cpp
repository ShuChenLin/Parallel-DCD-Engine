#include "scene.h"
#include "timer.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

static const char* scenario_name(Scenario s) {
    switch (s) {
        case Scenario::RANDOM_WALK: return "random_walk";
        case Scenario::TWO_CLUSTER: return "two_cluster";
        case Scenario::AVALANCHE:   return "avalanche";
    }
    return "unknown";
}

static void run_scenario(Scenario scenario, int n, int frames, bool validate) {
    printf("\n=== Scenario: %s | N=%d | Frames=%d ===\n",
           scenario_name(scenario), n, frames);

    Scene scene;
    scene.init(n, scenario);

    Timer t_total, t_step, t_detect;
    double total_step_ms = 0, total_detect_ms = 0;
    int total_collisions = 0;
    Scene::StageTimes acc_stages;  // accumulated across all frames

    t_total.start("Total");
    for (int f = 0; f < frames; f++) {
        t_step.start("Step");
        scene.step();
        total_step_ms += t_step.stop();

        t_detect.start("Detect (seq)");
        auto collisions = scene.detect_collisions_seq();
        total_detect_ms += t_detect.stop();

        // Accumulate per-stage times
        acc_stages.aabb_ms     += scene.stage_times.aabb_ms;
        acc_stages.morton_ms   += scene.stage_times.morton_ms;
        acc_stages.sort_ms     += scene.stage_times.sort_ms;
        acc_stages.build_ms    += scene.stage_times.build_ms;
        acc_stages.traverse_ms += scene.stage_times.traverse_ms;
        acc_stages.gjk_ms      += scene.stage_times.gjk_ms;

        total_collisions += static_cast<int>(collisions.size());

        if (validate && f == 0) {
            auto bf = scene.detect_collisions_bruteforce();
            // Sort both for comparison
            std::sort(collisions.begin(), collisions.end());
            std::sort(bf.begin(), bf.end());
            if (collisions == bf) {
                printf("  [PASS] Frame %d: BVH matches brute-force (%zu collisions)\n",
                       f, collisions.size());
            } else {
                printf("  [FAIL] Frame %d: BVH=%zu vs brute-force=%zu collisions\n",
                       f, collisions.size(), bf.size());
            }
        }

        if (f < 3 || f == frames - 1) {
            printf("  Frame %3d: %zu collisions | detect: %.3f ms\n",
                   f, collisions.size(), t_detect.elapsed_ms);
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

int main(int argc, char** argv) {
    int n = 1000;
    int frames = 10;
    bool validate = true;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) n = atoi(argv[++i]);
        else if (strcmp(argv[i], "-f") == 0 && i + 1 < argc) frames = atoi(argv[++i]);
        else if (strcmp(argv[i], "--no-validate") == 0) validate = false;
    }

    printf("Parallel DCD Engine - Sequential Baseline\n");
    printf("Objects: %d | Frames: %d | Validate: %s\n", n, frames, validate ? "yes" : "no");

    run_scenario(Scenario::RANDOM_WALK, n, frames, validate);
    run_scenario(Scenario::TWO_CLUSTER, n, frames, validate);
    run_scenario(Scenario::AVALANCHE, n, frames, validate);

    return 0;
}
