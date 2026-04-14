#include "scene.h"
#include "timer.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <omp.h>

static void run_scenario(Scenario scenario, int n, int frames, bool validate) {
    printf("\n=== Scenario: %s | N=%d | Frames=%d | Threads=%d ===\n",
           scenario_name(scenario), n, frames, omp_get_max_threads());

    Scene scene;
    scene.init(n, scenario);

    Timer t_total, t_step, t_detect;
    double total_step_ms = 0, total_detect_ms = 0;
    int total_collisions = 0;
    StageTimes acc_stages;

    t_total.start("Total");
    for (int f = 0; f < frames; f++) {
        t_step.start("Step");
        scene.step_omp();
        total_step_ms += t_step.stop();

        t_detect.start("Detect (omp)");
        auto collisions = scene.detect_collisions_omp();
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
                printf("  [PASS] Frame %d: BVH matches brute-force (%zu collisions)\n",
                       f, collisions.size());
            } else {
                printf("  [FAIL] Frame %d: BVH=%zu vs brute-force=%zu collisions\n",
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

int main(int argc, char** argv) {
    int n = 1000;
    int frames = 10;
    bool validate = true;
    int threads = 0;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) n = atoi(argv[++i]);
        else if (strcmp(argv[i], "-f") == 0 && i + 1 < argc) frames = atoi(argv[++i]);
        else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) threads = atoi(argv[++i]);
        else if (strcmp(argv[i], "--no-validate") == 0) validate = false;
    }

    if (threads > 0) omp_set_num_threads(threads);

    printf("Parallel DCD Engine - OpenMP\n");
    printf("Objects: %d | Frames: %d | Threads: %d | Validate: %s\n",
           n, frames, omp_get_max_threads(), validate ? "yes" : "no");

    run_scenario(Scenario::RANDOM_WALK, n, frames, validate);
    run_scenario(Scenario::TWO_CLUSTER, n, frames, validate);
    run_scenario(Scenario::AVALANCHE, n, frames, validate);

    return 0;
}
