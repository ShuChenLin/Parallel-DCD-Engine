#include "scene.h"
#include "timer.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <cuda_runtime.h>

static const char* scenario_name(Scenario s) {
    switch (s) {
        case Scenario::RANDOM_WALK: return "random_walk";
        case Scenario::TWO_CLUSTER: return "two_cluster";
        case Scenario::AVALANCHE:   return "avalanche";
    }
    return "unknown";
}

static void run_scenario(Scenario scenario, int n, int frames, bool validate) {
    printf("\n=== Scenario: %s | N=%d | Frames=%d | CUDA ===\n",
           scenario_name(scenario), n, frames);

    Scene scene;
    scene.init(n, scenario);

    Timer t_total, t_step, t_detect;
    double total_step_ms = 0, total_detect_ms = 0;
    int total_collisions = 0;

    t_total.start("Total");
    for (int f = 0; f < frames; f++) {
        t_step.start("Step");
        scene.step_cuda();
        total_step_ms += t_step.stop();

        t_detect.start("Detect (cuda)");
        auto collisions = scene.detect_collisions_cuda();
        total_detect_ms += t_detect.stop();

        total_collisions += (int)collisions.size();

        if (validate && f == 0) {
            auto bf = scene.detect_collisions_bruteforce();
            std::sort(collisions.begin(), collisions.end());
            std::sort(bf.begin(), bf.end());
            if (collisions == bf) {
                printf("  [PASS] Frame %d: CUDA matches brute-force (%zu collisions)\n",
                       f, collisions.size());
            } else {
                printf("  [FAIL] Frame %d: CUDA=%zu vs brute-force=%zu collisions\n",
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

    /* Print GPU info */
    int dev = 0;
    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, dev);
    printf("Parallel DCD Engine - CUDA\n");
    printf("GPU: %s | SMs: %d | Mem: %zu MB\n",
           prop.name, prop.multiProcessorCount,
           prop.totalGlobalMem / (1024 * 1024));
    printf("Objects: %d | Frames: %d | Validate: %s\n",
           n, frames, validate ? "yes" : "no");

    run_scenario(Scenario::RANDOM_WALK, n, frames, validate);
    run_scenario(Scenario::TWO_CLUSTER, n, frames, validate);
    run_scenario(Scenario::AVALANCHE, n, frames, validate);

    return 0;
}
