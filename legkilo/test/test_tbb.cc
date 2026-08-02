// Simple TBB timing example: compare serial vs parallel element-wise sum.

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <chrono>
#include <iostream>
#include <random>
#include <vector>

int main() {
    using clock = std::chrono::high_resolution_clock;

    const std::size_t N = 100'000'000;  // adjust size if needed
    std::vector<float> a(N), b(N), c(N, 0.0f);

    // Initialize with random data
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    for (std::size_t i = 0; i < N; ++i) {
        a[i] = dist(rng);
        b[i] = dist(rng);
    }

    // Serial
    auto t0 = clock::now();
    for (std::size_t i = 0; i < N; ++i) { c[i] = a[i] + b[i]; }
    auto t1 = clock::now();
    double ms_serial = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // Parallel with TBB
    std::fill(c.begin(), c.end(), 0.0f);
    auto t2 = clock::now();
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, N), [&](const tbb::blocked_range<std::size_t>& r) {
        for (std::size_t i = r.begin(); i < r.end(); ++i) { c[i] = a[i] + b[i]; }
    });
    auto t3 = clock::now();
    double ms_parallel = std::chrono::duration<double, std::milli>(t3 - t2).count();

    // simple checksum to keep compiler honest
    double checksum = 0.0;
    for (std::size_t i = 0; i < 10; ++i) checksum += c[i];

    std::cout << "Serial  time:   " << ms_serial << " ms\n";
    std::cout << "Parallel time:  " << ms_parallel << " ms\n";
    std::cout << "Speedup:        " << (ms_serial / ms_parallel) << "x\n";
    std::cout << "Checksum(first 10): " << checksum << "\n";
    return 0;
}

