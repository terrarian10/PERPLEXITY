#include <random>

inline std::mt19937& rng() {
    static std::mt19937 eng{ std::random_device{}() }; // seeded once
    return eng;
}

inline float randNormal(float mean, float stddev) {
    std::normal_distribution<float> dist(mean, stddev); // cheap to create
    return dist(rng());
}
