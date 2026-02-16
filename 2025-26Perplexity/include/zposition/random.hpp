#include <random>

inline std::mt19937& rng() {
    static std::mt19937 eng{ std::random_device{}() };
    return eng;
}

inline float randNormal(float mean, float stddev) {
    std::normal_distribution<float> dist(mean, stddev);
    return dist(rng());
}
