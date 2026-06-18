#pragma once

#include <chrono>
#include <string>

// ============================================================================
// Timer de alta resolución para benchmarking
// ============================================================================

class Timer {
public:
    Timer() = default;

    /// Inicia la medición
    void start() {
        m_start = std::chrono::high_resolution_clock::now();
    }

    /// Detiene la medición
    void stop() {
        m_end = std::chrono::high_resolution_clock::now();
    }

    /// Retorna el tiempo transcurrido en milisegundos
    double elapsedMs() const {
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(m_end - m_start);
        return duration.count() / 1000.0;
    }

    /// Retorna el tiempo transcurrido en microsegundos
    double elapsedUs() const {
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(m_end - m_start);
        return static_cast<double>(duration.count());
    }

private:
    std::chrono::high_resolution_clock::time_point m_start;
    std::chrono::high_resolution_clock::time_point m_end;
};
