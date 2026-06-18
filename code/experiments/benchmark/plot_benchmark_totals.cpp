#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

struct PlotRow {
    int threads = 0;
    int resolution = 0;
    double totalMs = 0.0;
};

std::vector<std::string> splitCSVLine(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string field;
    while (std::getline(ss, field, ',')) {
        fields.push_back(field);
    }
    return fields;
}

std::string xmlEscape(const std::string& value) {
    std::string out;
    out.reserve(value.size());
    for (char c : value) {
        switch (c) {
            case '&': out += "&amp;"; break;
            case '<': out += "&lt;"; break;
            case '>': out += "&gt;"; break;
            case '"': out += "&quot;"; break;
            case '\'': out += "&apos;"; break;
            default: out += c; break;
        }
    }
    return out;
}

std::vector<PlotRow> loadRows(const fs::path& csvPath) {
    std::ifstream input(csvPath);
    if (!input) {
        throw std::runtime_error("no se pudo abrir el CSV: " + csvPath.string());
    }

    std::string headerLine;
    if (!std::getline(input, headerLine)) {
        throw std::runtime_error("el CSV esta vacio");
    }

    std::vector<std::string> headers = splitCSVLine(headerLine);
    int threadsIndex = -1;
    int resolutionIndex = -1;
    int totalMsIndex = -1;

    for (size_t i = 0; i < headers.size(); ++i) {
        if (headers[i] == "threads") {
            threadsIndex = static_cast<int>(i);
        } else if (headers[i] == "resolution") {
            resolutionIndex = static_cast<int>(i);
        } else if (headers[i] == "total_ms") {
            totalMsIndex = static_cast<int>(i);
        }
    }

    if (threadsIndex < 0 || resolutionIndex < 0 || totalMsIndex < 0) {
        throw std::runtime_error("el CSV no contiene las columnas threads, resolution y total_ms");
    }

    std::vector<PlotRow> rows;
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty()) {
            continue;
        }

        std::vector<std::string> fields = splitCSVLine(line);
        if (static_cast<int>(fields.size()) <= std::max({threadsIndex, resolutionIndex, totalMsIndex})) {
            continue;
        }

        PlotRow row;
        row.threads = std::stoi(fields[threadsIndex]);
        row.resolution = std::stoi(fields[resolutionIndex]);
        row.totalMs = std::stod(fields[totalMsIndex]);
        if (row.totalMs > 0.0) {
            rows.push_back(row);
        }
    }

    if (rows.empty()) {
        throw std::runtime_error("el CSV no contiene filas validas para graficar");
    }

    return rows;
}

double mapX(int threads,
            const std::vector<int>& threadValues,
            double left,
            double plotWidth) {
    if (threadValues.size() == 1) {
        return left + plotWidth * 0.5;
    }

    auto it = std::find(threadValues.begin(), threadValues.end(), threads);
    size_t index = static_cast<size_t>(std::distance(threadValues.begin(), it));
    return left + plotWidth * static_cast<double>(index) / static_cast<double>(threadValues.size() - 1);
}

double mapY(double value,
            double minLog,
            double maxLog,
            double top,
            double plotHeight) {
    double logValue = std::log10(value);
    double t = (logValue - minLog) / (maxLog - minLog);
    return top + plotHeight * (1.0 - t);
}

std::string formatDouble(double value, int decimals = 2) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(decimals) << value;
    return out.str();
}

void writeSVG(const std::vector<PlotRow>& rows, const fs::path& outputPath) {
    const int width = 1400;
    const int height = 900;
    const double left = 120.0;
    const double right = 320.0;
    const double top = 100.0;
    const double bottom = 110.0;
    const double plotWidth = width - left - right;
    const double plotHeight = height - top - bottom;

    std::set<int> threadSet;
    std::map<int, std::vector<std::pair<int, double>>> grouped;
    double minValue = std::numeric_limits<double>::max();
    double maxValue = std::numeric_limits<double>::lowest();

    for (const PlotRow& row : rows) {
        threadSet.insert(row.threads);
        grouped[row.resolution].push_back({row.threads, row.totalMs});
        minValue = std::min(minValue, row.totalMs);
        maxValue = std::max(maxValue, row.totalMs);
    }

    std::vector<int> threadValues(threadSet.begin(), threadSet.end());
    if (threadValues.empty() || minValue <= 0.0 || maxValue <= 0.0) {
        throw std::runtime_error("no hay datos positivos para construir la grafica");
    }

    double minLog = std::floor(std::log10(minValue));
    double maxLog = std::ceil(std::log10(maxValue));
    if (minLog == maxLog) {
        maxLog += 1.0;
    }

    const std::vector<std::string> colors = {
        "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728",
        "#9467bd", "#8c564b", "#e377c2", "#17becf"
    };

    std::ofstream out(outputPath);
    if (!out) {
        throw std::runtime_error("no se pudo crear el SVG: " + outputPath.string());
    }

    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width
        << "\" height=\"" << height << "\" viewBox=\"0 0 " << width << ' ' << height << "\">\n";
    out << "<rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n";
    out << "<style>\n"
        << "text { font-family: Arial, sans-serif; fill: #111827; }\n"
        << ".grid { stroke: #d1d5db; stroke-dasharray: 6 6; }\n"
        << ".axis { stroke: #111827; stroke-width: 2; }\n"
        << ".tick { stroke: #111827; stroke-width: 1.5; }\n"
        << ".series { fill: none; stroke-width: 3; }\n"
        << ".point { stroke: white; stroke-width: 1.5; }\n"
        << "</style>\n";

    out << "<text x=\"" << width / 2 << "\" y=\"50\" text-anchor=\"middle\" font-size=\"30\" font-weight=\"600\">"
        << xmlEscape("Tiempo total del proyecto por cantidad de hilos") << "</text>\n";

    for (int exponent = static_cast<int>(minLog); exponent <= static_cast<int>(maxLog); ++exponent) {
        double value = std::pow(10.0, exponent);
        double y = mapY(value, minLog, maxLog, top, plotHeight);
        out << "<line class=\"grid\" x1=\"" << left << "\" y1=\"" << y
            << "\" x2=\"" << (left + plotWidth) << "\" y2=\"" << y << "\"/>\n";
        out << "<line class=\"tick\" x1=\"" << (left - 8) << "\" y1=\"" << y
            << "\" x2=\"" << left << "\" y2=\"" << y << "\"/>\n";
        out << "<text x=\"" << (left - 18) << "\" y=\"" << (y + 6)
            << "\" text-anchor=\"end\" font-size=\"20\">10^" << exponent << "</text>\n";
    }

    for (int threads : threadValues) {
        double x = mapX(threads, threadValues, left, plotWidth);
        out << "<line class=\"grid\" x1=\"" << x << "\" y1=\"" << top
            << "\" x2=\"" << x << "\" y2=\"" << (top + plotHeight) << "\"/>\n";
        out << "<line class=\"tick\" x1=\"" << x << "\" y1=\"" << (top + plotHeight)
            << "\" x2=\"" << x << "\" y2=\"" << (top + plotHeight + 8) << "\"/>\n";
        out << "<text x=\"" << x << "\" y=\"" << (top + plotHeight + 36)
            << "\" text-anchor=\"middle\" font-size=\"20\">" << threads << "</text>\n";
    }

    out << "<line class=\"axis\" x1=\"" << left << "\" y1=\"" << top
        << "\" x2=\"" << left << "\" y2=\"" << (top + plotHeight) << "\"/>\n";
    out << "<line class=\"axis\" x1=\"" << left << "\" y1=\"" << (top + plotHeight)
        << "\" x2=\"" << (left + plotWidth) << "\" y2=\"" << (top + plotHeight) << "\"/>\n";

    out << "<text x=\"" << (left + plotWidth / 2.0) << "\" y=\"" << (height - 30)
        << "\" text-anchor=\"middle\" font-size=\"24\">Numero de threads</text>\n";
    out << "<text x=\"40\" y=\"" << (top + plotHeight / 2.0)
        << "\" text-anchor=\"middle\" font-size=\"24\" transform=\"rotate(-90 40 "
        << (top + plotHeight / 2.0) << ")\">Tiempo total (ms)</text>\n";

    size_t colorIndex = 0;
    for (auto& [resolution, points] : grouped) {
        std::sort(points.begin(), points.end(), [](const auto& a, const auto& b) {
            return a.first < b.first;
        });

        const std::string& color = colors[colorIndex % colors.size()];
        ++colorIndex;

        out << "<polyline class=\"series\" stroke=\"" << color << "\" points=\"";
        for (const auto& [threads, totalMs] : points) {
            double x = mapX(threads, threadValues, left, plotWidth);
            double y = mapY(totalMs, minLog, maxLog, top, plotHeight);
            out << formatDouble(x) << "," << formatDouble(y) << " ";
        }
        out << "\"/>\n";

        for (const auto& [threads, totalMs] : points) {
            double x = mapX(threads, threadValues, left, plotWidth);
            double y = mapY(totalMs, minLog, maxLog, top, plotHeight);
            out << "<circle class=\"point\" cx=\"" << formatDouble(x)
                << "\" cy=\"" << formatDouble(y) << "\" r=\"6\" fill=\"" << color << "\"/>\n";
        }
    }

    const double legendX = left + plotWidth + 40.0;
    const double legendY = top + 20.0;
    const double legendLine = 34.0;
    out << "<rect x=\"" << (legendX - 20.0) << "\" y=\"" << (legendY - 30.0)
        << "\" width=\"240\" height=\"" << (grouped.size() * legendLine + 40.0)
        << "\" rx=\"12\" fill=\"white\" stroke=\"#d1d5db\"/>\n";

    colorIndex = 0;
    size_t legendIndex = 0;
    for (const auto& [resolution, points] : grouped) {
        const std::string& color = colors[colorIndex % colors.size()];
        ++colorIndex;
        double y = legendY + legendIndex * legendLine;
        out << "<line x1=\"" << legendX << "\" y1=\"" << y
            << "\" x2=\"" << (legendX + 38.0) << "\" y2=\"" << y
            << "\" stroke=\"" << color << "\" stroke-width=\"4\"/>\n";
        out << "<circle cx=\"" << (legendX + 19.0) << "\" cy=\"" << y
            << "\" r=\"5\" fill=\"" << color << "\" stroke=\"white\" stroke-width=\"1.5\"/>\n";
        out << "<text x=\"" << (legendX + 52.0) << "\" y=\"" << (y + 7.0)
            << "\" font-size=\"20\">Resolucion " << resolution << "</text>\n";
        ++legendIndex;
    }

    out << "</svg>\n";
}

void printUsage(const char* programName) {
    std::cout << "Uso: " << programName << " <input.csv> <output.svg>\n";
}

} // namespace

int main(int argc, char* argv[]) {
    try {
        if (argc != 3) {
            printUsage(argv[0]);
            return 1;
        }

        fs::path inputPath = argv[1];
        fs::path outputPath = argv[2];

        std::vector<PlotRow> rows = loadRows(inputPath);
        fs::create_directories(outputPath.parent_path());
        writeSVG(rows, outputPath);

        std::cout << "Grafico generado en: " << outputPath << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
