#pragma once
#include <string>
#include <vector>

// ─── Coordinates ────────────────────────────────────────────────────────────
struct Coordinates {
    double x;
    double y;
    double altitude;

    Coordinates(double x = 0.0, double y = 0.0, double altitude = 0.0)
        : x(x), y(y), altitude(altitude) {}
};

// ─── Intelligence Data ───────────────────────────────────────────────────────
struct IntelData {
    int         id;
    Coordinates location;
    std::string description;
    std::string timestamp;
    std::string imageDataBase64;  // Base64-encoded sensor image
};

// ─── Threat Data ─────────────────────────────────────────────────────────────
struct ThreatData {
    int         id;
    Coordinates location;
    double      riskFactor;  // Feeds into Cost = Distance + Risk_Factor + Energy
};

// ─── Path / Waypoint ─────────────────────────────────────────────────────────
using Waypoint  = Coordinates;
using Path      = std::vector<Waypoint>;
