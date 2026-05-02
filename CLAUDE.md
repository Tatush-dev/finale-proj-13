# CLAUDE.md - Project Instructions

## 1. Project Context
- **Name:** Autonomous Intelligence Gathering Drone
- **Objective:** Imagery Intelligence (IMINT) collection in urban environments (Zaytun Quarter model).
- **Core System:** Autonomous drone with Thermal/EO sensors, focusing on navigation and risk mitigation.

## 2. Technical Stack & Architecture
- **Architecture:** MVC (Model-View-Controller) pattern.
- **Model:** Contains navigation logic, Kalman Filter, and Path Planning.
- **View:** Handles SAR/EO data visualization and reporting.
- **Controller:** Manages the "Sense-Think-Act" loop and mission FSM.

## 3. Algorithms to Implement (From Project Book)
- **Localization:** Kalman Filter (Prediction & Correction cycles).
- **Mapping:** Occupancy Grid Mapping (Probabilistic grid for obstacles).
- **Global Planning:** A* Algorithm (Initial path based on distance/risk).
- **Local/Dynamic Planning:** D* Lite (Real-time path updates for new threats).

## 4. Logic & Data Rules
- **Cost Function:** `Cost = Distance + Risk_Factor + Energy_Consumption`.
- **Data Handling:** Use Hashing for O(1) intelligence retrieval.
- **Communication:** Base64 encoding for image/data transfer between components.
- **Mission States:** IDLE, OUTBOUND, RECON, EVADE, RETURN, LANDED.

## 5. Coding Standards
- Follow Clean Code principles.
- Use explicit naming conventions for autonomous logic (e.g., `updateOccupancyGrid`).
- Implement modularity to allow switching between simulation and real sensor data.