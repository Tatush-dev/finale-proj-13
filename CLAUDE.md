# CLAUDE.md - Project Instructions

## 1. Project Context
- **Name:** Autonomous Intelligence Gathering Drone
- **Objective:** Imagery Intelligence (IMINT) collection in urban environments (Zaytun Quarter model).
- **Core System:** Autonomous drone with Thermal/EO sensors, focusing on navigation and risk mitigation.

## 2. Technical Stack & Architecture
- **Architecture:** MVC (Model-View-Controller) pattern.
- **Primary Language:** C++ / Java / C# (Use strict OOP principles).
- **Model:** Contains navigation logic, Kalman Filter, and Path Planning.
- **View:** Handles SAR/EO data visualization and reporting.
- **Controller:** Manages the "Sense-Think-Act" loop and mission FSM.

## 3. Core Classes & Functions (Per Project Book)
### Model
- `NavigationModel`:
  - `CalculateInitialPath(start, target)` -> Uses A*.
  - `UpdateDynamicPath(currentLocation, newThreat)` -> Uses D* Lite.
- `KalmanFilter`:
  - `Predict(motionModel)`
  - `Update(sensorMeasurements)`
- `IntelligenceManager`:
  - `StoreIntel(IntelData)` -> O(1) insertion using HashTable.
  - `RetrieveIntel(Location)`
  - `GetAllIntel()` -> Returns List<IntelData>.
- `OccupancyGrid`:
  - `UpdateGridProbability(cell, sensorData)`

### Controller
- `MissionController` (The FSM Manager):
  - `StartMission()`
  - `ExecuteSenseThinkActLoop()` -> The main Outbound/Recon loop.
  - `HandleThreatDetection()` -> Triggers EVADE state and D* Lite recalculation.

### View
- `MissionView`:
  - `UpdateDronePosition(Coordinates)`
  - `DisplayAlert(String message)`
  - `GenerateFinalReport(List<IntelData> finalIntel)`

## 4. Key Data Structures
- **WeightedGraph:** Represented using an **Adjacency List** for the urban map.
- **PriorityQueue:** Used as the engine for A* and D* Lite algorithms (Open List).
- **HashSet (Set):** Used for the `ClosedList` in pathfinding for O(1) lookups.
- **Queue (FIFO):** Used to store and execute the calculated flight path `Waypoints`.
- **Dynamic List:** Used to accumulate dynamic threat and sensor objects.
- **HashTable / Unordered Map:** Core of the `IntelligenceManager` class.

## 5. Logic & Domain Rules
- **Cost Function:** `Cost = Distance + Risk_Factor + Energy_Consumption`.
- **Communication:** Base64 encoding for image/data transfer.
- **Mission States:** IDLE, OUTBOUND, RECON, EVADE, RETURN, LANDED.

## 6. Coding Standards
- Follow Clean Code principles.
- Implement modularity to allow switching between simulation and real sensor data.
- Add descriptive comments for complex algorithmic steps (especially in A*, D* Lite, and Kalman).