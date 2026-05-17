#ifndef INTEL_DATA_H
#define INTEL_DATA_H

/**
 * @file IntelData.h
 * @brief Data structure for storing intelligence/sensor data collected during missions.
 * 
 * Encapsulates all relevant information about a single piece of intelligence data:
 * location, timestamp, sensor type, and raw payload (Base64 encoded or binary).
 */

#include "../Common/AppEnums.h"
#include "../Utils/SpatialUtilities.h"
#include <string>
#include <ctime>

namespace AIGD {

/**
 * @class IntelData
 * @brief Represents a single unit of collected intelligence/sensor data.
 * 
 * Each instance stores:
 * - The geographic location where data was collected (Point2D)
 * - The timestamp of collection
 * - The type of sensor (IMINT, SIGINT, or THERMAL)
 * - The raw sensor payload (typically Base64 encoded for binary data)
 * 
 * This class follows the MVC architecture (Model layer) and is managed by
 * the IntelligenceManager (repository pattern).
 */
class IntelData {
private:
    Point2D location_;           ///< Geographic location of the collected data
    long long timestamp_;        ///< Unix timestamp (milliseconds) of collection
    SensorType sensorType_;      ///< Type of sensor that collected this data
    std::string rawData_;        ///< Base64 encoded or raw binary data payload

public:
    /**
     * @brief Default constructor.
     * 
     * Initializes all fields to default values:
     * - location: (0, 0)
     * - timestamp: 0
     * - sensorType: IMINT
     * - rawData: empty string
     */
    IntelData();

    /**
     * @brief Parameterized constructor.
     * 
     * @param location The geographic point where data was collected
     * @param timestamp Unix timestamp (milliseconds) of collection time
     * @param sensorType The type of sensor that collected the data
     * @param rawData Base64 encoded or binary data string
     */
    IntelData(const Point2D& location, long long timestamp, 
              SensorType sensorType, const std::string& rawData);

    /**
     * @brief Copy constructor.
     * @param other The IntelData object to copy
     */
    IntelData(const IntelData& other);

    /**
     * @brief Copy assignment operator.
     * @param other The IntelData object to assign
     * @return Reference to this object
     */
    IntelData& operator=(const IntelData& other);

    /**
     * @brief Destructor.
     */
    ~IntelData() = default;

    // --- Getters ---

    /**
     * @brief Get the location where this data was collected.
     * @return Const reference to the Point2D location
     */
    const Point2D& getLocation() const;

    /**
     * @brief Get the X coordinate of the collection location.
     * @return X coordinate value
     */
    double getLocationX() const;

    /**
     * @brief Get the Y coordinate of the collection location.
     * @return Y coordinate value
     */
    double getLocationY() const;

    /**
     * @brief Get the timestamp of data collection.
     * @return Unix timestamp in milliseconds
     */
    long long getTimestamp() const;

    /**
     * @brief Get the sensor type that collected this data.
     * @return The SensorType enum value
     */
    SensorType getSensorType() const;

    /**
     * @brief Get the sensor type as a human-readable string.
     * @return String representation ("IMINT", "SIGINT", or "THERMAL")
     */
    std::string getSensorTypeString() const;

    /**
     * @brief Get the raw sensor data payload.
     * @return Const reference to the raw data string
     */
    const std::string& getRawData() const;

    /**
     * @brief Get the size of the raw data payload in bytes.
     * @return Number of bytes in the raw data
     */
    size_t getDataSize() const;

    // --- Setters ---

    /**
     * @brief Update the location of this intel data.
     * @param location The new geographic location
     */
    void setLocation(const Point2D& location);

    /**
     * @brief Update the timestamp of this intel data.
     * @param timestamp The new Unix timestamp in milliseconds
     */
    void setTimestamp(long long timestamp);

    /**
     * @brief Update the sensor type.
     * @param sensorType The new sensor type
     */
    void setSensorType(SensorType sensorType);

    /**
     * @brief Update the raw data payload.
     * @param rawData The new Base64 encoded or binary data string
     */
    void setRawData(const std::string& rawData);

    /**
     * @brief Generate a debug string representation of this intel data.
     * @return String containing all relevant information in human-readable format
     */
    std::string toString() const;
};

} // namespace AIGD

#endif // INTEL_DATA_H
