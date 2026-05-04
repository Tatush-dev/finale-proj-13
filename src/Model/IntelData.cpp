#include "IntelData.h"
#include <sstream>
#include <iomanip>

namespace AIGD {

// --- Constructors ---

IntelData::IntelData() 
    : location_(0.0, 0.0), timestamp_(0), sensorType_(SensorType::IMINT), rawData_("") {
}

IntelData::IntelData(const Point2D& location, long long timestamp,
                     SensorType sensorType, const std::string& rawData)
    : location_(location), timestamp_(timestamp), sensorType_(sensorType), rawData_(rawData) {
}

IntelData::IntelData(const IntelData& other)
    : location_(other.location_), timestamp_(other.timestamp_),
      sensorType_(other.sensorType_), rawData_(other.rawData_) {
}

IntelData& IntelData::operator=(const IntelData& other) {
    if (this != &other) {
        location_ = other.location_;
        timestamp_ = other.timestamp_;
        sensorType_ = other.sensorType_;
        rawData_ = other.rawData_;
    }
    return *this;
}

// --- Getters ---

const Point2D& IntelData::getLocation() const {
    return location_;
}

double IntelData::getLocationX() const {
    return location_.x;
}

double IntelData::getLocationY() const {
    return location_.y;
}

long long IntelData::getTimestamp() const {
    return timestamp_;
}

SensorType IntelData::getSensorType() const {
    return sensorType_;
}

std::string IntelData::getSensorTypeString() const {
    switch (sensorType_) {
        case SensorType::IMINT:
            return "IMINT";
        case SensorType::SIGNAL:
            return "SIGNAL";
        case SensorType::THERMAL:
            return "THERMAL";
        default:
            return "UNKNOWN";
    }
}

const std::string& IntelData::getRawData() const {
    return rawData_;
}

size_t IntelData::getDataSize() const {
    return rawData_.size();
}

// --- Setters ---

void IntelData::setLocation(const Point2D& location) {
    location_ = location;
}

void IntelData::setTimestamp(long long timestamp) {
    timestamp_ = timestamp;
}

void IntelData::setSensorType(SensorType sensorType) {
    sensorType_ = sensorType;
}

void IntelData::setRawData(const std::string& rawData) {
    rawData_ = rawData;
}

// --- Utility Methods ---

std::string IntelData::toString() const {
    std::ostringstream oss;
    oss << "IntelData {\n"
        << "  Location: " << location_ << "\n"
        << "  Timestamp: " << timestamp_ << " ms\n"
        << "  Sensor Type: " << getSensorTypeString() << "\n"
        << "  Data Size: " << getDataSize() << " bytes\n"
        << "}";
    return oss.str();
}

} // namespace AIGD
