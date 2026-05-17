#ifndef SPATIAL_UTILITIES_H
#define SPATIAL_UTILITIES_H

/**
 * @file SpatialUtilities.h
 * @brief Spatial data structures for coordinate and vector operations.
 * 
 * Provides Point2D for representing geographic coordinates and Vector2D for
 * vector operations (addition, subtraction, magnitude calculations).
 */

#include <cmath>
#include <ostream>

namespace AIGD {

/**
 * @struct Point2D
 * @brief Represents a 2D coordinate point in the urban environment.
 * 
 * Used for drone location, target coordinates, and threat positions.
 * Coordinates are typically in meters relative to mission origin.
 */
struct Point2D {
    double x;  ///< X coordinate (typically East-West in meters)
    double y;  ///< Y coordinate (typically North-South in meters)

    /**
     * @brief Default constructor initializing to origin (0, 0).
     */
    Point2D() : x(0.0), y(0.0) {}

    /**
     * @brief Parameterized constructor.
     * @param x X coordinate value
     * @param y Y coordinate value
     */
    Point2D(double x, double y) : x(x), y(y) {}

    /**
     * @brief Equality comparison operator.
     * @param other The point to compare with
     * @return true if coordinates are equal within floating point precision
     */
    bool operator==(const Point2D& other) const {
        const double epsilon = 1e-9;
        return std::abs(x - other.x) < epsilon && std::abs(y - other.y) < epsilon;
    }

    /**
     * @brief Inequality comparison operator.
     * @param other The point to compare with
     * @return true if coordinates are not equal
     */
    bool operator!=(const Point2D& other) const {
        return !(*this == other);
    }

    /**
     * @brief Addition operator for vector translation.
     * @param vec The vector to add
     * @return New point at the translated location
     */
    Point2D operator+(const class Vector2D& vec) const;

    /**
     * @brief Subtraction operator to get vector between points.
     * @param other The point to subtract
     * @return Vector from other point to this point
     */
    Vector2D operator-(const Point2D& other) const;

    /**
     * @brief Stream output operator for debugging.
     * @param os Output stream
     * @param p The point to output
     * @return Reference to the output stream
     */
    friend std::ostream& operator<<(std::ostream& os, const Point2D& p);
};

/**
 * @class Vector2D
 * @brief Represents a 2D vector with magnitude and direction operations.
 * 
 * Supports vector arithmetic (addition, subtraction) and distance calculations
 * needed for path planning and threat assessment.
 */
class Vector2D {
private:
    double dx;  ///< X component (delta x)
    double dy;  ///< Y component (delta y)

public:
    /**
     * @brief Default constructor initializing to zero vector.
     */
    Vector2D() : dx(0.0), dy(0.0) {}

    /**
     * @brief Parameterized constructor.
     * @param dx X component value
     * @param dy Y component value
     */
    Vector2D(double dx, double dy) : dx(dx), dy(dy) {}

    /**
     * @brief Get the X component of the vector.
     * @return The X component value
     */
    double getDX() const { return dx; }

    /**
     * @brief Get the Y component of the vector.
     * @return The Y component value
     */
    double getDY() const { return dy; }

    /**
     * @brief Calculate the magnitude (length) of the vector.
     * 
     * Uses the Euclidean distance formula: magnitude = sqrt(dx^2 + dy^2)
     * Commonly used for distance calculations in path planning.
     * 
     * @return The magnitude of the vector in meters
     */
    double magnitude() const {
        return std::sqrt(dx * dx + dy * dy);
    }

    /**
     * @brief Addition operator for vector composition.
     * @param other The vector to add
     * @return New vector representing the sum
     */
    Vector2D operator+(const Vector2D& other) const {
        return Vector2D(dx + other.dx, dy + other.dy);
    }

    /**
     * @brief Subtraction operator for vector difference.
     * @param other The vector to subtract
     * @return New vector representing the difference
     */
    Vector2D operator-(const Vector2D& other) const {
        return Vector2D(dx - other.dx, dy - other.dy);
    }

    /**
     * @brief Scalar multiplication operator.
     * @param scalar The scalar value to multiply by
     * @return New vector with scaled components
     */
    Vector2D operator*(double scalar) const {
        return Vector2D(dx * scalar, dy * scalar);
    }

    /**
     * @brief Scalar division operator.
     * @param scalar The scalar value to divide by
     * @return New vector with scaled components
     */
    Vector2D operator/(double scalar) const {
        if (scalar == 0.0) {
            return Vector2D(0.0, 0.0);  // Handle division by zero
        }
        return Vector2D(dx / scalar, dy / scalar);
    }

    /**
     * @brief Dot product with another vector.
     * @param other The vector to compute dot product with
     * @return The scalar dot product value
     */
    double dotProduct(const Vector2D& other) const {
        return dx * other.dx + dy * other.dy;
    }

    /**
     * @brief Normalize the vector to unit length.
     * @return New normalized vector (magnitude = 1.0)
     */
    Vector2D normalize() const {
        double mag = magnitude();
        if (mag == 0.0) {
            return Vector2D(0.0, 0.0);  // Zero vector remains zero
        }
        return Vector2D(dx / mag, dy / mag);
    }

    /**
     * @brief Stream output operator for debugging.
     * @param os Output stream
     * @param v The vector to output
     * @return Reference to the output stream
     */
    friend std::ostream& operator<<(std::ostream& os, const Vector2D& v);
};

// Inline implementations of Point2D methods that depend on Vector2D
inline Point2D Point2D::operator+(const Vector2D& vec) const {
    return Point2D(x + vec.getDX(), y + vec.getDY());
}

inline Vector2D Point2D::operator-(const Point2D& other) const {
    return Vector2D(x - other.x, y - other.y);
}

} // namespace AIGD

#endif // SPATIAL_UTILITIES_H
