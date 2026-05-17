#include "SpatialUtilities.h"
#include <ostream>

namespace AIGD {

/**
 * @brief Stream output operator for Point2D.
 * 
 * Outputs point coordinates in the format: (x, y)
 */
std::ostream& operator<<(std::ostream& os, const Point2D& p) {
    os << "(" << p.x << ", " << p.y << ")";
    return os;
}

/**
 * @brief Stream output operator for Vector2D.
 * 
 * Outputs vector components in the format: <dx, dy>
 */
std::ostream& operator<<(std::ostream& os, const Vector2D& v) {
    os << "<" << v.getDX() << ", " << v.getDY() << ">";
    return os;
}

} // namespace AIGD
