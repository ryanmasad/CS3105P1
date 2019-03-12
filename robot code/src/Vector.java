


/**
 * Represents a 2D vector.
 */
public class Vector {
    double x;
    double y;

    public Vector(double _x, double _y) {
        x = _x;
        y = _y;
    }

    public Vector add(Vector other) {
        return new Vector(x + other.x, y + other.y);
    }

    public Vector diff(Vector other) {
        return new Vector(x - other.x, y - other.y);
    }

    public double dot(Vector other) {
        return other.x * x + other.y * y;
    }

    public double getMagnitude() {
        return Math.sqrt(x * x + y * y);
    }

    public String toString() {
        return "[" + x + ", " + y + "]";
    }

    public Vector midPoint(Vector other) {
        return new Vector((x + other.x) / 2, (y + other.y) / 2);
    }

    public Vector multiplyByScalar(double scalar) {
        return new Vector(scalar * x, scalar * y);
    }
}