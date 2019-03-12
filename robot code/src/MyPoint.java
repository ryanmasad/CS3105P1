

class MyPoint {
    double x, y;

    MyPoint(double px, double py) {
        x = px;
        y = py;
    }

    MyPoint(Vector v) {
        x = v.x;
        y = v.y;
    }

    MyPoint sub(MyPoint p2) {
        return new MyPoint(x - p2.x, y - p2.y);
    }

    MyPoint add(MyPoint p2) {
        return new MyPoint(x + p2.x, y + p2.y);
    }

    double distance(MyPoint p2) {
        return Math.sqrt((x - p2.x) * (x - p2.x) + (y - p2.y) * (y - p2.y));
    }

    MyPoint normal() {
        double length = Math.sqrt(x * x + y * y);
        return new MyPoint(x / length, y / length);
    }

    MyPoint scale(double s) {
        return new MyPoint(x * s, y * s);
    }

    public double getAngleTo(MyPoint o) {
        double angle = Math.atan2(o.y - y, o.x - x);
        if (angle < 0)
            angle += 2 * Math.PI;
        return angle;
    }

    @Override
    public String toString() {
        return "(" + x + "," + y + ")";
    }
}    
