import renderables.RenderablePolyline;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents a single arc.
 */
class MyArc {
    // Data Members:

    final public MyPoint p1;
    final public MyPoint p2;
    final double startHeading;
    final double arcLength;
    final double curvature;

    List<MyPoint> points = new ArrayList<MyPoint>();

    // Construction:

    /**
     * Constructs a MyArc instance.
     */

    MyArc(MyPoint p1, MyPoint p2, double startHeading, boolean draw) {

        startHeading = Calculator.mod(startHeading, (2 * Math.PI));
        this.p1 = p1;
        this.p2 = p2;
        double distance = p1.distance(p2);
        this.startHeading = startHeading;

        double step = 1.0;

        double XvalHeading = 1.0 * Math.cos(startHeading) - 0.0 * Math.sin(startHeading);
        double YvalHeading = 1.0 * Math.sin(startHeading) + 0.0 * Math.cos(startHeading);
        MyPoint VectorFromP1TpP2 = new MyPoint(p2.x - p1.x, p2.y - p1.y); // p2.sub(p1)
        // ;
        MyPoint headingVector = new MyPoint(XvalHeading, YvalHeading);

        double anglec = computeAngle(headingVector, VectorFromP1TpP2);

        double curvature = Calculator.getCurvature(anglec, distance);

        this.arcLength = Math.abs(Calculator.getArcLength(anglec, curvature, distance));

        if (anglec == Math.PI && curvature == 0)
            curvature = 1 / distance;
        this.curvature = curvature;
        if (curvature == 0 || anglec == 0) {

            MyPoint VectorX = new MyPoint(p2.x - p1.x, p2.y - p1.y);
            for (double i = 0; i < distance; i = i + step) {
                double x = p1.x + VectorX.x * i / distance;
                double y = p1.y + VectorX.y * i / distance;
                points.add(new MyPoint(x, y));
            }
            points.add(p2);
            return;

        }
        double raduis = 1.0 / curvature;
        double halfdiameter = Math.PI * raduis;

        MyPoint Center = circleCenter(p1, p2, startHeading, raduis);
        if (!draw)
            return;

        if (curvature != 0 && (halfdiameter <= arcLength || anglec == Math.PI / 2.0)) {

            points = ArcDrawing(p1, p2, Center, raduis, step, startHeading, false);

        } else if (curvature != 0) {

            points = ArcDrawing(p1, p2, Center, raduis, step, startHeading, true);
        }

    }

    /* compute signed angle */

    private static double computeAngleDirected(MyPoint vector1, MyPoint vector2) {
        double angle = Math.atan2(vector2.y, vector2.x) - Math.atan2(vector1.y, vector1.x);
        return angle;
    }

    /* rotate P1 by angle */
    private static MyPoint rotate(MyPoint P1, double angle) {
        double x = P1.x * Math.cos(angle) - P1.y * Math.sin(angle);
        double y = P1.x * Math.sin(angle) + P1.y * Math.cos(angle);
        return new MyPoint(x, y);

    }
    /* get the points on the arc -clockwise- */

    private static List<MyPoint> backwordOrder(double raduis, MyPoint center1, MyPoint point1, MyPoint point2,
                                               double heading, double step, boolean small) {

        int n = (int) (Math.abs(raduis) / step * 2.0 * 2.0 + .5 * 10);
        if (n == 0)
            n = 100;
        step = raduis / n;

        MyPoint P1ToC1Vector = new MyPoint(point1.x - center1.x, point1.y - center1.y);

        MyPoint P2ToC1Vector = new MyPoint(point2.x - center1.x, point2.y - center1.y);

        double angle = computeAngle(P1ToC1Vector, P2ToC1Vector);

        if (!small)
            angle = 2 * Math.PI - angle;
        MyPoint xaxis = new MyPoint(1.0, 0.0);
        double rotatedAngle = -1.0 * computeAngleDirected(P1ToC1Vector, xaxis);

        MyPoint rotatedPoint = new MyPoint(raduis, 0.0);
        rotatedPoint = rotate(rotatedPoint, angle);

        List<MyPoint> arc1 = new ArrayList<MyPoint>();

        double movedstep = raduis;

        while (movedstep > 0) {
            MyPoint point = new MyPoint(movedstep, -1 * Math.sqrt(raduis * raduis - (movedstep * movedstep)));
            movedstep = movedstep - step;
            if (Math.abs(point.y) > Math.abs(rotatedPoint.y) && angle < Math.PI / 2.0)
                break;
            arc1.add(point);

        }

        movedstep = 0;
        if (angle > Math.PI / 2.0) {
            while (movedstep > -raduis) {
                if (angle < Math.PI && movedstep < rotatedPoint.x) {
                    break;
                }

                MyPoint point = new MyPoint(movedstep, -1 * Math.sqrt(raduis * raduis - (movedstep * movedstep)));
                movedstep = movedstep - step;
                arc1.add(point);

            }
        }

        if (angle > Math.PI) {
            boolean stopCondtionRequire = angle < Math.PI * 3.0 / 2.0;

            movedstep = -raduis;
            while (movedstep < 0) {
                if (movedstep > rotatedPoint.x && stopCondtionRequire)
                    break;
                MyPoint point = new MyPoint(movedstep, Math.sqrt(raduis * raduis - (movedstep * movedstep)));
                movedstep = movedstep + step;
                arc1.add(point);

            }
        }

        if (angle > 3.0 * Math.PI / 2.0) {
            movedstep = 0;
            while (movedstep < rotatedPoint.x) {
                MyPoint point = new MyPoint(movedstep, Math.sqrt(raduis * raduis - (movedstep * movedstep)));
                movedstep = movedstep + step;
                arc1.add(point);
            }

        }
        List<MyPoint> arc1x = new ArrayList<MyPoint>();
        for (int i = 0; i < arc1.size(); i++) {
            MyPoint RotatedP = rotate(arc1.get(i), rotatedAngle);
            RotatedP = new MyPoint(RotatedP.x + center1.x, RotatedP.y + center1.y);
            arc1x.add(RotatedP);

        }

        return arc1x;
    }

    /* get the points on the arc -anti clockwise- */
    private static List<MyPoint> forwardOrder(double raduis, MyPoint center1, MyPoint point1, MyPoint point2,
                                              double heading, double step, boolean small) {
        int n = (int) (Math.abs(raduis) / step * 2.0 * 2.0 + .5 * 10);
        if (n == 0)
            n = 100;
        step = raduis / n;

        MyPoint P1ToC1Vector = new MyPoint(point1.x - center1.x, point1.y - center1.y);
        MyPoint P2ToC1Vector = new MyPoint(point2.x - center1.x, point2.y - center1.y);

        double angle = computeAngle(P1ToC1Vector, P2ToC1Vector);

        if (!small) {
            angle = 2 * Math.PI - angle;
        }

        MyPoint xx = new MyPoint(1.0, 0.0);

        double rotatedAngle = -1.0 * computeAngleDirected(P1ToC1Vector, xx);

        MyPoint rotatedPoint = new MyPoint(raduis, 0.0);
        rotatedPoint = rotate(rotatedPoint, angle);

        List<MyPoint> arc1 = new ArrayList<MyPoint>();

        double movedstep = raduis;

        while (movedstep > 0) {
            MyPoint Point = new MyPoint(movedstep, Math.sqrt(raduis * raduis - (movedstep * movedstep)));
            if (Math.abs(Point.y) > Math.abs(rotatedPoint.y) && angle < Math.PI / 2.0)
                break;
            arc1.add(Point);

            movedstep = movedstep - step;
        }

        movedstep = 0;
        if (angle > Math.PI / 2.0) {
            while (movedstep > -raduis) {

                if (angle < Math.PI && movedstep < rotatedPoint.x)
                    break;
                MyPoint Point = new MyPoint(movedstep, Math.sqrt(raduis * raduis - (movedstep * movedstep)));
                arc1.add(Point);

                movedstep = movedstep - step;
            }
        }

        if (angle > Math.PI) {
            boolean stopCondtionRequire = angle < Math.PI * 3.0 / 2.0;

            movedstep = -raduis;
            while (movedstep < 0) {
                if (movedstep > rotatedPoint.x && stopCondtionRequire)
                    break;
                MyPoint Point = new MyPoint(movedstep, -1 * Math.sqrt(raduis * raduis - (movedstep * movedstep)));
                arc1.add(Point);

                movedstep = movedstep + step;

            }

        }

        if (angle > 3.0 * Math.PI / 2.0) {

            movedstep = 0;
            while (movedstep < rotatedPoint.x) {
                MyPoint Point = new MyPoint(movedstep, -1 * Math.sqrt(raduis * raduis - (movedstep * movedstep)));
                arc1.add(Point);

                movedstep = movedstep + step;
            }
        }

        List<MyPoint> arc1x = new ArrayList<MyPoint>();
        for (int i = 0; i < arc1.size(); i++) {
            MyPoint RotatedP = rotate(arc1.get(i), rotatedAngle);
            RotatedP = new MyPoint(RotatedP.x + center1.x, RotatedP.y + center1.y);
            arc1x.add(RotatedP);
        }

        return arc1x;
    }

    /* get the points on the arc */
    private static List<MyPoint> ArcDrawing(MyPoint p1, MyPoint p2, MyPoint center1, double raduis, double step,
                                            double startHeading, boolean small) {

        List<MyPoint> arc1X = new ArrayList<MyPoint>();
        List<MyPoint> arc2X = new ArrayList<MyPoint>();

        arc1X = backwordOrder(raduis, center1, p1, p2, startHeading, step, small);
        arc2X = forwardOrder(raduis, center1, p1, p2, startHeading, step, small);
        if (arc1X.size() < 2 && arc2X.size() < 2) {
            arc1X.add(p2);
            return arc1X;
        }

        arc1X.add(p2);
        arc2X.add(p2);

        if (arc1X.size() < 2)
            return arc2X;
        if (arc2X.size() < 2)
            return arc1X;

        MyPoint secoundPointarc1 = arc1X.get(1);
        MyPoint secoundPointarc2 = arc2X.get(1);

        MyPoint headingPoint = rotate(new MyPoint(1, 0), startHeading);
        MyPoint newPosition1Vector = new MyPoint(secoundPointarc1.x - p1.x, secoundPointarc1.y - p1.y);
        MyPoint newPosition2Vector = new MyPoint(secoundPointarc2.x - p1.x, secoundPointarc2.y - p1.y);

        double angle1 = computeAngle(newPosition1Vector, headingPoint);
        double angle2 = computeAngle(newPosition2Vector, headingPoint);

        if (angle1 < angle2) {
            return arc1X;
        }
        return arc2X;

    }

    /* compute unsigned angle */
    private static double computeAngle(MyPoint V1, MyPoint V2) {
        double dotProduct = V1.x * V2.x + V1.y * V2.y;
        double norm1 = Math.sqrt(V1.x * V1.x + V1.y * V1.y);
        double norm2 = Math.sqrt(V2.x * V2.x + V2.y * V2.y);
        if ((dotProduct / (norm1 * norm2)) > 1)
            return Math.acos(1.0);
        else if ((dotProduct / (norm1 * norm2)) < -1)
            return Math.acos(-1.0);

        return Math.acos(dotProduct / (norm2 * norm1));

    }

    /*
     * find the circle center for the arc .
     *
     */

    private static MyPoint circleCenter(MyPoint p1, MyPoint p2, double startHeading, double radius) {

        MyPoint VectorFromP1TpP2 = new MyPoint(p2.x - p1.x, p2.y - p1.y);
        MyPoint headingVector = rotate(new MyPoint(1, 0), startHeading);
        double distance = p1.distance(p2);
        double Angle = computeAngle(VectorFromP1TpP2, headingVector);
        MyPoint c1;
        // if the angle between the two vectors is 90 degress then the circle
        // center is between P1 and P2
        if (Angle == Math.PI / 2.0) {
            double cx1 = VectorFromP1TpP2.x / 2.0 + p1.x;
            double cy1 = VectorFromP1TpP2.y / 2.0 + p1.y;
            c1 = new MyPoint(cx1, cy1);

            return c1;

        }

        // Angle between the two vectors is <90

        if (Angle < Math.PI / 2.0) {
            double lengthOfHeading = .5 * distance / Math.cos(Angle);
            MyPoint thirdVertex = new MyPoint(p1.x + (headingVector.x * lengthOfHeading),
                    p1.y + (headingVector.y * lengthOfHeading));
            MyPoint midChordPoint = new MyPoint(p1.x + (VectorFromP1TpP2.x * .5), p1.y + (VectorFromP1TpP2.y * .5));
            MyPoint VectorToCenter = new MyPoint(midChordPoint.x - thirdVertex.x, midChordPoint.y - thirdVertex.y);
            double normOfCenter = Math.sqrt(VectorToCenter.x * VectorToCenter.x + VectorToCenter.y * VectorToCenter.y);
            double lengthToCenter = Math.sqrt(radius * radius - (distance * .5) * (distance * .5));
            c1 = new MyPoint(midChordPoint.x + (VectorToCenter.x * lengthToCenter / normOfCenter),
                    midChordPoint.y + (VectorToCenter.y * lengthToCenter / normOfCenter));

            return c1;

        }
        // Angle between the two vectors is >90
        else {
            double anglex = Math.PI - Angle;
            double lengthOfHeading = .5 * distance / Math.cos(anglex);
            MyPoint thirdVertex = new MyPoint(p1.x - (headingVector.x * lengthOfHeading),
                    p1.y - (headingVector.y * lengthOfHeading));
            MyPoint midChordPoint = new MyPoint(p1.x + (VectorFromP1TpP2.x * .5), p1.y + (VectorFromP1TpP2.y * .5));
            MyPoint VectorToCenter = new MyPoint(thirdVertex.x - midChordPoint.x, thirdVertex.y - midChordPoint.y);
            double normOfCenter = Math.sqrt(VectorToCenter.x * VectorToCenter.x + VectorToCenter.y * VectorToCenter.y);
            double lengthToCenter = Math.sqrt(radius * radius - (distance * .5) * (distance * .5));
            c1 = new MyPoint(midChordPoint.x + (VectorToCenter.x * lengthToCenter / normOfCenter),
                    midChordPoint.y + (VectorToCenter.y * lengthToCenter / normOfCenter));

            return c1;
        }

    }

    public RenderablePolyline getRenderablePolyline(boolean srcDrawingEnabled) {
        RenderablePolyline polyline = new RenderablePolyline();
        if (srcDrawingEnabled)
            for (MyPoint p : points)
                polyline.addPoint((int) p.x, (int) p.y);
        else {
            polyline.addPoint((int) p1.x, (int) p1.y);
            polyline.addPoint((int) p2.x, (int) p2.y);
        }
        return polyline;
    }
}