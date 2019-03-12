import dataStructures.RRNode;
import dataStructures.RRTree;
import easyGui.EasyGui;
import geometry.IntPoint;
import renderables.*;

import java.awt.*;
import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.Random;

public class RRT {
    private final EasyGui gui;
    private final int buttonId;
    private final int circleSId;
    private final int circleLId;
    private final int squareSId;
    private final int squareLId;
    private final int randomLineId;
    private final int clearObsId;
    private final int easyCourseId;
    private final int medCourseId;
    private final int hardCourseId;
    private final int enGeomId;
    private final int disGeomId;
    private final int noBiasId;
    private final int weakBiasId;
    private final int strongBiasId;
    private final int startXId;
    private final int startYId;
    private final int goalXId;
    private final int goalYId;
    private final int treeSpeedId;
    private final int stepSizeId;
    private final int frameLength = 1200;
    private final int frameHeight = 900;
    private final int graphicsHeight = 700;
    private RRTree tree;
    private int goalBias;
    private boolean geomMove;

    private ArrayList<Renderable> goalPathRender;

    private ArrayList<Renderable> obstacles;

    public RRT() {
        //Generate the GUI, buttons, labels, etc.
        gui = new EasyGui(frameLength, frameHeight);

        //Robot options
        buttonId = gui.addButton(2, 5, "Generate Tree", this, "buttonAction");
        gui.addButton(2, 4, "Clear Fields", this, "clearButtonAction");

        gui.addLabel(0, 0, "Starting X:");
        startXId = gui.addTextField(0, 1, null);
        gui.addLabel(1, 0, "Starting Y:");
        startYId = gui.addTextField(1, 1, null);

        gui.addLabel(0, 2, "Goal X:");
        goalXId = gui.addTextField(0, 3, null);
        gui.addLabel(1, 2, "Goal Y:");
        goalYId = gui.addTextField(1, 3, null);

        gui.addLabel(0, 4, "Tree Expansion Speed:");
        treeSpeedId = gui.addTextField(0, 5, null);

        gui.addLabel(1, 4, "Step Size:");
        stepSizeId = gui.addTextField(1, 5, null);

        //More options
        gui.addLabel(0, 6, "Geometric Movement: ");
        enGeomId = gui.addButton(0, 7, "On", this, "enableGeom");
        disGeomId = gui.addButton(0, 8, "Off", this, "disableGeom");
        gui.addLabel(1, 6, "Goal Bias: ");
        noBiasId = gui.addButton(1, 7, "None", this, "noBias");
        weakBiasId = gui.addButton(1, 8, "Weak", this, "weakBias");
        strongBiasId = gui.addButton(1, 9, "Strong", this, "strongBias");
        this.geomMove = true;
        this.goalBias = 0;
        gui.setButtonEnabled(enGeomId, false);
        gui.setButtonEnabled(noBiasId, false);

        gui.addLabel(2, 6, "(Leave fields blank for random values!)");

        //Pre-made courses
        gui.addLabel(3, 0, "Select a pre-made course: ");
        easyCourseId = gui.addButton(3, 1, "Easy", this, "easyCourse");
        medCourseId = gui.addButton(3, 2, "Medium", this, "medCourse");
        hardCourseId = gui.addButton(3, 3, "Hard", this, "hardCourse");

        //Custom obstacles
        gui.addLabel(4, 0, "Or add in your own obstacles: ");
        circleSId = gui.addButton(4, 1, "Circle (S)", this, "genCircleS");
        circleLId = gui.addButton(4, 2, "Circle (L)", this, "genCircleL");
        squareSId = gui.addButton(4, 3, "Square (S)", this, "genSquareS");
        squareLId = gui.addButton(4, 4, "Square (L)", this, "genSquareL");
        randomLineId = gui.addButton(4, 5, "Line", this, "genLine");
        clearObsId = gui.addButton(4, 6, "Clear Obstacles", this, "clearObs");

        gui.addButton(4, 8, "Quit", this, "quit");

        obstacles = new ArrayList<Renderable>();

    }

    /**
     * Generate a random point between 0 and maxX, and 0 and maxY. For obstacle creation.
     **/
    private static IntPoint randomPoint(int maxX, int maxY) {
        Random rand = new Random();
        IntPoint point = new IntPoint();
        point.x = rand.nextInt(maxX + 1);
        point.y = rand.nextInt(maxY + 1);
        return point;
    }

    /**
     * Generate a random point within radius r = 2*dist(start, goal)
     **/
    private static IntPoint randomPoint(IntPoint start, IntPoint goal) {
        Random rand = new Random();
        IntPoint point = new IntPoint();
        double angle = Math.random() * Math.PI * 2;
        int r = (int) (distance(start, goal) * 2);
        point.x = start.x + (int) (Math.cos(angle) * (double) rand.nextInt(r));
        point.y = start.y + (int) (Math.sin(angle) * (double) rand.nextInt(r));
        return point;
    }

    /**
     * Generate a random point within radius r with bias towards the goal based on the ratio of a:b:c
     **/
    private static IntPoint randomPointBias(IntPoint start, IntPoint goal, int a, int b, int c) {
        Random rand = new Random();
        IntPoint[] points = new IntPoint[a + b + c];
        //'a' random points
        for (int i = 0; i < a; i++) {
            IntPoint ri = new IntPoint();
            double angle = Math.random() * Math.PI * 2;
            int r = (int) (distance(start, goal) * 2);
            ri.x = start.x + (int) (Math.cos(angle) * (double) rand.nextInt(r));
            ri.y = start.y + (int) (Math.sin(angle) * (double) rand.nextInt(r));
            points[i] = ri;
        }
        //'b' points within dist(start, end) of the goal
        for (int i = 0; i < b; i++) {
            IntPoint ri = new IntPoint();
            double angle = Math.random() * Math.PI * 2;
            int r = (int) (distance(start, goal));
            ri.x = goal.x + (int) (Math.cos(angle) * (double) rand.nextInt(r));
            ri.y = goal.y + (int) (Math.sin(angle) * (double) rand.nextInt(r));
            points[i + a] = ri;
        }
        //'c' points within dist(start, end)/2 of the goal
        for (int i = 0; i < c; i++) {
            IntPoint ri = new IntPoint();
            double angle = Math.random() * Math.PI * 2;
            int r = (int) (distance(start, goal) / 2);
            ri.x = goal.x + (int) (Math.cos(angle) * (double) rand.nextInt(r));
            ri.y = goal.y + (int) (Math.sin(angle) * (double) rand.nextInt(r));
            points[i + b + a] = ri;
        }
        return points[rand.nextInt(a + b + c)];
    }

    /**
     * Check if a given point is within the goal radius
     */
    private static boolean isInGoal(IntPoint point, IntPoint goal, int radius) {
        int l = Math.abs(goal.x - point.x);
        int h = Math.abs(goal.y - point.y);
        double dist = Math.sqrt(Math.pow(l, 2) + Math.pow(h, 2));
        return dist <= radius;
    }

    /**
     * Get the distance (in pixels) between two points
     **/
    private static double distance(IntPoint a, IntPoint b) {
        return Math.sqrt(Math.pow((a.x - b.x), 2) + Math.pow((a.y - b.y), 2));
    }

    /**
     * Get the difference between two angles, measured between 0 and 2PI.
     **/
    private static double angleDifference(double angleA, double angleB) {
        return Math.abs(mod((angleA + Math.PI - angleB), (2 * Math.PI)) - Math.PI);

    }

    /**
     * a % b but always with a positive answer - Java's default % syntax gives a negative answer if the
     * dividend is < 0.
     **/
    private static double mod(double a, double b) {
        return ((a % b) + b) % b;
    }

    public void enableGeom() {
        this.geomMove = true;
        gui.setButtonEnabled(enGeomId, false);
        gui.setButtonEnabled(disGeomId, true);
    }

    public void disableGeom() {
        this.geomMove = false;
        gui.setButtonEnabled(enGeomId, true);
        gui.setButtonEnabled(disGeomId, false);
    }

    /*These buttons create shapes in random places on the gui - circles, squares and lines*/

    public void noBias() {
        this.goalBias = 0;
        gui.setButtonEnabled(weakBiasId, true);
        gui.setButtonEnabled(strongBiasId, true);
        gui.setButtonEnabled(noBiasId, false);
    }

    public void weakBias() {
        this.goalBias = 1;
        gui.setButtonEnabled(weakBiasId, false);
        gui.setButtonEnabled(strongBiasId, true);
        gui.setButtonEnabled(noBiasId, true);
    }

    public void strongBias() {
        this.goalBias = 2;
        gui.setButtonEnabled(weakBiasId, true);
        gui.setButtonEnabled(strongBiasId, false);
        gui.setButtonEnabled(noBiasId, true);
    }

    public void quit() {
        gui.hide();
        System.exit(0);
    }

    /**
     * Setup the 'easy' obstacle course on the GUI
     **/
    public void easyCourse() {
        gui.setTextFieldContent(startXId, "100");
        gui.setTextFieldContent(startYId, "100");
        gui.setTextFieldContent(goalXId, "1000");
        gui.setTextFieldContent(goalYId, "500");

        RenderableOval r = new RenderableOval(frameLength / 2, graphicsHeight / 2, 150, 150);
        r.setProperties(Color.DARK_GRAY, 1f, true);
        obstacles.add(r);
        gui.draw(r);
        gui.update();
    }

    /**
     * Set up the 'medium' premade course
     */
    public void medCourse() {
        gui.setTextFieldContent(startXId, "0");
        gui.setTextFieldContent(startYId, "0");
        gui.setTextFieldContent(goalXId, "1400");
        gui.setTextFieldContent(goalYId, "700");

        RenderableRectangle r = new RenderableRectangle(150, 150, 150, 150);
        r.setProperties(Color.DARK_GRAY, 1f, true, false);
        RenderableRectangle r2 = new RenderableRectangle(400, 400, 150, 150);
        r2.setProperties(Color.DARK_GRAY, 1f, true, false);
        RenderableRectangle r3 = new RenderableRectangle(700, 425, 150, 150);
        r3.setProperties(Color.DARK_GRAY, 1f, true, false);
        RenderableRectangle r4 = new RenderableRectangle(400, 250, 50, 50);
        r4.setProperties(Color.DARK_GRAY, 1f, true, false);
        RenderableRectangle r5 = new RenderableRectangle(600, 100, 150, 150);
        r5.setProperties(Color.DARK_GRAY, 1f, true, false);
        RenderableRectangle r6 = new RenderableRectangle(1000, 600, 150, 150);
        r6.setProperties(Color.DARK_GRAY, 1f, true, false);
        RenderableRectangle r7 = new RenderableRectangle(900, 250, 50, 50);
        r7.setProperties(Color.DARK_GRAY, 1f, true, false);
        RenderableRectangle r8 = new RenderableRectangle(1000, 450, 50, 50);
        r8.setProperties(Color.DARK_GRAY, 1f, true, false);
        RenderableRectangle r9 = new RenderableRectangle(1150, 350, 50, 50);
        r9.setProperties(Color.DARK_GRAY, 1f, true, false);
        obstacles.add(r);
        obstacles.add(r2);
        obstacles.add(r3);
        obstacles.add(r4);
        obstacles.add(r5);
        obstacles.add(r6);
        obstacles.add(r7);
        obstacles.add(r8);
        obstacles.add(r9);
        gui.draw(obstacles);
        gui.update();
    }

    /**
     * Set up the 'hard' premade course
     */
    public void hardCourse() {
        gui.setTextFieldContent(startXId, "0");
        gui.setTextFieldContent(startYId, "0");
        gui.setTextFieldContent(goalXId, "1400");
        gui.setTextFieldContent(goalYId, "850");

        RenderableRectangle r = new RenderableRectangle(150, 200, 150, 150);
        r.setProperties(Color.DARK_GRAY, 1f, true, false);
        RenderableRectangle r2 = new RenderableRectangle(400, 400, 150, 150);
        r2.setProperties(Color.DARK_GRAY, 1f, true, false);
        RenderableRectangle r4 = new RenderableRectangle(400, 250, 50, 50);
        r4.setProperties(Color.DARK_GRAY, 1f, true, false);
        RenderableRectangle r5 = new RenderableRectangle(600, 100, 150, 150);
        r5.setProperties(Color.DARK_GRAY, 1f, true, false);
        RenderableRectangle r7 = new RenderableRectangle(900, 350, 50, 50);
        r7.setProperties(Color.DARK_GRAY, 1f, true, false);
        obstacles.add(r);
        obstacles.add(r2);
        obstacles.add(r4);
        obstacles.add(r5);
        obstacles.add(r7);
        gui.draw(obstacles);
        gui.update();

        IntPoint p0 = new IntPoint(800, 700);
        IntPoint p1 = new IntPoint(800, 900);
        IntPoint p2 = new IntPoint(1300, 800);
        IntPoint p3 = new IntPoint(1400, 300);
        IntPoint p4 = new IntPoint(1200, 350);
        RenderablePolyline line = new RenderablePolyline();
        line.addPoint(p0.x, p0.y);
        line.addPoint(p1.x, p1.y);
        line.addPoint(p2.x, p2.y);
        line.addPoint(p3.x, p3.y);
        line.addPoint(p4.x, p4.y);
        line.setProperties(Color.DARK_GRAY, 2f);
        obstacles.add(line);
        gui.draw(line);
        gui.update();
    }

    public void genCircleS() {
        IntPoint centre = randomPoint(frameLength, frameHeight);
        RenderableOval o = new RenderableOval(centre.x, centre.y, 50, 50);
        o.setProperties(Color.MAGENTA, 1f, true);
        obstacles.add(o);
        gui.draw(o);
        gui.update();
    }

    public void genCircleL() {
        IntPoint centre = randomPoint(frameLength, frameHeight);
        RenderableOval o = new RenderableOval(centre.x, centre.y, 150, 150);
        o.setProperties(Color.MAGENTA, 1f, true);
        obstacles.add(o);
        gui.draw(o);
        gui.update();
    }

    public void genSquareS() {
        IntPoint origin = randomPoint(frameLength - 50, frameHeight - 5);
        RenderableRectangle r = new RenderableRectangle(origin.x, origin.y, 50, 50);
        r.setProperties(Color.CYAN, 1f, true, false);
        obstacles.add(r);
        gui.draw(r);
        gui.update();
    }

    public void genSquareL() {
        IntPoint origin = randomPoint(frameLength - 150, frameHeight - 150);
        RenderableRectangle r = new RenderableRectangle(origin.x, origin.y, 150, 150);
        r.setProperties(Color.CYAN, 1f, true, false);
        obstacles.add(r);
        gui.draw(r);
        gui.update();
    }

    public void genLine() {
        IntPoint p1 = randomPoint(frameLength - 200, frameHeight - 200);
        IntPoint p2 = randomPoint(frameLength - 200, frameHeight - 200);
        RenderablePolyline p = new RenderablePolyline();
        p.addPoint(p1.x, p1.y);
        p.addPoint(p2.x, p2.y);
        p.setProperties(Color.ORANGE, 2f);
        obstacles.add(p);
        gui.draw(p);
        gui.update();

    }

    /**
     * Clear all obstacles
     **/
    public void clearObs() {
        gui.unDraw(obstacles);
        obstacles = new ArrayList<Renderable>();
        gui.update();
    }

    /**
     * Clear all text fields
     **/
    public void clearButtonAction() {
        gui.setTextFieldContent(startXId, "");
        gui.setTextFieldContent(startYId, "");
        gui.setTextFieldContent(goalXId, "");
        gui.setTextFieldContent(goalYId, "");
        gui.setTextFieldContent(treeSpeedId, "");
        gui.setTextFieldContent(stepSizeId, "");
    }

    /**
     * Get the parameters from the text fields and set the path finding algorithm going
     **/
    public void buttonAction() throws InterruptedException {
        int startX, startY, goalX, goalY, radius, stepSize, speed;
        Random rand = new Random();
        String startXs = gui.getTextFieldContent(startXId);
        String startYs = gui.getTextFieldContent(startYId);
        String goalXs = gui.getTextFieldContent(goalXId);
        String goalYs = gui.getTextFieldContent(goalYId);
        String speeds = gui.getTextFieldContent(treeSpeedId);
        String stepSizes = gui.getTextFieldContent(stepSizeId);
        if (startXs.equals("")) startX = rand.nextInt(frameLength);
        else startX = Integer.parseInt(startXs);
        gui.setTextFieldContent(startXId, "" + startX);
        if (startYs.equals("")) startY = rand.nextInt(frameHeight);
        else startY = Integer.parseInt(startYs);
        gui.setTextFieldContent(startYId, "" + startY);
        if (goalXs.equals("")) goalX = rand.nextInt(frameLength);
        else goalX = Integer.parseInt(goalXs);
        gui.setTextFieldContent(goalXId, "" + goalX);
        if (goalYs.equals("")) goalY = rand.nextInt(frameHeight);
        else goalY = Integer.parseInt(goalYs);
        gui.setTextFieldContent(goalYId, "" + goalY);
        if (speeds.equals("")) speed = 100; //100 actions per second default
        else speed = Integer.parseInt(speeds);
        gui.setTextFieldContent(treeSpeedId, "" + speed);
        radius = 60;
        if (stepSizes.equals("")) stepSize = rand.nextInt(radius - 5) + 20; //Step size is between 10 and the
            //radius size-5, so it never overshoots
        else stepSize = Integer.parseInt(stepSizes);
        gui.setTextFieldContent(stepSizeId, "" + stepSize);

        makeTree(new IntPoint(startX, startY), new IntPoint(goalX, goalY), radius, stepSize, speed);
    }

    /**
     * Create an RRT tree between the start and goal points.
     *
     * @param start      The coordinates of the start point
     * @param goal       The coordinates of the end point
     * @param goalRadius The radius of the goal
     * @param stepSize   How far to move each step
     * @param speed      How many samples to take per second
     **/
    private void makeTree(IntPoint start, IntPoint goal, int goalRadius, int stepSize, int speed) throws InterruptedException {

        //Initialise tree, random point, text strings, goal route
        RenderablePoint randomRendered = new RenderablePoint(0, 0);
        randomRendered.setProperties(Color.RED, 10.0f);
        RenderableString rs = new RenderableString(800, 20, "Goal Path Length: 0");
        RenderableString rs2 = new RenderableString(800, 0, "Smoothness: 0");
        RenderableString rs3 = new RenderableString(800, 40, "Used Nodes/Total Nodes: 0/0");
        rs.setLayer(456);
        rs.setProperties(Color.BLUE, new Font(Font.SERIF, Font.BOLD, 14));
        rs2.setLayer(456);
        rs2.setProperties(Color.BLUE, new Font(Font.SERIF, Font.BOLD, 14));
        rs3.setLayer(456);
        rs3.setProperties(Color.BLUE, new Font(Font.SERIF, Font.BOLD, 14));
        boolean fin = false;
        tree = new RRTree(Color.BLACK);
        tree.setStartAndGoal(start, goal, goalRadius);
        goalPathRender = new ArrayList<Renderable>();
        int robotRadius = 20;
        int nodes = 0;

        //Draw the tree.
        gui.clearGraphicsPanel();
        gui.draw(obstacles);
        gui.draw(tree);

        //Disable all buttons while the program is running.
        gui.setButtonEnabled(buttonId, false);
        gui.setButtonEnabled(circleSId, false);
        gui.setButtonEnabled(circleLId, false);
        gui.setButtonEnabled(squareSId, false);
        gui.setButtonEnabled(squareLId, false);
        gui.setButtonEnabled(randomLineId, false);
        gui.setButtonEnabled(clearObsId, false);
        gui.setButtonEnabled(easyCourseId, false);
        gui.setButtonEnabled(medCourseId, false);
        gui.setButtonEnabled(hardCourseId, false);

        do {
            Thread.sleep(1000 / speed); //[speed] actions per second
            //Create a random point with variable goal bias
            IntPoint randomGoal;
            if (goalBias == 0) { //no goal bias - sample whole area
                randomGoal = randomPoint(start, goal);
            } else if (goalBias == 2) { //strong bias - sample with strong weights to points around the goal
                randomGoal = randomPointBias(start, goal, 1, 4, 8);
            } else { //weak bias - sample with weak weight towards points around the goal.
                randomGoal = randomPointBias(start, goal, 1, 1, 1);
            }


            // Draws the current target point
            gui.unDraw(randomRendered);
            randomRendered.x = randomGoal.x;
            randomRendered.y = randomGoal.y;
            gui.draw(randomRendered);

            // Returns the nearest node to the random point.
            RRNode nearest = tree.getNearestNeighbour(randomGoal);

            //Calculate position of new node based on angle + position of nearest and random
            IntPoint nearestPoint = new IntPoint(nearest.x, nearest.y);

            if (distance(nearestPoint, randomGoal) < stepSize) continue; //Ignore the step if it's too close

            IntPoint newNode = getNode(nearestPoint, randomGoal, stepSize);

            //Don't walk too close to obstacles - stay 2*radius away, if possible
            if (intersectsObstacle(newNode, nearestPoint) || closestObstacle(newNode) < 2 * robotRadius) continue;

            //Add the new node to the tree
            tree.addNode(nearest, newNode);

            //Highlight the current best path
            gui.unDraw(goalPathRender);
            ArrayList<IntPoint> goalPath = tree.getPathFromRootTo(tree.getNearestNeighbour(goal));
            goalPathRender = new ArrayList<Renderable>();
            for (IntPoint node : goalPath) {
                RenderablePoint r = new RenderablePoint(node.x, node.y);
                r.setProperties(Color.RED, 10f);
                goalPathRender.add(r);
            }
            gui.draw(goalPathRender);

            //Draw used/unused node ratio
            gui.unDraw(rs3);
            rs3.string = "Used Nodes/Total Nodes: " + goalPath.size() + "/" + ++nodes;
            gui.draw(rs3);
            gui.update();

            //Check for win
            fin = isInGoal(newNode, goal, goalRadius);
        } while (!fin);
        //Now make a robot and have him follow the path
        PotentialFieldsRobot rob = new PotentialFieldsRobot(null, start, goal, robotRadius, 200, 90, 2 * robotRadius + 50, obstacles, 0);
        double headingR = rob.calculateHeading(goal);
        rob.setHeading(headingR);
        RenderablePolyline line = new RenderablePolyline();
        line.setProperties(Color.RED, 2f);
        ArrayList<IntPoint> goalPath = tree.getPathFromRootTo(tree.getNearestNeighbour(goal));
        goalPath.add(goal);
        int l = 0;
        for (int i = 0; i < goalPath.size(); i++) {
            rob.setGoal(goalPath.get(i));
            if (i == goalPath.size() - 1)
                rob.setGoalRadius(goalRadius); //Reset radius to original goalrad for last goal
            while (!rob.inGoal()) {
                Thread.sleep(25);
                gui.clearGraphicsPanel();
                //Draw the world
                gui.draw(goalPathRender);
                gui.draw(obstacles);
                gui.draw(tree);
                gui.draw(rs3);
                drawRobot(rob);
                //Draw the current goal path length
                gui.unDraw(rs);
                l += rob.getStepSize();
                rs.string = "Distance Travelled (pixels): " + l;
                gui.draw(rs);
                //Draw the current goal path smoothness
                gui.unDraw(rs2);
                l += rob.getStepSize();
                rs2.string = "Path Smoothness: " + calculateSmoothness(line);
                gui.draw(rs2);
                //Draw the current path
                line.addPoint(rob.getPosition().x, rob.getPosition().y);
                gui.draw(line);
                gui.update();
                rob.move(); //Take a step
            }
            gui.clearGraphicsPanel();
            //Draw the world
            gui.draw(goalPathRender);
            gui.draw(obstacles);
            gui.draw(tree);
            gui.draw(rs3);
            drawRobot(rob);
            //Draw the current goal path length
            gui.unDraw(rs);
            l += rob.getStepSize();
            rs.string = "Distance Travelled (pixels):" + l;
            gui.draw(rs);
            //Draw the current goal path smoothness
            gui.unDraw(rs2);
            l += rob.getStepSize();
            rs2.string = "Path Smoothness: " + calculateSmoothness(line);
            gui.draw(rs2);
            //Draw the current path
            line.addPoint(rob.getPosition().x, rob.getPosition().y);
            gui.draw(line);
            gui.update();
        }

        //Print the metrics to the console
        System.out.println("Used Nodes/Total Nodes: " + goalPath.size() + "/" + ++nodes);
        System.out.println("Distance Travelled (pixels): " + l);
        System.out.println("Path Smoothness: " + calculateSmoothness(line));

        //Turn the buttons back on
        gui.setButtonEnabled(buttonId, true);
        gui.setButtonEnabled(circleSId, true);
        gui.setButtonEnabled(circleLId, true);
        gui.setButtonEnabled(squareSId, true);
        gui.setButtonEnabled(squareLId, true);
        gui.setButtonEnabled(randomLineId, true);
        gui.setButtonEnabled(clearObsId, true);
        gui.setButtonEnabled(easyCourseId, true);
        gui.setButtonEnabled(medCourseId, true);
        gui.setButtonEnabled(hardCourseId, true);

    }

    /**
     * Get the new node obtained from moving stepSize pixels from start towards end. If the
     * end point is >30 degrees turn from the current heading, it is ignored unless geometric movement is enabled
     * - the robot can only move in 30 degree (PI/6) steps to simulate continuous movement.
     *
     * @param start    The starting point
     * @param end      The goal for this step
     * @param stepSize How far to move towards the goal
     * @return The coordinates of the calculated position.
     **/
    private IntPoint getNode(IntPoint start, IntPoint end, int stepSize) {
        double angle = calculateHeading(start, end);
        double heading;
        //Get the current heading by finding the angle of the line formed by the previous two points on this branch
        ArrayList<IntPoint> pointsToStart = tree.getPathFromRootTo(tree.getNearestNeighbour(start));
        if (pointsToStart.size() < 2) heading = angle; //If at the start of the tree, we can go any direction
        else {
            IntPoint previousPoint = pointsToStart.get(pointsToStart.size() - 2); //Get the node before 'start' node
            heading = calculateHeading(previousPoint, start); //get the robot's heading
        }

        //If geometric movement is not enabled, enforce maximum of 30 degree turns
        if (!geomMove) {
            //If the required angle > 30 degrees, turn 30 degrees towards the point
            if (angleDifference(angle, heading) > Math.PI / 6) {
                //If it's closer to turn left than right, tuirn left, else turn right
                if (angleDifference(angle, mod(heading + (Math.PI / 6), 2 * Math.PI)) <=
                        angleDifference(angle, mod(heading - (Math.PI / 6), 2 * Math.PI)))
                    angle = mod(heading + (Math.PI / 6), 2 * Math.PI);
                else
                    angle = mod(heading - (Math.PI / 6), 2 * Math.PI);
            }
        }

        //Get the number of pixels to travel along/up, and add to current heading
        int length = (int) (stepSize * Math.cos(angle));
        int height = (int) (stepSize * Math.sin(angle));
        return new IntPoint(start.x + length, start.y + height);
    }

    /**
     * Find the heading that the robot must be on to travel between 'start' and 'end' coordinates
     */
    private double calculateHeading(IntPoint start, IntPoint end) {
        double grad = Math.abs(((double) end.y - (double) start.y)
                / ((double) end.x - (double) start.x));
        double angle = Math.atan(grad);

        //The previous calculation only gives the absolute angle - use a bit of trigonometric know-how to get the actual angle
        if (end.x - start.x < 0) {
            if (end.y - start.y < 0) {
                angle = Math.PI + angle;
            } else {
                angle = Math.PI - angle;
            }
        } else {
            if (end.y - start.y < 0) {
                angle = (Math.PI * 2) - angle;
            }
        }

        return angle;
    }

    /**
     * Check if the line between 'start' and 'goal' intersects an obstacle. This is achieved by
     * turning the shapes into their constituent polygons. Each polygon is then broken down into a
     * set of lines. These lines are compared with the start/goal line to determine if they intersect.
     **/
    private boolean intersectsObstacle(IntPoint start, IntPoint goal) {
        Line2D.Double line = new Line2D.Double();
        line.setLine(start.x, start.y, goal.x, goal.y);
        for (Renderable obstacle : obstacles) {
            if (obstacle.getClass() == RenderablePolyline.class) {
                //Polylines are broken up into regular lines
                ArrayList<Integer> xs = ((RenderablePolyline) obstacle).xPoints;
                ArrayList<Integer> ys = ((RenderablePolyline) obstacle).yPoints;
                for (int i = 0; i < xs.size() - 1; i++) {
                    Line2D.Double obsLine = new Line2D.Double(xs.get(i), ys.get(i), xs.get(i + 1), ys.get(i + 1));
                    if (line.intersectsLine(obsLine)) return true;
                }
            } else if (obstacle.getClass() == RenderableRectangle.class) {
				/* Rectangle is treated like a polygon but since because it's a
				   different class it has to be handled separately - we've got to construct the
				   polypoints separately (annoyingly) */
                ArrayList<Integer> xs = new ArrayList<Integer>();
                ArrayList<Integer> ys = new ArrayList<Integer>();
                xs.add(((RenderableRectangle) obstacle).bottomLeftX);
                xs.add(((RenderableRectangle) obstacle).bottomLeftX);
                xs.add(((RenderableRectangle) obstacle).bottomLeftX + ((RenderableRectangle) obstacle).width);
                xs.add(((RenderableRectangle) obstacle).bottomLeftX + ((RenderableRectangle) obstacle).width);

                ys.add(((RenderableRectangle) obstacle).bottomLeftY);
                ys.add(((RenderableRectangle) obstacle).bottomLeftY + ((RenderableRectangle) obstacle).height);
                ys.add(((RenderableRectangle) obstacle).bottomLeftY + ((RenderableRectangle) obstacle).height);
                ys.add(((RenderableRectangle) obstacle).bottomLeftY);

                for (int i = 0; i < xs.size(); i++) {
                    Line2D.Double obsLine = new Line2D.Double(xs.get(i), ys.get(i),
                            xs.get((i + 1) % xs.size()), ys.get((i + 1) % ys.size()));
                    if (line.intersectsLine(obsLine)) return true;
                }

            } else if (obstacle.getClass() == RenderablePolygon.class) {
                ArrayList<Integer> xs = ((RenderablePolygon) obstacle).xPoints;
                ArrayList<Integer> ys = ((RenderablePolygon) obstacle).yPoints;
                for (int i = 0; i < xs.size(); i++) {
                    Line2D.Double obsLine = new Line2D.Double(xs.get(i), ys.get(i),
                            xs.get((i + 1) % xs.size()), ys.get((i + 1) % ys.size()));
                    if (line.intersectsLine(obsLine)) return true;
                }
            } else if (obstacle.getClass() == RenderableOval.class) {
                //ovals are treated as their bounding polygons (90-sided is sufficient) and they have to be circles
                ArrayList<Integer> xs = new ArrayList<Integer>();
                ArrayList<Integer> ys = new ArrayList<Integer>();
                RenderableOval roval = (RenderableOval) obstacle;

                for (int i = 0; i < 90; i++) {
                    int trigPoint = (int) (roval.width / 2 * Math.cos(i * Math.PI / 45));
                    xs.add(roval.centreX + trigPoint);
                }

                for (int i = 0; i < 90; i++) {
                    int trigPoint = (int) (roval.width / 2 * Math.sin(i * Math.PI / 45));
                    ys.add(roval.centreY + trigPoint);
                }

                for (int i = 0; i < xs.size(); i++) {
                    Line2D.Double obsLine = new Line2D.Double(xs.get(i), ys.get(i),
                            xs.get((i + 1) % xs.size()), ys.get((i + 1) % ys.size()));
                    if (line.intersectsLine(obsLine)) return true;
                }
            }
        }
        return false;
    }

    /**
     * Entry point for this class - make the GUI visible.
     **/
    public void runRobot() {
        gui.show();
    }

    /**
     * Draw the robot, it's sensors (in green), and all of the points it can move to (in blue)
     */
    private void drawRobot(PotentialFieldsRobot rob) {
        gui.draw(rob.getImage());
        for (IntPoint p : rob.getSensorablePoints()) {
            RenderablePolyline r = new RenderablePolyline();
            r.addPoint(rob.getPosition().x, rob.getPosition().y);
            r.addPoint(p.x, p.y);
            r.setProperties(Color.GREEN, 1f);
            RenderablePoint pp = new RenderablePoint(p.x, p.y);
            pp.setProperties(Color.GREEN, 5f);
            gui.draw(r);
            gui.draw(pp);
        }
        for (IntPoint p : rob.getSamplePoints()) {
            RenderablePolyline r = new RenderablePolyline();
            r.addPoint(rob.getPosition().x, rob.getPosition().y);
            r.addPoint(p.x, p.y);
            r.setProperties(Color.BLUE, 3f);
            RenderablePoint pp = new RenderablePoint(p.x, p.y);
            pp.setProperties(Color.BLUE, 6f);
            gui.draw(r);
            gui.draw(pp);
        }
    }

    /**
     * Get the closest point where this line crosses an obstacle - this varies based on the obstacle type
     * In general, this is achieved by turning the obstacle into a series of lines and calling
     * getIntersectionPoint() on the target line and each of the polygon's lines. Once all intersection
     * points are found, the closest to the robot is returned. It is assumed all polygons are convex.
     */
    private double closestObstacle(IntPoint coords) {
        double closest = Integer.MAX_VALUE;
        for (Renderable obstacle : obstacles) {
            if (obstacle.getClass() == RenderablePolyline.class) {
                ArrayList<Integer> xs = ((RenderablePolyline) obstacle).xPoints;
                ArrayList<Integer> ys = ((RenderablePolyline) obstacle).yPoints;
                for (int i = 0; i < xs.size() - 1; i++) {
                    double d = distanceToClosestObstacle(new IntPoint(xs.get(i), ys.get(i)), coords,
                            new IntPoint(xs.get(i + 1), ys.get(i + 1)));
                    if (d < closest) closest = d;
                }
            } else if (obstacle.getClass() == RenderableRectangle.class) {
                /* Rectangle is treated like a polygon but since because it's a
                 * different class it has to be handled separately - we've got to construct the
                 * polypoints separately (annoyingly)*/
                ArrayList<Integer> xs = new ArrayList<Integer>();
                ArrayList<Integer> ys = new ArrayList<Integer>();
                xs.add(((RenderableRectangle) obstacle).bottomLeftX);
                xs.add(((RenderableRectangle) obstacle).bottomLeftX);
                xs.add(((RenderableRectangle) obstacle).bottomLeftX + ((RenderableRectangle) obstacle).width);
                xs.add(((RenderableRectangle) obstacle).bottomLeftX + ((RenderableRectangle) obstacle).width);

                ys.add(((RenderableRectangle) obstacle).bottomLeftY);
                ys.add(((RenderableRectangle) obstacle).bottomLeftY + ((RenderableRectangle) obstacle).height);
                ys.add(((RenderableRectangle) obstacle).bottomLeftY + ((RenderableRectangle) obstacle).height);
                ys.add(((RenderableRectangle) obstacle).bottomLeftY);

                for (int i = 0; i < xs.size(); i++) {
                    double d = distanceToClosestObstacle(new IntPoint(xs.get(i), ys.get(i)), coords,
                            new IntPoint(xs.get((i + 1) % xs.size()), ys.get((i + 1) % ys.size())));
                    if (d < closest) closest = d;
                }

            } else if (obstacle.getClass() == RenderablePolygon.class) {
                ArrayList<Integer> xs = ((RenderablePolygon) obstacle).xPoints;
                ArrayList<Integer> ys = ((RenderablePolygon) obstacle).yPoints;
                for (int i = 0; i < xs.size(); i++) {
                    distanceToClosestObstacle(new IntPoint(xs.get(i), ys.get(i)), coords,
                            new IntPoint(xs.get((i + 1) % xs.size()), ys.get((i + 1) % ys.size())));
                }
            } else if (obstacle.getClass() == RenderableOval.class) {
                //ovals are treated as their bounding polygons (90-sided) and they have to be circles
                ArrayList<Integer> xs = new ArrayList<Integer>();
                ArrayList<Integer> ys = new ArrayList<Integer>();
                RenderableOval roval = (RenderableOval) obstacle;

                for (int i = 0; i < 90; i++) {
                    int trigPoint = (int) (roval.width / 2 * Math.cos(i * Math.PI / 45));
                    xs.add(roval.centreX + trigPoint);
                }

                for (int i = 0; i < 90; i++) {
                    int trigPoint = (int) (roval.width / 2 * Math.sin(i * Math.PI / 45));
                    ys.add(roval.centreY + trigPoint);
                }

                for (int i = 0; i < xs.size(); i++) {
                    double d = distanceToClosestObstacle(new IntPoint(xs.get(i), ys.get(i)), coords,
                            new IntPoint(xs.get((i + 1) % xs.size()), ys.get((i + 1) % ys.size())));
                    if (d < closest) closest = d;
                }

            }
        }
        return closest;
    }

    private double distanceToClosestObstacle(IntPoint p1, IntPoint pmid, IntPoint p2) {
        // Return minimum distance between line segment vw and point p
        double l2 = Math.pow(p2.x - p1.x, 2) + Math.pow(p2.y - p1.y, 2);  // i.e. |w-v|^2 -  avoid a sqrt
        if (l2 == 0) return distance(pmid, p1);   // v == w case
        // Consider the line extending the segment, parameterized as v + t (w - v).
        // We find projection of point p onto the line.
        // It falls where t = [(p-v) . (w-v)] / |w-v|^2
        double t = ((pmid.x - p1.x) * (p2.x - p1.x) + (pmid.y - p1.y) * (p2.y - p1.y)) / l2;
        if (t < 0) return distance(pmid, p1);       // Beyond the 'v' end of the segment
        else if (t > 1) return distance(pmid, p2);  // Beyond the 'w' end of the segment
        IntPoint projection = new IntPoint((int) Math.round(p1.x + t * (p2.x - p1.x)), (int) Math.round(p1.y + t * (p2.y - p1.y)));  // Projection falls on the segment
        return distance(pmid, projection);
    }

    /**
     * Evaluate the smoothness of the line so far
     **/
    private double calculateSmoothness(RenderablePolyline line) {
        if (line.xPoints.size() < 20) return 0;
        double totalDiff = 0;
        for (int i = 0; i < line.xPoints.size() - 20; i += 10) {
            IntPoint p1 = new IntPoint(line.xPoints.get(i), line.yPoints.get(i));
            IntPoint pmid = new IntPoint(line.xPoints.get(i + 10), line.yPoints.get(i + 10));
            IntPoint p2 = new IntPoint(line.xPoints.get(i + 20), line.yPoints.get(i + 20));
            Line2D pline = new Line2D.Double();
            pline.setLine(p1.x, p1.y, p2.x, p2.y);

            double distance = Math.abs((p2.y - p1.y) * pmid.x - (p2.x - p1.x) * pmid.y + p2.x * p1.y - p2.y * p1.x)
                    / Math.sqrt(Math.pow(p2.y - p1.y, 2) + Math.pow(p2.x - p1.x, 2));
            totalDiff += distance;
        }
        return totalDiff / line.xPoints.size();
    }

}
