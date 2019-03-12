import dataStructures.RRTree;
import easyGui.EasyGui;
import geometry.IntPoint;
import renderables.*;

import java.awt.*;
import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.Random;

public class PotentialFields {

    //------------//
    // Attributes //
    //------------//

    static final int frameLength = 1200;
    static final int frameHeight = 900;
    static final int graphicsHeight = 700;
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
    private final int newobsticalShape;
    private final int enMikeId;
    private final int disMikeId;
    private final int enArcDrawingId;
    private final int disArcDrawingId;
    private final int enArcId;
    private final int disArcId;
    private final int enFPId;
    private final int startXId;
    private final int startYId;
    //private final int powerId;
    //private final int goalId;
    //private final int boxId;
    private final int goalXId;
    private final int goalYId;
    private final int goalRadiusId;
    private final int robotRadiusId;
    private final int robotSensorRangeId;
    private final int robotSensorDensityId;
    private final int robotSpeedId;
    private final int headingR;
    private EasyGui newGUI;
    private int buttonIdNEW;
    private int x1newID, x2newID, x3newID, x4newID, x5newID;
    private int y1newID, y2newID, y3newID, y4newID, y5newID;
    private int messageLabel;
    private boolean arcs;
    private boolean mike;

    private int ArcPlannerInt;


    private boolean FractionalProgress;
    private ArrayList<Renderable> obstacles;

    private boolean stop;

    //-------------//
    // Constructor //
    //-------------//

    public PotentialFields() {
        // Set up the GUI, labels, buttons
        gui = new EasyGui(frameLength, frameHeight);

        buttonId = gui.addButton(5, 5, "Go Little Robot!", this, "buttonAction");
        gui.addButton(2, 5, "Clear Fields", this, "clearButtonAction");

        gui.addLabel(0, 0, "Starting X:");
        startXId = gui.addTextField(0, 1, null);
        gui.addLabel(1, 0, "Starting Y:");
        startYId = gui.addTextField(1, 1, null);

        gui.addLabel(0, 2, "Goal X:");
        goalXId = gui.addTextField(0, 3, null);
        gui.addLabel(1, 2, "Goal Y:");
        goalYId = gui.addTextField(1, 3, null);


        gui.addLabel(1, 4, "heading:");
        headingR = gui.addTextField(1, 5, null);


        gui.addLabel(0, 4, "Goal Radius:");
        goalRadiusId = gui.addTextField(0, 5, null);

        gui.addLabel(0, 6, "Robot Radius:");
        robotRadiusId = gui.addTextField(0, 7, null);

        gui.addLabel(1, 6, "Robot Sensor Range:");
        robotSensorRangeId = gui.addTextField(1, 7, null);

        gui.addLabel(1, 8, "Robot Sensor Density:");
        robotSensorDensityId = gui.addTextField(1, 9, null);

        gui.addLabel(0, 8, "Robot Speed (moves/second):");
        robotSpeedId = gui.addTextField(0, 9, null);

        gui.addLabel(7, 1, "Plannar: ");
        enArcId = gui.addButton(7, 2, "Arc", this, "enableArc");
        disArcId = gui.addButton(7, 3, "Euclidean", this, "disableArc");
        enFPId = gui.addButton(7, 4, "Fractional Progress", this, "fractionalProgress");
        gui.setButtonEnabled(disArcId, false);


        // More options
        gui.addLabel(7, 5, "Arc drawing: ");
        enArcDrawingId = gui.addButton(7, 6, "Arcs", this, "enableArcDrawing");
        disArcDrawingId = gui.addButton(7, 7, "Lines", this, "disableArcDrawing");
        gui.setButtonEnabled(disArcDrawingId, false);
        arcs = false;

        // More options
        gui.addLabel(5, 0, "Mike Mode: ");
        enMikeId = gui.addButton(5, 1, "On", this, "enableMike");
        disMikeId = gui.addButton(5, 2, "Off", this, "disableMike");
        gui.setButtonEnabled(disMikeId, false);
        mike = false;

        gui.addLabel(0, -1, "Leave fields blank for random values!");

        // Pre-made courses
        gui.addLabel(3, 0, "Select a pre-made course: ");
        easyCourseId = gui.addButton(3, 1, "Easy", this, "easyCourse");
        medCourseId = gui.addButton(3, 2, "Medium", this, "medCourse");
        hardCourseId = gui.addButton(3, 3, "Hard", this, "hardCourse");
        newobsticalShape = gui.addButton(3, 4, "5-point obstacle ", this, "newobsticalShape");

        // Custom obstacles
        gui.addLabel(4, 0, "Or add in your own obstacles: ");
        circleSId = gui.addButton(4, 1, "Circle (S)", this, "genCircleS");

        circleLId = gui.addButton(4, 2, "Circle (L)", this, "genCircleL");
        squareSId = gui.addButton(4, 3, "Square (S)", this, "genSquareS");
        squareLId = gui.addButton(4, 4, "Square (L)", this, "genSquareL");
        randomLineId = gui.addButton(4, 5, "Line", this, "genLine");
        clearObsId = gui.addButton(4, 6, "Clear Obstacles", this, "clearObs");

        gui.addButton(7, 8, "Pause", this, "pause");
        gui.addButton(6, 8, "Quit", this, "quit");


//		
        obstacles = new ArrayList<Renderable>();
    }

    //---------//
    // Methods //
    //---------//

    // Control:

    public static boolean getPoints(Vector[] points, String... numberString) {
        boolean isNumberAll = true;

        for (int i = 0; i < numberString.length - 1; i = i + 2) {

            try {
                points[i / 2] = new Vector(Integer.parseInt(numberString[i]), Integer.parseInt(numberString[i + 1]));

            } catch (Exception e) {
                System.out.println("e" + e);
                return false;
            }
        }

        return isNumberAll;

    }

    /**
     * Generate a random point in 2D space in the range ([0-maxX,], [0-maxY]) for
     * obstacle creation
     */
    private static IntPoint randomPoint(int maxX, int maxY) {
        Random rand = new Random();
        IntPoint point = new IntPoint();
        point.x = rand.nextInt(maxX + 1);
        point.y = rand.nextInt(maxY + 1);
        return point;
    }

    public void runRobot() {
        gui.show();
    }

    // Mike:

    public void pause() {
        stop = !stop;
        setButtons(stop);
    }

    public void quit() {
        gui.hide();
        System.exit(0);
    }

    public void enableMike() {
        setMike(true);
    }

    public void disableMike() {
        setMike(false);
    }

    private void setMike(boolean mike) {
        this.mike = mike;
        gui.setButtonEnabled(enMikeId, !mike);
        gui.setButtonEnabled(disMikeId, mike);
    }

    public void enableArcDrawing() {
        setArcDrawing(true);
    }

    public void disableArcDrawing() {
        setArcDrawing(false);
    }

    private void setArcDrawing(boolean arcs) {
        this.arcs = arcs;
        gui.setButtonEnabled(enArcDrawingId, !arcs);
        gui.setButtonEnabled(disArcDrawingId, arcs);
    }

    public void enableArc() {
        setPlanarButtons(1);
    }

    public void disableArc() {
        setPlanarButtons(2);
    }

    public void fractionalProgress() {
        setPlanarButtons(3);
    }

    // Pre-made courses:

    private void setPlanarButtons(int planar) {
        this.ArcPlannerInt = planar;

        switch (ArcPlannerInt) {
            case 1:
                gui.setButtonEnabled(enArcId, false);
                gui.setButtonEnabled(disArcId, true);
                gui.setButtonEnabled(enFPId, true);
                break;

            case 2:
                gui.setButtonEnabled(enArcId, true);
                gui.setButtonEnabled(disArcId, false);
                gui.setButtonEnabled(enFPId, true);
                break;

            case 3:
                gui.setButtonEnabled(enArcId, true);
                gui.setButtonEnabled(disArcId, true);
                gui.setButtonEnabled(enFPId, false);
                break;
        }


    }

    /**
     * Action when 'clear fields' button is pressed - reset all text fields in the GUI.
     **/
    public void clearButtonAction() {
        gui.setTextFieldContent(startXId, "");
        gui.setTextFieldContent(startYId, "");
        gui.setTextFieldContent(goalXId, "");
        gui.setTextFieldContent(goalYId, "");
        gui.setTextFieldContent(goalRadiusId, "");
        gui.setTextFieldContent(robotRadiusId, "");
        gui.setTextFieldContent(robotSensorRangeId, "");
        gui.setTextFieldContent(robotSensorDensityId, "");
        gui.setTextFieldContent(robotSpeedId, "");
    }

    /**
     * Set up the 'easy' pre-made course
     */
    public void easyCourse() {
        clearObs();
        setStart("100", "100");
        setGoal("1000", "500");

        RenderableOval r = new RenderableOval(frameLength / 2, graphicsHeight / 2, 150, 150);
        r.setProperties(Color.DARK_GRAY, 1f, true);
        setupObstacle(r);
    }

    /**
     * Set up the 'medium' pre-made course
     */
    public void medCourse() {
        clearObs();
        setStart("0", "0");
        setGoal("1400", "700");

        int[][] params = {{150, 150, 150, 150}, {400, 400, 150, 150}, {700, 425, 150, 150}, {400, 250, 50, 50},
                {600, 100, 150, 150}, {1000, 600, 150, 150}, {900, 250, 50, 50}, {1000, 450, 50, 50},
                {1150, 350, 50, 50}};

        setObstacles(params);
        gui.draw(obstacles);
        gui.update();
    }

    public void newobsticalShape() {
        newGUI = new EasyGui(100, 100);
        newGUI.addLabel(0, 0, "Enter the coordinates of the five points : ");

        newGUI.addLabel(1, 0, "Point1 X  : ");

        x1newID = newGUI.addTextField(1, 1, null);
        newGUI.addLabel(1, 2, "Point1 Y  : ");
        y1newID = newGUI.addTextField(1, 3, null);

        newGUI.addLabel(2, 0, "Point2 X  : ");
        x2newID = newGUI.addTextField(2, 1, null);
        newGUI.addLabel(2, 2, "Point2 Y  : ");
        y2newID = newGUI.addTextField(2, 3, null);


        newGUI.addLabel(3, 0, "Point3 X  : ");
        x3newID = newGUI.addTextField(3, 1, null);
        newGUI.addLabel(3, 2, "Point3 Y  : ");
        y3newID = newGUI.addTextField(3, 3, null);

        newGUI.addLabel(4, 0, "Point4 X  : ");
        x4newID = newGUI.addTextField(4, 1, null);
        newGUI.addLabel(4, 2, "Point4 Y  : ");
        y4newID = newGUI.addTextField(4, 3, null);

        newGUI.addLabel(5, 0, "Point5 X  : ");
        x5newID = newGUI.addTextField(5, 1, null);
        newGUI.addLabel(5, 2, "Point5 Y  : ");
        y5newID = newGUI.addTextField(5, 3, null);

        messageLabel = newGUI.addLabel(6, 0, "   ");

        buttonIdNEW = newGUI.addButton(7, 1, "done", this, "createObstical");

        newGUI.show();


    }

    public void createObstical() {
        String[] allVals = new String[10];
        allVals[0] = newGUI.getTextFieldContent(x1newID);
        allVals[1] = newGUI.getTextFieldContent(y1newID);
        allVals[2] = newGUI.getTextFieldContent(x2newID);
        allVals[3] = newGUI.getTextFieldContent(y2newID);
        allVals[4] = newGUI.getTextFieldContent(x3newID);
        allVals[5] = newGUI.getTextFieldContent(y3newID);
        allVals[6] = newGUI.getTextFieldContent(x4newID);
        allVals[7] = newGUI.getTextFieldContent(y4newID);
        allVals[8] = newGUI.getTextFieldContent(x5newID);
        allVals[9] = newGUI.getTextFieldContent(y5newID);
        Vector points[] = new Vector[5];

        boolean success = getPoints(points, allVals);

        if (!success) {
            newGUI.setLabelText(messageLabel, "All Field should be integers");
            return;
        }


        RenderablePolyline line = new RenderablePolyline();
        int[][] linePoints = {{(int) points[0].x, (int) points[0].y}, {(int) points[1].x, (int) points[1].y}, {(int) points[2].x, (int) points[2].y}, {(int) points[3].x, (int) points[3].y}, {(int) points[4].x, (int) points[4].y}};
        for (int[] p : linePoints)
            line.addPoint(p[0], p[1]);

        line.setProperties(Color.DARK_GRAY, 2f);
        setupObstacle(line);

        newGUI.hide();


        // if (  )


    }

    /**
     * Set up the 'hard' pre-made course
     */
    public void hardCourse() {
        clearObs();
        setStart("0", "0");
        setGoal("1500", "950");
        gui.setTextFieldContent(goalRadiusId, "20");

        int[][] params = {{150, 200, 150, 150}, {400, 400, 150, 150}, {400, 250, 50, 50}, {600, 100, 150, 150},
                {900, 350, 50, 50}};
        setObstacles(params);
        gui.draw(obstacles);
        gui.update();

        RenderablePolyline line = new RenderablePolyline();
        int[][] linePoints = {{800, 700}, {800, 900}, {1300, 800}, {1400, 300}, {1200, 350}};
        for (int[] p : linePoints) {
            line.addPoint(p[0], p[1]);
        }
        line.setProperties(Color.DARK_GRAY, 2f);
        setupObstacle(line);
    }

    /**
     * Set up initial robot position.
     */
    private void setStart(String x, String y) {
        gui.setTextFieldContent(startXId, x);
        gui.setTextFieldContent(startYId, y);
    }

    /**
     * Set up goal position.
     */
    private void setGoal(String x, String y) {
        gui.setTextFieldContent(goalXId, x);
        gui.setTextFieldContent(goalYId, y);
    }

    private void setObstacles(int[][] params) {
        for (int[] square : params) {
            RenderableRectangle r = new RenderableRectangle(square[0], square[1], square[2], square[3]);
            r.setProperties(Color.DARK_GRAY, 1f, true, false);
            obstacles.add(r);
        }
    }

    private void setupObstacle(Renderable r) {
        obstacles.add(r);
        gui.draw(r);
        gui.update();
    }

    /* Methods to generate random obstacles - circles, squares and lines */
    public void genCircleS() {
        genCircle(50, 50);
    }

    public void genCircleL() {
        genCircle(150, 150);
    }

    private void genCircle(int width, int height) {
        IntPoint centre = randomPoint(frameLength, frameHeight);
        RenderableOval o = new RenderableOval(centre.x, centre.y, width, height);
        o.setProperties(Color.MAGENTA, 1f, true);
        setupObstacle(o);
    }

    public void genSquareS() {
        IntPoint origin = randomPoint(frameLength - 50, frameHeight - 5);
        RenderableRectangle r = new RenderableRectangle(origin.x, origin.y, 50, 50);
        r.setProperties(Color.CYAN, 1f, true, false);
        setupObstacle(r);
    }

    public void genSquareL() {
        IntPoint origin = randomPoint(frameLength - 150, frameHeight - 150);
        RenderableRectangle r = new RenderableRectangle(origin.x, origin.y, 150, 150);
        r.setProperties(Color.CYAN, 1f, true, false);
        setupObstacle(r);
    }

    public void genLine() {
        IntPoint p1 = randomPoint(frameLength - 200, frameHeight - 200);
        IntPoint p2 = randomPoint(frameLength - 200, frameHeight - 200);
        RenderablePolyline p = new RenderablePolyline();
        p.addPoint(p1.x, p1.y);
        p.addPoint(p2.x, p2.y);
        p.setProperties(Color.ORANGE, 2f);
        setupObstacle(p);
    }

    /**
     * Clear all obstacles from the screen
     **/
    public void clearObs() {
        gui.unDraw(obstacles);
        obstacles = new ArrayList<Renderable>();
        gui.update();
    }

    private int getProp(String var, Random rand, int randParam) {
        if (var.isEmpty())
            return rand.nextInt(randParam);
        return Integer.parseInt(var);
    }

    /**
     * Get the parameters from the text fields and use these to set the robot moving
     **/
    public void buttonAction() throws InterruptedException {
        int startX, startY, goalX, goalY, radius, robotRadius, robotSensorRange, robotSensorDensity, robotSpeed, power, box, goal;
        Random rand = new Random();
        String startXs = gui.getTextFieldContent(startXId);
        String startYs = gui.getTextFieldContent(startYId);
        String goalXs = gui.getTextFieldContent(goalXId);
        String goalYs = gui.getTextFieldContent(goalYId);
        String radiuss = gui.getTextFieldContent(goalRadiusId);
        String robotRadiuss = gui.getTextFieldContent(robotRadiusId);
        String robotSensorRanges = gui.getTextFieldContent(robotSensorRangeId);
        String robotSensorDensitys = gui.getTextFieldContent(robotSensorDensityId);
        String robotSpeeds = gui.getTextFieldContent(robotSpeedId);
//		String powers = gui.getTextFieldContent(powerId);
//		String boxes = gui.getTextFieldContent(boxId);
//		String goals = gui.getTextFieldContent(goalId);
//                

        String headingStr = gui.getTextFieldContent(headingR);
        double headingD = 0.0;
        if (!headingStr.isEmpty())
            headingD = Double.parseDouble(headingStr);
        gui.setTextFieldContent(headingR, "" + headingD);


        startX = getProp(startXs, rand, frameLength);
        gui.setTextFieldContent(startXId, "" + startX);

        startY = getProp(startYs, rand, frameHeight);
        gui.setTextFieldContent(startYId, "" + startY);

        goalX = getProp(goalXs, rand, frameLength);
        gui.setTextFieldContent(goalXId, "" + goalX);

        goalY = getProp(goalYs, rand, frameHeight);
        gui.setTextFieldContent(goalYId, "" + goalY);

        if (radiuss.isEmpty())
            radius = rand.nextInt(70) + 30; // Radius is between 30 and 100
        else
            radius = Integer.parseInt(radiuss);
        gui.setTextFieldContent(goalRadiusId, "" + radius);

        if (robotRadiuss.isEmpty())
            robotRadius = rand.nextInt(20) + 20; // Robot radius between 20 and 40
        else
            robotRadius = Integer.parseInt(robotRadiuss);
        gui.setTextFieldContent(robotRadiusId, "" + robotRadius);

        if (robotSensorRanges.isEmpty())
            robotSensorRange = rand.nextInt(300) + 100; // between 50 and 200
        else
            robotSensorRange = Integer.parseInt(robotSensorRanges);
        gui.setTextFieldContent(robotSensorRangeId, "" + robotSensorRange);

        if (robotSensorDensitys.isEmpty())
            robotSensorDensity = rand.nextInt(175) + 5; // between 5 and 180
        else
            robotSensorDensity = Integer.parseInt(robotSensorDensitys);
        gui.setTextFieldContent(robotSensorDensityId, "" + robotSensorDensity);

        if (robotSpeeds.isEmpty())
            robotSpeed = 40; // Default speed is 40 moves per second
        else
            robotSpeed = Integer.parseInt(robotSpeeds);
        gui.setTextFieldContent(robotSpeedId, "" + robotSpeed);

        if (robotSpeeds.isEmpty())
            robotSpeed = 40; // Default speed is 40 moves per second
        else
            robotSpeed = Integer.parseInt(robotSpeeds);
        gui.setTextFieldContent(robotSpeedId, "" + robotSpeed);


        String image = mike ? "mike.png" : null;

        goLittleRobot(new IntPoint(startX, startY), new IntPoint(goalX, goalY), radius, robotRadius, robotSensorRange,
                robotSensorDensity, robotSpeed, image, /*power, goal, box,*/headingD);
    }

    /**
     * Set the robot moving towards a goal on the screen with set radius, step size,
     * etc.
     *
     * @param start              The coordinates of the starting point
     * @param goal               The coordinates of the goal
     * @param goalRad            The radius of the goal - if the robot falls within this, it wins
     * @param robotRadius        The width of the robot
     * @param robotSensorRange   How far the robot can 'see'
     * @param robotSensorDensity The number of sensor lines the robot can use
     * @param robotSpeed         The number of moves per second
     */
    public void goLittleRobot(IntPoint start, IntPoint goal, int goalRad, int robotRadius, int robotSensorRange,
                              int robotSensorDensity, int robotSpeed, String image,/* int power, int goalFactor, int box ,*/ double headingR) throws InterruptedException {
        // Disable all buttons while robot is active TODO
        setButtons(false);
        stop = false;

        // Create the robot, start & end points, renderables

        PotentialFieldsRobot rob = new PotentialFieldsRobot(image, start, goal, robotRadius, robotSensorRange,
                robotSensorDensity, goalRad, obstacles,/* power, goalFactor, box,*/headingR);

        RRTree startAndGoal = new RRTree(Color.black);
        startAndGoal.setStartAndGoal(start, goal, goalRad);
        RenderableString rs = null;
        RenderableString rs2 = null;
        RenderablePolyline path = new RenderablePolyline();
        path.setProperties(Color.BLACK, 1f);
        path.addPoint(start.x, start.y);

        // Draw the initial set up
        gui.clearGraphicsPanel();
        gui.draw(startAndGoal);
        gui.draw(path);
        gui.draw(obstacles);
        drawRobot(rob);
        gui.update();

        int l = 0;
        // Loop until the robot reaches the goal or gets stuck
        while (!rob.inGoal()) {
            do {
                Thread.sleep(1000 / robotSpeed);
            }
            while (stop);

            boolean move = true;

            switch (ArcPlannerInt) {
                case 1:
                    move = rob.ArcMove();
                    break;
                case 2:
                    move = rob.move();
                    break;
                case 3:
                    move = rob.fpMove();
            }

            // If robot has crashed:
            if (!move) {
                // Draw message to let the user know that Rob is stuck
                rs = new RenderableString(500, 500, "I'm Stuck :(");
                rs.setLayer(456);
                rs.setProperties(Color.BLUE, new Font(Font.SERIF, Font.BOLD, 32));
                gui.draw(rs);
                gui.update();
                break; // Stop as no actions are available
            }

            // Draw the path from start to Rob's position
            path.addPoint(rob.getPosition().x, rob.getPosition().y);
            gui.clearGraphicsPanel();
            gui.draw(startAndGoal);
            gui.draw(path);
            gui.draw(obstacles);
            drawRobot(rob);

            // Draw movement arcs
            if (true /* ArcPlanner*/) {
                drawArc(rob.getFirstArc(), Color.BLACK);
                if (ArcPlannerInt != 1 && !arcs) {


                    MyArc newArc = new MyArc(rob.getSecondArc().p1, rob.getThirdArc().p2, rob.getSecondArc().startHeading, true);


                    drawArc(newArc, Color.PINK);
                } else {
                    drawArc(rob.getSecondArc(), Color.PINK);
                    drawArc(rob.getThirdArc(), Color.DARK_GRAY);
                }
            }

            // Print path length so far
            gui.unDraw(rs);
            l += rob.getStepSize();
            rs = new RenderableString(820, 20, "Distance Travelled (pixels): " + l);
            rs.setLayer(456);
            rs.setProperties(Color.BLUE, new Font(Font.SERIF, Font.BOLD, 14));
            gui.draw(rs);

            // Print current goal path smoothness
            gui.unDraw(rs2);
            //l += rob.getStepSize();TODO
            rs2 = new RenderableString(820, 0, "Path Smoothness Rating: " + (calculateSmoothness(path)));
            rs2.setLayer(456);
            rs2.setProperties(Color.BLUE, new Font(Font.SERIF, Font.BOLD, 14));
            gui.draw(rs2);

            gui.update();
        }

        // Print metrics to console
        System.out.println("Distance Travelled (pixels): " + l);
        System.out.println("Path Smoothness: " + calculateSmoothness(path));

        // Re-enable buttons when finished
        setButtons(true);
    }

    private void drawArc(MyArc arc, Color c) {
        if (arc != null) {


            RenderablePolyline polyline = arc.getRenderablePolyline(arcs);

//			addVectorToLine(arc.getStartPoint(), polyline);
//			addVectorToLine(arc.getEndPoint(), polyline);
            polyline.setProperties(c, 5f);
            gui.draw(polyline);
        }
    }


    public void setButtons(boolean enabled) {
        gui.setButtonEnabled(buttonId, enabled);
        gui.setButtonEnabled(circleSId, enabled);
        gui.setButtonEnabled(circleLId, enabled);
        gui.setButtonEnabled(squareSId, enabled);
        gui.setButtonEnabled(squareLId, enabled);
        gui.setButtonEnabled(randomLineId, enabled);
        gui.setButtonEnabled(clearObsId, enabled);
        gui.setButtonEnabled(easyCourseId, enabled);
        gui.setButtonEnabled(medCourseId, enabled);
        gui.setButtonEnabled(hardCourseId, enabled);
        gui.setButtonEnabled(newobsticalShape, enabled);
    }

    /**
     * Draw the robot, its sensors (in green), and all of the points it can move to
     * (in blue)
     */
    private void drawRobot(PotentialFieldsRobot rob) {
        gui.draw(rob.getImage());
        try {
            Thread.sleep(1); // necessary when rendering images to give them time to load
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
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
     * Smoothness metric. 0 = completely smooth, high values = not very smooth
     **/
    private double calculateSmoothness(RenderablePolyline line) {
        if (line.xPoints.size() < 20)
            return 0;
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
