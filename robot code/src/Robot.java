


import easyGui.EasyGui;

public class Robot {
    private final EasyGui gui;

    public Robot() {
        gui = new EasyGui(100, 100);

        gui.addLabel(0, 0, "Select an algorithm: ");

        gui.addButton(1, 0, "RRT", this, "rrt");

        gui.addButton(1, 1, "Potential Fields", this, "potentialFields");

    }

    // MAIN
    public static void main(String[] args) {
        Robot robot = new Robot();
        robot.runProject();
    }

    public void runProject() {
        gui.show();
    }

    public void rrt() {
        gui.hide();
        RRT rt = new RRT();
        rt.runRobot();
    }

    public void potentialFields() {
        gui.hide();
        PotentialFields fields = new PotentialFields();
        fields.runRobot();
    }
}

        
        

