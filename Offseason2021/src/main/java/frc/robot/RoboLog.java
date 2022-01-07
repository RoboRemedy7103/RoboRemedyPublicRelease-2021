package frc.robot;

import edu.wpi.first.wpilibj.*;

public class RoboLog {

    private Timer timer = new Timer();
    private String shortDescription = "INIT";

    public RoboLog() {
        timer.start();
    }

    public void setRobotMode(String robotMode, String longDescription) {
        print("New Mode: " + longDescription);
        shortDescription = robotMode;
        timer.reset();
    }

    public void print(String s) {
        System.out.printf("%3.3f ", timer.get());
        System.out.print(shortDescription + ": ");
        System.out.println(s);
    }

    public void printf(String format, Object... arguments) {
        System.out.printf("%3.3f ", timer.get());
        System.out.print(shortDescription + ": ");
        System.out.printf(format, arguments);
        System.out.println();
    }

}