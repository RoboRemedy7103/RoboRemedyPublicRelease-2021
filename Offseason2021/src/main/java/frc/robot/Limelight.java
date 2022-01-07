package frc.robot;

import edu.wpi.first.networktables.*;

/**
 * Limelight Vision Control
 */
public class Limelight {

    public enum VisionPipeline {
        TargetOne, TargetTwo
    }

    private static final double LimeLightInchesLeftOfCenter = 9.75;
    private static final double LimeLightInchesFrontOfPivot = -2.0;

    private static RoboLog rLog = null;
    static int currentPipeline = 0;
    static VisionPipeline currentVisionPipeline = VisionPipeline.TargetOne;

    static void limelightPeriodic() {
    }

    static boolean isTargetFound() {
        double tv = getValue("tv", 0);
        return tv > 0.5;
    }

    private static double getTargetDistanceInInches() {
        double width;
        if (currentPipeline == 0) {
            width = 3360 / getValue("thor", 0);
        } else if (currentPipeline == 8) {
            width = 10153 / getValue("thor", 0);
        } else if (currentPipeline == 4) {
            width = 3170 / getValue("tvert", 0);
        } else if (currentPipeline == 5) {
            width = 1168 / getValue("thor", 0);
        } else {
            width = 0;
        }
        return width;
    }

    static double getAdjustedDistanceInInches() {
        return getTargetDistanceInInches() + LimeLightInchesFrontOfPivot;
    }

    static double getAdjustedTargetDistanceInInches(double gyroAngle, double facingAngle) {
        // Take the average between the unadjusted distance and the cosine-adjusted
        // difference
        double multiplier = 1 - 0.5 * (1 - Math.cos(Math.toRadians(gyroAngle - facingAngle)));
        double d = getTargetDistanceInInches();
        return d * multiplier + LimeLightInchesFrontOfPivot;
    }

    static double getAdjustedWallDistanceInInches(double gyroAngle, double facingAngle) {
        return getAdjustedTargetDistanceInInches(gyroAngle, facingAngle)
                * Math.cos(Math.toRadians(gyroAngle - facingAngle));
    }

    public static double getTargetHorizontalAngle() {
        return getValue("tx", 0);
    }

    static double getTargetVerticalAngle() {
        return getValue("ty", 0);
    }

    static double getInchesRightOfRobot(double gyroAngle, double facingAngle) {
        return (getAdjustedWallDistanceInInches(gyroAngle, facingAngle)
                * Math.tan(Math.toRadians(getTargetHorizontalAngle())) - LimeLightInchesLeftOfCenter);
    }

    static void printRawValues() {
        double tx = getValue("tx", 999);
        double ty = getValue("ty", 999);
        double thor = getValue("thor", 999);
        double tvert = getValue("tvert", 999);
        System.out.println("tx:" + tx + " ty:" + ty + " thor:" + thor + " tvert:" + tvert);
    }

    public static double getLatencyInSeconds() {
        return getValue("tl", 0);
    }

    public static void setLEDON() {
        setValue("ledMode", 3);
    }

    public static void setLEDOFF() {
        setValue("ledMode", 1);
    }

    public static void setPipeline(VisionPipeline pipeline) {
        if (pipeline == currentVisionPipeline) {
            return;
        }
        currentVisionPipeline = pipeline;

        if (pipeline == VisionPipeline.TargetOne) {
            currentPipeline = 4;
        } else if (pipeline == VisionPipeline.TargetTwo) {
            currentPipeline = 8;
        }
        setValue("pipeline", currentPipeline);
        if (rLog != null)
            rLog.print("New Vision Pipeline: " + currentVisionPipeline + "," + currentPipeline);
    }

    public static double getValue(String key, double defaultValue) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        return table.getEntry(key).getDouble(defaultValue);
    }

    private static boolean setValue(String key, int value) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        return table.getEntry(key).setNumber(value);
    }

    public static void setCameraMode1() {
        setValue("stream", 1); // Limelight big, driver is PIP
    }

    public static void setCameraMode2() {
        setValue("stream", 2); // Driver big, Limelight is PIP
    }

    public static void setRobotLog(RoboLog robotLog) {
        rLog = robotLog;
    }
}