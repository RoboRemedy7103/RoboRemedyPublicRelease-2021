/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import frc.robot.Limelight.VisionPipeline;

/**
 * This class is for testing each motor and sensor as it is connected without
 * affecting our existing code and to make it easy to test various parts of the
 * robot during the season as we have issues or replace parts
 */
public class TestRobot {

    Electronics e;
    Action act;
    RoboLog rLog;
    Joystick joystick1;
    Timer timer1 = new Timer();

    public TestRobot(Electronics e, RoboLog rLog, Action act, Joystick joystick) {
        this.e = e;
        this.rLog = rLog;
        this.act = act;
        this.joystick1 = joystick;
    }

    public void testInit() {
        e.resetGyro();
        act.actionReset();
        timer1.reset();
        timer1.start();  
    }

    public void testPeriodic() {
        testDistanceSensor();
    }

    public void testSwerve() {
        int module = 0;
        if (joystick1.getPOV() == 135) {
            module = 1;
        } else if (joystick1.getPOV() == 180) {
            module = 2;
        } else if (joystick1.getPOV() == 225) {
            module = 3;
        }

        double pow = -1 * joystick1.getY();
        double mag = joystick1.getMagnitude();
        double goalAng = joystick1.getDirectionDegrees();
        if (joystick1.getRawButton(1)) {
            e.setDrivePercent(module, 0.2);
            e.setTurnPercent(module, 0);
        } else if (joystick1.getRawButton(2)) {
            e.setTurnPercent(module, 0.2);
            e.setDrivePercent(module, 0);
        } else if (joystick1.getRawButton(3)) {
            e.setDrivePercent(module, 0.8 * pow);
            e.setTurnPercent(module, 0);
        } else if (joystick1.getRawButton(4)) {
            e.setTurnPercent(module, 0.8 * pow);
            e.setDrivePercent(module, 0);
        } else if (joystick1.getRawButton(5)) {
            e.setAllHeadings(0);
        } else if (joystick1.getRawButton(6)) {
            e.setAllHeadings(90);
        } else if (joystick1.getRawButton(7)) {
            if (mag > 0.3) {
                e.setAllHeadings(goalAng);
            }
        } else if (joystick1.getPOV(0) == 0) {
            e.setDriveSpeed(module, 4);
        } else if (joystick1.getPOV(0) == 45) {
            e.setDriveSpeed(module, 12);
        } else if (joystick1.getPOV(0) == 90) {
            double velocity = pow * 100.0;
            e.setDriveSpeed(module, velocity);
        } else {
            e.stopMotors();
        }
        if (joystick1.getRawButton(9)) {
            Limelight.setPipeline(VisionPipeline.TargetOne);
            Limelight.setLEDOFF();
            rLog.print("setPipeline0");
        } else if (joystick1.getRawButton(10)) {
            Limelight.setPipeline(VisionPipeline.TargetOne);
            Limelight.setLEDON();
            rLog.print("setPipeline4");
        } else if (joystick1.getPOV(0) == 270) {
            rLog.print("Abs Encoders Front Left: " + e.getAbsoluteTurnEncoderPosition(0) + "Front Right: "
                    + e.getAbsoluteTurnEncoderPosition(1) + "Back Right: " + e.getAbsoluteTurnEncoderPosition(2)
                    + "Back Left: " + e.getAbsoluteTurnEncoderPosition(3));
        } else if (joystick1.getPOV(0) == 315) {
            rLog.print("non-ABS Encoedrs Front Left:" + e.getTurnEncoderPosition(0) + "Front Right: "
            + e.getTurnEncoderPosition(1) + "Back Right: " + e.getTurnEncoderPosition(2)
            + "Back Left: " + e.getTurnEncoderPosition(3));
        }
    }

    public void testAngle() {
        if (joystick1.getRawButton(1))
            e.assignRobotMotionAndHeadingField(0, 0, -1); 
        else if (joystick1.getRawButton(2))
            e.assignRobotMotionAndHeadingField(0, 0, -1.5);
        else if (joystick1.getRawButton(3))
            e.assignRobotMotionAndHeadingField(0, 0, -3);
        else if (joystick1.getRawButton(4))
            e.assignRobotMotionAndHeadingField(0, 0, -12);
        else
            e.assignRobotMotionAndHeadingField(0, 0, 0);
    }

    public void testDistanceSensor() {
        if (joystick1.getRawButtonPressed(1)) {
            if (e.isDistanceSensorRangeValid())
                rLog.print("Distance Sensor: " + e.getDistanceInInches());
            else {
                rLog.print("Distance Sensor: not valid");
                rLog.print("Distance Sensor: " + e.getDistanceInInches());
            }
        }
    }
}
