/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;

/**
 * Creates and controls joysticks and buttons
 */
public class OI {

    OI() {
    }

    public Joystick driver = new Joystick(0);

    double getDriveMagnitude() {
        return driver.getMagnitude();
    }

    double getDriveDirectionDegrees() {
        return driver.getDirectionDegrees();
    }

    double getFacingJoystickMagnitude() {
        return Math.sqrt(Math.pow(driver.getRawAxis(2), 2) + Math.pow(driver.getRawAxis(3), 2));
    }

    double getFacingJoystickDegrees() {
        double x = driver.getRawAxis(2);
        double y = driver.getRawAxis(3);
        double angleRad = Math.atan2(y, x);
        double angleDeg = Math.toDegrees(angleRad);
        double finalAng = angleDeg + 90;
        return finalAng;
    }

    boolean getLockButtonPressed() {
        return (driver.getRawButton(7));
    }

    boolean getDriveSlowButtonPressed() {
        return driver.getRawButton(6);
    }

    boolean getDriveFastButtonPressed() {
        return driver.getRawButton(8);
    }

    boolean getGyroTo180Pressed(){
        return driver.getRawButton(1);
    }

    boolean getAllignWheelsButtonPressed() {
        return driver.getRawButton(2);
    }

    boolean getResetGyroButtonPressed() {
        return driver.getRawButton(5);
    }

    boolean getCameraButtonPressed() {
        return driver.getRawButton(4);
    }
}
