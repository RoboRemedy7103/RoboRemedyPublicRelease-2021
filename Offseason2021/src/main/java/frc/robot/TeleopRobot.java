/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;

/**
 * Code for Teleop Mode
 */
public class TeleopRobot {

Electronics e;
Action act;
RoboLog rLog;
OI oi;
private double facing = 0;
private double previousFacingJoystickMagnitude = 0;
private double previousFacingJoystickDirection = 0;
private Timer facingTimer = new Timer();
private Timer testTimer = new Timer();

public TeleopRobot(Electronics e, RoboLog rLog, Action act, OI oi) {
    this.e = e;
    this.rLog = rLog;
    this.act = act;
    this.oi = oi;
}

public void teleopInit(boolean wasAuto) {
    if (!wasAuto) {
        e.resetGyro();
        rLog.print("Done with reset Gyro");
    }
    act.actionReset();
    e.fasterTurning = false;
    testTimer.start();
}

public void teleopPeriodic() {
    // testLimelight();
    double joyMag = Math.pow(oi.getDriveMagnitude(), 2);
    
    // driving
    if (oi.getDriveFastButtonPressed() || oi.getDriveSlowButtonPressed()) {
        double driveSpeed;

        if (joyMag < 0.2) {
            driveSpeed = 0;
        } else {
            double maxSpeed = (oi.getDriveSlowButtonPressed() ? 37 : 
                                e.getMaxTeleopInchesPerSecond());
            driveSpeed = (joyMag - 0.2) * (maxSpeed / 0.8);
        }

        double driveDirection = oi.getDriveDirectionDegrees();
        double facingMagnitude = oi.getFacingJoystickMagnitude();
        double facingAngle = oi.getFacingJoystickDegrees();

        if (facingMagnitude > 0.5 && previousFacingJoystickMagnitude < 0.5) {
            facingTimer.reset();
            facingTimer.start();
        }

        if (facingMagnitude > 0.5 && facingTimer.get() > 0.2) {
            facing = facingAngle;
        } else if (facingMagnitude < 0.5 && previousFacingJoystickMagnitude > 0.5 && facingTimer.get() < 0.2) {
            if (previousFacingJoystickDirection > 90 && previousFacingJoystickDirection < 270) {
                if (facing != 180) {
                    rLog.print("Facing 180 Tap Button Pressed");
                }
                facing = 180;
            } else {
                if (facing != 0) {
                    rLog.print("Facing 0 Tap Button Pressed");
                    facing = 0;
                }
            }
        }
        double actualFacingAngle = facing;

        while (facing < -180) {
            facing += 360;
        }
        while (facing > 180) {
            facing -= 360;
        }

        e.assignRobotMotionAndHeadingField(driveDirection, driveSpeed, actualFacingAngle);
        previousFacingJoystickMagnitude = facingMagnitude;
        previousFacingJoystickDirection = facingAngle;
        } else if (oi.getAllignWheelsButtonPressed()) {
            e.allignMotorsForward();
        } else if (oi.getLockButtonPressed()) {
            e.lockWheels();
        } else {
            facing = e.getGyro();
            e.stopMotors();
        }        
    }
}