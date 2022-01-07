/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import frc.robot.RobotMotor.*;

/**
 * This is used to control one corner of a swerve drive (one drive and one turn
 * motor)
 */
public class SwerveModule {
    private RobotMotor driveMotor;
    private RobotMotor turnMotor;
    private PWMInput pWMInput = null;
    private CANCoder turnEncoder = null;
    private RoboLog rLog;
    private double lastTurnDifference;
    private RobotEncoderType turnEncoderType;
    private double offsetDegrees;

    private static double[] percentValue;
    private static double[] velocityValue;
    private static final double[] percentNeoValue = { 0.03, 0.07, 0.12, 0.20, 0.40, 0.50, 0.75, 1.00 };
    private static final double[] velocityNeoValue = { 3.6, 9.45, 16.6, 28.2, 56.1, 70.1, 104.4, 139.0 };
    private static final double[] percentFalconValue = { .02, .067, .119, .198, .399, .499, .748, 1.0 };
    private static final double[] velocityFalconValue = { 1.5, 12.5, 23.2, 38.4, 76.9, 95.9, 143.8, 191.0 };

    SwerveModule(int driveID, int turnID, int absoluteEncoderID, double offsetDegrees, RoboLog rLog,
            RobotMotor.RobotMotorType driveMotorType, RobotMotor.RobotMotorType turnMotorType,
            RobotEncoderType turnEncoderType, double driveWheelDiameter, double driveGearRatio, double turnEncoderRatio) {
        this.rLog = rLog;
        this.rLog.print("SwerveModule Created");
        
        if (driveMotorType == RobotMotorType.SparkMax) {
            percentValue = percentNeoValue;
            velocityValue = velocityNeoValue;
        } else {
            percentValue = percentFalconValue;
            velocityValue = velocityFalconValue;
        }
        driveMotor = new RobotMotor(driveMotorType, driveID, false, true, rLog, (driveWheelDiameter * Math.PI) / driveGearRatio, 11,
                RobotEncoderType.Internal, 0);
        driveMotor.setPID(0, 0, 0, 0, false);
        driveMotor.setMaxOutput(1.0);
        driveMotor.setPercentVelocityMapping(percentValue, velocityValue);
        driveMotor.burnFlash();

        turnMotor = new RobotMotor(turnMotorType, turnID, true, true, rLog, turnEncoderRatio, 11,
            RobotEncoderType.Internal, 0);
        turnMotor.setPID(0.04, 0.0001, 0, 3.0, true);
        turnMotor.setMaxOutput(0.8);
        turnMotor.burnFlash();

        this.turnEncoderType = turnEncoderType;
        if (turnEncoderType == RobotEncoderType.Cancoder) {
            turnEncoder = new CANCoder(absoluteEncoderID);
            turnEncoder.configSensorDirection(true);
        } else {
            pWMInput = new PWMInput(absoluteEncoderID, 1.0E-6, 1.0E-6, 1024);
        }

        this.offsetDegrees = offsetDegrees;
    }

    public void setDrivePercent(double percent) {
        driveMotor.setPercent(percent);
    }

    public void setTurnPower(double percent) {
        turnMotor.setPercent(percent);
    }

    public void resetDriveEncoderValue() {
        driveMotor.setEncoderValue(0);
    }

    public void setDriveEncoderValue(double position) {
        driveMotor.setEncoderValue(position);
    }

    public void setTurnEncoderValue(double position) {
        turnMotor.setEncoderValue(position);
    }

    public double getTurnEncoderPosition() {
        return turnMotor.getEncoderPosition();
    }

    public double getDriveEncoderPosition() {
        return driveMotor.getEncoderPosition();
    }

    public double getDriveVelocity() {
        return driveMotor.getEncoderVelocity();
    }

    public double getAbsoluteTurnEncoderPosition() {
        if (turnEncoderType == RobotEncoderType.Cancoder) {
            return (turnEncoder.getAbsolutePosition() - offsetDegrees);
        } else {
            return ((pWMInput.getLastPulse() / 1024.0) * 360.0) - offsetDegrees;
        }
    }

    public static double getMaxVelocity() {
        return velocityValue[velocityValue.length - 1];
    }

    /*
     * 
     * Methods to use when we are using feeback
     * 
     */

    public void setDriveSpeed(double velocity) {
        driveMotor.setVelocity(velocity);
    }

    public void setTurnHeading(double angle) {
        double currentHeading = getTurnEncoderPosition();
        while (angle < (currentHeading - 180)) {
            angle = angle + 360;
        }
        while (angle > (currentHeading + 180)) {
            angle = angle - 360;
        }
        double difference = (angle - currentHeading);
        if (((difference <= 0 && lastTurnDifference >= 0) || (difference >= 0 && lastTurnDifference <= 0))
                && Math.abs(difference) < 0.55) {
            setTurnPower(0);
        } else {
            lastTurnDifference = difference;
            turnMotor.setPosition(angle);
        }
    }

    public void setSpeedAndHeading(double velocity, double angle) {
        if (velocity < 0.01) {
            setDriveSpeed(0);
            setTurnPower(0);
        } else {
            double currentHeading = getTurnEncoderPosition();
            double currentVelocity = getDriveVelocity();
            while (angle < (currentHeading - 180)) {
                angle = angle + 360;
            }
            while (angle > (currentHeading + 180)) {
                angle = angle - 360;
            }
            double angleDifference = Math.abs(angle - currentHeading);
            if (angleDifference > 90.0 && currentVelocity < 20.0) {
                angle = angle + 180;
                velocity = -velocity;
                angleDifference = 180.0 - angleDifference;
            }
            if (angleDifference > 10.0 && currentVelocity < 20.0) {
                setDriveSpeed(0);
            } else {
                setDriveSpeed(velocity);
            }
            setTurnHeading(angle);
        }
    }

    /*
     * 
     * Methods to use when tuning new modules
     * 
     */

    public void setDriveFPID(double F, double P, double I, double D, double iZone, boolean isVelocity) {
        driveMotor.setPID(P, I, D, iZone, isVelocity);
    }

    public void setTurnFPID(double F, double P, double I, double D, double iZone, boolean isVelocity) {
        turnMotor.setPID(P, I, D, iZone, isVelocity);
    }

    public void setDriveOpenLoopRamp(double rate) {
        driveMotor.setDriveOpenLoopRamp(rate);
    }

    public void setTurnOpenLoopRamp(double rate) {
        turnMotor.setDriveOpenLoopRamp(rate);
    }

    public void setDriveAndTurnNeutralMode(boolean isCoast) {
        driveMotor.setNeutralMode(isCoast);
        turnMotor.setNeutralMode(isCoast);
    }
}
