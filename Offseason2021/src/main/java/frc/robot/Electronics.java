/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import frc.robot.RobotMotor.RobotEncoderType;
import frc.robot.RobotMotor.RobotMotorType;
import java.io.File;
import com.kauailabs.navx.frc.*;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

/**
 * This is used to control the motors, gyro, encoders, solenoids, etc.
 */
public class Electronics {
    public enum RobotName {
        LIVEROBOT, OLDSWERVE
    }

    private RobotName robotName;

    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule backRight;
    private SwerveModule backLeft;
    private SwerveModule[] swerveDrives;
    private SwerveState state;
    AHRS gyro = new AHRS(SPI.Port.kMXP);
    private RoboLog rLog;
    private double lastTravelVelocity = 0;
    private Timer commandTimer = new Timer();
    private double lastCommandTime = 0;
    Compressor compressor;
    DoubleSolenoid solenoidOne;
    private boolean compressorOn = true;
    public boolean fasterTurning = false;
    private double gyroOffset = 0;
    private double maxTeleopInchesPerSecond = 0;
    TimeOfFlight distSensor = new TimeOfFlight(1);

    static final double DISTANCE_SENSOR_OFFSET = 7.5;

    Electronics(boolean fullSwerve, RoboLog rLog) {
        File o = new File("/home/lvuser/OldSwerve.txt");

        if (o.isFile()) {
            System.out.println("Robot = Old Swerve");
            robotName = RobotName.OLDSWERVE;
        } else {
            System.out.println("Robot = Live Robot");
            robotName = RobotName.LIVEROBOT;
        }

        this.rLog = rLog;
        if (robotName == RobotName.OLDSWERVE) {
            System.out.println("OLD");
            state = new SwerveState(13.5, 13.5);
            // frontLeft = new SwerveModule(41, 11, 1, 88.3, rLog, RobotMotorType.Falcon, RobotMotorType.SparkMax,
            //         RobotEncoderType.Cancoder, 4.0, 8.308, 20);
            // frontRight = new SwerveModule(42, 12, 2, 56.5, rLog, RobotMotorType.Falcon, RobotMotorType.SparkMax,
            //         RobotEncoderType.Cancoder, 4.0, 8.308, 20);
            // backRight = new SwerveModule(43, 13, 3, 296.0, rLog, RobotMotorType.Falcon, RobotMotorType.SparkMax,
            //         RobotEncoderType.Cancoder, 4.0, 8.308, 20);
            // backLeft = new SwerveModule(44, 14, 4, 260.3, rLog, RobotMotorType.Falcon, RobotMotorType.SparkMax,
            //         RobotEncoderType.Cancoder, 4.0, 8.308, 20);

            frontLeft = new SwerveModule(41, 51, 01, 133, rLog, RobotMotorType.Falcon, RobotMotorType.Falcon,
                RobotEncoderType.Cancoder, 3.77, 6.12, 28.14);
            frontRight = new SwerveModule(42, 52, 2, 81, rLog, RobotMotorType.Falcon, RobotMotorType.Falcon,
                RobotEncoderType.Cancoder, 3.77, 6.12, 28.14);
            backRight = new SwerveModule(43, 53, 3, 71, rLog, RobotMotorType.Falcon, RobotMotorType.Falcon,
                RobotEncoderType.Cancoder, 3.77, 6.12, 28.14);
            backLeft = new SwerveModule(44, 54, 4, 177, rLog, RobotMotorType.Falcon, RobotMotorType.Falcon,
                RobotEncoderType.Cancoder, 3.77, 6.12, 28.14);

            maxTeleopInchesPerSecond = 77.0;

            compressor = null;
            solenoidOne = null;
        } else {
            System.out.println("LIVE");
            state = new SwerveState(21.5, 21.5);
            frontLeft = new SwerveModule(41, 11, 31, 160.14, rLog, RobotMotorType.Falcon, RobotMotorType.SparkMax,
                    RobotEncoderType.Cancoder, 4.0, 8.308, 20);
            frontRight = new SwerveModule(42, 12, 32, 70.93, rLog, RobotMotorType.Falcon, RobotMotorType.SparkMax,
                    RobotEncoderType.Cancoder, 4.0, 8.308, 20);
            backRight = new SwerveModule(43, 13, 33, 288.46, rLog, RobotMotorType.Falcon, RobotMotorType.SparkMax,
                    RobotEncoderType.Cancoder, 4.0, 8.308, 20);
            backLeft = new SwerveModule(44, 14, 34, 157.24, rLog, RobotMotorType.Falcon, RobotMotorType.SparkMax,
                    RobotEncoderType.Cancoder, 4.0, 8.308, 20);

            maxTeleopInchesPerSecond = 115.0;

            compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
            solenoidOne = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 0, 1);
            }

        if (fullSwerve) {
            swerveDrives = new SwerveModule[] { frontLeft, frontRight, backRight, backLeft };
        } else {
            swerveDrives = new SwerveModule[] { backLeft };
        }

        commandTimer.start();
        distSensor.setRangingMode(RangingMode.Short, 24);
    }

    public void stopMotors() {
        lastTravelVelocity = 0;
        lastCommandTime = commandTimer.get();
        for (SwerveModule m : swerveDrives) {
            m.setDriveSpeed(0);
            m.setTurnPower(0);
            if (Math.abs(m.getDriveVelocity()) <= 0.25) {
                // If we are trying to stop the motor and the motor
                // is still not moving, then use Coast Mode
                m.setDriveAndTurnNeutralMode(true);
            } else {
                // If we are trying to stop the motor and the motor
                // is still spinning, then use Break Mode
                m.setDriveAndTurnNeutralMode(false);
            }
        }
    }

    public void allignMotorsForward() {
        lastTravelVelocity = 0;
        lastCommandTime = commandTimer.get();
        for (SwerveModule m : swerveDrives) {
            m.setDrivePercent(0);
            m.setTurnHeading(0);
        }
    }

    public void resetGyro() {
        setGyro(0);
    }

    public void setGyro(double gyroAngle) {
        gyroOffset = gyro.getAngle() - gyroAngle;
    }

    public double getGyro() {
        return gyro.getAngle() - gyroOffset;
    }

    public double getGyroCenteredOnGoal(double goalAngle) {
        double gyroValue = getGyro();

        while (gyroValue < (goalAngle - 180)) {
            gyroValue += 360;
        }
        while (gyroValue > (goalAngle + 180)) {
            gyroValue -= 360;
        }

        return gyroValue;
    }

    private SwerveModule getModule(int module) {
        if (module >= swerveDrives.length || module < 0) {
            return null;
        } else {
            return swerveDrives[module];
        }
    }

    public void setDrivePercent(int moduleNumber, double power) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Module ");
        } else {
            swerveDriveModule.setDrivePercent(power);
        }
    }

    public void setDriveSpeed(int moduleNumber, double velocity) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Module ");
        } else {
            swerveDriveModule.setDriveSpeed(velocity);
        }
    }

    public void setTurnPercent(int moduleNumber, double power) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
        } else {
            swerveDriveModule.setTurnPower(power);
        }
    }

    public void setTurnHeading(int moduleNumber, double angle) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
        } else {
            swerveDriveModule.setTurnHeading(angle);
        }
    }

    public void setAllHeadings(double angle) {
        for (int i = 0; i < swerveDrives.length; i++) {
            setTurnHeading(i, angle);
        }
    }

    public void setTurnEncoder(int moduleNumber, double angle) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
        } else {
            swerveDriveModule.setTurnEncoderValue(angle);
        }
    }

    public void assignAllEncoderValues() {
        for (int i = 0; i < 4; i++) {
            setTurnEncoder(i, getAbsoluteTurnEncoderPosition(i));
            System.out.println("Wheel " + i + ": " + getAbsoluteTurnEncoderPosition(i));
        }
    }

    public void setAllTurnEncoders(double angle) {
        for (int i = 0; i < swerveDrives.length; i++) {
            setTurnEncoder(i, angle);
        }
    }

    public double[] getAllTurnEncoders() {
        double[] positions = new double[swerveDrives.length];
        for (int i = 0; i < swerveDrives.length; i++) {
            positions[i] = swerveDrives[i].getTurnEncoderPosition();
        }
        return positions;
    }

    public double[] getAllDriveEncoders() {
        double[] positions = new double[swerveDrives.length];
        for (int i = 0; i < swerveDrives.length; i++) {
            positions[i] = swerveDrives[i].getDriveEncoderPosition();
        }
        return positions;
    }

    public double getDriveVelocity(int moduleNumber) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
            return 0;
        } else {
            return swerveDriveModule.getDriveVelocity();
        }
    }

    public boolean areWheelsStopped() {
        for (SwerveModule m : swerveDrives) {
            if (Math.abs(m.getDriveVelocity()) > 0.5) {
                return false;
            }
        }
        return true;
    }

    public double getAbsoluteTurnEncoderPosition(int moduleNumber) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
            return 0;
        } else {
            return swerveDriveModule.getAbsoluteTurnEncoderPosition();
        }
    }

    public double getTurnEncoderPosition(int moduleNumber) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
            return 0;
        } else {
            return swerveDriveModule.getTurnEncoderPosition();
        }
    }

    public void assignRobotMotionField(double travelAngle, double travelInchesPerSecond, double degreesPerSecond) {
        lastTravelVelocity = travelInchesPerSecond;
        lastCommandTime = commandTimer.get();
        state.assignSwerveModulesField(travelAngle, travelInchesPerSecond, degreesPerSecond, getGyro(),
                SwerveModule.getMaxVelocity());
        for (int i = 0; i < swerveDrives.length; i++) {
            swerveDrives[i].setSpeedAndHeading(state.getMagnitude(i), state.getAngle(i));
        }
    }

    public void assignRobotMotionAndHeadingField(double travelAngle, double travelInchesPerSecond, double facingAngle) {
        double degreesPerSecond = 0;
        double angleDifference = facingAngle - getGyroCenteredOnGoal(facingAngle);
        double acceptableRange = Math.max(Math.min((0.1042 * travelInchesPerSecond), 1.0), 0.6);
        if (Math.abs(angleDifference) < acceptableRange) {
            degreesPerSecond = 0;
        } else {
            double calcDPS = angleDifference * 6.0;
            if (calcDPS < 0) {
                degreesPerSecond = Math.min(Math.max(-150, calcDPS), -10);
            } else {
                degreesPerSecond = Math.max(Math.min(150, calcDPS), 10);
            }
        }
        assignRobotMotionField(travelAngle, travelInchesPerSecond, degreesPerSecond);
    }

    public void assignRobotMotionAndHeadingFieldAccel(double travelAngle, double travelInchesPerSecond,
            double facingAngle, double maxTravelAcceleration) {
        double degreesPerSecond = 0;
        double angleDifference = facingAngle - getGyroCenteredOnGoal(facingAngle);
        double acceptableRange = Math.max(Math.min((0.1042 * travelInchesPerSecond), 2.5), 1);
        if (Math.abs(angleDifference) < acceptableRange) {
            degreesPerSecond = 0;
        } else {
            double calcDPS = angleDifference * 2.0;
            if (calcDPS < 0) {
                degreesPerSecond = Math.min(Math.max(-120, calcDPS), -25);
            } else {
                degreesPerSecond = Math.max(Math.min(120, calcDPS), 25);
            }
        }
        assignRobotMotionField(travelAngle, limitAccel(travelInchesPerSecond, maxTravelAcceleration), degreesPerSecond);
    }

    public void lockWheels() {
        lastTravelVelocity = 0;
        lastCommandTime = commandTimer.get();
        state.lockWheels();
        for (int i = 0; i < swerveDrives.length; i++) {
            swerveDrives[i].setSpeedAndHeading(state.getMagnitude(i), state.getAngle(i));
        }
    }

    public double getLastTravelVelocityCommand() {
        return lastTravelVelocity;
    }

    public double getCalculatedTravelSinceLastCommand() {
        return lastTravelVelocity * (commandTimer.get() - lastCommandTime);
    }

    public double getMaxSpeedChange(double maxTravelAcceleration) {
        return maxTravelAcceleration * (commandTimer.get() - lastCommandTime);
    }

    public double limitAccel(double goalInPerSec, double maxTravelAcceleration) {
        double calcInPerSec;
        if (goalInPerSec >= getLastTravelVelocityCommand()) {
            calcInPerSec = Math.min(goalInPerSec,
                    getLastTravelVelocityCommand() + getMaxSpeedChange(maxTravelAcceleration));
        } else {
            calcInPerSec = Math.max(goalInPerSec,
                    getLastTravelVelocityCommand() - getMaxSpeedChange(maxTravelAcceleration));
        }
        return calcInPerSec;
    }

    public void startCompressor() {
        if (!compressorOn) {
            compressor.enableDigital();
            compressorOn = true;
        }
    }

    public void stopCompressor() {
        if (compressorOn) {
            compressor.disable();
            compressorOn = false;
        }
    }

    public void retractSolenoidOne() {
        solenoidOne.set(Value.kReverse);
    }

    public void deploySolenoidOne() {
        solenoidOne.set(Value.kForward);
    }

    public boolean isDistanceSensorRangeValid() {
        return distSensor.isRangeValid();
    }
    public double getDistanceInInches() {
        double distance = distSensor.getRange();
        distance = distance * 0.0393701; // Convert mm to inches
        double robotCenterDistance = distance + DISTANCE_SENSOR_OFFSET;
        return robotCenterDistance;
    }

    public double getMaxTeleopInchesPerSecond() {
        return maxTeleopInchesPerSecond;
    }
}
