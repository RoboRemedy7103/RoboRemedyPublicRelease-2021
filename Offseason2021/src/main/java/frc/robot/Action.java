/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Timer;

/*
 * This is used to define methods used in autonomated behavior
 */

public class Action {
    private Electronics e;
    private double[] driveEncoderZeros;
    private Timer actionTimer = new Timer();
    private double calcDistTraveled = 0;
    RoboLog rLog;

    private int step = 1;
    private static final double TARGET_RANGE = 0;
    private static final double PIVOT_TO_FRONT = 0;

    Action(Electronics elec, RoboLog rLog) {
        this.rLog = rLog;
        e = elec;
        actionTimer.start();
    }

    public void actionReset() {
        step = 1;
        subActionReset();
    }

    public void subActionReset() {
        driveEncoderZeros = e.getAllDriveEncoders();
        calcDistTraveled = 0;
        actionTimer.reset();
        actionTimer.start();
    }

    public double getActTime() {
        return actionTimer.get();
    }

    public boolean driveStraightNoFacing(double travelAngle, double travelInchesPerSecond, double maxTravelAcceleration,
            double distance, double endingInchesPerSecond) {
        double distTraveled = Math.abs((e.getAllDriveEncoders()[0] - driveEncoderZeros[0]));
        double distYetToGo = distance - distTraveled;
        if (distTraveled > distance - TARGET_RANGE) {
            e.assignRobotMotionField(travelAngle, endingInchesPerSecond, 0);
            return true;
        } else {
            if (travelInchesPerSecond < 0) {
                travelInchesPerSecond = -1 * travelInchesPerSecond;
                travelAngle = travelAngle + 180;
            }
            double upInPerSec = limitAccel(travelInchesPerSecond, maxTravelAcceleration);
            double dwnInPerSec = endingInchesPerSecond + (distYetToGo * 2);
            double calcInPerSec = Math.min(upInPerSec, dwnInPerSec);
            e.assignRobotMotionField(travelAngle, calcInPerSec, 0);
        }
        return false;
    }

    public boolean driveCurveWithFacing(double startingAngle, double targetAngle, double travelInchesPerSecond,
            double facingAngle, double maxTravelAcceleration, double degreesPerSecond) {
        boolean goalReached = false;
        if (startingAngle > targetAngle && degreesPerSecond > 0) {
            degreesPerSecond = -degreesPerSecond;
        }
        double goalHeading = startingAngle + (getActTime() * degreesPerSecond);
        if (degreesPerSecond > 0) {
            goalReached = targetAngle - goalHeading < 0.5;
        } else {
            goalReached = targetAngle - goalHeading > -0.5;
        }
        if (goalReached) {
            e.assignRobotMotionAndHeadingField(targetAngle, travelInchesPerSecond, facingAngle);
            return true;
        } else {
            double calcInPerSec = limitAccel(travelInchesPerSecond, maxTravelAcceleration);
            e.assignRobotMotionAndHeadingField(goalHeading, calcInPerSec, facingAngle);
        }
        return false;
    }

    public boolean driveStraightWithFacing(double travelAngle, double travelInchesPerSecond, double facingAngle,
            double maxTravelAcceleration, double distance, double endingInchesPerSecond) {
        calcDistTraveled += e.getCalculatedTravelSinceLastCommand();
        double distYetToGo = distance - calcDistTraveled;
        if (calcDistTraveled > distance - TARGET_RANGE) {
            e.assignRobotMotionAndHeadingField(travelAngle, endingInchesPerSecond, facingAngle);
            return true;
        } else {
            if (travelInchesPerSecond < 0) {
                travelInchesPerSecond = -1 * travelInchesPerSecond;
                travelAngle = travelAngle + 180;
            }
            double upInPerSec = limitAccel(travelInchesPerSecond, maxTravelAcceleration);
            double dwnInPerSec = endingInchesPerSecond + (distYetToGo * 2);
            double calcInPerSec = Math.min(upInPerSec, dwnInPerSec);
            e.assignRobotMotionAndHeadingField(travelAngle, calcInPerSec, facingAngle);
        }
        return false;
    }

    public void driveStraightContinuousWithFacing(double travelAngle, double travelInchesPerSecond, double facingAngle,
            double maxTravelAcceleration) {
        double calcInPerSec = limitAccel(travelInchesPerSecond, maxTravelAcceleration);
        e.assignRobotMotionAndHeadingField(travelAngle, calcInPerSec, facingAngle);
    }

    public boolean driveCurveNoFacing(double startingAngle, double targetAngle, double travelInchesPerSecond,
            double maxTravelAcceleration, double degreesPerSecond) {
        return driveCurveWithFacing(startingAngle, targetAngle, travelInchesPerSecond, e.getGyro(),
                maxTravelAcceleration, degreesPerSecond);
    }

    public boolean driveCurveCenterVision(double travelAngle, double targetInchesRightOfRobot, double travelInchesPerSecond, 
            double maxTravelAcceleration, double endingInchesPerSecond, double maxDistance, Limelight.VisionPipeline visionPipeline) {
                Limelight.setPipeline(visionPipeline);
                double gyroAngle = e.getGyroCenteredOnGoal(travelAngle);
                double vertDistanceToDesiredPoint = maxDistance - calcDistTraveled;
                double horzDistToTarget = Limelight.getInchesRightOfRobot(gyroAngle, travelAngle) - targetInchesRightOfRobot;
                double adjustmentAngle = Math.min(Math.max(horzDistToTarget * 5, -75), 75);
                double calcHeading = travelAngle + adjustmentAngle;
                rLog.print("distToGo" + vertDistanceToDesiredPoint + " " + "gyro" + gyroAngle + " " + "horzDist" + horzDistToTarget);
                if (Limelight.isTargetFound()){
                    if (calcDistTraveled >= maxDistance || Math.abs(horzDistToTarget) <= 1.5) {
                        e.assignRobotMotionAndHeadingField(travelAngle, endingInchesPerSecond, travelAngle);
                        return true;
                    } else {
                        e.assignRobotMotionAndHeadingField(calcHeading, calcFinalVel(endingInchesPerSecond, vertDistanceToDesiredPoint, travelInchesPerSecond, maxTravelAcceleration), travelAngle);
                    }
                } else if (Math.abs(gyroAngle - travelAngle) <= 10) {
                        e.assignRobotMotionAndHeadingField(travelAngle, calcFinalVel(endingInchesPerSecond, vertDistanceToDesiredPoint, travelInchesPerSecond, maxTravelAcceleration), travelAngle);
                } else {
                    e.assignRobotMotionAndHeadingField(0, 0, travelAngle);
                }
                calcDistTraveled += e.getCalculatedTravelSinceLastCommand() * Math.cos(adjustmentAngle);
                return false;
            }

    public boolean driveCurveRadiusWithFacing(double startingAngle, double targetAngle, double travelInchesPerSecond,
            double facingAngle, double maxTravelAcceleration, double turnRadius) {
        double degreesPerSecond = (travelInchesPerSecond * 180) / (Math.PI * turnRadius);
        return driveCurveWithFacing(startingAngle, targetAngle, travelInchesPerSecond, facingAngle,
                maxTravelAcceleration, degreesPerSecond);
    }

    public boolean driveCurveRadiusNoFacing(double startingAngle, double targetAngle, double travelInchesPerSecond,
            double maxTravelAcceleration, double turnRadius) {
        return driveCurveRadiusWithFacing(startingAngle, targetAngle, travelInchesPerSecond, e.getGyro(),
                maxTravelAcceleration, turnRadius);
    }

    public boolean driveToVisionTarget(double facingAngle, double targetDistanceInInches,
            double targetInchesRightOfRobot, double distanceRange, double horzDistRange, double maxTravelSpeed,
            double maxTravelAcceleration, Limelight.VisionPipeline visionPipeline) {
        return driveToVisionTargetNonStop(facingAngle, targetDistanceInInches, targetInchesRightOfRobot, distanceRange,
                horzDistRange, maxTravelSpeed, maxTravelAcceleration, 0, visionPipeline);
    }

    public boolean driveToVisionTargetNonStop(double facingAngle, double targetDistanceInInches,
    double targetInchesRightOfRobot, double distanceRange, double horzDistRange, double maxTravelSpeed,
    double maxTravelAcceleration, double endingInchesPerSecond, Limelight.VisionPipeline visionPipeline) {
        Limelight.setPipeline(visionPipeline);
        double gyroAngle = e.getGyroCenteredOnGoal(facingAngle);
        double forwardDistToTarget = Limelight.getAdjustedDistanceInInches();
        if(Limelight.isTargetFound() && forwardDistToTarget > PIVOT_TO_FRONT) {
            double horzDistToTarget = Limelight.getInchesRightOfRobot(gyroAngle, facingAngle) - targetInchesRightOfRobot;
            double forwardDistToGoal = forwardDistToTarget - targetDistanceInInches;
            double calcDistToGo = Math.sqrt((Math.pow(horzDistToTarget, 2)) + Math.pow(forwardDistToGoal, 2));
            double calcHeading = gyroAngle + (90 - Math.toDegrees(Math.atan2(forwardDistToGoal, horzDistToTarget)));
            if(Math.abs(horzDistToTarget) < horzDistRange && Math.abs(forwardDistToGoal) < distanceRange && Math.abs(facingAngle - e.getGyroCenteredOnGoal(facingAngle)) < 2) {
                e.assignRobotMotionAndHeadingField(calcHeading, endingInchesPerSecond, facingAngle);
                return true;
            } else {
                double dwnInPerSec = endingInchesPerSecond + (calcDistToGo * 1.6);
                double calcInPerSec = Math.min(maxTravelSpeed, dwnInPerSec);
                double finalVel = limitAccel(calcInPerSec, maxTravelAcceleration);
                e.assignRobotMotionAndHeadingField(calcHeading, finalVel, facingAngle);
            }
        } else {
            e.assignRobotMotionAndHeadingField(0, 0, facingAngle);
        }
        return false;
    }

    public boolean driveStraightWithVisionGuide(double facingAngle, double distance, double targetInchesRightOfRobot, 
    double travelSpd, double maxTravelAcceleration, double endingInchesPerSecond, Limelight.VisionPipeline visionPipeline) {
        double calcHeading;
        Limelight.setPipeline(visionPipeline);
        double travelAngle = facingAngle;
        double forwardDistToGoal = distance - calcDistTraveled;
        double gyroAngle = e.getGyroCenteredOnGoal(facingAngle);
        double dwnInPerSec = endingInchesPerSecond + (forwardDistToGoal * 1.6);
        double calcInPerSec = Math.min(travelSpd, dwnInPerSec);
        double finalVel = limitAccel(calcInPerSec, maxTravelAcceleration);
        if (Limelight.isTargetFound()) {
            double horzDistToTarget = Limelight.getInchesRightOfRobot(gyroAngle, facingAngle) - targetInchesRightOfRobot;
            double adjustmentAngle = Math.min(Math.max(horzDistToTarget * 5, -75), 75);
            calcHeading = travelAngle - adjustmentAngle;
            if (forwardDistToGoal <= 0) {
                e.assignRobotMotionAndHeadingField(calcHeading, endingInchesPerSecond, facingAngle);
                return true;
            } else {
                e.assignRobotMotionAndHeadingField(calcHeading, finalVel, facingAngle);
            }
        } else {
            calcHeading = travelAngle;
            if (forwardDistToGoal <= 0) {
                e.assignRobotMotionAndHeadingField(calcHeading, endingInchesPerSecond, facingAngle);
                return true;
            } else {
                e.assignRobotMotionAndHeadingField(calcHeading, finalVel, facingAngle);
            }
        }
        calcDistTraveled += e.getCalculatedTravelSinceLastCommand() / Math.sin(calcHeading);
        return false;
    }

    public double limitAccel(double goalInPerSec, double maxTravelAcceleration) {
        double calcInPerSec;
        if (goalInPerSec >= e.getLastTravelVelocityCommand()) {
            calcInPerSec = Math.min(goalInPerSec,
                    e.getLastTravelVelocityCommand() + e.getMaxSpeedChange(maxTravelAcceleration));
        } else {
            calcInPerSec = Math.max(goalInPerSec,
                    e.getLastTravelVelocityCommand() - e.getMaxSpeedChange(maxTravelAcceleration));
        }
        return calcInPerSec;
    }

    public double calcFinalVel(double endingInchesPerSecond, double distToGo, double maxTravelSpeed, double maxTravelAcceleration) {
        double dwnInPerSec = endingInchesPerSecond + (distToGo * 1.6);
        double calcInPerSec = Math.min(maxTravelSpeed, dwnInPerSec);
        double finalVel = limitAccel(calcInPerSec, maxTravelAcceleration);
        return finalVel;
    }

    public double limitUpAccel(double goalInPerSec, double maxTravelAcceleration) {
        return (Math.min(goalInPerSec, e.getLastTravelVelocityCommand() + e.getMaxSpeedChange(maxTravelAcceleration)));
    }

    public void setAutoStepNumber(int number) {
        step = number;
        subActionReset();
        rLog.print("New auto action step " + step);
    }

    void setNextAutoStepNumber() {
        setAutoStepNumber(step + 1);
    }

    public boolean allignWheelsForawrd() {
        e.allignMotorsForward();
        return true;
    }

    public double getDistanceTraveled() {
        return calcDistTraveled;
    }
}