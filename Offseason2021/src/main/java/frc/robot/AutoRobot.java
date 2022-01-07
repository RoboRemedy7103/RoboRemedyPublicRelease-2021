//The purpose of this class is to store the autonomus code

package frc.robot;

import edu.wpi.first.wpilibj.*;
import frc.robot.Limelight.VisionPipeline;

public class AutoRobot {

    private Electronics e;
    private Action act;
    private RoboLog rLog;
    private int step_number;
    private Timer timer1 = new Timer();
    private String autoSelection;

    public AutoRobot(Electronics e, RoboLog rLog, Action act) {
        this.e = e;
        this.rLog = rLog;
        this.act = act;
    }

    public void autonomousInit(String autoSelection) {
        this.autoSelection = autoSelection;
        e.resetGyro();
        act.actionReset();
        e.stopMotors();
        setStepNumber(1);
        Limelight.setPipeline(VisionPipeline.TargetTwo);
        rLog.print("New AutoProgram Step Number: " + autoSelection);    
    }

    public void autonomousPeriodic() {
        if (autoSelection.equals("na")) {
            e.allignMotorsForward();
        } else if (autoSelection.equals("auto1")) {
            autoModeOne();
        } else if (autoSelection.equals("auto2")) {
            autoModeTwo();
        } else {
            rLog.print("Your selection for the Autonomus program is incorrect. Selected: " + autoSelection);
        }
    }

    void setStepNumber(int number) {
        step_number = number;
        act.actionReset();
        timer1.reset();
        timer1.start();
        rLog.print("New Auto Step Number: " + step_number);
    }

    void setNextStepNumber() {
        setStepNumber(step_number + 1);
    }

    void autoModeOne() {
        switch (step_number) {
            case 1:
                if (act.driveStraightWithFacing(2, 90, 90, 60, 105, 90)) {
                    setNextStepNumber();
                }
                break;
            case 2:
                if (act.driveCurveWithFacing(2, 90, 90, 90, 50, 150)) {
                    setNextStepNumber();
                }
                break;
            case 3:
                e.stopMotors();
                e.allignMotorsForward();
                break;
        }
    }

    void autoModeTwo() {
        if (step_number == 1) {
            if (act.driveToVisionTarget(0, 30, 0, 3, 3, 60, 50, VisionPipeline.TargetTwo)) {
                setNextStepNumber();
            }
        } else if (step_number == 2) {
            if (timer1.get() < 1.5) {
                act.driveStraightNoFacing(0, 20, 50, 50, 0);
            } else {
                setNextStepNumber();
            }
        } else {
            e.stopMotors();
        }
    }
}