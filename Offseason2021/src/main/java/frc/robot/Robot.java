/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* 2021 Offseason Robot */
public class Robot extends TimedRobot {

    private final String projectName = "2021 Offseason";
    private Electronics e;
    private Action act;
    private RoboLog rLog = new RoboLog();
    private TestRobot test;
    private OI oi = new OI();
    private AutoRobot auto;
    private TeleopRobot teleop;
    private Dashboard dash;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();
    private boolean wasAuto = false;
    private boolean lastButtonCamera = false;
    private int camMode = 2;
    private Timer autoTimer = new Timer();
    private Timer autoDisplayTimer = new Timer();
    private String lastAuto = "";

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        rLog.print(projectName + " robotInit");
        e = new Electronics(true, rLog);
        act = new Action(e, rLog);
        e.assignAllEncoderValues();
        test = new TestRobot(e, rLog, act, oi.driver);
        auto = new AutoRobot(e, rLog, act);
        teleop = new TeleopRobot(e, rLog, act, oi);
        dash = new Dashboard(e);
        m_chooser.setDefaultOption("None", "na");
        m_chooser.addOption("Auto One", "auto1");
        m_chooser.addOption("Auto Two", "auto2");
        SmartDashboard.putData("Auto choices", m_chooser);
        Limelight.setRobotLog(rLog);
        Limelight.setCameraMode2();
        Limelight.setLEDOFF();
        autoDisplayTimer.start();
        LiveWindow.disableAllTelemetry(); // Improve performance
        SmartDashboard.updateValues(); // Improve performance
    }

    /* Called periodically in all modes */
    @Override
    public void robotPeriodic() {
        if (DriverStation.isDSAttached())
        {
            dash.dashboardPeriodic();
            displayDashboard();
            if (oi.getResetGyroButtonPressed()) {
                System.out.println("Resetting");
                e.resetGyro();
            }

            if (oi.getGyroTo180Pressed()) {
                e.setGyro(180);
            }

            if (!lastButtonCamera && oi.getCameraButtonPressed()) {
                if (camMode == 1) {
                    Limelight.setCameraMode2();
                    camMode = 2;
                    rLog.print("Camera Mode Button Pressed: Now Driver's Camera");
                } else if (camMode == 2) {
                    Limelight.setCameraMode1();
                    camMode = 1;
                    rLog.print("Camera Mode Button Pressed: Now Vision Camera");
                }
            }
            lastButtonCamera = oi.getCameraButtonPressed();
            e.getDistanceInInches();
        }
    }

    /* Called once whenever robot is disabled */
    @Override
    public void disabledInit() {
        rLog.setRobotMode("DISA", "Disabled");
        rLog.print(projectName + " disabledInit");
        Limelight.setLEDOFF();
    }

    /* Called periodically while robot is disabled */
    @Override
    public void disabledPeriodic() {
        if (autoDisplayTimer.get() > 30 || !lastAuto.equals(m_chooser.getSelected())) {
            autoDisplayTimer.reset();
            autoDisplayTimer.start();
            lastAuto = m_chooser.getSelected();
            rLog.print("Auto Selection = " + lastAuto);
        }
    }

    /* Called once when autonomous is started */
    @Override
    public void autonomousInit() {
        wasAuto = true;
        rLog.setRobotMode("AUTO", "Autonomus");
        rLog.print(projectName + " autonomousInit");
        String autoSelected = m_chooser.getSelected();
        rLog.print("Auto selected: " + autoSelected);
        auto.autonomousInit(autoSelected);
        Limelight.setLEDON();
    }

    /* Called periodically during autonomous */
    @Override
    public void autonomousPeriodic() {
        auto.autonomousPeriodic();
        autoTimer.reset();
        autoTimer.start();
    }

    /* Called once when operator control is started */
    @Override
    public void teleopInit() {
        rLog.setRobotMode("TELE", "Teleop");
        rLog.print(projectName + " teleopInit");
        teleop.teleopInit(wasAuto && autoTimer.get() < 3.0);
        Limelight.setLEDON();
    }

    /* Called periodically during operator control */
    @Override
    public void teleopPeriodic() {
        teleop.teleopPeriodic();
    }

    /* Called once when test mode is started */
    @Override
    public void testInit() {
        rLog.setRobotMode("TEST", "Test");
        rLog.print(projectName + " testInit");
        test.testInit();
        Limelight.setLEDON();
    }

    /* Called periodically during test mode */
    @Override
    public void testPeriodic() {
        test.testPeriodic();
    }

    void displayDashboard() {
        SmartDashboard.putNumber("Gyro Angle", e.getGyroCenteredOnGoal(0.0));
        SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
        SmartDashboard.putNumber("Left Distance Sensor", e.getDistanceInInches());
        SmartDashboard.putBoolean("Test", true);
    }
}
