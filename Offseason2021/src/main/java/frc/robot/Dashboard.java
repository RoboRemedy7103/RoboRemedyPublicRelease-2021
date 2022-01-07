/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.NetworkTableEntry;

/**
 * Creates and controls the Shuffleboard dashboard
 */
public class Dashboard {

    Electronics e;
    NetworkTableEntry gyroEntry;

    Dashboard(Electronics e) {
        this.e = e;

        Shuffleboard.getTab("Test")
            .add("Some Number", 1)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withPosition(2, 0)
            .withSize(2, 1);

        gyroEntry = Shuffleboard.getTab("Test")
            .add("Gyro", -999)
            .withWidget(BuiltInWidgets.kGyro)
            .withPosition(0, 0)
            .getEntry();
    }

    public void dashboardPeriodic() {
        gyroEntry.setDouble(Math.round(e.getGyro()));
    }
}
