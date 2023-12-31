// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.drive.Drive;

public class DriveWithFlywheelAuto extends SequentialCommandGroup {
  private static final double drivePercent = 0.5;
  private static final double driveDuration = 3.0;
  // private static final double flywheelSpeed = 1500.0;
  // private static final double flywheelDuration = 10.0;

  /**
   * Creates a new DriveWithFlywheelAuto, which drives forward for three seconds and then runs the
   * flywheel for ten seconds.
   */
  public DriveWithFlywheelAuto(Drive drive) {
    addCommands(
        new StartEndCommand(
                () -> drive.drivePercent(drivePercent, -drivePercent), drive::stop, drive)
            .withTimeout(driveDuration));
    // new StartEndCommand(() -> flywheel.runVelocity(flywheelSpeed), flywheel::stop, flywheel)
    // .withTimeout(flywheelDuration));
  }
}
