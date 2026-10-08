// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  /** Drives for a certain number of seconds. */
  public static Command driveStraight(DriveSubsystem drive) {
    return new InstantCommand(() -> drive.drive(0.0, 0.5, 0.0))
               .andThen(new WaitCommand(1.8))
               .andThen(() -> drive.drive(0.0, 0.0, 0.0));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

}
