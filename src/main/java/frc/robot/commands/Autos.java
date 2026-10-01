// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
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
  public static Command driveStraight(DriveSubsystem drive, ArmSubsystem arm) {
    return new InstantCommand(() -> drive.drive(0.0, 0.5, 0.0))
               .andThen(new WaitCommand(1.8))
               .andThen(() -> drive.drive(0.0, 0.0, 0.0));
  }

  public static Command coralAuto(DriveSubsystem drive, ArmSubsystem arm, String startPos) {
    double turnMultiplier = 0;
    if (startPos.equals("right")) {
      turnMultiplier = 1;
    } else if (startPos.equals("left")) {
      turnMultiplier = -1;
    }
    return new InstantCommand(() -> arm.setArmSpeed(0))
               .andThen(() -> drive.drive(0, -0.7, 0))
               .andThen(new WaitCommand(0.82))
                
               //turns 45 degrees
               .andThen(() -> drive.drive(0, 0, 0.5))
               .andThen(new WaitCommand(0.8))

                //drives forward to coral branch get in the silly car
               .andThen(() -> drive.drive(0, -0.7, 0))
               .andThen(new WaitCommand(0.78))
               .andThen(() -> drive.drive(0, -0, 0))
               .andThen(new WaitCommand(0.3))

                //stops and lifts
               .andThen(() -> drive.drive(0, 0, 0))
               .andThen(() -> arm.setArmSpeed(-0.12))
               .andThen(new WaitCommand(2.5))

               //stops lift, spins outtake, stops outtake
               .andThen(() -> arm.setArmSpeed(0))
               .andThen(new WaitCommand(1))
               .andThen(() -> arm.spinOuttake(true))
               .andThen(new WaitCommand(0.5))
               .andThen(() -> arm.spinOuttake(false))
               .andThen(new WaitCommand(2))

               //moves arm back down and stops
               .andThen(() -> arm.setArmSpeed(0.1))
               .andThen(new WaitCommand(1.5))
               .andThen(() -> arm.setArmSpeed(0));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

}
