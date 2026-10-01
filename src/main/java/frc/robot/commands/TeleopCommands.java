package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.InputSubsystem;

/**
 * Contains static command methods meant to be used by triggers during teleop.
 */
public class TeleopCommands {
    public static Command outtakeCommand(InputSubsystem inputSubsystem, ArmSubsystem armSubsystem) {
        return new ConditionalCommand(new InstantCommand(() -> armSubsystem.spinOuttake(true), armSubsystem),
                                      new InstantCommand(() -> armSubsystem.spinOuttake(false), armSubsystem),
                                      inputSubsystem::isCoralIntakeActivated);
    }
}
