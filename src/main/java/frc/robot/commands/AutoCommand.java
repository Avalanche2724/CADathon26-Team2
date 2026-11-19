
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SwerveSubsystem;

public class AutoCommand {

    public static Command driveForward(SwerveSubsystem swerveSubsystem) {

        return Commands.runEnd(
                () -> swerveSubsystem.drive(1.0, 0.0, 0.0),
                () -> swerveSubsystem.drive(0.0, 0.0, 0.0),
                swerveSubsystem).withTimeout(2.0);
    }
}
