
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAlignCommand {

    public static Command align(
            VisionSubsystem visionSubsystem,
            SwerveSubsystem swerveSubsystem) {

        return Commands.runEnd(

                () -> {
                    if (!visionSubsystem.hasTarget()) {
                        swerveSubsystem.drive(0.0, 0.0, 0.0);
                        return;
                    }

                    double rotation = MathUtil.clamp(
                            -visionSubsystem.getHorizontalOffset() * 0.04,
                            -1.5,
                            1.5);

                    swerveSubsystem.drive(0.0, 0.0, rotation);
                },

                () -> swerveSubsystem.drive(0.0, 0.0, 0.0),

                swerveSubsystem

        ).until(visionSubsystem::isTargetCentered)
                .withTimeout(4.0);
    }
}
