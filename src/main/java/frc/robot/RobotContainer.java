
package frc.robot;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final PivotSubsystem pivotSubsystem = new PivotSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(
      Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    // Drivetrain control
    swerveSubsystem.setDefaultCommand(
        new RunCommand(
            () -> {
              double forward = MathUtil.applyDeadband(
                  -driverController.getLeftY(), 0.08);

              double sideways = MathUtil.applyDeadband(
                  -driverController.getLeftX(), 0.08);

              double rotation = MathUtil.applyDeadband(
                  -driverController.getRightX(), 0.08);

              swerveSubsystem.drive(
                  forward * Constants.SwerveConstants.MAX_SPEED,
                  sideways * Constants.SwerveConstants.MAX_SPEED,
                  rotation * Constants.SwerveConstants.MAX_ANGULAR_SPEED);
            },
            swerveSubsystem));

    // Run both intake rollers while right bumper is held
    driverController.rightBumper().whileTrue(
        new StartEndCommand(
            () -> intakeSubsystem.runIntake(),
            () -> intakeSubsystem.stopIntake(),
            intakeSubsystem));

    // Raise pivot while A is held
    driverController.a().whileTrue(
        new StartEndCommand(
            () -> pivotSubsystem.raise(),
            () -> pivotSubsystem.stop(),
            pivotSubsystem));

    // Lower pivot while B is held
    driverController.b().whileTrue(
        new StartEndCommand(
            () -> pivotSubsystem.lower(),
            () -> pivotSubsystem.stop(),
            pivotSubsystem));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
