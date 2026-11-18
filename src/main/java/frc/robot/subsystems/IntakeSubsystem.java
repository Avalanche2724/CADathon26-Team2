
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX leftMotor = new TalonFX(IntakeConstants.LEFT_MOTOR_ID);

    private final TalonFX rightMotor = new TalonFX(IntakeConstants.RIGHT_MOTOR_ID);

    public void runIntake() {
        leftMotor.set(IntakeConstants.INTAKE_SPEED);
        rightMotor.set(IntakeConstants.INTAKE_SPEED);
    }

    public void stopIntake() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }
}
