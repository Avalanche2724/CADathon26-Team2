
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {

    private final TalonFX pivotMotor = new TalonFX(PivotConstants.MOTOR_ID);

    public void raise() {
        pivotMotor.set(PivotConstants.PIVOT_SPEED);
    }

    public void lower() {
        pivotMotor.set(-PivotConstants.PIVOT_SPEED);
    }

    public void stop() {
        pivotMotor.stopMotor();
    }
}
