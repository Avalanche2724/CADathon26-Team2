
package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    private static final double ALIGNMENT_TOLERANCE = 2.0;

    public boolean hasTarget() {
        return limelight.getEntry("tv").getDouble(0.0) == 1.0;
    }

    public double getHorizontalOffset() {
        return limelight.getEntry("tx").getDouble(0.0);
    }

    public double getVerticalOffset() {
        return limelight.getEntry("ty").getDouble(0.0);
    }

    public double getTargetArea() {
        return limelight.getEntry("ta").getDouble(0.0);
    }

    // Check whether the target is centered
    public boolean isTargetCentered() {
        return hasTarget()
                && Math.abs(getHorizontalOffset()) <= ALIGNMENT_TOLERANCE;
    }

    // Return horizontal correction direction
    public double getAlignmentDirection() {

        if (!hasTarget() || isTargetCentered()) {
            return 0.0;
        }

        return getHorizontalOffset() > 0 ? 1.0 : -1.0;
    }

    // Publish vision information to SmartDashboard
    @Override
    public void periodic() {

        SmartDashboard.putBoolean("Vision/Target Detected", hasTarget());
        SmartDashboard.putNumber("Vision/Horizontal Offset", getHorizontalOffset());
        SmartDashboard.putNumber("Vision/Vertical Offset", getVerticalOffset());
        SmartDashboard.putNumber("Vision/Target Area", getTargetArea());
        SmartDashboard.putBoolean("Vision/Target Centered", isTargetCentered());
    }
}
