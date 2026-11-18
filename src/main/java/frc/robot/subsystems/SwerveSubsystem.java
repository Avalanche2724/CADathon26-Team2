
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {

    // Motor control requests
    private final PositionVoltage steeringRequest = new PositionVoltage(0);

    private final VelocityVoltage driveRequest = new VelocityVoltage(0);

    // Half of the robot's 22-inch wheelbase and trackwidth
    private final double halfDistance = Units.inchesToMeters(11.0);

    // Swerve module positions relative to robot center
    private final Translation2d frontLeftLocation = new Translation2d(halfDistance, halfDistance);

    private final Translation2d frontRightLocation = new Translation2d(halfDistance, -halfDistance);

    private final Translation2d backLeftLocation = new Translation2d(-halfDistance, halfDistance);

    private final Translation2d backRightLocation = new Translation2d(-halfDistance, -halfDistance);

    // Swerve kinematics
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            frontLeftLocation,
            frontRightLocation,
            backLeftLocation,
            backRightLocation);

    // Drive motors
    private final TalonFX frontLeftDrive = new TalonFX(SwerveConstants.FL_DRIVE_ID);
    private final TalonFX frontRightDrive = new TalonFX(SwerveConstants.FR_DRIVE_ID);
    private final TalonFX backLeftDrive = new TalonFX(SwerveConstants.BL_DRIVE_ID);
    private final TalonFX backRightDrive = new TalonFX(SwerveConstants.BR_DRIVE_ID);

    // Steering motors
    private final TalonFX frontLeftSteer = new TalonFX(SwerveConstants.FL_STEER_ID);
    private final TalonFX frontRightSteer = new TalonFX(SwerveConstants.FR_STEER_ID);
    private final TalonFX backLeftSteer = new TalonFX(SwerveConstants.BL_STEER_ID);
    private final TalonFX backRightSteer = new TalonFX(SwerveConstants.BR_STEER_ID);

    // Absolute encoders
    private final CANcoder frontLeftEncoder = new CANcoder(SwerveConstants.FL_ENCODER_ID);
    private final CANcoder frontRightEncoder = new CANcoder(SwerveConstants.FR_ENCODER_ID);
    private final CANcoder backLeftEncoder = new CANcoder(SwerveConstants.BL_ENCODER_ID);
    private final CANcoder backRightEncoder = new CANcoder(SwerveConstants.BR_ENCODER_ID);

    // Constructor
    public SwerveSubsystem() {

        frontLeftDrive.setNeutralMode(NeutralModeValue.Brake);
        frontRightDrive.setNeutralMode(NeutralModeValue.Brake);
        backLeftDrive.setNeutralMode(NeutralModeValue.Brake);
        backRightDrive.setNeutralMode(NeutralModeValue.Brake);

        frontLeftSteer.setNeutralMode(NeutralModeValue.Brake);
        frontRightSteer.setNeutralMode(NeutralModeValue.Brake);
        backLeftSteer.setNeutralMode(NeutralModeValue.Brake);
        backRightSteer.setNeutralMode(NeutralModeValue.Brake);

        configureSteeringMotor(frontLeftSteer, frontLeftEncoder);
        configureSteeringMotor(frontRightSteer, frontRightEncoder);
        configureSteeringMotor(backLeftSteer, backLeftEncoder);
        configureSteeringMotor(backRightSteer, backRightEncoder);
    }

    // Convert robot movement into individual wheel states
    public void drive(double xSpeed, double ySpeed, double rotation) {

        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rotation);

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(
                moduleStates,
                SwerveConstants.MAX_SPEED);

        setModuleStates(moduleStates);
    }

    // Send speed and angle commands to all four modules
    private void setModuleStates(SwerveModuleState[] states) {

        double wheelCircumference = Math.PI * Units.inchesToMeters(
                SwerveConstants.WHEEL_DIAMETER);

        // Drive motor speed commands
        frontLeftDrive.setControl(
                driveRequest.withVelocity(
                        states[0].speedMetersPerSecond
                                / wheelCircumference
                                * SwerveConstants.DRIVE_GEAR_RATIO));

        frontRightDrive.setControl(
                driveRequest.withVelocity(
                        states[1].speedMetersPerSecond
                                / wheelCircumference
                                * SwerveConstants.DRIVE_GEAR_RATIO));

        backLeftDrive.setControl(
                driveRequest.withVelocity(
                        states[2].speedMetersPerSecond
                                / wheelCircumference
                                * SwerveConstants.DRIVE_GEAR_RATIO));

        backRightDrive.setControl(
                driveRequest.withVelocity(
                        states[3].speedMetersPerSecond
                                / wheelCircumference
                                * SwerveConstants.DRIVE_GEAR_RATIO));

        // Steering motor position commands
        // CANcoder measures module rotation directly
        frontLeftSteer.setControl(
                steeringRequest.withPosition(
                        states[0].angle.getRotations()));

        frontRightSteer.setControl(
                steeringRequest.withPosition(
                        states[1].angle.getRotations()));

        backLeftSteer.setControl(
                steeringRequest.withPosition(
                        states[2].angle.getRotations()));

        backRightSteer.setControl(
                steeringRequest.withPosition(
                        states[3].angle.getRotations()));
    }

    // Configure steering motor feedback
    private void configureSteeringMotor(
            TalonFX steeringMotor,
            CANcoder encoder) {

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        // Motor rotations per CANcoder rotation
        config.Feedback.RotorToSensorRatio = SwerveConstants.STEER_GEAR_RATIO;

        // CANcoder rotations per module rotation
        config.Feedback.SensorToMechanismRatio = 1.0;

        steeringMotor.getConfigurator().apply(config);
    }
}
