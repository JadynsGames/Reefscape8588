package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants;

public class SwerveModule {
    private final SparkMax m_turningSparkMax;
    private final SparkMax m_drivingSparkMax;

    private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningEncoder;

    private final SparkClosedLoopController m_drivingPidController;
    private final SparkClosedLoopController m_turningPidController;

    private double m_chassisAngularOffset;

    public final Translation2d m_moduleLocation;

    //private SwerveModuleState m_targetState = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModule(int kTurningCanID, int kDrivingCanID, double chassisAngularOffset,
            Translation2d m_moduleLocation) {
        m_turningSparkMax = new SparkMax(kTurningCanID, MotorType.kBrushless);
        m_drivingSparkMax = new SparkMax(kDrivingCanID, MotorType.kBrushless);

        // Reset all SPARKS MAX
        m_turningSparkMax.restoreFactoryDefaults();
        m_drivingSparkMax.restoreFactoryDefaults();

        m_drivingEncoder = m_drivingSparkMax.getEncoder();
        m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

        m_drivingPidController = m_drivingSparkMax.getPIDController();
        m_turningPidController = m_turningSparkMax.getPIDController();

        m_drivingEncoder.setPositionConversionFactor(Constants.DriveConstants.kDrivePositionConversionFactor);
        m_drivingEncoder.setVelocityConversionFactor(Constants.DriveConstants.kDriveVelocityConversionFactor);

        m_turningEncoder.setPositionConversionFactor(Constants.DriveConstants.kAnglePositionConversionFactor);
        m_turningEncoder.setVelocityConversionFactor(Constants.DriveConstants.kAngleVelocityConversionFactor);

        m_drivingPidController.setFeedbackDevice(m_drivingEncoder);
        m_turningPidController.setFeedbackDevice(m_turningEncoder);

        m_turningEncoder.setInverted(true);

        m_turningPidController.setPositionPIDWrappingEnabled(true);
        m_turningPidController
                .setPositionPIDWrappingMinInput(Constants.DriveConstants.kTurningEncoderPositionPIDMinInput);
        m_turningPidController
                .setPositionPIDWrappingMaxInput(Constants.DriveConstants.kTurningEncoderPositionPIDMaxInput);

        m_drivingPidController.setP(Constants.DriveConstants.kDrivingP);
        m_drivingPidController.setI(Constants.DriveConstants.kDrivingI);
        m_drivingPidController.setD(Constants.DriveConstants.kDrivingD);
        m_drivingPidController.setFF(Constants.DriveConstants.kDrivingFF);
        m_drivingPidController.setOutputRange(-1, 1);

        m_turningPidController.setP(Constants.DriveConstants.kTurningP);
        m_turningPidController.setI(Constants.DriveConstants.kTurningI);
        m_turningPidController.setD(Constants.DriveConstants.kTurningD);
        m_turningPidController.setFF(Constants.DriveConstants.kTurningFF);
        m_turningPidController.setOutputRange(-1, 1);

        m_drivingSparkMax.setSmartCurrentLimit(Constants.DriveConstants.kDrivingMaxCurrent);
        m_turningSparkMax.setSmartCurrentLimit(Constants.DriveConstants.kTurningMaxCurrent);
        m_drivingSparkMax.setIdleMode(IdleMode.kBrake);
        m_turningSparkMax.setIdleMode(IdleMode.kBrake);

        m_drivingSparkMax.burnFlash();
        m_turningSparkMax.burnFlash();

        m_chassisAngularOffset = chassisAngularOffset;

        this.m_moduleLocation = m_moduleLocation;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_drivingEncoder.getVelocity(),
                new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_drivingEncoder.getPosition(),
                new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedState = new SwerveModuleState();
        correctedState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedState,
                new Rotation2d(m_turningEncoder.getPosition()));

        m_drivingPidController.setReference(optimizedDesiredState.speedMetersPerSecond,
                SparkMax.ControlType.kVelocity);
        m_turningPidController.setReference(optimizedDesiredState.angle.getRadians(),
                SparkMax.ControlType.kPosition);

        m_targetState = desiredState;
    }
}
