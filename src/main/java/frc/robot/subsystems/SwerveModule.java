package frc.robot.subsystems;

// Import necessary libraries
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import java.io.ObjectInputFilter.Config;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
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

         //private final SparkBaseConfig sBaseConfig_turningSparkMax;
        // private final SparkBaseConfig sBaseConfig_drivingSparkMax;

        private final SparkMaxConfig c_turningSparkMax;
        private final SparkMaxConfig c_drivingSparkMax;

        private final RelativeEncoder m_drivingEncoder;
        private final AbsoluteEncoder m_turningEncoder;

        private final SparkClosedLoopController m_drivingPidController;
        private final SparkClosedLoopController m_turningPidController;

        private final ClosedLoopConfig m_turningClosedLoopConfig;
        private final ClosedLoopConfig m_drivingClosedLoopConfig;

        private double m_chassisAngularOffset;

        public final Translation2d m_moduleLocation;

        private SwerveModuleState m_targetState = new SwerveModuleState(0.0, new Rotation2d());

        public SwerveModule(int kTurningCanID, int kDrivingCanID, double chassisAngularOffset,
                        Translation2d m_moduleLocation) {

                m_turningSparkMax = new SparkMax(kTurningCanID, MotorType.kBrushless);
                m_drivingSparkMax = new SparkMax(kDrivingCanID, MotorType.kBrushless);

                m_drivingEncoder = m_drivingSparkMax.getEncoder();

                

                m_turningClosedLoopConfig = new ClosedLoopConfig();
                m_drivingClosedLoopConfig = new ClosedLoopConfig();

                c_drivingSparkMax = new SparkMaxConfig();
                
                c_drivingSparkMax.signals.primaryEncoderPositionPeriodMs(kDrivingCanID); // Set encoder update period
                m_drivingSparkMax.configure(c_drivingSparkMax, null, null);

                // WE NEED TO FIX THIS IDK WHY THESE TWO LINES ARENT WORKING BUT THEY ARE INTEGRAL
                // IT DOESN't INSTANTIATE FOR SOME REASON
                // sBaseConfig_turningSparkMax = new SparkBaseConfig(); 
                // sBaseConfig_drivingSparkMax = new SparkBaseConfig();

                c_turningSparkMax = new SparkMaxConfig();
                c_turningSparkMax.signals.primaryEncoderPositionPeriodMs(kDrivingCanID); // Set encoder update period
                m_turningSparkMax.configure(c_turningSparkMax, null, null);
                
                
                m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();

                m_turningPidController = m_turningSparkMax.getClosedLoopController();

                c_turningSparkMax.inverted(true).idleMode(IdleMode.kBrake); // Invert motor direction, set brake mode

                c_turningSparkMax.encoder.positionConversionFactor(chassisAngularOffset)
                                .velocityConversionFactor(chassisAngularOffset);

                c_turningSparkMax.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(
                                Constants.DriveConstants.kTurningP, Constants.DriveConstants.kTurningI,
                                Constants.DriveConstants.kTurningD);

                m_drivingPidController = m_drivingSparkMax.getClosedLoopController();

                c_drivingSparkMax.idleMode(IdleMode.kBrake);

                c_drivingSparkMax.encoder
                                .positionConversionFactor(Constants.DriveConstants.kDrivePositionConversionFactor)
                                .velocityConversionFactor(Constants.DriveConstants.kDriveVelocityConversionFactor);

                c_drivingSparkMax.closedLoop
                                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                .pid(Constants.DriveConstants.kDrivingP, Constants.DriveConstants.kDrivingI,
                                                Constants.DriveConstants.kDrivingD);

                c_turningSparkMax.smartCurrentLimit(kDrivingCanID);
                m_turningClosedLoopConfig.positionWrappingEnabled(true);

                m_turningClosedLoopConfig
                                .positionWrappingMinInput(Constants.DriveConstants.kTurningEncoderPositionPIDMinInput);
                m_turningClosedLoopConfig
                                .positionWrappingMaxInput(Constants.DriveConstants.kTurningEncoderPositionPIDMaxInput);

                m_drivingClosedLoopConfig.minOutput(-1);
                m_drivingClosedLoopConfig.maxOutput(1);

                c_drivingSparkMax.smartCurrentLimit(kDrivingCanID);

                m_chassisAngularOffset = chassisAngularOffset;
                this.m_moduleLocation = m_moduleLocation;
        }

        // Method to get the current state (speed and angle) of the module
        public SwerveModuleState getState() {
                return new SwerveModuleState(m_drivingEncoder.getVelocity(),
                                new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
        }

        // Method to get the current position (distance and angle) of the module
        public SwerveModulePosition getPosition() {
                return new SwerveModulePosition(m_drivingEncoder.getPosition(),
                                new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
        }

        // Method to set the desired state (speed and angle) of the module
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
