// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS5Controller.Button;
//import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.PhotonVision;
import frc.robot.Constants.SubsystemConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import frc.robot.subsystems.Dumpster;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Dumpster m_dumpster = new Dumpster();
  private final VisionSubsystem m_photonVisionCam1 = new VisionSubsystem("Cam 1");
  private final VisionSubsystem m_photonVisionCam2 = new VisionSubsystem("Cam 2");
  private final PIDController m_visionTurnController = new PIDController(PhotonVision.visionTurnkP, 0, PhotonVision.visionTurnkD);
  private final PIDController m_visionDriveController = new PIDController(PhotonVision.visionDrivekP, 0, PhotonVision.visionDrivekD);


  private final SendableChooser<Command> autoChooser;
  // The driver's controller
  //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  PS5Controller m_driverController = new PS5Controller(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_visionTurnController.setTolerance(1);
    // Configure the button bindings
    configureButtonBindings();
    // XBOX VERSION
    //configureButtonBindingsXbox();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY() * DriveConstants.kDriveThrottle, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX() * DriveConstants.kDriveThrottle, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX() * DriveConstants.kDriveThrottle, OIConstants.kDriveDeadband),
                true),
            m_robotDrive));

            
            NamedCommands.registerCommand("ReleaseCoral", m_dumpster.startDumpsterCommand());
            NamedCommands.registerCommand("StopDumpster",m_dumpster.stopDumpsterCommand());
            NamedCommands.registerCommand("LockCoral",m_dumpster.lockDumpsterCommand());

            autoChooser = AutoBuilder.buildAutoChooser();
            SmartDashboard.putData("Auto Mode", autoChooser); 
  }   

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // SYS ID
    /*
    new JoystickButton(m_driverController, Button.kTriangle.value)
      .whileTrue(new ParallelCommandGroup(
        new RunCommand( () ->
          m_robotDrive.resetEncoders()
        ),
        m_robotDrive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
      ));
    new JoystickButton(m_driverController, Button.kCross.value)
      .whileTrue(m_robotDrive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    new JoystickButton(m_driverController, Button.kSquare.value)
      .whileTrue(m_robotDrive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    new JoystickButton(m_driverController, Button.kCircle.value)
      .whileTrue(m_robotDrive.sysIdDynamic(SysIdRoutine.Direction.kReverse));*/

    // TINY DUMPSTER (LOCK CORAL)
    new JoystickButton(m_driverController, Button.kSquare.value)
    .whileTrue(new InstantCommand(
        () -> {
          m_dumpster.runDumpster(-1*SubsystemConstants.kDumpsterLock);
        }));


    // SET ZERO YAW
    new JoystickButton(m_driverController, Button.kTriangle.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));
    
    // DRIVE KILLSWITCH
    new JoystickButton(m_driverController, Button.kL3.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    // LOWER DUMPSTER SPEED
    new JoystickButton(m_driverController, Button.kR3.value)
        .whileTrue(new StartEndCommand(
            () -> {
              m_dumpster.fastMode(true);
            },
            () -> {
              m_dumpster.fastMode(false);
            }));

    // DUMPSTER
    new JoystickButton(m_driverController, Button.kR2.value)
        .whileTrue(new StartEndCommand(
            () -> {
              m_dumpster.runDumpster(-1);
            },
            () -> {
              m_dumpster.runDumpster(0);
            }));

    // REVERSE DUMPSTER
    new JoystickButton(m_driverController, Button.kL2.value)
    .whileTrue(new StartEndCommand(
        () -> {
          m_dumpster.runDumpster(1);
        },
        () -> {
          m_dumpster.runDumpster(0);
        }));

    // VISION
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.drive(
              -MathUtil.applyDeadband(m_driverController.getLeftY() * DriveConstants.kDriveThrottle, OIConstants.kDriveDeadband),
              //-MathUtil.applyDeadband(m_visionDriveController.calculate(m_photonVisionCam1.getDistance(),0) * DriveConstants.kDriveThrottle, OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getLeftX() * DriveConstants.kDriveThrottle, OIConstants.kDriveDeadband),
              1.0 * m_visionTurnController.calculate(m_photonVisionCam2.getYaw(),0),
              true
            ), 
            m_robotDrive));
    
    new JoystickButton(m_driverController, Button.kL1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.drive(
              -MathUtil.applyDeadband(m_driverController.getLeftY() * DriveConstants.kDriveThrottle, OIConstants.kDriveDeadband),
              //-MathUtil.applyDeadband(m_visionDriveController.calculate(m_photonVisionCam2.getDistance(),0) * DriveConstants.kDriveThrottle, OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getLeftX() * DriveConstants.kDriveThrottle, OIConstants.kDriveDeadband),
              1.0 * m_visionTurnController.calculate(m_photonVisionCam1.getYaw(),0),
              true
            ), 
            m_robotDrive));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
