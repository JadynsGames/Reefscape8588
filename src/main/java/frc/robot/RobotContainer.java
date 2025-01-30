// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Import necessary classes and subsystems
/*import frc.robot.Constants.OperatorConstants; // Constants for operator settings
import edu.wpi.first.wpilibj2.command.Command; // Command base class
import edu.wpi.first.wpilibj2.command.InstantCommand; // Command that runs once and immediately finishes
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // SmartDashboard
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup; // Parallel command group
import edu.wpi.first.wpilibj2.command.WaitCommand; */// Wait command
import edu.wpi.first.wpilibj2.command.StartEndCommand; // Command that runs a start action while active
import edu.wpi.first.wpilibj2.command.button.CommandXboxController; // Xbox controller support
import frc.robot.commands.AutonCommand; // Autonomous command
import frc.robot.commands.DefaultDrive; // Default driving command
import frc.robot.subsystems.Dumpster;
import frc.robot.subsystems.SwerveSubsystem; // Swerve drive subsystem for movement


/**
 * The RobotContainer class is responsible for declaring the robot's subsystems,
 * commands, and configuring the input bindings for the controller.
 */
public class RobotContainer {
  // Define robot subsystems as public static final to allow access throughout the
  // class
  public static final SwerveSubsystem m_Swerb = new SwerveSubsystem();
  public static final Dumpster dumpster = new Dumpster();


  public static final AutonCommand m_autonCommand = new AutonCommand();
  // Create an Xbox controller instance to handle driver input (0 is the port
  // number)
  public static CommandXboxController driverController = new CommandXboxController(0);

  /**
   * Constructor for RobotContainer. This is called when the robot is initialized.
   * It sets up the command bindings for the robot's controls.
   */
  public RobotContainer() {
    // Call the method to configure the input bindings
    configureBindings();
  }

  /**
   * This method maps controller inputs to commands that control the robot's
   * subsystems.
   * It defines the behavior for each button press and its associated actions.
   */
  private void configureBindings() {
    // Set the default command for the swerve drive subsystem
    m_Swerb.setDefaultCommand(new DefaultDrive());

    
    //Drops Coral
    driverController.a()
        .whileTrue(new StartEndCommand(
            () -> {
              dumpster.releaseCoral(0.2);
            },
            () -> {
              dumpster.releaseCoral(0);
            }));

  }

  /**
   * This method provides the command to run during the autonomous phase.
   *
   * @return The command to execute in autonomous mode.
   */
  public AutonCommand getAutonomousCommand() {
    return m_autonCommand;
  }
}