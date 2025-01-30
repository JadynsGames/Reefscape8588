// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Dumpster extends SubsystemBase {
  private SparkMax m_dumpsterDown;

  /** Creates a new ExampleSubsystem. */
  public Dumpster() {
    m_dumpsterDown =  new SparkMax(Constants.SubsystemConstants.kDumpsterID, MotorType.kBrushless);

  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void releaseCoral(double speed) {

    m_dumpsterDown.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}