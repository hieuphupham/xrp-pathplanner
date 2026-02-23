// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;

public class TurnArmDegree extends Command {
  private final Arm m_arm;
  private final double m_degrees;
  

  /**
   * Creates a new TurnDegrees. This command will turn your robot for a desired rotation (in
   * degrees) and rotational speed.
   *
   * @param speed The speed which the robot will drive. Negative is in reverse.
   * @param degrees Degrees to turn. Leverages encoders to compare distance.
   * @param drive The drive subsystem on which this command will run
   */
  public TurnArmDegree(double degrees, Arm arm) {
    m_degrees = degrees;
    m_arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set motors to stop, read encoder values for starting point
    m_arm.setAngle(m_degrees);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setAngle(m_degrees);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setAngle(0);
  }

  public boolean isFinished() {
    // We are finished when the current angle is "close enough" to the target.
    // Using an error threshold (e.g., 1.0 degree) prevents the robot from 
    // jittering if it never hits the exact decimal value.
    return Math.abs(m_arm.getMeasurement() - m_degrees) < 1.0;
}
}