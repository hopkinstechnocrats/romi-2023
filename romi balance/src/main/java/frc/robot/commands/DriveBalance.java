// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveBalance extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_deadzoneAngle;
  private final double m_speed;

  /**
   * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param inches The number of inches the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveBalance(double speed, double deadzoneDegrees, Drivetrain drive) {
    m_deadzoneAngle = deadzoneDegrees;
    m_speed = speed;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.drive(0, 0);
    m_drive.resetEncoders();
    m_drive.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle_degrees = m_drive.getGyroAngleY();
    if(angle_degrees > m_deadzoneAngle/2){
        m_drive.drive(m_speed, m_speed);
    }
    else if(angle_degrees < -m_deadzoneAngle/2){
        m_drive.drive(-m_speed, -m_speed);
    }
    else {
        m_drive.drive(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0);
  }

  
  @Override
  public boolean isFinished() {
    //never finished
    return false;

  }
}
