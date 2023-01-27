// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import javax.lang.model.UnknownEntityException;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveBalance extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_deadzoneAngle;
  private final double m_speed;
  private boolean m_falling = false;
  private boolean m_direction;
  private double m_waitTime;
  private double m_maxWaitTime;

  /**
   * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param deadzoneDegrees The angle at which the robot will start moving
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveBalance(double speed, double deadzoneDegrees, double maxWaitTime, Drivetrain drive) {
    m_deadzoneAngle = deadzoneDegrees;
    m_speed = speed;
    m_drive = drive;
    //Don't wait initially
    m_waitTime = maxWaitTime;
    m_maxWaitTime = maxWaitTime;
    addRequirements(drive);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(m_falling){
      // If already falling
      m_falling = Math.abs(m_drive.getGyroRateY())>20;
    }
    else{
      // Start of falling?????
      m_falling = Math.abs(m_drive.getGyroRateY())>20;
      if(m_falling){
        // Oh yeah we def started falling
        m_direction = m_drive.getLeftEncoderRateMps()>0;
        m_waitTime = 0;
      } else {
        m_waitTime ++;
      }
    }


    if(m_falling == true){
      if(m_direction){
        m_drive.arcadeDrive(
          -1, 0);
      }
      else{
        m_drive.arcadeDrive(1, 0);
      }
    }
    else{
      if (m_waitTime > m_maxWaitTime){
        final double angle_degrees = m_drive.getGyroAngleY();
    
        if(angle_degrees > m_deadzoneAngle/2){
         m_drive.pidDrive(m_speed, m_speed);
        }
        else if(angle_degrees < -m_deadzoneAngle/2){
            m_drive.pidDrive(-m_speed, -m_speed);
        }
        else {
            m_drive.pidDrive(0, 0);
        }
      }
      else{
        m_drive.arcadeDrive(0, 0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  
  @Override
  public boolean isFinished() {
    //never finished
    return false;

  }
}
