// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.sensors.RomiGyro;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousBalance extends SequentialCommandGroup {
  NetworkTableInstance m_insta = NetworkTableInstance.getDefault();
  NetworkTable m_autoTable = m_insta.getTable("Auto_Table");

  public AutonomousBalance(Drivetrain drivetrain) {

    m_autoTable.getEntry("Speed").setDouble(Constants.defaultSpeed);
    m_autoTable.getEntry("Deadzone_Degrees").setDouble(Constants.defaultDegrees);
    m_autoTable.getEntry("Max_Wait_Time").setDouble(Constants.maxWaitTime);

    addCommands(
        
        new DriveBalance(
          m_autoTable.getEntry("Speed").getDouble(Constants.defaultSpeed),
          m_autoTable.getEntry("Deadzone_Degrees").getDouble(Constants.defaultDegrees),
          m_autoTable.getEntry("Max_Wait_Time").getDouble(Constants.maxWaitTime),
          drivetrain));
  }
}