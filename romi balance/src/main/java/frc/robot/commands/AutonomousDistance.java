// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousDistance extends SequentialCommandGroup {
  NetworkTableInstance m_insta = NetworkTableInstance.getDefault();
  NetworkTable m_autoTable = m_insta.getTable("Auto_Table");


  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousDistance(Drivetrain drivetrain) {
    m_autoTable.getEntry("Speed").setDouble(Constants.defaultSpeed);

    addCommands(
        new DriveDistance(m_autoTable.getEntry("Speed").getDouble(0.05),
         10, drivetrain)
    );
  }
}
