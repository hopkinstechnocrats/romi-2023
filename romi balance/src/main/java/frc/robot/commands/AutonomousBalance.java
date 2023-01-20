// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.sensors.RomiGyro;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousBalance extends SequentialCommandGroup {

  public AutonomousBalance(Drivetrain drivetrain) {
    addCommands(
        
        new DriveBalance(0.05, 10, drivetrain));
  }
}