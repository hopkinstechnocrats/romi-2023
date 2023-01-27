package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

public class ResetGyro extends CommandBase {
    private final Drivetrain m_drivetrain;

    public ResetGyro(
      Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_drivetrain.resetGyro();
    }
  
}