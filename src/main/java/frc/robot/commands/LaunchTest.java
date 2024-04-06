package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
//import frc.robot.subsystems.LauncherSubsystem.Launcher;

public class LaunchTest extends Command {
  private final LauncherSubsystem m_launch;
  private final IntakeSubsystem m_intake;
  
  public LaunchTest(LauncherSubsystem launcher, IntakeSubsystem intake) {
    m_launch = launcher;
    m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    m_launch.runLauncher();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
    //return Math.abs(m_index.getIndexMotorSpeed() + IntakeConstants.kIndexMotorSpeed) <= IntakeConstants.kIndexMotorDCTolerance;
  }
}