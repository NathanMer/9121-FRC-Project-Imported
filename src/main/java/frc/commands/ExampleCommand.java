
package frc.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
//import frc.robot.subsystems.IntakeSubsystem.feedLauncher;



/** An example command that uses an example subsystem. 
 * @param <IntakeSubsytem>*/
public class ExampleCommand<IntakeSubsytem> extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsytem m_intake;
  private final LauncherSubsystem m_launcher;
  /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleCommand(LauncherSubsystem launcher, IntakeSubsytem intake) {
    m_launcher = launcher;
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(launcher);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Commands.print("Finished");
      //m_intake.feedLauncher(m_launcher);
    //new ParallelCommandGroup(new RunCommand(() -> m_launcher.runLauncher()).withTimeout(2.0), new RunCommand(() -> m_intake.feedLauncher(m_launcher)).withTimeout(2.0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
