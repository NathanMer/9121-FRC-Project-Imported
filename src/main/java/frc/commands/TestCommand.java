package frc.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class TestCommand extends Command {
    private IntakeSubsystem m_intake;

    public TestCommand(IntakeSubsystem intake) {
        this.m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
       // m_intake.set(1.0);
       System.out.println("Initalized");
    }

    @Override
    public void end(boolean interrupted) {
       // m_intake.set(0.0);
    }

    @Override
    public boolean isFinished() {
        System.out.println("Finished Command");
        return true;
    }

    // execute() defaults to do nothing
    // isFinished() defaults to return false
}