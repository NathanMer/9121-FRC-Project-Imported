// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
//import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.utils.GamepadUtils;

//import java.nio.file.Path;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

//
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  public final LauncherSubsystem m_launcher = new LauncherSubsystem();
    private final SendableChooser<Command> choose;
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  //PS5Controller m_driverController = new PS5Controller(OIConstants.kDriverControllerPort); 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    getNamedCommands();
    


    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -GamepadUtils.squareInput(
                        m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -GamepadUtils.squareInput(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -GamepadUtils.squareInput(
                        m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true,
                    false),
            m_robotDrive));

    // set the arm subsystem to run the "runAutomatic" function continuously when no other command
    // is running
    
    m_arm.setDefaultCommand(new RunCommand(() -> m_arm.runAutomatic(), m_arm));

    // set the intake to stop (0 power) when no other command is running
    m_intake.setDefaultCommand(new RunCommand(() -> m_intake.setPower(0.0), m_intake));

    // configure the launcher to stop when no other command is running
    m_launcher.setDefaultCommand(new RunCommand(() -> m_launcher.stopLauncher(), m_launcher));

    //SequentialCommandGroup shootCommand = new SequentialCommandGroup(null)
    


    

    //Chooser
    choose = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(choose);

  }

  public void getNamedCommands(){
    NamedCommands.registerCommand("launch", new RunCommand(()->m_launcher.runLauncher(),m_launcher));
    NamedCommands.registerCommand("launch1", new InstantCommand(() -> m_launcher.runLauncher()).withTimeout(2));
    NamedCommands.registerCommand("stopLaunch", new RunCommand(() -> m_launcher.stopLauncher()));
    
    NamedCommands.registerCommand("armDown", new RunCommand(()-> m_arm.setTargetPosition(Constants.Arm.kIntakePosition)));
    NamedCommands.registerCommand("armUp", new RunCommand(()-> m_arm.setTargetPosition(Constants.Arm.kScoringPosition)));
    
    NamedCommands.registerCommand("intake", new InstantCommand(() -> m_intake.setPower(-1.0),m_intake));
    NamedCommands.registerCommand("stopIntake", new InstantCommand(() -> m_intake.setPower(-1.0),m_intake));
    NamedCommands.registerCommand("feedLauncher", new InstantCommand(() -> m_intake.feedLauncher(m_launcher)));

    //NamedCommands.registerCommand("launch", new RunCommand(() -> m_launcher.runLauncher()));
    //NamedCommands.registerCommand("launchCon", new SequentialCommandGroup(new RunCommand(()-> m_launcher.runLauncher().withTimeout(2.0))));
    NamedCommands.registerCommand("shoot", new ParallelCommandGroup(new RunCommand(() -> m_launcher.runLauncher()).withTimeout(2.0), new RunCommand(() -> m_intake.feedLauncher(m_launcher)).withTimeout(2.0)));
    NamedCommands.registerCommand("marker1", Commands.print("Finished"));
    NamedCommands.registerCommand("marker2", new PrintCommand("Finished"));
  }




  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // button to put swerve modules in an "x" configuration to hold position
    new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

    // set up arm preset positions
    //new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        //.onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kScoringPosition)));
        //changing intake to left bumper instead of trigger
        //Bad
        // Change left trigger to reverseLAUNCHER /* */
    //new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
      //  .onTrue(new InstantCommand(() -> m_launcher.reverseLauncher()));

    /*new Trigger(
            () ->
                m_driverController.getLeftTriggerAxis()
                    > Constants.OIConstants.kTriggerButtonThreshold)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kIntakePosition)));
    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kHomePosition))); */

    // intake controls (run while button is held down, run retract command once when the button is
    // released)
    new Trigger(
            () ->
                m_driverController.getRightTriggerAxis()
                    > Constants.OIConstants.kTriggerButtonThreshold)
        .whileTrue(new RunCommand(() -> m_intake.setPower(Constants.Intake.kIntakePower), m_intake));
        //.onFalse(m_intake.retract());

    

    // launcher controls (button to pre-spin the launcher and button to launch)
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(() -> m_launcher.runLauncher(), m_launcher));

        //Enables intake from leftBumper
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value).whileTrue(new RunCommand(()-> m_arm.setTargetPosition(Constants.Arm.kScoringPosition)));
    //Set arm pos from x button
    //maybe something wrong with running setPosition command
    //new JoystickButton(m_driverController, XboxController.Button.kB.value).onTrue(new RunCommand(()-> m_arm.setTargetPosition(Constants.Arm.kIntakePosition)));
   

    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .whileTrue((new RunCommand(()-> m_arm.setTargetPosition(Constants.Arm.kIntakePosition))));
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .whileTrue(new RunCommand(()-> m_arm.setTargetPosition(Constants.Arm.kHomePosition)));
    
        //trying to set start button to reset odometry
    //new JoystickButton(m_driverController, XboxController.Button.kStart.value).onTrue(new InstantCommand(()-> m_robotDrive.resetOdometry(new Pose2d(m_robotDrive.getPose().getTranslation(), new Rotation2d(0)))));   
    //new JoystickButton(m_driverController, XboxController.Button.kStart.value).onTrue(new InstantCommand(()-> m_robotDrive.resetOdometry(new Pose2d(m_robotDrive.getPose().getTranslation(), m_robotDrive.getPose().getRotation()))));
    new JoystickButton(m_driverController, XboxController.Button.kStart.value).onTrue(new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .onTrue(m_intake.feedLauncher(m_launcher));

    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .whileTrue(new RunCommand(() -> m_intake.setPower(-1.0)));
  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            //0 x -3.5 x
            //List.of(new Translation2d(0, 0), new Translation2d(-2, 0)),
            List.of(),
            // End 3 meters straight ahead of where we started, facing forward


            //can change rotation2d value to 180 to turn around
            //3.5
            new Pose2d(2, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose()); 
    m_robotDrive.zeroHeading();
    //Trying to call registerNamedCommands to get commands
   // registerNamedCommands();
    // Run path following command, then stop at the end.
    //m_robotDrive.alterHeading();


    //return NamedCommands.getCommand("launch").andThen(()-> Timer.delay(2)).andThen(NamedCommands.getCommand("intake")).andThen(()-> Timer.delay(2)).andThen(NamedCommands.getCommand("stopLaunch")).andThen(swerveControllerCommand).andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    
   
    //return new RunCommand(() -> m_intake.setPower(-1.0)).andThen(Timer.delay(2.0)).andThen(swerveControllerCommand.andThen(()-> m_robotDrive.drive(.25,.25,0,false,false)));
    //return new RunCommand(() -> m_launcher.runLauncher(), m_launcher).withTimeout(2).andThen(m_intake.feedLauncher(m_launcher));//.andThen(swerveControllerCommand.andThen(()-> m_robotDrive.drive(.25,.25,0,false,false)))
   
   
    //run & go forward (wrong way)
    
    //   return new RunCommand(() -> m_launcher.runLauncher(), m_launcher).withTimeout(2).andThen(m_intake.feedLauncher(m_launcher)).andThen(swerveControllerCommand.andThen(()-> m_robotDrive.drive(.25,.25,0,false,false)));
    
   //reliable MOVE FORWARD
   //return swerveControllerCommand.andThen(()-> m_robotDrive.drive(.25,.25,0,false,false));
   
   //Daniels auto
   //  return new RunCommand(() -> m_launcher.runLauncher(), m_launcher).withTimeout(2).andThen(m_intake.feedLauncher(m_launcher)).andThen(swerveControllerCommand.andThen(()-> m_robotDrive.drive(0,.15,0,false,false))).withTimeout(2).andThen(new RunCommand(()-> m_arm.setTargetPosition(Constants.Arm.kIntakePosition))).andThen(new RunCommand(() -> m_intake.setPower(Constants.Intake.kIntakePower), m_intake)).andThen(new RunCommand(()-> m_arm.setTargetPosition(Constants.Arm.kScoringPosition))).andThen(new RunCommand(() -> m_launcher.runLauncher(), m_launcher).withTimeout(1)).andThen(m_intake.feedLauncher(m_launcher)); 

 
/* 
   //Calls path and commands manually 
   PathPlannerPath right = PathPlannerPath.fromPathFile("Test Right");
   PathPlannerPath rightBack = PathPlannerPath.fromPathFile("Comeback Test");
// :D
    return Commands.sequence(
        new RunCommand(() -> m_launcher.runLauncher(), m_launcher).withTimeout(2), //rev up launcher
     Commands.runOnce(()-> m_intake.feedLauncher(m_launcher).withTimeout(2)), //run launcher with note
        new RunCommand(()-> m_arm.setTargetPosition(Constants.Arm.kIntakePosition)), //moves arm to ground position
        AutoBuilder.followPath(right), //runs path to right note
        new RunCommand(() -> m_intake.setPower(-1.0)).withTimeout(2), //intakes at same time
    
        
        //new RunCommand(() -> m_intake.setPower(-1.0)).withTimeout(2),
        new RunCommand(()-> m_arm.setTargetPosition(Constants.Arm.kHomePosition)), //brings arm back up
        AutoBuilder.followPath(rightBack), //path to go back to subwoofer
        m_intake.feedLauncher(m_launcher).withTimeout(2) //can take out if not working

    );*/
//usable middle THIS IS WHAT WE HAVE BEEN USING I AM WRITING IN ALL CAPS BECAUSE THERE ARE SO MANY OTHER FAILED AUTOS
     //  return new RunCommand(() -> m_launcher.runLauncher(), m_launcher).withTimeout(2).andThen(m_intake.feedLauncher(m_launcher)).andThen(AutoBuilder.buildAuto("Only Left"));//.andThen(new RunCommand(()-> m_arm.setTargetPosition(Constants.Arm.kIntakePosition))).andThen(new RunCommand(() -> m_intake.setPower(-1.0))).withTimeout(2).andThen(new RunCommand(()-> m_arm.setTargetPosition(Constants.Arm.kHomePosition)));//.andThen(AutoBuilder.buildAuto("Comeback Left"));
   
    //Test Maybe get another note?
   //   return new RunCommand(() -> m_launcher.runLauncher(), m_launcher).withTimeout(2).andThen(m_intake.feedLauncher(m_launcher)).andThen(new RunCommand(()-> m_arm.setTargetPosition(Constants.Arm.kIntakePosition))).andThen(AutoBuilder.buildAuto("Test Left")).andThen(new RunCommand(() -> m_intake.setPower(-1.0))).withTimeout(2).andThen(new RunCommand(()-> m_arm.setTargetPosition(Constants.Arm.kHomePosition)));//.andThen(AutoBuilder.buildAuto("Comeback Left"));
   
   // return new RunCommand(() -> m_launcher.runLauncher(), m_launcher).withTimeout(2).andThen(m_intake.feedLauncher(m_launcher)).andThen(AutoBuilder.buildAuto("Test Left")).andThen(AutoBuilder.buildAuto("Comeback Test"));

   //Nathan's Pathplanner auto
     // return choose.getSelected();
      
    //pathplanner auto with shoot before and after
    //return new RunCommand(() -> m_launcher.runLauncher(), m_launcher).withTimeout(2).andThen(m_intake.feedLauncher(m_launcher)).andThen(choose.getSelected()).andThen(new RunCommand(() -> m_launcher.runLauncher(), m_launcher)).withTimeout(2).andThen(m_intake.feedLauncher(m_launcher));

//github test

         return new RunCommand(() -> m_launcher.runLauncher(), m_launcher).withTimeout(2).andThen(m_intake.feedLauncher(m_launcher)).andThen(AutoBuilder.buildAuto("Only Left"));
   
   
   //return new InstantCommand(()->Timer.delay(5.0)).andThen(swerveControllerCommand.andThen(()-> m_robotDrive.drive(.25,.25,0,false,false)));
    //return NamedCommands.getCommand("shoot").andThen(swerveControllerCommand.andThen(()-> m_robotDrive.drive(.25,.25,0,false,false)));
    /* 
    //long millisecondsToRun = 1000;
    //long initTime = Utility.getFPGATime();
    //return (m_intake.feedLauncher(m_launcher).andThen((Timer.delay(1)m_robotDrive.drive(.25,.25, 0, false, false)));
   // PathPlannerPath path = PathPlannerPath.fromPathFile("Shoot Middle");
  //return AutoBuilder.followPath(path);

    */
  
     //PathPlannerPath path = PathPlannerPath.fromPathFile("Shoot Middle");
    //return NamedCommands.getCommand("launch").andThen()
    

     //Create a path following command using AutoBuilder. This will also trigger event markers.
     //return AutoBuilder.followPath(path);

}
} 
