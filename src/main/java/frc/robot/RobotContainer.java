// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.
/* Github testing */
package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.DriveToDistance;
import frc.robot.commands.auto.TurnToAngle;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.Swerve;


public class RobotContainer {

  private final SendableChooser<Command> chooser = new SendableChooser<Command>();
  private Swerve m_swerve = new Swerve();

  // Subsystem creation
  private final PneumaticsSubsystem m_PneumaticsSubsystem = new PneumaticsSubsystem();

  private final CommandXboxController m_driverController = new CommandXboxController(Constants.Controllers.kDriverControllerPort);
  //private final CommandXboxController m_operatorController = new CommandXboxController(Constants.Controllers.kOperatorControllerPort);

  
  public RobotContainer() {
    setupDrive(); 
    configureButtonBindings();
    //Creating a dropdown for autonomous commands to choose from
    addCommandDropdown();
    
  }

  private void setupDrive() {
    m_swerve.setDefaultCommand(
      new DriveWithJoysticks(
        m_swerve,
        () -> modifyAxis(-m_driverController.getLeftY()),
        () -> modifyAxis(m_driverController.getLeftX()),
        () -> modifyAxis(-m_driverController.getRightX())
      )
    );
  }

  private void configureButtonBindings() {
    //DRIVER
    m_driverController.y().onTrue(new InstantCommand(() -> m_swerve.zeroHeading(), m_swerve));
    m_driverController.a().onTrue(new InstantCommand(() -> m_swerve.resetEncoders(), m_swerve));
    m_driverController.x().onTrue(new InstantCommand(() -> m_swerve.setX(), m_swerve));

    // Triggers solenoid on press of button b.
    m_driverController.b().onTrue(new InstantCommand(() -> m_PneumaticsSubsystem.toggle()));
   

    //m_operatorController.x().onTrue(new InstantCommand(() -> m_arm.getExtensionEncoder().setPosition(0), m_arm));

    //TODO: Fix Arm Angle Offsets in Arm.java first before uncommenting
    // m_operatorController.a().onTrue(new MoveToGoal(m_arm, Row.BOTTOM))
    // .or(m_operatorController.b().onTrue(new MoveToGoal(m_arm, Row.MIDDLE)))
    // .or(m_operatorController.y().onTrue(new MoveToGoal(m_arm, Row.TOP)));
    
    //slow mode for right bumper, medium slow for left bumper
    m_driverController.rightBumper().onTrue(new InstantCommand(() -> m_swerve.slowMode(), m_swerve))
    .or(m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_swerve.mediumMode(), m_swerve)))
    .onFalse(new InstantCommand(() -> m_swerve.fastMode(), m_swerve));
    
  }

  private void addCommandDropdown()
  {
    chooser.setDefaultOption("Drive To Distance", new DriveToDistance(Units.feetToMeters(12), m_swerve));
    chooser.addOption("Turn To Angle", new TurnToAngle(90, m_swerve));

    SmartDashboard.putData(chooser);
  }

  public Command getAutonomousCommand() {
     return chooser.getSelected();
  }

  public void disabledInit()
  {}

  public void enabledInit()
  { }

  private static double modifyAxis(double value)
  {
    value = deadBand(value, 0.075);

    value = Math.copySign(value * value, value);

    return value;
  }

  private static double deadBand(double value, double deadband)
  {
    if (Math.abs(value) > deadband)
    {
      if (value > 0.0)
        return (value - deadband)/(1.0 - deadband);
      else
        return (value + deadband)/(1.0 - deadband);
    }
    else
      return 0.0;
  }
}
