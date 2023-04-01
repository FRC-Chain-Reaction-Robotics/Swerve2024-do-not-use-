// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.arm.MoveToGoal;
import frc.robot.commands.arm.MoveToGoal.Row;
import frc.robot.commands.auto.DriveToDistance;
import frc.robot.commands.auto.MoveToChargeStation;
import frc.robot.commands.auto.PickupAndScore;
import frc.robot.commands.auto.TurnToAngle;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.commands.intake.PickupGamePiece;
import frc.robot.commands.intake.ReleaseGamePiece;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;


public class RobotContainer {

  private final SendableChooser<Command> chooser = new SendableChooser<Command>();
  private Swerve m_swerve = new Swerve();
  private Arm m_arm = new Arm();
  private Intake m_intake = new Intake();
  

  private final CommandXboxController m_driverController = new CommandXboxController(Constants.Controllers.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(Constants.Controllers.kOperatorControllerPort);

  
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
   
    //Arm
    m_operatorController.povUp().whileTrue(new RunCommand(() -> m_arm.moveShoulder(0.75), m_arm))
    .or(m_operatorController.povDown().whileTrue(new RunCommand(() -> m_arm.moveShoulder(-0.75), m_arm)))
    .whileFalse(new RunCommand(() -> m_arm.moveShoulder(0), m_arm));
    
    //move extension back and forth
    m_operatorController.leftBumper().whileTrue(new RunCommand(() -> m_arm.moveExtensionArm(0.5), m_arm)) //takes in cube, removes cone
    .or(m_operatorController.rightBumper().whileTrue(new RunCommand(() -> m_arm.moveExtensionArm(-0.5), m_arm))) //removes cube, takes in cone
    .whileFalse(new RunCommand(() -> m_arm.moveExtensionArm(0), m_arm));
    

    //m_operatorController.x().onTrue(new InstantCommand(() -> m_arm.getExtensionEncoder().setPosition(0), m_arm));

    //TODO: Fix Arm Angle Offsets in Arm.java first before uncommenting
    // m_operatorController.a().onTrue(new MoveToGoal(m_arm, Row.BOTTOM))
    // .or(m_operatorController.b().onTrue(new MoveToGoal(m_arm, Row.MIDDLE)))
    // .or(m_operatorController.y().onTrue(new MoveToGoal(m_arm, Row.TOP)));
    
    //slow mode for right bumper, medium slow for left bumper
    m_driverController.rightBumper().onTrue(new InstantCommand(() -> m_swerve.slowMode(), m_swerve))
    .or(m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_swerve.mediumMode(), m_swerve)))
    .onFalse(new InstantCommand(() -> m_swerve.fastMode(), m_swerve));

    //Intake Button
    m_operatorController.rightTrigger().whileTrue(new RunCommand(() -> m_intake.On(1), m_intake))
    .or(m_operatorController.leftTrigger().whileTrue(new RunCommand(() -> m_intake.Reverse(1), m_intake)))
    .onFalse(new RunCommand(() -> m_intake.Off(), m_intake));
    
  }

  private void addCommandDropdown()
  {
    chooser.setDefaultOption("Drive To Distance", new DriveToDistance(Units.feetToMeters(12), m_swerve));
    chooser.addOption("Turn To Angle", new TurnToAngle(90, m_swerve));
    chooser.addOption("Score and Move to Charge Station", new MoveToChargeStation(m_swerve, m_arm, m_intake));
    chooser.addOption("Pickup game piece and score", new PickupAndScore(m_swerve, m_arm, m_intake));
    chooser.addOption("Move to Hybrid", new MoveToGoal(m_arm, Row.HYBRID));
    chooser.addOption("Move to Middle", new MoveToGoal(m_arm, Row.MIDDLE));
    chooser.addOption("Move to High", new MoveToGoal(m_arm, Row.TOP));
    chooser.addOption("Move Arm up", new MoveToGoal(m_arm, Row.NONE));
    chooser.addOption("Pickup Game Piece", new PickupGamePiece(m_intake));
    chooser.addOption("Release Game Piece", new ReleaseGamePiece(m_intake));
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
