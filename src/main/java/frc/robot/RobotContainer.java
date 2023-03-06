// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// import frc.robot.commands.arm.ExtendArm;
// import frc.robot.commands.arm.RetractArm;
// import frc.robot.commands.auto.FollowTrajectory;
import frc.robot.commands.drive.DriveWithJoysticks;
//import frc.robot.commands.drive.ZeroHeading;
// import frc.robot.commands.suction.DisableSuction;
// import frc.robot.commands.suction.EnableSuction;
//import frc.robot.lib.Utils;
import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.Suction;

public class RobotContainer {
  private Swerve m_swerve = new Swerve();
  // private Suction m_suction = new Suction();
  // private Arm m_arm = new Arm();
  private Elevator m_elevator = new Elevator();

  private final CommandXboxController m_driverController = new CommandXboxController(Constants.Controllers.kDriverControllerPort);
  private final CommandXboxController m_manipulatorController = new CommandXboxController(Constants.Controllers.kManipulatorControllerPort);

  private final PathPlannerTrajectory simplePath = PathPlanner.loadPath("SimplePath", 1, 1);
  
  public RobotContainer() {
    //m_elevator.setDefaultCommand(new RunCommand(() -> m_elevator.motorsOff(), m_elevator));
    setupDrive(); 
    configureButtonBindings();
  }

  private void setupDrive() {
    m_swerve.setDefaultCommand(
      new DriveWithJoysticks(
        m_swerve,
        () -> modifyAxis(-m_driverController.getLeftY()),
        () -> modifyAxis(-m_driverController.getLeftX()),
        () -> modifyAxis(-m_driverController.getRightX())
      )
    );
  }

  private void configureButtonBindings() {
    //DRIVER
    m_driverController.y().onTrue(new InstantCommand(() -> m_swerve.zeroHeading(), m_swerve));
    // Hi! Check this out! It's an important button to press...
    // Zeroes out the gyro
    // Yup. You want to press that when the robot is facing North to ensure alignment.
    // Also, when setting up the robot (if you don't have an auto for swerve)
    // you have to set it parallel to driver station lol yea
    // and facing the right direction. Everything field oriented would then be based on that.

    // ORRR if you forget or something, just have a driver align it against a wall or visually, and then have them hit this Zero button. Should reset the gyro.

    // is makes sense, yes?
    // ✅
    // Now. This does affect the BEHAVIOR of the modules.
    // By resetting the gyro, you're also telling the odometry, "Hey, THIS is actually north! Please remember that"
    // Which means your modules are going to behave accordingly. yeah?


    // ahh, lol. kind of, yes!

    // They should ONLY really matter when setting your offsets. the way we were doing it at Waco is that the bevel gears all face one cardinal-ish direction

    // so, like, they all face to the RIGHT of the robot. This means two would face the inside, and the others face the outside
    // YES
    // the paint is good

    // Yup. once the offsets are configured, you can turn the modules however you want in a match, and be totally fine through a power cycle.
    // HOWEVER
    // You'd have to reset offsets IF you meddle with a module (in terms of REMOVING it.) So, say you need to swap or repair a module,
    // then you have to redo its offsets 
    // massively time consuming to set offsets. yup. Technically it's like a 5-10 minute job, but do you really have that in a pit? Nope.
    
    // Ooooh hmmmm hm hm hm
    // is that how this code does it? I don't think any code base I know of allows that, lol
   
   
    //!!
    // resettting module angle offsets is different from GYRO reset.
    // nerp
    // The gyro can only handle "all of the wheels relative to where you want North to be"
    // Module offsets are on a per module basis. Those regard the CANCoder!
    // True!


// we gotta clean up these comments LMAO and leave notes in here?

// Fair. Although, I will say - if you ever come BACK to this, you'll want some *relevant* notes here. <<<<<<<<<

// Okay thrifty bot huehuehue
// amarillo...
// yippee! Ask around if you can acquire some from another team, idk about their shipping
// not really... sort of? lol
// now... I could go on a MASSIVE explanation
// Simple details:
// The CANCoders function very well with Falcons. Falcons can directly read data from a CANCoder on the same network. Yes, CTRE. >:(
// Sooo. Technically, of course you can *use* CANCoders and NEOs. People are doing it all over the place.
// TTB encoder is a more "old school analog encoder". And! It can plug in directly to the Rio in an analog in port
// OR!
// Directly to a smax.
// Meaning, you get the same benefit of keeping any PID business on the motor controller if you plug it in to the smax;;;
  // You do need an adapter board though
// https://www.revrobotics.com/rev-11-1881/

// COOL MONEY POINT!
// Sell the cancoders for a profit LMFAO
// hj^ like literally, people need them BAD
// so you genuinely could sell them for a profit IF that's okay with coaches
// ask Brian (ladycans mentor) before buying those, idk if it's actually worth it to use ttb over cancoder for sure. on paper it should be...
// but ofc... just ttb enc to rio otherwise! that totally works :) (kind of a lot of wires but ehhh lol)



// which is why i mention cleanup
// maybe
    // Reset offset after any swerve module repairs [per module basis]

    //MANIPULATOR
    // new Trigger(m_manipulatorController::getAButton).onTrue(new EnableSuction(m_suction));
    // new Trigger(m_manipulatorController::getBButton).onTrue(new DisableSuction(m_suction));
    // new Trigger(m_manipulatorController::getYButton).whileTrue(new ExtendArm(m_arm, 0));
    // new Trigger(m_manipulatorController::getXButton).whileTrue(new RetractArm(m_arm, 0));

    //Elevator
    m_driverController.povUp().whileTrue(new RunCommand(() -> m_elevator.motorsOn(0.5), m_elevator)).or(m_driverController.povDown().whileTrue(new RunCommand(() -> m_elevator.motorsOn(-0.5), m_elevator)))
    .whileFalse(new RunCommand(m_elevator::motorsOff, m_elevator));
  }

  // public Command getAutonomousCommand() {
  //    return new FollowTrajectory(simplePath, true, m_drive);
  // }

  public void disabledInit()
  {}

  public void enabledInit()
  { }

  private static double modifyAxis(double value)
  {
    value = deadBand(value, 0.075);

    value = value * value * value;

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
