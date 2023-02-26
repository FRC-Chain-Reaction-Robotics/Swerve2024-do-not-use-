// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveWithJoysticks extends CommandBase {
  private final Swerve m_swerve;
  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final DoubleSupplier m_rotationSupplier;

// Supplier is an interface that has a method that returns <T>
// Double Supplier extends Supplier and defines that method to return <Double>
// Functional Interfaces are FUNcKY 😎😎😎
// Double Supplier has the getAsDouble() method

  public DriveWithJoysticks(Swerve swerve,
  DoubleSupplier translationXSupplier,
  DoubleSupplier translationYSupplier,
  DoubleSupplier rotationSupplier) {
    m_swerve = swerve;
    m_translationXSupplier = translationXSupplier;
    m_translationYSupplier = translationYSupplier;
    m_rotationSupplier = rotationSupplier;
    
    addRequirements(m_swerve);
  }

  @Override
  public void execute() {
   m_swerve.drive(m_translationXSupplier.getAsDouble() * Constants.Swerve.kMaxSpeedMetersPerSecond, 
   m_translationYSupplier.getAsDouble() * Constants.Swerve.kMaxSpeedMetersPerSecond, 
   m_rotationSupplier.getAsDouble() * Constants.Swerve.kMaxAngularSpeed, 
   false);
  }

  @Override
  public void end(boolean interrupted) {
   m_swerve.drive(0, 0, 0, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
