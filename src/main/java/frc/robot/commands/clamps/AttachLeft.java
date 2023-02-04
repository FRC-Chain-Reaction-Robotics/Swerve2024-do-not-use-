// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.clamps;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Clamps;

public class AttachLeft extends CommandBase {
  /** Creates a new AttachLeft. */
  public AttachLeft() {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Clamps);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
