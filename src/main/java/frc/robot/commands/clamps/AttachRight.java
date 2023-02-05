// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.clamps;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Clamps;

public class AttachRight extends CommandBase {
  private Clamps m_clamps;
  /** Creates a new AttachRight. */
  public AttachRight(Clamps clamps) {
    m_clamps = clamps;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_clamps);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_clamps.engageRight(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_clamps.engageRight(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
