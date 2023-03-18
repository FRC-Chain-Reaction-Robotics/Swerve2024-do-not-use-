// // Copyright (c) 2023 FRC Team 2881 - The Lady Cans
// //
// // Open Source Software; you can modify and/or share it under the terms of BSD
// // license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ExtendArm extends CommandBase {
    private Arm m_arm;
    private double m_position;
   /** Creates a new Lower. */
   public ExtendArm(Arm arm, double position) {
     // Use addRequirements() here to declare subsystem dependencies.
     m_arm = arm;
     m_position = position;
   }

   // Called when the command is initially scheduled.
   @Override
   public void initialize() {
     m_arm.moveExtensionArm(0.5);
   }

   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute() {}

   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
     m_arm.setExtendedBoolean(true);
     m_arm.moveExtensionArm(0);
   }

   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
     return !(m_arm.getExtensionEncoder().getPosition() < Math.PI * 6);
   }
 }
