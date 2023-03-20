 // Copyright (c) 2023 FRC Team 2881 - The Lady Cans
 //
 // Open Source Software; you can modify and/or share it under the terms of BSD
 // license file in the root directory of this project.

 package frc.robot.commands.arm;

 import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Arm;

public class RetractArm extends PIDCommand {
   private Arm m_arm;
   /** Creates a new Lower. */
   public RetractArm(Arm arm) {
     super(new PIDController(1.5, 0, 0),
      () -> arm.getExtensionEncoder().getPosition(),
      0,
      output -> arm.moveExtensionArm(output),
      arm
     );

     getController().setTolerance(Units.inchesToMeters(3));

     m_arm = arm;
     
   }

   // Called once the command ends or is interrupted.
  @Override
   public void end(boolean interrupted) {
     m_arm.setExtendedBoolean(false);
     m_arm.moveExtensionArm(0);
   }

   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
     return getController().atSetpoint();
   }
 }
