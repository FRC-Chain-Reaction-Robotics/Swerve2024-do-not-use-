 // Copyright (c) 2023 FRC Team 2881 - The Lady Cans
 //
 // Open Source Software; you can modify and/or share it under the terms of BSD
 // license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

   private CANSparkMax intakeMotor = new CANSparkMax(Constants.Intake.kIntakeMotorId, null);
   
   public Intake() 
   {
        intakeMotor.burnFlash();
        intakeMotor.restoreFactoryDefaults();
   }

   public void On()
   {
        intakeMotor.set(1);
   }

   public void off()
   {
        intakeMotor.set(0);
   }

   @Override
   public void periodic() {
     // This method will be called once per scheduler run
   }
 }
