 // Copyright (c) 2023 FRC Team 2881 - The Lady Cans
 //
 // Open Source Software; you can modify and/or share it under the terms of BSD
 // license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

   private TalonSRX intakeMotor = new TalonSRX(Constants.Intake.kIntakeMotorId);
   
   public Intake() 
   {
        intakeMotor.configFactoryDefault();
   }

   public void On()
   {
        intakeMotor.set(ControlMode.Velocity, 1);
   }

   public void off()
   {
        intakeMotor.set(ControlMode.Velocity, 0);
   }

   @Override
   public void periodic() {
     // This method will be called once per scheduler run
   }
 }
