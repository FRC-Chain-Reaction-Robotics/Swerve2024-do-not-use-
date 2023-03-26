 // Copyright (c) 2023 FRC Team 2881 - The Lady Cans
 //
 // Open Source Software; you can modify and/or share it under the terms of BSD
 // license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

   private WPI_TalonSRX intakeMotor = new WPI_TalonSRX(Constants.Intake.kIntakeMotorId);
   
   public Intake() 
   {
        intakeMotor.configFactoryDefault();
   }

   public void On(double speed)
   {
        intakeMotor.setInverted(false);
        intakeMotor.set(speed);
   }

   public void Off()
   {
        intakeMotor.set(0);
   }

   public void Reverse(double speed)
   {
        intakeMotor.setInverted(true);
        intakeMotor.set(speed);
   }

   @Override
   public void periodic() {
     // This method will be called once per scheduler run
     SmartDashboard.putNumber("Intake motor value", intakeMotor.getSelectedSensorPosition());
   }

   public double getSelectedSensorPosition()
   {
     return intakeMotor.getSelectedSensorPosition();
   }
 }
