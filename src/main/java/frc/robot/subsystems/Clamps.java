// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Clamps extends SubsystemBase {
   private final CANSparkMax m_left;
   private final CANSparkMax m_right;
  /** Creates a new Clamps. */
  public Clamps() {
    m_left = new CANSparkMax(0, MotorType.kBrushless);
    m_right = new CANSparkMax(0, MotorType.kBrushless);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void engageLeft(double speed) {
    m_left.set(speed);
  }
}
