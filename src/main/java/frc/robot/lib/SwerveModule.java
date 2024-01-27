// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.lib;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

////indicates intentional annotations made by the Chain Reaction Robotics team
//indicates code
// indicates annotations made by the LadyCans

public class SwerveModule {

  ////CANSparkMax motor controllers, ID them on REV Hardware Client or SparkMax Client
  private final CANSparkMax m_drivingSparkMax; ////controls driving on a swerve module
  private final CANSparkMax m_turningSparkMax; ////controls turning on a swerve module

  private final RelativeEncoder m_drivingEncoder; ////encoder for driving
  private final RelativeEncoder m_turningEncoder; ////encoder for turning
  private final CANCoder m_canCoder; ////another turning encoder for the absolute position, ID on phoenix tuner

  ////PID means Proportional Integral Derivative; uses an equation; accounts for "close enough"
  ////formula is u(t) = kP(e(t)) + kI(integral 0 to t of (e(t)dt)) + kD(d(e(t))/dt), 
  ////tuning integral constant NOT recommended
  private final SparkMaxPIDController m_drivingPIDController; ////PID for driving
  private final SparkMaxPIDController m_turningPIDController; ////PID for turning

  private double m_chassisAngularOffset = 0; ////allows individual wheels to offset correctly

  ////SwerveModuleState objects stores the desired speed in meters per second and angle in radians for your wheels
  ////Used to help set your wheels running at a certain speed in a direction
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
  
  ////allows you to apply settings you made to the cancoder
  private CANCoderConfiguration config = new CANCoderConfiguration();
  
  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   * @param drivingCANId the ID for the drive controller
   * @param turningCANId the ID for the turn controller
   * @param canCoderCANId the ID for the cancoder
   * @param chassisAngularOffset the offset to make the wheels face forward
   */
  public SwerveModule(int drivingCANId, int turningCANId, int canCoderCANId, double chassisAngularOffset) {
    config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;

    ////boots the wheel to its current position rather than zero
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    
    m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_drivingSparkMax.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    // SDS Module is inverted relative to the MAXSwerve
    m_drivingSparkMax.setInverted(true);;
    m_turningSparkMax.setInverted(true);;

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getEncoder();
    m_canCoder = new CANCoder(canCoderCANId);
    m_drivingPIDController = m_drivingSparkMax.getPIDController();
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_drivingEncoder.setPositionConversionFactor(Constants.SwerveModule.kDrivingEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(Constants.SwerveModule.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_turningEncoder.setPositionConversionFactor(Constants.SwerveModule.kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(Constants.SwerveModule.kTurningEncoderVelocityFactor);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(Constants.SwerveModule.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(Constants.SwerveModule.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_drivingPIDController.setP(Constants.SwerveModule.kDrivingP);
    m_drivingPIDController.setI(Constants.SwerveModule.kDrivingI);
    m_drivingPIDController.setD(Constants.SwerveModule.kDrivingD);
    m_drivingPIDController.setFF(Constants.SwerveModule.kDrivingFF);
    m_drivingPIDController.setOutputRange(Constants.SwerveModule.kDrivingMinOutput,
      Constants.SwerveModule.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(Constants.SwerveModule.kTurningP);
    m_turningPIDController.setI(Constants.SwerveModule.kTurningI);
    m_turningPIDController.setD(Constants.SwerveModule.kTurningD);
    m_turningPIDController.setFF(Constants.SwerveModule.kTurningFF);
    m_turningPIDController.setOutputRange(Constants.SwerveModule.kTurningMinOutput,
        Constants.SwerveModule.kTurningMaxOutput);

    m_drivingSparkMax.setIdleMode(Constants.SwerveModule.kDrivingMotorIdleMode);
    m_turningSparkMax.setIdleMode(Constants.SwerveModule.kTurningMotorIdleMode);

    //// Set the current limit to avoid current spikes so you wont damage the motor
    m_drivingSparkMax.setSmartCurrentLimit(Constants.SwerveModule.kDrivingMotorCurrentLimit);
    m_turningSparkMax.setSmartCurrentLimit(Constants.SwerveModule.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_drivingSparkMax.burnFlash();
    m_turningSparkMax.burnFlash();

    // This allows time for the absolute position to be sent by the CANcoder (we know this isn't the best solution, we'll fix it later)
    Timer.delay(1);

    // CANcoder angle is measured in degrees so we need to convert that into radians
    m_chassisAngularOffset = chassisAngularOffset; 
    m_desiredState.angle = Rotation2d.fromDegrees(m_canCoder.getAbsolutePosition());
    m_drivingEncoder.setPosition(0);
    m_turningEncoder.setPosition(Math.toRadians(m_canCoder.getAbsolutePosition()));
    
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    

    // Optimize the reference state to avoid spinning further than 90 degrees (allow shortcuts for the wheels to turn to).
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setP(Constants.SwerveModule.kDrivingP);
    m_drivingPIDController.setD(Constants.SwerveModule.kDrivingD);

    //another way to run your motors. first parameter is the value, second parameter is the control type
    m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);


    m_desiredState = desiredState;

  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  public double getSteeringRelativePosition(){
   return m_turningEncoder.getPosition();
  }

  public double getSteeringAbsolutePosition(){
    return m_canCoder.getAbsolutePosition();
  }

  public double getDrivingRelativePosition(){
    return m_drivingEncoder.getPosition();
  }

  public double getDrivingVelocity(){
    return m_drivingEncoder.getVelocity();
  }
  
  public void evilMode() //Typhoon Reference :)
  {
    m_drivingSparkMax.setSmartCurrentLimit(Constants.SwerveModule.kDrivingMotorCurrentLimit + 20);
    m_turningSparkMax.setSmartCurrentLimit(Constants.SwerveModule.kTurningMotorCurrentLimit + 20);
  }

  public void goodMode()
  {
    m_drivingSparkMax.setSmartCurrentLimit(Constants.SwerveModule.kDrivingMotorCurrentLimit);
    m_turningSparkMax.setSmartCurrentLimit(Constants.SwerveModule.kTurningMotorCurrentLimit);
  }

  public RelativeEncoder getTurningEncoder()
  {
    return  m_turningEncoder;
  }
}