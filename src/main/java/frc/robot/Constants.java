// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;


/** Add your docs here. */
public class Constants {

    public static final class Controllers {
        public static final int kDriverControllerPort = 0; 
        public static final int kManipulatorControllerPort = 1; 
        public static final double kDeadband = 0.1; 
    }

    public static final class Arm {
        /**
         * The CAN ID of the extension motor.
         */
        public static final int kExtensionMotorId = 41;
    }

    public static final class Clamps {

    }

    public static final class Drive {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    
        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(26.5);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(26.5);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    
        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.toRadians(72.7);
        public static final double kFrontRightChassisAngularOffset = -Math.toRadians(210.4);
        public static final double kBackLeftChassisAngularOffset = -Math.toRadians(281.2);
        public static final double kBackRightChassisAngularOffset = -Math.toRadians(160.0);
    
        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 1;
        public static final int kRearLeftDrivingCanId = 3;
        public static final int kFrontRightDrivingCanId = 5;
        public static final int kRearRightDrivingCanId = 7;
    
        public static final int kFrontLeftTurningCanId = 2;
        public static final int kRearLeftTurningCanId = 4;
        public static final int kFrontRightTurningCanId = 6;
        public static final int kRearRightTurningCanId = 8;

        public static final int kFrontLeftCanCoderId = 2;
        public static final int kRearLeftCanCoderId = 4;
        public static final int kFrontRightCanCoderId = 6;
        public static final int kRearRightCanCoderId = 8;
    
        public static final boolean kGyroReversed = false;
      }
    
      public static final class SwerveModule {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;
    
        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;
    
        // Calculations required for driving motor conversion factors and feed forward
        public static final double kNeoMotorFreeSpeedRpm = 5676 * 0.9;

        public static final double kDrivingMotorFreeSpeedRps = kNeoMotorFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.1016; // 3 : 0.0762; // 3.75 : 0.09525; //3.8 : 0.09652; // 4 : 0.1016; //Units.inchesToMeters(4.0);
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (50.0 * 17.0 * 45.0) / (kDrivingMotorPinionTeeth * 27.0 * 15.0);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
            / kDrivingMotorReduction;
    
        public static final double kSteeringMotorReduction = 150.0 / 7.0;
    
        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction) / 60.0; // meters per second
    
        public static final double kTurningEncoderPositionFactor = (2 * Math.PI) / kSteeringMotorReduction; // radians
        public static final double kTurningEncoderVelocityFactor = ((2 * Math.PI) / kSteeringMotorReduction) / 60.0; // radians per second
    
        public static final double kTurningEncoderPositionPIDMinInput = -Math.PI; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = Math.PI; // radians
    
        public static final double kDrivingP = 0.1;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0.01;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;
    
        public static final double kTurningP = 2.295;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0.2295;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;
    
        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;
    
        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
      }

      public static final class Intake {

      }

      public static final class PrettyLights {

      }

    public static final class Suction {
        /**
         * The CAN ID of the first suction motor.
         */
        public static final int kMotor1Id = 40;

        /**
         * The CAN ID of the second suction motor.
         */
        public static final int kMotor2Id = 41; //TODO check the actual ID number

        /**
         * The pneumatic hub channel ID of the first suction solenoid.
         */
        public static final int kSolenoid1Id = 0;

        /**
         * The pneumatic hub channel ID of the second suction solenoid.
         */
        public static final int kSolenoid2Id = 1;

        /**
         * The maximum current to send to the suction motor.
         */
        public static final int kCurrentLimit = 30;

        /**
         * The maximum speed that the suction motor runs.
         */
        public static final double kMaxSpeed = 0.33;

        /**
         * The pneumatic hub channel ID of the first pressure sensor.
         */
        public static final int kPressureSensor1Id = 0;

        /**
         * The pneumatic hub channel ID of the first pressure sensor.
         */
        public static final int kPressureSensor2Id = 1;

        /*
         * The target pressure for vacuum state
         */
        public static final double kTargetPressure = 17;

        /*
         * The minimum pressure for vacuum state
         */
        public static final double kMinimumPressure = 20;

      }

      public static final class Vision {
        public static AprilTagFieldLayout kAprilTagFieldLayout = null;
        static {
            try {
                kAprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        public static final String kLeftCameraName = "LEFT";
        public static final Transform3d kLeftRobotToCamera =
            new Transform3d(
                new Translation3d(0.5, 0.0, 0.5),
                new Rotation3d(0, 0,0)); 

        public static final String kRightCameraName = "RIGHT";
        public static final Transform3d kRightRobotToCamera =
            new Transform3d(
                new Translation3d(0.5, 0.0, 0.5),
                new Rotation3d(0, 0,0)); 

      }

}
