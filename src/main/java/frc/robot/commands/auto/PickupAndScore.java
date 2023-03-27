package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.MoveToGoal;
import frc.robot.commands.arm.MoveToGoal.Row;
import frc.robot.commands.intake.PickupGamePiece;
import frc.robot.commands.intake.ReleaseGamePiece;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class PickupAndScore extends SequentialCommandGroup{
    public PickupAndScore(Swerve m_swerve, Arm m_arm, Intake m_intake)
    {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.Swerve.kMaxSpeedMetersPerSecond, Constants.Swerve.kMaxAccel);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
                new Translation2d(0, Units.feetToMeters(5)),
                new Translation2d(Units.feetToMeters(5), Units.feetToMeters(5))
            ), 
            new Pose2d(Units.feetToMeters(5), 0, new Rotation2d(0)), 
            trajectoryConfig);
        
        Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.feetToMeters(5), 0, new Rotation2d(0)), 
            List.of(
                new Translation2d(5, Units.feetToMeters(5)),
                new Translation2d(Units.feetToMeters(0), Units.feetToMeters(5))
            ), 
            new Pose2d(0, 0, new Rotation2d(0)), 
            trajectoryConfig);

        PIDController xPID = new PIDController(1.5, 0, 0);
        PIDController yPID = new PIDController(1.5, 0, 0);
        ProfiledPIDController thetaPID = new ProfiledPIDController(3, 0, 0, 
        new Constraints(Constants.Swerve.kMaxAngularSpeed, Constants.Swerve.kMaxAngularAccel));
        
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory, 
            m_swerve::getPose, 
            Constants.Swerve.kDriveKinematics, 
            xPID,
            yPID, 
            thetaPID, 
            m_swerve::setModuleStates,
            m_swerve);
        
         SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
            trajectory2, 
            m_swerve::getPose, 
            Constants.Swerve.kDriveKinematics, 
            xPID,
            yPID, 
            thetaPID, 
            m_swerve::setModuleStates,
            m_swerve);

        addCommands(
            new MoveToGoal(m_arm, Row.TOP),
            new ReleaseGamePiece(m_intake),
            new TurnToAngle(180, m_swerve),
            new InstantCommand(m_swerve::zeroHeading, m_swerve),
            new InstantCommand(() -> m_swerve.resetPose(trajectory.getInitialPose())),
            swerveControllerCommand,
            new InstantCommand(() -> m_swerve.stopModules()),
            new MoveToGoal(m_arm, Row.GROUND),
            new PickupGamePiece(m_intake),
            new TurnToAngle(180, m_swerve),      
            swerveControllerCommand2,
            new InstantCommand(() -> m_swerve.stopModules()),
            new MoveToGoal(m_arm, Row.TOP),
            new ReleaseGamePiece(m_intake),
            //optional
            new TurnToAngle(180, m_swerve));
    }
}
