package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.MoveToGoal;
import frc.robot.commands.arm.MoveToGoal.Row;
import frc.robot.commands.intake.ReleaseGamePiece;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class MoveToChargeStation extends SequentialCommandGroup{

    public MoveToChargeStation(Swerve m_swerve, Arm m_arm, Intake m_intake)
    {
        addCommands(
            new MoveToGoal(m_arm, Row.TOP),
            new ReleaseGamePiece(m_intake),
            new TurnToAngle(180, m_swerve),
            new DriveToDistance(Units.feetToMeters(7), m_swerve),
            new DriveToDistance(Units.feetToMeters(-3), m_swerve),
            new AutoAligner(m_swerve)
        );
    }
    
}
