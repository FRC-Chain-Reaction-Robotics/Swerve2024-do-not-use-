package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class MoveToChargeStation extends SequentialCommandGroup{

    public MoveToChargeStation(Swerve m_swerve)
    {
        addCommands(
            new DriveToDistance(Units.feetToMeters(2), m_swerve),
            new AutoAligner(m_swerve)
        );
    }
    
}
