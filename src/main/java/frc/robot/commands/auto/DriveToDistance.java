package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveToDistance extends PIDCommand{
    Swerve m_swerve;
    
    public DriveToDistance(double distMeters, Swerve m_swerve)
    {
        super(
            new PIDController(Constants.SwerveModule.kDrivingP, Constants.SwerveModule.kDrivingI, Constants.SwerveModule.kDrivingD),
            m_swerve::getDistanceMeters,
            distMeters + m_swerve.getDistanceMeters(),
            output -> m_swerve.drive(output, 0, 0, true),
            m_swerve
        );

        this.m_swerve = m_swerve;
    }

    @Override
    public boolean isFinished()
    {
        return getController().atSetpoint();    //  This command will terminate once the desired distance has been reached.
    }
    
    @Override
    public void end(boolean interrupted) {
        m_swerve.drive(0, 0, 0, true);
    }
}
