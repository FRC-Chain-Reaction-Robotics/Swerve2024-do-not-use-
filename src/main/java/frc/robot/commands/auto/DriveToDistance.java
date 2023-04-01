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
            new PIDController(1.15, 0, 0),
            m_swerve::getDistanceMeters,
            distMeters,
            output -> m_swerve.drive(output, 0, 0, false),
            m_swerve
        );

        this.m_swerve = m_swerve;
    }
    
    @Override
    public void initialize()
    {
        m_swerve.resetEncoders();
    }

    @Override
    public boolean isFinished()
    {
        return getController().atSetpoint();    //  This command will terminate once the desired distance has been reached.
    }
    
    @Override
    public void end(boolean interrupted) {
        m_swerve.drive(0, 0, 0, false);
        m_swerve.resetEncoders();
    }
}
