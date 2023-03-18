package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TurnToAngle extends PIDCommand{
    Swerve m_swerve;
    
    public TurnToAngle(double angle, Swerve m_swerve)
    {
        super(
            new PIDController(Constants.SwerveModule.kTurningP, Constants.SwerveModule.kTurningI, Constants.SwerveModule.kTurningD),
            m_swerve::getHeading,
            angle,
            output -> m_swerve.drive(0, 0, output, false),
            m_swerve
        );

        getController().setTolerance(3);

        this.m_swerve = m_swerve;
    }

    @Override
    public boolean isFinished()
    {
        return getController().atSetpoint();    //  This command will terminate once the desired distance has been reached.
    }
    
    @Override
    public void end(boolean interrupted) {
        m_swerve.drive(0, 0, 0, false);
    }
}
