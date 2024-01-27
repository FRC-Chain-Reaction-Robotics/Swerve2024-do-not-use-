package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PIDCommand;
//import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveToDistance extends PIDCommand{
    Swerve m_swerve;
    
    public DriveToDistance(double distMeters, Swerve m_swerve)
    {
        super(
            new PIDController(1.15, 0, 0),
            m_swerve::getDistanceMeters, ////what is being measured
            distMeters, ////where you want to be
            output -> m_swerve.drive(output, 0, 0, false), ////the output to get to distMeters
            m_swerve
        );
        
        ////sets tolerable error
        getController().setTolerance(Units.inchesToMeters(5)); 

        this.m_swerve = m_swerve;
    }
    
    //when
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
