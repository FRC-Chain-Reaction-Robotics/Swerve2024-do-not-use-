package frc.robot.commands.auto;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class AutoAligner extends CommandBase {
    public final AHRS navx;
    public final Swerve m_swerve;
    public PIDController pid = new PIDController(0.01, 0, 0);
    public ChassisSpeeds cs = new ChassisSpeeds(0, 0, 0);

    public AutoAligner(Swerve m_swerve) {
        this.m_swerve = m_swerve;
        this.navx = m_swerve.getGyro();
        pid.setTolerance(2.5, 0.1);
    }

    @Override
    public void execute() {
        cs.vxMetersPerSecond = pid.calculate(navx.getPitch(), 0);
        m_swerve.drive(cs.vxMetersPerSecond, cs.vyMetersPerSecond, cs.omegaRadiansPerSecond, true);
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

}