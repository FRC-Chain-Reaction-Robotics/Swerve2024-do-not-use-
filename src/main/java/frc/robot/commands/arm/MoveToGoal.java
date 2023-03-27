package frc.robot.commands.arm;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class MoveToGoal extends CommandBase {
   
    private Arm m_arm;

    
    //The Arm Angle PID
    private final PIDController m_angle_controller;
    private DoubleSupplier m_angle_measurement;
    private DoubleSupplier m_angle_setpoint;
    private DoubleConsumer m_angle_useOutput;
    //The Arm Extension PID 
    private final PIDController m_extend_controller;
    private DoubleSupplier m_extend_measurement;
    private DoubleSupplier m_extend_setpoint;
    private DoubleConsumer m_extend_useOutput;

    public static enum Row{BOTTOM, MIDDLE, TOP, GROUND}
    
    public MoveToGoal(Arm m_arm, Row row, boolean extended) {
       
       //Angle PIDController
        m_angle_controller = new PIDController(3, 0, 0);
        m_angle_measurement = () -> m_arm.getLiftThroughBEncoder().getPosition();
        m_angle_setpoint = () -> 0;
        m_angle_useOutput = output -> m_arm.moveShoulder(output);

       //Extend PIDController
        m_extend_controller = new PIDController(1.5, 0, 0);
        m_extend_measurement = () -> m_arm.getExtensionEncoder().getPosition();
        m_extend_setpoint = () -> 0;
        m_extend_useOutput = output -> m_arm.moveExtensionArm(output);
        switch (row)
        //Tune setpoints manually
        {
            case BOTTOM:
                m_angle_setpoint = () -> Constants.Arm.kBottomAngle;
                m_extend_setpoint = () -> Constants.Arm.kBottomLength;
                break;
            case MIDDLE:
                m_angle_setpoint = () -> Constants.Arm.kMiddleAngle;
                m_extend_setpoint = () -> Constants.Arm.kMiddleLength;
                break;
            case TOP:
                m_angle_setpoint = () -> Constants.Arm.kTopAngle;
                m_extend_setpoint = () -> Constants.Arm.kTopLength;
                break;
            // AUTON Only (Tune for intake)
            case GROUND:
                m_angle_setpoint = () -> 0;
                m_extend_setpoint = () -> m_arm.getExtensionEncoder().getPosition();
                break;
                
            // case PLAYER_STATION:
        }

        m_angle_controller.setTolerance(Units.degreesToRotations(5));
        m_extend_controller.setTolerance(.5);

        if (!extended)
            m_extend_setpoint = () -> m_arm.getExtensionEncoder().getPosition();

        this.m_arm = m_arm;
        //TODO Auto-generated constructor stub
    }

    //Overloaded 3 parameter version without extension
    public MoveToGoal(Arm m_arm, Row row){
        this(m_arm, row, false);
    }

    @Override
    public void execute()
    {
        m_angle_useOutput.accept(m_angle_controller.calculate(m_angle_measurement.getAsDouble(), m_angle_setpoint.getAsDouble()));
        m_extend_useOutput.accept(m_extend_controller.calculate(m_extend_measurement.getAsDouble(), m_extend_setpoint.getAsDouble()));
    }

    @Override
    public void end(boolean interrupted)
    {
        m_arm.moveShoulder(0);
        m_arm.moveExtensionArm(0);
    }

    @Override
    public boolean isFinished()
    {
        return m_angle_controller.atSetpoint() && m_extend_controller.atSetpoint();
    }

}
