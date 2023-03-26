package frc.robot.commands.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class MoveToGoal extends PIDCommand{
   
    private Arm m_arm;

    public static enum Row{BOTTOM, MIDDLE, TOP}
    
    public MoveToGoal(Arm m_arm, Row row) {
        super(
            m_arm.getPidController(), 
            () -> m_arm.getLiftThroughBEncoder().getPosition(), 
            0, 
            output -> m_arm.moveShoulder(output), 
            m_arm
        );

        switch (row)
        //Tune setpoints manually
        {
            case BOTTOM:
                m_setpoint = () -> Units.radiansToRotations(Math.asin(Constants.Game.kBottomRowHeight/Constants.Arm.kArmlength))+Constants.Arm.armEncoderOffset;
                break;
            case MIDDLE:
                m_setpoint = () -> Units.radiansToRotations(Math.asin(Constants.Game.kMiddleRowHeight/Constants.Arm.kArmlength))+Constants.Arm.armEncoderOffset;
                break;
            case TOP:
                m_setpoint = () -> Units.radiansToRotations(Math.asin(Constants.Game.kTopRowHeight/Constants.Arm.kArmlength))+Constants.Arm.armEncoderOffset;
                break;
        }

        this.m_arm = m_arm;
        //TODO Auto-generated constructor stub
    }

    @Override
    public void end(boolean interrupted)
    {
        m_arm.moveShoulder(0);
    }

    @Override
    public boolean isFinished()
    {
        return getController().atSetpoint();
    }

}
