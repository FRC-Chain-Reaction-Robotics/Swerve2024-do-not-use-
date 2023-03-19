package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class MoveToGoal extends PIDCommand{
   
    private Arm m_arm;

    public static enum Row{BOTTOM, MIDDLE, TOP}
    
    public MoveToGoal(Arm m_arm, Row row) {
        super(
            m_arm.getPidController(), 
            () -> m_arm.getThroughBEncoder().getAbsolutePosition(), 
            0, 
            output -> m_arm.move(output), 
            m_arm
        );

        switch (row)
        {
            case BOTTOM:
                m_setpoint = () -> Math.asin(Constants.Game.kBottomRowHeight/Constants.Arm.kArmlength);
                break;
            case MIDDLE:
                m_setpoint = () -> Math.asin(Constants.Game.kMiddleRowHeight/Constants.Arm.kArmlength);
                break;
            case TOP:
                m_setpoint = () -> Math.asin(Constants.Game.kTopRowHeight/Constants.Arm.kArmlength);
                break;
        }

        this.m_arm = m_arm;
        //TODO Auto-generated constructor stub
    }

    @Override
    public void end(boolean interrupted)
    {
        m_arm.move(0);
    }

    @Override
    public boolean isFinished()
    {
        return getController().atSetpoint();
    }

}
