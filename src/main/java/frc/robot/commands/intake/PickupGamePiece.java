package frc.robot.commands.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Intake;

public class PickupGamePiece extends PIDCommand
{
    private Intake m_intake;

    public PickupGamePiece(Intake m_intake)
    {
        super(new PIDController(1, 0, 0),
        m_intake::getSelectedSensorPosition, 
        m_intake.getSelectedSensorPosition() + 1,
        output -> m_intake.On(output), 
        m_intake);
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_intake.Off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}