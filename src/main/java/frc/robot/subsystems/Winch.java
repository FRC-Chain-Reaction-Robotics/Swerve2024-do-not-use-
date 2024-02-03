package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Winch extends SubsystemBase {
    
    //TODO: Set the correct deviceID
    CANSparkMax winchCanSparkMax; 

    public Winch(){
     winchCanSparkMax = new CANSparkMax(0, MotorType.kBrushless);
     winchCanSparkMax.setInverted(false);
     winchCanSparkMax.setSmartCurrentLimit(60);
     winchCanSparkMax.setIdleMode(IdleMode.kBrake);
    }
    
    public void winchExtend(CommandXboxController operator) {
    
     if(deadBand(operator.getRightY(), .3) > 0) {
       winchCanSparkMax.set(1);
     }
     else if(deadBand(operator.getRightY(), .3) < 0) {
        winchCanSparkMax.set(-1);
     }
     else {
        winchCanSparkMax.set(0);
     }
    }

    private static double deadBand(double value, double deadband) {
        if(Math.abs(value) > deadband) {
            if(value > 0.0) {
                return (value - deadband) / (1 -deadband);
            }
            else {
                return (value + deadband) / (1 - deadband);
            }
        }

        return 0.0;
    }

    

}
