package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* Github testing */
public class Winch extends SubsystemBase {
    
    //TODO: Set the correct deviceID
    CANSparkMax winchCanSparkMax; 


    public Winch(){
     winchCanSparkMax = new CANSparkMax(0, MotorType.kBrushless);
    }

    public void winchExtend() {
     winchCanSparkMax.set(.8);
    }

}
