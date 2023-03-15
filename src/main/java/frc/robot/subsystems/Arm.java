package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase
{
    private CANSparkMax rightMotor;
    private CANSparkMax leftMotor;
    
    private RelativeEncoder rightEncoder;
    private RelativeEncoder leftEncoder;

    private DutyCycleEncoder throughBEncoder;

    public Arm()
    {
        rightMotor = new CANSparkMax(Constants.Arm.kRightMotorId, MotorType.kBrushless);
        leftMotor = new CANSparkMax(Constants.Arm.kLeftMotorId, MotorType.kBrushless);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.burnFlash();
        rightMotor.burnFlash();

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);

        
        rightEncoder = rightMotor.getEncoder();
        leftEncoder = leftMotor.getEncoder();

        //throughBEncoder = new DutyCycleEncoder();
    }
}