package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private CANSparkMax rightMotor;
    private CANSparkMax leftMotor;
    private CANSparkMax extensionMotor;
    
    private RelativeEncoder rightEncoder;
    private RelativeEncoder leftEncoder;
    private RelativeEncoder extensionEncoder;
    

    private DutyCycleEncoder throughBEncoder;

    private final PIDController pid = new PIDController(0.1, 0, 0);

    public Arm() {
        rightMotor = new CANSparkMax(Constants.Arm.kRightMotorId, MotorType.kBrushless);
        leftMotor = new CANSparkMax(Constants.Arm.kLeftMotorId, MotorType.kBrushless);
        extensionMotor = new CANSparkMax(Constants.Arm.kExtensionMotorId, MotorType.kBrushless);
        
        rightMotor.follow(leftMotor, false);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        extensionMotor.restoreFactoryDefaults();

        leftMotor.burnFlash();
        rightMotor.burnFlash();
        extensionMotor.burnFlash();

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);
        extensionMotor.setInverted(false);
        
        
        rightEncoder = rightMotor.getEncoder();
        leftEncoder = leftMotor.getEncoder();
        extensionEncoder = extensionMotor.getEncoder();

        rightEncoder.setPositionConversionFactor(Constants.Arm.kArmEncoderPositionFactor);

        throughBEncoder = new DutyCycleEncoder(0);
    }

    public boolean atSetpoint() {
        return pid.atSetpoint();
    }

    @Override

    public void periodic() {
        SmartDashboard.putNumber("Through Bore Encoder Value", throughBEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Extension Motor Position", extensionEncoder.getPosition());
    }

    public void move(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);     
    }

    public void toggle() {
        if (extensionEncoder.getPosition() > -0.1 && extensionEncoder.getPosition() < 0.1) {
            
        }
    }

    public void moveExtensionArm(double speed) {
        extensionMotor.set(speed);
    }
    

    



}