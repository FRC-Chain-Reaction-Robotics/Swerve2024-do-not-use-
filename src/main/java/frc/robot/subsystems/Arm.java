package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

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
    
    private RelativeEncoder leftEncoder;
    private RelativeEncoder extensionEncoder;

    private SparkMaxPIDController leftPIDController;
    private SparkMaxPIDController extensionPIDController;

    private AbsoluteEncoder liftThroughBEncoder;


    public Arm() {
        rightMotor = new CANSparkMax(Constants.Arm.kRightMotorId, MotorType.kBrushless);
        leftMotor = new CANSparkMax(Constants.Arm.kLeftMotorId, MotorType.kBrushless);
        rightMotor.follow(leftMotor, true);

        extensionMotor = new CANSparkMax(Constants.Arm.kExtensionMotorId, MotorType.kBrushless);
        
        //rightMotor.follow(leftMotor, false);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        extensionMotor.restoreFactoryDefaults();

        leftMotor.setInverted(true);
        extensionMotor.setInverted(false);
        
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
        extensionMotor.setIdleMode(IdleMode.kBrake);
        
        leftMotor.setSmartCurrentLimit(60, 40); // CHANGE THIS
        rightMotor.setSmartCurrentLimit(60, 40);
        extensionMotor.setSmartCurrentLimit(40, 30);
        
        leftEncoder = leftMotor.getEncoder();
        extensionEncoder = extensionMotor.getEncoder();
        
        leftEncoder.setPositionConversionFactor(Constants.Arm.kArmEncoderPositionFactor);
        // TODO: Calculate this factor "solve for average radius";
        extensionEncoder.setPositionConversionFactor(Constants.Arm.kArmLengthConversionFactor);

        leftEncoder.setVelocityConversionFactor(Constants.Arm.kArmEncoderVelocityFactor);
        extensionEncoder.setVelocityConversionFactor(Constants.Arm.kArmEncoderVelocityFactor);

        leftPIDController = leftMotor.getPIDController();
        extensionPIDController = extensionMotor.getPIDController();

        leftPIDController.setP(2.4);
        leftPIDController.setI(0);
        leftPIDController.setD(0);
        leftPIDController.setFF(0.1);

        extensionPIDController.setP(3);
        extensionPIDController.setI(0);
        extensionPIDController.setD(1.5);
        //extensionPIDController.setFF(0.1);
       
        liftThroughBEncoder = leftMotor.getAbsoluteEncoder(Type.kDutyCycle);  //  TODO: what?? doesn't this need 4 dio ports NOO! It needs an adapter board for the sparkmaxes data port       
        liftThroughBEncoder.setInverted(true);
    

        leftMotor.burnFlash();
        rightMotor.burnFlash();
        extensionMotor.burnFlash();
    }


    public AbsoluteEncoder getLiftThroughBEncoder()
    {
        return liftThroughBEncoder;
    }


    @Override

    public void periodic() {
        SmartDashboard.putNumber("Lift Through Bore Encoder Value", liftThroughBEncoder.getPosition());
        SmartDashboard.putNumber("Extension Motor Position", extensionEncoder.getPosition());
    }

    public void moveShoulder(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);     
    }

    public void moveExtensionArm(double speed) {
        extensionMotor.set(speed);
    }
    

    public RelativeEncoder getExtensionEncoder()
    {
        return extensionEncoder;
    }
    
    



}