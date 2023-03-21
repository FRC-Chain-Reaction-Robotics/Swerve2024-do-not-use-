package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
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

    private SparkMaxPIDController leftPIDController;
    private SparkMaxPIDController rightPIDController;
    private SparkMaxPIDController extensionPIDController;

    private DutyCycleEncoder throughBEncoder;

    private final PIDController pid = new PIDController(0.1, 0, 0);
    private boolean extended = false;

    public Arm() {
        rightMotor = new CANSparkMax(Constants.Arm.kRightMotorId, MotorType.kBrushless);
        leftMotor = new CANSparkMax(Constants.Arm.kLeftMotorId, MotorType.kBrushless);
        extensionMotor = new CANSparkMax(Constants.Arm.kExtensionMotorId, MotorType.kBrushless);
        
        //rightMotor.follow(leftMotor, false);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        extensionMotor.restoreFactoryDefaults();

        leftMotor.burnFlash();
        rightMotor.burnFlash();
        extensionMotor.burnFlash();

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);
        extensionMotor.setInverted(false);
        
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
        extensionMotor.setIdleMode(IdleMode.kBrake);
        
        leftMotor.setSmartCurrentLimit(20, 20); // CHANGE THIS
        rightMotor.setSmartCurrentLimit(20, 20);
        extensionMotor.setSmartCurrentLimit(20, 20);
        
        rightEncoder = rightMotor.getEncoder();
        leftEncoder = leftMotor.getEncoder();
        extensionEncoder = extensionMotor.getEncoder();
        
        leftEncoder.setPositionConversionFactor(Constants.Arm.kArmEncoderPositionFactor);
        rightEncoder.setPositionConversionFactor(Constants.Arm.kArmEncoderPositionFactor);
        extensionEncoder.setPositionConversionFactor(Constants.Arm.kArmEncoderPositionFactor);

        leftEncoder.setVelocityConversionFactor(Constants.Arm.kArmEncoderVelocityFactor);
        rightEncoder.setVelocityConversionFactor(Constants.Arm.kArmEncoderVelocityFactor);
        extensionEncoder.setVelocityConversionFactor(Constants.Arm.kArmEncoderVelocityFactor);

        leftPIDController = leftMotor.getPIDController();
        rightPIDController = rightMotor.getPIDController();
        extensionPIDController = extensionMotor.getPIDController();

        leftPIDController.setP(2.4);
        leftPIDController.setI(0);
        leftPIDController.setD(0);
        leftPIDController.setFF(0.1);
       
        throughBEncoder = new DutyCycleEncoder(0);
        
    }

    public PIDController getPidController()
    {
        return pid;
    }

    public DutyCycleEncoder getThroughBEncoder()
    {
        return throughBEncoder;
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
/** 
    public void toggle() {
        if (extensionEncoder.getPosition() > -0.1 && extensionEncoder.getPosition() < 0.1) {
            while (extensionEncoder.getPosition() < 6 * Math.PI)
            {
                moveExtensionArm(0.5);
            }
        }
    }
*/
    public void moveExtensionArm(double speed) {
        extensionMotor.set(speed);
    }
    
    public void setExtendedBoolean(boolean extended)
    {
        this.extended = extended;
    }

    public boolean extended()
    {
        return extended;
    }

    public RelativeEncoder getExtensionEncoder()
    {
        return extensionEncoder;
    }
    



}