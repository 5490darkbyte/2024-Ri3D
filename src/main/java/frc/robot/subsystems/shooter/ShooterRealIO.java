package frc.robot.subsystems.shooter;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;



public class ShooterRealIO implements ShooterIO {

    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private SparkPIDController rightPIDController;

    private RelativeEncoder rightMotorEncoder;

    private double p = 0.0009;
    private double i = 0.00001;
    private double d = 0.0;

    private double setPoint = 0;

    public ShooterRealIO(int IDleftMotor, int IDrightMotor) {
        leftMotor = new CANSparkMax(IDleftMotor, MotorType.kBrushless);
        rightMotor = new CANSparkMax(IDrightMotor, MotorType.kBrushless);

        rightPIDController = rightMotor.getPIDController();
        rightMotorEncoder = rightMotor.getEncoder();

        leftMotor.setInverted(false);
        rightMotor.setInverted(true);

        rightPIDController.setOutputRange(-1, 1);

        leftMotor.setSmartCurrentLimit(80);
        rightMotor.setSmartCurrentLimit(80);

        rightMotorEncoder.setVelocityConversionFactor(SHOOTER_RADIO*2*Math.PI/60);
        rightMotorEncoder.setPositionConversionFactor(SHOOTER_RADIO*2*Math.PI);

        rightPIDController.setP(p);
        rightPIDController.setI(i);
        rightPIDController.setD(d);

        rightPIDController.setFeedbackDevice(rightMotorEncoder);

        leftMotor.setIdleMode(IdleMode.kCoast);
        rightMotor.setIdleMode(IdleMode.kCoast);

        leftMotor.follow(rightMotor);
    }

    public void updateInputs(ShooterIOInputs inputs) {
        inputs.rightCurent = rightMotor.getOutputCurrent();
        inputs.rightVelocity = rightMotorEncoder.getVelocity();
        inputs.rightPosition = rightMotorEncoder.getPosition();
        inputs.rightApppliedPower = rightMotor.getAppliedOutput();
        inputs.setPoint = setPoint;
    }

    public void setPowers(double rightPower) {
        rightMotor.set(rightPower);
    }

    public void setRPS(double rightRPS) {
        rightPIDController.setReference(rightRPS, ControlType.kVelocity);
        this.setPoint = rightRPS;
    }
}
