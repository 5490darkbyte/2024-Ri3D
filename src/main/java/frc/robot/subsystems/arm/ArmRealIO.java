package frc.robot.subsystems.arm;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */
public class ArmRealIO implements ArmIO {

        boolean isBrake;
        double targetAngle;

        //Motor controllers for arm
        private CANSparkMax armMotor_l;
        private CANSparkMax armMotor_r;
        //WPILib PID for controlling arm
        private PIDController armPIDController;
        //Encoder for arm absolute position and velocity
        private CANcoder cancoder;

        public ArmRealIO(int IDleftMotor, int IDrightMotor) {
            //Initialize arm motor controller objects
            armMotor_l = new CANSparkMax(IDleftMotor, CANSparkLowLevel.MotorType.kBrushless);
            armMotor_r = new CANSparkMax(IDrightMotor, CANSparkLowLevel.MotorType.kBrushless);

            //Arm motors originally set to brake when in idle
            armMotor_l.setIdleMode(CANSparkMax.IdleMode.kBrake);
            armMotor_r.setIdleMode(CANSparkMax.IdleMode.kBrake);
            isBrake = true;

            //Invert the right motor (motors will spin in opposite directions)
            armMotor_r.setInverted(true);

            //PID constant settings
            armPIDController = new PIDController(0.4,0.0,0.00001);

            //Set the left motor to follow the right motors movements, but inverted
            armMotor_l.follow(armMotor_r, true);

            //Initialize arm cancoder
            cancoder = new CANcoder(0); //Need to input proper device ID here
            //Configure cancoder settings
            CANcoderConfiguration CANconfig = new CANcoderConfiguration();
            CANconfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
            cancoder.getConfigurator().apply(CANconfig);
        
            //Set current limit for arm motors
            armMotor_r.setSmartCurrentLimit(80);
            armMotor_l.setSmartCurrentLimit(80);
        }


        public void updateInputs(ArmIOInputs inputs) {
            inputs.isBrake = isBrake;
            inputs.current = armMotor_r.getOutputCurrent()+ armMotor_l.getOutputCurrent();
            inputs.currentAngle = cancoder.getAbsolutePosition().getValueAsDouble(); //or should this be getPosition()?
            inputs.velocity = cancoder.getVelocity().getValueAsDouble();
            inputs.targetAngle = targetAngle;
            inputs.appliedPower = armMotor_r.getAppliedOutput();
            inputs.relativePos_l = armMotor_l.getEncoder().getPosition();
            inputs.relativePos_r = armMotor_r.getEncoder().getPosition();
        }
        
        //Method to change idle mode 
        public void setBreakMode(boolean isBrake) {
            this.isBrake = isBrake;
            if (isBrake) {
                armMotor_l.setIdleMode(CANSparkMax.IdleMode.kBrake);
            } else {
                armMotor_l.setIdleMode(CANSparkMax.IdleMode.kCoast);
            }
        }

        //Set the target angle and move the motor towards the angle
        public void setAngle(double angle) {
            targetAngle = angle;
            armMotor_r.set(armPIDController.calculate(cancoder.getAbsolutePosition().getValueAsDouble(), targetAngle)); //or should this be getPosition()?
        }

}
