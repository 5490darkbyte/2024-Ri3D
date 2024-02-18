package frc.robot.subsystems.arm;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */
public class ArmRealIO implements ArmIO {

        boolean isBrake;
        double targetAngle;

        private CANSparkMax armMotor_l;
        private CANSparkMax armMotor_r;
        private PIDController armPIDController;
        private CANcoder cancoder;

        public ArmRealIO(int IDleftMotor, int IDrightMotor) {
            armMotor_l = new CANSparkMax(IDleftMotor, MotorType.kBrushless);
            armMotor_r = new CANSparkMax(IDrightMotor, MotorType.kBrushless);

            armMotor_l.setIdleMode(CANSparkMax.IdleMode.kBrake);
            armMotor_r.setIdleMode(CANSparkMax.IdleMode.kBrake);
            isBrake = true;

            armMotor_r.setInverted(true);

            armPIDController = new PIDController(0.4,0.0,0.00001);

            armMotor_l.follow(armMotor_r, true);

            /*
            armAbsoluteEncoder.setInverted(true);
            armAbsoluteEncoder.setPositionConversionFactor(Math.PI*2);
            armAbsoluteEncoder.setVelocityConversionFactor(Math.PI*2/60);
            */

            cancoder = new CANcoder(0); //Need to input proper device ID here
            CANcoderConfiguration CANconfig = new CANcoderConfiguration();
            cancoder.getConfigurator().apply(CANconfig);
        
            
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

        public void setBreakMode(boolean isBrake) {
            this.isBrake = isBrake;
            if (isBrake) {
                armMotor_l.setIdleMode(CANSparkMax.IdleMode.kBrake);
            } else {
                armMotor_l.setIdleMode(CANSparkMax.IdleMode.kCoast);
            }
        }

        public void setAngle(double angle) {
            armMotor_r.set(armPIDController.calculate(cancoder.getAbsolutePosition().getValueAsDouble(), angle)); //or should this be getPosition()?
        }

}
