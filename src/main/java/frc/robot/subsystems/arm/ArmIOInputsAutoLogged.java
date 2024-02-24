package frc.robot.subsystems.arm;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ArmIOInputsAutoLogged extends ArmIO.ArmIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("IsBrake", isBrake);
    table.put("Current", current);
    table.put("CurrentAngle", currentAngle);
    table.put("Velocity", velocity);
    table.put("TargetAngle", targetAngle);
    table.put("AppliedPower", appliedPower);
    table.put("RelativePos_l", relativePos_l);
    table.put("RelativePos_r", relativePos_r);
  }

  @Override
  public void fromLog(LogTable table) {
    isBrake = table.get("IsBrake", isBrake);
    current = table.get("Current", current);
    currentAngle = table.get("CurrentAngle", currentAngle);
    velocity = table.get("Velocity", velocity);
    targetAngle = table.get("TargetAngle", targetAngle);
    appliedPower = table.get("AppliedPower", appliedPower);
    relativePos_l = table.get("RelativePos_l", relativePos_l);
    relativePos_r = table.get("RelativePos_r", relativePos_r);
  }

  public ArmIOInputsAutoLogged clone() {
    ArmIOInputsAutoLogged copy = new ArmIOInputsAutoLogged();
    copy.isBrake = this.isBrake;
    copy.current = this.current;
    copy.currentAngle = this.currentAngle;
    copy.velocity = this.velocity;
    copy.targetAngle = this.targetAngle;
    copy.appliedPower = this.appliedPower;
    copy.relativePos_l = this.relativePos_l;
    copy.relativePos_r = this.relativePos_r;
    return copy;
  }
}
