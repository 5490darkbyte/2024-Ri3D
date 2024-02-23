package frc.lib.config;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue; 
import com.ctre.phoenix6.configs.CANcoderConfiguration;

import frc.robot.Constants;

public final class CTREConfigs {
    public CANcoderConfiguration swerveCANcoderConfiguration; 

    public CTREConfigs()
    {
        swerveCANcoderConfiguration =  new CANcoderConfiguration();
        swerveCANcoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCANcoderConfiguration.MagnetSensor.SensorDirection = Constants.SwerveConstants.canCoderInvert;
        /* Note: Check if Phoenix Tuner X can change InitializationStrategy & SensorTimeBase */
    }
}
