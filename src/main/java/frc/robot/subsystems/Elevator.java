package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    public TalonFX rightMotor;
    public TalonFX leftMotor;
    public MotionMagicVoltage request;
    public ElevatorState state = ElevatorState.STOW;

    public enum ElevatorState{
        STOW(0),
        L2(ElevatorConstants.inchesPerRotation * 5),
        L3(ElevatorConstants.inchesPerRotation * 10),
        MAX(ElevatorConstants.inchesPerRotation * 29);

        public double position;

        private ElevatorState(double position){
            this.position = position;
        }
    }



    public Elevator(){
        request = new MotionMagicVoltage(0);
        bootUpTalons();
        configTalons();

    }

    public void bootUpTalons(){
        rightMotor = new TalonFX(ElevatorConstants.kRightID, ArmConstants.kSuperStructureCANName); 
        leftMotor = new TalonFX(ElevatorConstants.kLeftID, ArmConstants.kSuperStructureCANName);
        rightMotor.setPosition(0);
        leftMotor.setPosition(0);
    }

    @Override
    public void periodic() {
        rightMotor.setControl(request.withPosition(this.state.position));
        leftMotor.setControl(request.withPosition(this.state.position));
    }

    public Command setElevatorState(ElevatorState state){
        return runOnce(()->this.state = state);
    }

    public void configTalons(){
        var leftMotorConfig = new TalonFXConfiguration();
        var leftSlotConfigs = leftMotorConfig.Slot0;
        leftSlotConfigs.kS = 0.25; 
        leftSlotConfigs.kV = 0.12;
        leftSlotConfigs.kA = 0.01; 
        leftSlotConfigs.kP = 8;
        leftSlotConfigs.kI = 0; 
        leftSlotConfigs.kD = 0.1; 
        var leftMotionMagicConfigs = leftMotorConfig.MotionMagic;
        leftMotionMagicConfigs.MotionMagicCruiseVelocity = 120; 
        leftMotionMagicConfigs.MotionMagicAcceleration = 600;
        leftMotionMagicConfigs.MotionMagicJerk = 1600;
        var leftCurrentLimitconfigs = leftMotorConfig.CurrentLimits;
        leftCurrentLimitconfigs.SupplyCurrentLimit = 110;
        leftCurrentLimitconfigs.SupplyCurrentLimitEnable = true;
        var leftInvertConfig = leftMotorConfig.MotorOutput;
        leftInvertConfig.Inverted = InvertedValue.Clockwise_Positive;


        var rightMotorConfig = new TalonFXConfiguration();
        var rightSlotConfigs = rightMotorConfig.Slot0;
        rightSlotConfigs.kS = 0.25; 
        rightSlotConfigs.kV = 0.12;
        rightSlotConfigs.kA = 0.01; 
        rightSlotConfigs.kP = 8;
        rightSlotConfigs.kI = 0; 
        rightSlotConfigs.kD = 0.1; 
        var rightMotionMagicConfigs = rightMotorConfig.MotionMagic;
        rightMotionMagicConfigs.MotionMagicCruiseVelocity = 120; 
        rightMotionMagicConfigs.MotionMagicAcceleration = 600;
        rightMotionMagicConfigs.MotionMagicJerk = 1600;
        var rightCurrentLimitconfigs = rightMotorConfig.CurrentLimits;
        rightCurrentLimitconfigs.SupplyCurrentLimit = 110;
        rightCurrentLimitconfigs.SupplyCurrentLimitEnable = true;
        var rightInvertConfig = rightMotorConfig.MotorOutput;
        rightInvertConfig.Inverted = InvertedValue.CounterClockwise_Positive;

        leftMotor.getConfigurator().apply(leftMotorConfig);
        rightMotor.getConfigurator().apply(rightMotorConfig);

    }

}
