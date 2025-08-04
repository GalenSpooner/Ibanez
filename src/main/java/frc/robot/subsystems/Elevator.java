package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


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

    }

    public void bootUpTalons(){
        rightMotor = new TalonFX(ElevatorConstants.kRightID, ArmConstants.kSuperStructureCANName); 
        leftMotor = new TalonFX(ElevatorConstants.kLeftID, ArmConstants.kSuperStructureCANName);
        rightMotor.setPosition(0);
        leftMotor.setPosition(0);
        rightMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        rightMotor.setControl(request.withPosition(this.state.position));
        leftMotor.setControl(request.withPosition(this.state.position));
    }

    public Command setElevatorState(ElevatorState state){
        return runOnce(()->this.state = state);
    }


}
