package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ArmConstants;

public class Climb extends SubsystemBase {
    public TalonFX rightMotor;
    public TalonFX leftMotor;
    public MotionMagicVoltage request;
    public ClimbState state = ClimbState.STOW;

    public enum ClimbState {
        STOW(0),
        DEPLOY(75);

        public double position;

        private ClimbState(double position) {
            this.position = position;
        }
    }

    public Climb() {
        request = new MotionMagicVoltage(0);
        bootUpTalons();
    }

    public void bootUpTalons() {
        rightMotor = new TalonFX(ClimbConstants.kRightID, ArmConstants.kSuperStructureCANName);
        leftMotor = new TalonFX(ClimbConstants.kLeftID, ArmConstants.kSuperStructureCANName);
        rightMotor.setPosition(0);
        leftMotor.setPosition(0);
        
    }

    @Override
    public void periodic() {
        rightMotor.setControl(request.withPosition(this.state.position));
        leftMotor.setControl(request.withPosition(this.state.position));
    }

    public Command setClimbState(ClimbState state) {
        return runOnce(() -> this.state = state);
    }
}
