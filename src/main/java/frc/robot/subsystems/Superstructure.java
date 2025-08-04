package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Elevator.ElevatorState;

public class Superstructure extends SubsystemBase{
    Elevator elevator;
    Arm arm;
    public Superstructure(Elevator elevator, Arm arm){
        this.elevator = elevator;
        this.arm = arm;
        CommandScheduler.getInstance().schedule(this.elevator.setElevatorState(ElevatorState.STOW));
        CommandScheduler.getInstance().schedule(this.arm.setArmState(ArmState.STOW));
    }


    public enum SuperstructureState{
        STOW(ElevatorState.STOW,ArmState.STOW),
        L1(ElevatorState.STOW,ArmState.L1),
        L2(ElevatorState.L2,ArmState.L2),
        L3(ElevatorState.L3,ArmState.L3),
        L4(ElevatorState.MAX, ArmState.L4),
        ALGLOW(ElevatorState.L2,ArmState.LOWALG),
        ALGHIGH(ElevatorState.L3,ArmState.HIGHALG),
        INTAKE(ElevatorState.STOW,ArmState.INTAKE),
        BARGE(ElevatorState.MAX,ArmState.BARGE);

        public ElevatorState elevatorState;
        public ArmState armState;

        private SuperstructureState(ElevatorState elevatorState, ArmState armState){
            this.elevatorState = elevatorState;
            this.armState = armState;
        }
    }

    public Command requestSuperstructureState(SuperstructureState state){
        return runOnce( ()->{
            this.elevator.setElevatorState(state.elevatorState);
            this.arm.setArmState(state.armState);
        }
        );
    }

    

}
