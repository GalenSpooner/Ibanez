// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Climb.ClimbState;
import frc.robot.subsystems.EndEffector.EndEffectorState;
import frc.robot.subsystems.Superstructure.SuperstructureState;

import com.ctre.phoenix6.hardware.CANdle;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public CANdle leds = new CANdle(LEDConstants.kCandleID);
  public Elevator elevator = new Elevator();
  public Arm arm = new Arm(leds);
  public EndEffector endEffector = new EndEffector(leds);
  public Climb climb = new Climb();
  public Superstructure superstructure = new Superstructure(elevator, arm);
  
  

  
  private final CommandXboxController controller =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandJoystick guitar = 
      new CommandJoystick(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
      controller.a().onTrue(superstructure.requestSuperstructureState(SuperstructureState.L1))
      .onFalse(superstructure.requestSuperstructureState(SuperstructureState.STOW));

      controller.b().onTrue(superstructure.requestSuperstructureState(SuperstructureState.L2))
      .onFalse(superstructure.requestSuperstructureState(SuperstructureState.STOW));

      controller.x().onTrue(superstructure.requestSuperstructureState(SuperstructureState.L3))
      .onFalse(superstructure.requestSuperstructureState(SuperstructureState.STOW));

      controller.y().onTrue(superstructure.requestSuperstructureState(SuperstructureState.L4))
      .onFalse(superstructure.requestSuperstructureState(SuperstructureState.STOW));

      controller.povUp().onTrue(superstructure.requestSuperstructureState(SuperstructureState.BARGE))
      .onFalse(superstructure.requestSuperstructureState(SuperstructureState.STOW));

      controller.povLeft().onTrue(superstructure.requestSuperstructureState(SuperstructureState.ALGLOW)
      .alongWith(endEffector.requestEndEffectorState(EndEffectorState.INTAKE)))
      .onFalse(superstructure.requestSuperstructureState(SuperstructureState.STOW));

      controller.povRight().onTrue(superstructure.requestSuperstructureState(SuperstructureState.ALGHIGH)
      .alongWith(endEffector.requestEndEffectorState(EndEffectorState.INTAKE)))
      .onFalse(superstructure.requestSuperstructureState(SuperstructureState.STOW));

      controller.leftTrigger().onTrue(superstructure.requestSuperstructureState(SuperstructureState.INTAKE)
      .alongWith(endEffector.requestEndEffectorState(EndEffectorState.INTAKE)))
      .onFalse(superstructure.requestSuperstructureState(SuperstructureState.STOW));

      controller.rightTrigger().onTrue(endEffector.requestEndEffectorState(EndEffectorState.OUTTAKE))
      .onFalse(endEffector.requestEndEffectorState(EndEffectorState.HOLD));



      guitar.button(0).onTrue(climb.setClimbState(ClimbState.DEPLOY));

      guitar.button(1).onTrue(climb.setClimbState(ClimbState.STOW));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
