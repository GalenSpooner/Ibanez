package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.RGBWColor;

import dev.doglog.DogLog;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    public boolean armZeroed = false;
    public boolean armConfiged = false;
    private CANdle led;
    private TalonFX leftX44;
    private TalonFX rightX44;
    final MotionMagicVoltage request;
    public ArmState state = ArmState.STOW;

    public Arm(CANdle led){
        bootupKrakens();
        this.led = led;
        request = new MotionMagicVoltage(0);
    }

    public enum ArmState{
        STOW(20),
        INTAKE(45),
        L1(25),
        L2(125),
        L3(125),
        L4(125),
        LOWALG(65),
        HIGHALG(65),
        BARGE (125);

        public double angle;

        private ArmState(double angle){
            this.angle = angle;
        }
    }





    public void bootupKrakens(){
        leftX44 = new TalonFX(ArmConstants.kLeftID, ArmConstants.kSuperStructureCANName);
        rightX44 = new TalonFX(ArmConstants.kRightID, ArmConstants.kSuperStructureCANName);
        leftX44.setNeutralMode(NeutralModeValue.Coast);
        rightX44.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Arm Zeroed?", this.armZeroed);
        
        if(DriverStation.isDisabled()){
            led.setControl(new SolidColor(0,100).withColor(new RGBWColor(Color.kPurple)));
            if(RobotController.getUserButton()){
                requestZeroArm();
            }
        }
        rightX44.setControl(request.withPosition(Units.degreesToRotations(this.state.angle)));


        DogLog.log("Arm State", this.state);
        

    }
    public void requestZeroArm(){
        var zero = leftX44.setPosition(0);
        if(zero.isError()){
            handleError(leftX44);
        } 
        armZeroed = true; 

        leftX44.setNeutralMode(NeutralModeValue.Brake);
        rightX44.setNeutralMode(NeutralModeValue.Brake);

        led.setControl(new StrobeAnimation(0,100).withColor(new RGBWColor(Color.kGreen)));
        if(config().isOK()){
            armConfiged = true;
            led.setControl(new SolidColor(0,100).withColor(new RGBWColor(Color.kGreen)));
        }

    }

    public void handleError(TalonFX motor){
        if(!motor.isConnected()){
            DriverStation.reportError("Arm Zero failed: TalonFX disconnected", true);
            return;
        } else{
            DriverStation.reportError("Arm Zero failed: Unknown Error. Recommend turn robot off and on in zero position.", true);
            motor.clearStickyFaults();
            return;
        }
        

    }

    public StatusCode config(){
        var motorConfig = new TalonFXConfiguration();
        //var leftConfig = new TalonFXConfiguration();


        var slot0Configs = motorConfig.Slot0;
        slot0Configs.kS = 0.25; 
        slot0Configs.kV = 0.12;
        slot0Configs.kA = 0.01; 
        slot0Configs.kP = 8;
        slot0Configs.kI = 0; 
        slot0Configs.kD = 0.1; 
        var motionMagicConfigs = motorConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 120; 
        motionMagicConfigs.MotionMagicAcceleration = 600;
        motionMagicConfigs.MotionMagicJerk = 1600;
        var currentLimitconfigs = motorConfig.CurrentLimits;
        currentLimitconfigs.SupplyCurrentLimit = 65;
        currentLimitconfigs.SupplyCurrentLimitEnable = true;
        var feedbackConfigs = motorConfig.Feedback;
        feedbackConfigs.SensorToMechanismRatio = 60;




        
        var status = this.rightX44.getConfigurator().apply(motorConfig);
        this.leftX44.getConfigurator().apply(motorConfig);
        leftX44.setControl(new Follower(ArmConstants.kRightID, true));

        return status;
    }

    public Command setArmState(ArmState state){
        return runOnce(()->this.state = state);
    }
    
}
