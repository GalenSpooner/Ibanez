package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffector extends SubsystemBase  {
    private CANrange sensor;
    private TalonFX roller;
    private CANdle leds;
    public EndEffectorState state = EndEffectorState.HOLD;

    public enum EndEffectorState{
        INTAKE(new SolidColor(0,100).withColor(RGBWColor.fromHex("#FFA500").get()),8),//orange
        OUTTAKE(new SolidColor(0,100).withColor(RGBWColor.fromHex("#FFA500").get()), -5), //orange
        HOLD(new SolidColor(0,100).withColor(RGBWColor.fromHex("#00FF00").get()),1), //green
        NONE(new SolidColor(0,100).withColor(RGBWColor.fromHex("#FF0000").get()),0); //red

        public SolidColor color;
        public double volts;

        private EndEffectorState(SolidColor color, double volts){
            this.color = color;
            this.volts = volts;
        }


    }

    public EndEffector(CANdle leds){
        sensor = new CANrange(EndEffectorConstants.kSensorID, ArmConstants.kSuperStructureCANName);
        this.leds = leds;
        roller = new TalonFX(EndEffectorConstants.kMotorID,ArmConstants.kSuperStructureCANName);
        configMotor(roller);
        
        
    }

    @Override
    public void periodic() {
        if(DriverStation.isEnabled()){
            this.leds.setControl(this.state.color); //if disabled let arm handle LEDS for zeroing purposes
        }
        this.roller.setVoltage(this.state.volts);
        if(hasCoral() && this.state == EndEffectorState.INTAKE){
            requestStateInternal(EndEffectorState.HOLD);
        }
        if(!hasCoral() && this.state == EndEffectorState.HOLD){
            requestStateInternal(EndEffectorState.NONE);
        }

        if(!this.sensor.isConnected()){
            DriverStation.reportError("End Effector CANrange Disconnect", false);
        }
    }

    public boolean hasCoral(){
        return (this.sensor.isConnected()) && (this.sensor.getDistance().getValueAsDouble() < 3);
    }

    private void requestStateInternal(EndEffectorState state){
        this.state = state;
    }

    public Command requestEndEffectorState(EndEffectorState state){
        return runOnce(()->requestStateInternal(state));
    }

    public void configMotor(TalonFX motor){
        var motorConfig = new TalonFXConfiguration();
        var rollerCurrentLimit = motorConfig.CurrentLimits;
        rollerCurrentLimit.SupplyCurrentLimit = 35;
        rollerCurrentLimit.SupplyCurrentLowerLimit = 5;
        rollerCurrentLimit.SupplyCurrentLowerTime = 2;
        rollerCurrentLimit.SupplyCurrentLimitEnable = true;

        motor.getConfigurator().apply(motorConfig);
    }


}
