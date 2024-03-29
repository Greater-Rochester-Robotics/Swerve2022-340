// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib.rev;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import frc.robot.subsystems.swervelib.interfaces.SwerveMoveBase;

/** Add your docs here. */
public class SwerveMoveNEO extends SwerveMoveBase{
    private CANSparkMax driveMotor;
    private final double MINIMUM_DRIVE_SPEED = .2;
    private final double MINIMUM_DRIVE_DUTY_CYCLE = 0.2;
    private boolean areValuesUpdated = false;

    public SwerveMoveNEO(int driveMotorID){
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);

        // above uses configSelectedFeedbackCoefficient(), to scale the
        // driveMotor to real distance, DRIVE_ENC_TO_METERS_FACTOR
        driveMotor.setInverted(false);// Set motor inverted(set to false)
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 1000);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20); 
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        driveMotor.burnFlash();
    }

    public SwerveMoveNEO(int driveMotorID, NEOConfig config){
        areValuesUpdated = false;
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);

        
        //adjust PIDF if changed
        if(driveMotor.getPIDController().getP() != config.pidfConfig.P){
            driveMotor.getPIDController().setP(config.pidfConfig.P);
            areValuesUpdated = true;
        }
        if(driveMotor.getPIDController().getI() != config.pidfConfig.I){
            driveMotor.getPIDController().setI(config.pidfConfig.I);
            areValuesUpdated = true;
        }
        if(driveMotor.getPIDController().getD() != config.pidfConfig.D){
            driveMotor.getPIDController().setD(config.pidfConfig.D);
            areValuesUpdated = true;
        }
        if(driveMotor.getPIDController().getFF() != config.pidfConfig.FF){
            driveMotor.getPIDController().setFF(config.pidfConfig.FF);
            areValuesUpdated = true;
        }

        //confirm desired brake mode
        if((driveMotor.getIdleMode() == IdleMode.kBrake) != config.isBrakeMode){
            setDriveMotorBrake(config.isBrakeMode);
            areValuesUpdated = true;
        }
        //confirm if motor is inverted
        if(driveMotor.getInverted() != config.isInverted){
            driveMotor.setInverted(config.isInverted);// Set motor inverted(set to true)
            areValuesUpdated = true;
        }
        //confirm voltage compensation mode voltage
        if(driveMotor.getVoltageCompensationNominalVoltage() < config.maxVoltage-.01 
            || driveMotor.getVoltageCompensationNominalVoltage() > config.maxVoltage + .01){
            driveMotor.enableVoltageCompensation(config.maxVoltage);
            areValuesUpdated = true;
        }
        
        //TODO: WAIT FOR REV adjust for new sensor firmware to adjust frame periods
        driveMotor.setInverted(false);// Set motor inverted(set to false)
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 1000);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20); 
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
       
        //if values have changed burn NEO flash
        if(areValuesUpdated){
            driveMotor.burnFlash();
        }
    }


    public void setDriveDutyCycle(double dutyCycle){
        driveMotor.set(dutyCycle);
    }

    public void setDriveSpeed(double speed){
        driveMotor.set(speed);//TODO: this is not right, pid contoller
    }

    public double getDriveDistance(){
        return driveMotor.getEncoder().getPosition();//TODO: this is not scalled to meters
    }

    public void setDriveMotorBrake(boolean brakeOn){
        if(brakeOn){
            driveMotor.setIdleMode(IdleMode.kBrake);
        }
        else{
            driveMotor.setIdleMode(IdleMode.kCoast);
        }
    }

    public double getDriveVelocity(){
        return driveMotor.getEncoder().getVelocity();//TODO: this is not scalled to meters per second
    }

    public void resetDriveMotorEncoder(){
        driveMotor.getEncoder().setPosition(0.0);
    }
    
    public void setDriveMotorPIDF(double P, double I, double D, double F){
        if(driveMotor.getPIDController().getP() != P){
            driveMotor.getPIDController().setP(P);
            areValuesUpdated = true;
        }
        if(driveMotor.getPIDController().getI() != I){
            driveMotor.getPIDController().setI(I);
            areValuesUpdated = true;
        }
        if(driveMotor.getPIDController().getD() != D){
            driveMotor.getPIDController().setD(D);
            areValuesUpdated = true;
        }
        if(driveMotor.getPIDController().getFF() != F){
            driveMotor.getPIDController().setFF(F);
            areValuesUpdated = true;
        }
    }

    public void enableVoltageCompensation(double maximumVoltage){
        driveMotor.enableVoltageCompensation(maximumVoltage);

    }

    @Override
    public void stopMotor() {
        driveMotor.stopMotor();       
    }

    @Override
    public double getMinimumDriveSpeed() {
        return MINIMUM_DRIVE_SPEED;
    }

    @Override
    public double getMinimumDutyCycle() {
        return MINIMUM_DRIVE_DUTY_CYCLE;
    }
}
