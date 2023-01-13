// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib.rev;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.subsystems.swervelib.interfaces.SwerveAbsoluteSensor;
import frc.robot.subsystems.swervelib.interfaces.SwerveRotationMotor;

/** Add your docs here. */
public class SwerveRotationNEO implements SwerveRotationMotor , SwerveAbsoluteSensor {
    private CANSparkMax rotationMotor;
    public final double RAD_TO_ENC_CONV_FACTOR;

    public SwerveRotationNEO(int rotationMotorID, double radToEncConvFactor){
        this(rotationMotorID, radToEncConvFactor, new NEOConfig());
    }

    public SwerveRotationNEO(int rotationMotorID, double radToEncConvFactor, NEOConfig config){
        //contruct and setup rotation NEO
        rotationMotor = new CANSparkMax(rotationMotorID, MotorType.kBrushless);
        
        RAD_TO_ENC_CONV_FACTOR = radToEncConvFactor;
        // use the integrated sensor with the primary closed loop and timeout is 0.
        boolean areValuesUpdated = false;

        //adjust PIDF if changed
        if(rotationMotor.getPIDController().getP() != config.pidfConfig.P){
            rotationMotor.getPIDController().setP(config.pidfConfig.P);
            areValuesUpdated = true;
        }
        if(rotationMotor.getPIDController().getI() != config.pidfConfig.I){
            rotationMotor.getPIDController().setI(config.pidfConfig.I);
            areValuesUpdated = true;
        }
        if(rotationMotor.getPIDController().getD() != config.pidfConfig.D){
            rotationMotor.getPIDController().setD(config.pidfConfig.D);
            areValuesUpdated = true;
        }
        if(rotationMotor.getPIDController().getFF() != config.pidfConfig.FF){
            rotationMotor.getPIDController().setFF(config.pidfConfig.FF);
            areValuesUpdated = true;
        }

        //confirm desired brake mode
        if((rotationMotor.getIdleMode() == IdleMode.kBrake) != config.isBrakeMode){
            setRotationMotorBrake(config.isBrakeMode);
            areValuesUpdated = true;
        }
        //confirm if motor is inverted
        if(rotationMotor.getInverted() != config.isInverted){
            rotationMotor.setInverted(config.isInverted);// Set motor inverted(set to true)
            areValuesUpdated = true;
        }
        //confirm voltage compensation mode voltage
        if(rotationMotor.getVoltageCompensationNominalVoltage() < config.maxVoltage-.01 
            || rotationMotor.getVoltageCompensationNominalVoltage() > config.maxVoltage + .01){
            rotationMotor.enableVoltageCompensation(config.maxVoltage);
            areValuesUpdated = true;
        }
        
        //TODO: WAIT FOR REV adjust for new sensor firmware to adjust frame periods
        rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 1000);
        rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);
        rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);

        //if values have changed burn NEO flash
        if(areValuesUpdated){
            rotationMotor.burnFlash();
        }
    }

    public void setRotationMotorBrake(boolean brakeOn){
        if(brakeOn){
            rotationMotor.setIdleMode(IdleMode.kBrake);
        }
        else{
            rotationMotor.setIdleMode(IdleMode.kCoast);
        }
    }

    public void setRotationMotorPIDF(double P, double I, double D, double F){
        rotationMotor.getPIDController().setP(P);
        rotationMotor.getPIDController().setI(I);
        rotationMotor.getPIDController().setD(D);
        rotationMotor.getPIDController().setFF(F);
    }

    /**
     * 
     * @return get the relative encoder count in native units
     */
    public double getRelEncCount(){
        return rotationMotor.getEncoder().getPosition();
    }

    @Override
    public double getRelEncSpeed() {
        return rotationMotor.getEncoder().getVelocity() * RAD_TO_ENC_CONV_FACTOR;
    }

    /**
     * 
     */
    public void driveRotateMotor(double dutyCycle){
        this.rotationMotor.set(dutyCycle);
    }

    /**
     * 
     */
    public void setRotationMotorPosition(double output){
        rotationMotor.getPIDController().setReference(output, CANSparkMax.ControlType.kPosition);
    }

    /**
     * A method to stop the rotation motor from rotating
     */
    public void stopRotation(){
        rotationMotor.stopMotor();
    }

    public double radToEncConvFactor(){
        return RAD_TO_ENC_CONV_FACTOR;
    }

    //external sensor 

    /**
     * This method is used to change the offset of 
     * the absolute sensor so we dictate the zero 
     * position as the current position of the module.
     */
    public void zeroAbsPositionSensor(){
        //TODO: WAIT FOR REV write this
    }

    /**
     * The CANCoder reads the absolute rotational position
     * of the module. This method returns that positon in 
     * degrees.
     * note: NOT Inverted module safe (use getPosInRad())
     * 
     * @return the position of the module in degrees, should limit from -180 to 180
     */
    public double getAbsPosInDeg(){
        return 0.0;//TODO: WAIT FOR REV write this
    }

    /**
     * This method gets the current position in radians and 
     * normally the zero is at the front of the robot.
     * 
     * @return the position of the module in radians, should limit from -PI to PI
     */
    public double getPosInRad(){
        return 0.0;//TODO: WAIT FOR REV write this
    }

    /**
     * Returns the current angle of the swerve module, 
     * as read by the absolute rotational sensor, as a 
     * Rotation2d object. This is measured from the 
     * front of the robot, where counter-clockwise is 
     * positive.
     * 
     * @return A Rotation2d object, current position of the module
     */
    public Rotation2d getCurRot2d(){
        return new Rotation2d(getPosInRad());
    }

    public double getSpeedInRad(){
        return 0.0;//TODO: WAIT FOR REV write this
    }

    

}
