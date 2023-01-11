// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.swervelib.interfaces.SwerveAbsoluteSensor;
import frc.robot.subsystems.swervelib.interfaces.SwerveMoveBase;
import frc.robot.subsystems.swervelib.interfaces.SwerveRotationMotor;
import frc.robot.subsystems.swervelib.rev.SwerveRotationNEO;

/**
 * This is the class given both motor controller object, 
 * the absolute sensor and contains all functions needed to run one 
 * swerve module. This class handles all access to these objects.
 * 
 * 
 * More on swerve found here:
 * https://docs.google.com/presentation/d/1feVl0L5lgIKSZhKCheWgWhkOydIu-ibgdp7oqA0yqAQ/edit?usp=sharing
 */
public class SwerveModule {
    private SwerveMoveBase driveMotor;
    private SwerveRotationMotor rotationMotor;
    private SwerveAbsoluteSensor absSensor;
    private Rotation2d varOfRelToAbs;//a variable to maybe keep from over burning Sparkmaxes

    //The follwing constants are needed all the time, but are made once here
    static final double PI_OVER_TWO = Math.PI/2;
    static final double TWO_PI = 2*Math.PI;
    static final Rotation2d ROTATE_BY_PI = Rotation2d.fromDegrees(180);

    /**
     * Creates a new SwerveModule object. 
     * 
     * @param driveMotor a SwerveDriveBase object, which
     */
    public SwerveModule(SwerveMoveBase driveMotor, SwerveRotationMotor rotationMotor, SwerveAbsoluteSensor rotationAbsSensor) {
        this.driveMotor = driveMotor;
        this.rotationMotor = rotationMotor;
        this.absSensor = rotationAbsSensor;

    }

    public SwerveModule(SwerveMoveBase driveMotor, SwerveRotationNEO rotationNEO){
        this.driveMotor = driveMotor;
        this.rotationMotor = rotationNEO;
        this.absSensor = rotationNEO;
    }

    //TODO: Write this class for SwerrveRotateTalonSRX Good for thriftyBot swerves
    // public SwerveModule(SwerveMoveBase driveMotor, SwerveRotateTalonSRX rotationTalonSRX){
    //     this.driveMotor = driveMotor;
    //     this.rotationMotor = rotationTalonSRX;
    //     this.absSensor = rotationTalonSRX;
    // }

    /**
     * 
     * @return the drive motor object for this module
     */
    public SwerveMoveBase getDriveMotor(){
        return driveMotor;
    }

    /**
     * 
     * @return the rotation motor object for this module
     */
    public SwerveRotationMotor getRotationMotor(){
        return rotationMotor;
    }

    /**
     * 
     * @return the absolute rotation sensor object for this module
     */   
    public SwerveAbsoluteSensor getAbsSensor(){
        return absSensor;
    }

    /**
     * This method dictates the zero position as the 
     * current position of the module.
     */
    public void zeroAbsPositionSensor(){
        absSensor.zeroAbsPositionSensor();
    }

    /**
     * Returns the current state of the swerve module 
     * as a SwerveModuleState. The speed of the module 
     * should be in m/s and the rotational position is 
     * in the form of a Rotation2d object.
     * 
     * @return a SwerveModuleState
     */
    public SwerveModuleState getModuleState(){
        return new SwerveModuleState(driveMotor.getDriveVelocity(), absSensor.getCurRot2d());
    }

    /**
     * Returns the current position of the swerve module 
     * as a SwerveModulePosition. The position of the module 
     * should be in meters and the rotational position is 
     * in the form of a Rotation2d object.
     * 
     * @return a SwerveModuleState
     */
    //TODO: WAIT FOR 2023 Use this with the new SwerveOdometry
    // public SwerveModulePosition getModulePosition(){
    //     return new SwerveModuleState(driveMotor.getDrivePosition(), absSensor.getCurRot2d());
    // }

    /**
     * This testing class requires the absolute sensor 
     * and the rotation motor encoder to function. It 
     * rotates the module to an angle. The input is 
     * taken in a Rotation2d object, and should have 
     * the angle of 0 pointed toward the front of the 
     * robot. 
     * 
     * @param targetAngle
     */
    public void setModulePosition(Rotation2d targetAngle){
        setModulePosition(absSensor.getCurRot2d(), targetAngle, rotationMotor.getRelEncCount());
    }
    
    /**
     * This class requires the current position of the 
     * module and the rotation motor encoder to function. 
     * These may come from the same source. It rotates 
     * the module to an angle. The inputs are Rotation2d 
     * objects, and should have the angle of 0 pointed 
     * toward the front of the robot, and a current read 
     * of the rotationMotor's encoder.
     * 
     * @param currentAngle
     * @param targetAngle
     * @param currentRelPos
     */
    public void setModulePosition(Rotation2d currentAngle,Rotation2d targetAngle, double currentRelPos){

        // Find the difference between the target and current position
        double posDiff = targetAngle.getRadians() - currentAngle.getRadians();
        double absDiff = Math.abs(posDiff);

        // if the distance is more than a half circle, we are going the wrong way
        if (absDiff > Math.PI) {
            // the distance the other way around the circle
            posDiff = posDiff - (TWO_PI * Math.signum(posDiff));
        }

        // Convert the shortest distance of rotation to relative encoder value(use convertion factor)
        double setpointAngle = posDiff * rotationMotor.radToEncConvFactor();
        // add the encoder distance to the current encoder count
        double outputEncValue = setpointAngle + currentRelPos;

        // Set the setpoint using setReference on the rotation motor
        rotationMotor.setRotationMotorPosition(outputEncValue);
    }

    /**
     * The method to set the module to a position and speed. 
     * This method does the opitimization internally. The 
     * speed should be from -1.0 to 1.0 if isVeloMode is false, 
     * and should be between -MAX_VELOCITY and MAX_VELOCITY if 
     * isVeloMode is true.
     * 
     * @param targetState SwerveModuleState
     * @param isVeloMode true if velocity mode, false if percent output mode
     */
    public SwerveModuleState setModuleState(SwerveModuleState targetState, boolean isVeloMode){

        
        // Instatiate Rotation2d object and fill with call from getCurRot2d()
        Rotation2d currentAngle = absSensor.getCurRot2d();
        double currentRelPos = rotationMotor.getRelEncCount();

        //TODO: poll AbsEnc speed compare with RelEnc speed, if AbsEnc speed is 0ish, while other is NOT begin an if statement
        //TODO: check in nested if, varOfRelToAbs is null, if is, return, use DriverStationWarning that ABS sensor isn't working
        //TODO: use the varOfRelToAbs to get a new currentAngle, Hint use the modulo and RAD_TO_ENC_CONV_FACTOR
        //TODO: in and else statement of the if equate the varOfRelToAbs to the correct offset based on currrentRelPos and currentAngle
        if((absSensor.getSpeedInRad() > -.25 && absSensor.getSpeedInRad() < .25) && !(currentRelPos > -.25 && currentRelPos < .25)) {
            if(varOfRelToAbs == null) {
                DriverStation.reportWarning("Abs Sensor is not working!", false);
                return null;
            }
            currentAngle = new Rotation2d(varOfRelToAbs.getRadians() % Constants.RAD_TO_ENC_CONV_FACTOR);
        } else {
            varOfRelToAbs = currentAngle.minus(new Rotation2d(currentRelPos));
        }
        
        // Optimize targetState with Rotation2d object pulled from above
        targetState = optimize(targetState, currentAngle);
        
        //TODO:add this to a higher layer to optimize it
        if(Math.abs(targetState.speedMetersPerSecond) < 
            (isVeloMode?driveMotor.getMinimumDriveSpeed():driveMotor.getMinimumDutyCycle())) {
            stopAll();
        } else {
            // Set position
            this.setModulePosition(currentAngle, targetState.angle, currentRelPos);

            // Output to drive motor based on velomode or not
            if (isVeloMode) {
                driveMotor.setDriveSpeed(targetState.speedMetersPerSecond);
            } else {
                driveMotor.setDriveDutyCycle(targetState.speedMetersPerSecond);
            }
        }

        //TODO: WAIT FOR 2023 Switch this to use with the new SwerveOdometry
        //for simplicity use this time to drop the odometry back
        return new SwerveModuleState(driveMotor.getDriveVelocity(), currentAngle);
    }

    /**
     * This method is used to stop the module completely. The drive 
     * motor is switched to percent voltage and and output of 0.0 
     * percent volts. The rotation motor's PIDController is set to 
     * DutyCyclevoltage control mode, and output of 0.0% output.
     */
    public void stopAll() {
        driveMotor.stopMotor();
        rotationMotor.stopRotation();
    }

    /**
     * Minimize the change in heading the desired swerve module state would require by potentially
     * reversing the direction the wheel spins. If this is used with the PIDController class's
     * continuous input functionality, the furthest a wheel will ever rotate is 90 degrees.
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     * @return Optimized swerve module state.
     */
    private static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        var delta = desiredState.angle.minus(currentAngle);
        if (Math.abs(delta.getRadians()) > PI_OVER_TWO) {
            return new SwerveModuleState(
                -desiredState.speedMetersPerSecond,
                desiredState.angle.rotateBy(ROTATE_BY_PI));
        } else {
            return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        }
    }

}
