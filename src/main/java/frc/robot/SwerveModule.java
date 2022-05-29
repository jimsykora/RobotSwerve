// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import static frc.robot.RobotPreferences.pref_SwerveModule.*;

/** Swerve Module Class */
public class SwerveModule {

    private TalonFX driveMotor;
    private TalonFX turnMotor;

    private Encoder turnEncoder;

    private final PIDController drivePIDController = new PIDController(
            drivePIDControllerP.getValue(),
            drivePIDControllerI.getValue(),
            drivePIDControllerD.getValue());

    private final ProfiledPIDController turnPIDController = new ProfiledPIDController(
            turnPIDControllerP.getValue(),
            turnPIDControllerI.getValue(),
            turnPIDControllerD.getValue(),
            new TrapezoidProfile.Constraints(
                    maxAngularVelocity.getValue(),
                    maxAngularAcceleration.getValue()));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
            driveFeedforwardS.getValue(), driveFeedforwardV.getValue());
    private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(
            turnFeedforwardS.getValue(), turnFeedforwardV.getValue());

    /**
     * Creates Swerve Module with Drive Encoder
     * 
     * @param driveMotorID       CAN device number of driveMotor
     * @param turnMotorID        CAN device number of turnMotor
     * @param turnEncoderSourceA SourceA of turnEncoder
     * @param turnEncoderSourceB SourceB of turnEncoder
     */
    public SwerveModule(int driveMotorID, int turnMotorID, int turnEncoderSourceA, int turnEncoderSourceB) {

        driveMotor = new TalonFX(driveMotorID);
        turnMotor = new TalonFX(turnMotorID);

        turnEncoder = new Encoder(turnEncoderSourceA, turnEncoderSourceB);

        // Set the distance (in this case, angle) per pulse for the turning encoder.
        // This is the the angle through an entire rotation (2 * pi) divided by the
        // encoder resolution.
        turnEncoder.setDistancePerPulse(2 * Math.PI / encoderCountsPerRotation.getValue());

    }

    public void configMotors(TalonFXConfiguration driveConfig, TalonFXConfiguration turnConfig) {
        driveMotor.configFactoryDefault();
        turnMotor.configFactoryDefault();
        driveMotor.setNeutralMode(NeutralMode.Brake);
        turnMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setInverted(false);
        turnMotor.setInverted(false);
        driveMotor.configAllSettings(driveConfig);
        turnMotor.configAllSettings(turnConfig);
    }

    public double getDriveEncoderPosition() {
        return driveMotor.getSelectedSensorPosition()
                * (2 * Math.PI * wheelRadius.getValue() / encoderCountsPerRotation.getValue());
    }

    public double getTurnEncoderPosition() {
        return turnEncoder.get();
    }

    /**
     * Get the rate of the drive encoder
     * 
     * @return rate of drive encoder in meters per second
     */
    private double getDriveEncoderRate() {
        double rate = driveMotor.getSelectedSensorVelocity(); // sensor units per 100ms

        rate *= 10; // sensor units per second

        rate /= encoderCountsPerRotation.getValue(); // rotations per second

        rate *= (2 * Math.PI * wheelRadius.getValue()); // meters per second

        return rate;
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveEncoderRate(), new Rotation2d(getTurnEncoderPosition()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        // eg at 359 degrees instead of traveling 360 degrees to get to 0, just travel 1
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getTurnEncoderPosition()));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = drivePIDController.calculate(getDriveEncoderRate(), state.speedMetersPerSecond);
        final double driveOutputFeedforward = driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = turnPIDController.calculate(getTurnEncoderPosition(), state.angle.getRadians());
        final double turnOutputFeedforward = turnFeedforward.calculate(turnPIDController.getSetpoint().velocity);

        driveMotor.set(ControlMode.Current, driveOutput + driveOutputFeedforward);
        turnMotor.set(ControlMode.Current, turnOutput + turnOutputFeedforward);
    }
}
