// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.SuperStateSubsystem;
import frc.robot.subsystems.SuperStateSubsystem.FireIntent;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.AllianceUtil;

public class TeleopDriveCommand extends Command {
    private final SwerveDrive swerve;
    private final SuperStateSubsystem superState;
    private final DoubleSupplier   vX;
    private final DoubleSupplier   vY;
    private final DoubleSupplier   vZ;
    private FireIntent rawfireState;

    private final double maxSwerveVelocity = Constants.SwerveConfig.MAXIMUM_CHASSIS_VELOCITY;
    private final double maxSwerveAngularVelocity = Constants.SwerveConfig.MAXIMUM_CHASSIS_ANGULAR_VELOCITY;

    private double speedModifier = 1.0;
    private double rotationModifier = 1.0;

    private int timer = 0;

    private boolean isLocked;

    public TeleopDriveCommand(
        SwerveDrive swerve, 
        DoubleSupplier vX, 
        DoubleSupplier vY, 
        DoubleSupplier vZ,
        SuperStateSubsystem superState
    ) {
            this.swerve = swerve;
            this.vX = vX;
            this.vY = vY;
            this.vZ = vZ;
            this.superState = superState;
            addRequirements(swerve);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Field-relative driving: the field origin is always on the blue side, so a red
        // driver's "downfield" is the opposite direction. Flip translation only
        // rotation is not alliance-dependent.
        double allianceSign = AllianceUtil.isRed() ? 1.0 : -1.0;

        double xVelocity   = deadbandAndCube(vX.getAsDouble(), Constants.Controls.Y_DEADBAND) * allianceSign;
        double yVelocity   = deadbandAndCube(vY.getAsDouble(), Constants.Controls.Y_DEADBAND) * allianceSign;
        double angVelocity = -deadbandAndCube(vZ.getAsDouble(), Constants.Controls.ANGLE_JOYSTICK_DEADBAND);


        rawfireState = superState.getFireIntent();
        isLocked = superState.getTestingWheelsLocked(); //Only for Test Mode
        Logger.recordOutput("SuperState/FireIntent", rawfireState.toString());
        Logger.recordOutput("Joystick/xVelocity", xVelocity);
        Logger.recordOutput("Joystick/yVelocity", yVelocity);
        Logger.recordOutput("Joystick/angularVelocity", angVelocity);
        if (rawfireState == FireIntent.FIRE || rawfireState == FireIntent.FIREANDINTAKE) {
            double maxSpeedMeters = maxSwerveVelocity; //Change if you want it to slow down when shooting
            swerve.drive(
                xVelocity * maxSpeedMeters * speedModifier,
                yVelocity * maxSpeedMeters * speedModifier,
                angVelocity * maxSwerveAngularVelocity * rotationModifier
            );
        } else {
            swerve.drive(
                xVelocity * maxSwerveVelocity * speedModifier,
                yVelocity * maxSwerveVelocity * speedModifier, 
                angVelocity * maxSwerveAngularVelocity * rotationModifier 
                );

            /*
            if (xVelocity == 0.0 && yVelocity == 0.0 && angVelocity == 0.0) {
                if (!RobotState.isTest()) {
                    if (timer > 10) {
                        swerve.lockWheels();
                    } else {
                        //Ensures that drive is actually set to zero when it's locked, not something low, due to how the loop runs
                        swerve.drive(
                        0.0,
                        0.0,
                        0.0
                        );
                    }
                } else {swerve.lockWheelsForward();}
                
                
                timer++;
            } else {
                swerve.drive(
                xVelocity * maxSwerveVelocity * speedModifier,
                yVelocity * maxSwerveVelocity * speedModifier,
                angVelocity * maxSwerveAngularVelocity * rotationModifier 
                );
                timer = 0;
            }
                 */
            
        }
             
    }

    private double deadbandAndCube(double raw, double deadband) {
        return Math.pow(MathUtil.applyDeadband(raw, deadband), 3);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}