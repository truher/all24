package org.team100.lib.motion.drivetrain.module;

import org.team100.lib.encoder.SimulatedEncoder;
import org.team100.lib.motion.components.OutboardVelocityServo;
import org.team100.lib.motion.components.PositionServo;
import org.team100.lib.motion.components.VelocityServo;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motor.SimulatedMotor;
import org.team100.lib.profile.Profile100;
import org.team100.lib.units.Angle;
import org.team100.lib.units.Distance;

import edu.wpi.first.math.controller.PIDController;

public class SimulatedSwerveModule100 extends SwerveModule100 {

    public static SimulatedSwerveModule100 get(String name,
            SwerveKinodynamics kinodynamics) {
        VelocityServo<Distance> driveServo = simulatedDriveServo(name);
        PositionServo<Angle> turningServo = simulatedTurningServo(name, kinodynamics);
        return new SimulatedSwerveModule100(name, driveServo, turningServo);
    }

    private static VelocityServo<Distance> simulatedDriveServo(String name) {
        SimulatedMotor<Distance> driveMotor = new SimulatedMotor<>(drive(name));
        SimulatedEncoder<Distance> driveEncoder = new SimulatedEncoder<>(
                drive(name),
                driveMotor,
                1,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);
        return new OutboardVelocityServo<>(
                drive(name),
                driveMotor,
                driveEncoder);
    }

    private static PositionServo<Angle> simulatedTurningServo(String name,
            SwerveKinodynamics kinodynamics) {
        SimulatedMotor<Angle> turningMotor = new SimulatedMotor<>(turning(name));
        SimulatedEncoder<Angle> turningEncoder = new SimulatedEncoder<>(
                turning(name),
                turningMotor,
                1,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);
        VelocityServo<Angle> turningVelocityServo = new OutboardVelocityServo<>(
                turning(name),
                turningMotor,
                turningEncoder);
        PIDController turningPositionController = new PIDController(
                20, // kP
                0, // kI
                0, // kD
                dt);
        turningPositionController.enableContinuousInput(-Math.PI, Math.PI);
        // note low tolerance
        turningPositionController.setTolerance(0.05, 0.05);
        Profile100 profile = kinodynamics.getSteeringProfile();
        PositionServo<Angle> turningServo = new PositionServo<>(
                turning(name),
                turningVelocityServo,
                turningEncoder,
                kinodynamics.getMaxSteeringVelocityRad_S(),
                turningPositionController,
                profile,
                Angle.instance);
        turningServo.reset();
        return turningServo;
    }

    private SimulatedSwerveModule100(
            String name,
            VelocityServo<Distance> driveServo,
            PositionServo<Angle> turningServo) {
        super(name, driveServo, turningServo);
        //
    }

}
