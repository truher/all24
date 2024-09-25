package org.team100.lib.swerve;

import java.util.OptionalDouble;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * if any module is stopped and misaligned, stop everything until it's aligned.
 */
public class SteeringOverride implements Glassy {
    private static final double kEpsilon = 1e-3;

    private final SwerveKinodynamics m_limits;
    private final DoubleSupplierLogger2 m_log_s;

    public SteeringOverride(SupplierLogger2 parent, SwerveKinodynamics limits) {
        SupplierLogger2 child = parent.child(this);
        m_limits = limits;
        m_log_s = child.doubleLogger(Level.TRACE, "s");
    }

    /**
     * @param desiredModuleStates
     * @param prevModuleStates
     * @param overrideSteering    outvar, nullable entries
     * @param kDtSec
     */
    public double overrideIfStopped(
            SwerveModuleState100[] desiredModuleStates,
            SwerveModuleState100[] prevModuleStates,
            Rotation2d[] overrideSteering,
            double kDtSec) {
        // in one cycle we can go this many radians. note this assumes infinite
        // acceleration; if the steering axes are slow to accelerate, maybe change this?
        double maxThetaStepRad = kDtSec * m_limits.getMaxSteeringVelocityRad_S();
        double min_s = 1.0;
        for (int i = 0; i < prevModuleStates.length; ++i) {
            if (Math.abs(prevModuleStates[i].speedMetersPerSecond) <= kEpsilon) {
                // If module is stopped, we know that we will need to move straight to the final
                // steering angle, so limit based purely on rotation in place.
                if (Math.abs(desiredModuleStates[i].speedMetersPerSecond) <= kEpsilon) {
                    // Both previous and desired states are stopped.
                    // Just leave module at its current angle.
                    if (prevModuleStates[i].angle.isEmpty()) {
                        // there is no current angle, give up
                        overrideSteering[i] = null;
                        continue;
                    } else {
                        overrideSteering[i] = prevModuleStates[i].angle.get();
                        continue;
                    }
                }

                OptionalDouble rotationRad = rotationRad(desiredModuleStates[i], prevModuleStates[i]);
                if (rotationRad.isEmpty()) {
                    overrideSteering[i] = null;
                    continue;
                }

                double numStepsNeeded = Math.abs(rotationRad.getAsDouble()) / maxThetaStepRad;

                if (numStepsNeeded <= 1.0) {
                    // goal is achievable in one time step.
                    // note this angle is the *unflipped* one, which means that something downstream
                    // may decide to flip it.
                    if (desiredModuleStates[i].angle.isEmpty()) {
                        overrideSteering[i] = null;
                    } else {
                        overrideSteering[i] = desiredModuleStates[i].angle.get();
                    }
                } else {
                    // goal is not achievable, so move as much as possible in one step.
                    // note this moves in the "flipped" direction if required.
                    Rotation2d oneStepOfRotation = Rotation2d.fromRadians(Math.signum(rotationRad.getAsDouble()) * maxThetaStepRad);
                    overrideSteering[i] = prevModuleStates[i].angle.get().rotateBy(oneStepOfRotation);
                    // stop all drive motors until steering is aligned
                    min_s = 0.0;
                }
            }
        }
        final double s = min_s;
        m_log_s.log( () -> s);
        return min_s;
    }


    /** Actual rotation required, taking flipping into account. */
    private OptionalDouble rotationRad(SwerveModuleState100 desiredModuleState, SwerveModuleState100 prevModuleState) {
        if (desiredModuleState.angle.isEmpty() || prevModuleState.angle.isEmpty()) {
            return OptionalDouble.empty();
        }
        Rotation2d necessaryRotation = desiredModuleState.angle.get().minus(prevModuleState.angle.get());
        if (SwerveUtil.shouldFlip(necessaryRotation)) {
            necessaryRotation = necessaryRotation.rotateBy(GeometryUtil.kRotation180);
        }
        return OptionalDouble.of(MathUtil.angleModulus(necessaryRotation.getRadians()));
    }
}
