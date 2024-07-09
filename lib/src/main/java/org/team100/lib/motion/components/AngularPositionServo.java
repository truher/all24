package org.team100.lib.motion.components;

import java.util.OptionalDouble;

import org.team100.lib.controller.State100;
import org.team100.lib.dashboard.Glassy;

public interface AngularPositionServo extends Glassy {
    /**
     * It is essential to call this after a period of disuse, to prevent transients.
     * 
     * To prevent oscillation, the previous setpoint is used to compute the profile,
     * but there needs to be an initial setpoint.
     */
    void reset();

    /**
     * The angle measure here *does not* wind up, so 0 and 2pi are the same.
     * 
     * The measurements here are output measurements, e.g. shaft radians, not motor
     * radians.
     * 
     * @param goal              radians
     * @param feedForwardTorque used for gravity compensation
     */
    void setPosition(double goal, double feedForwardTorqueNm);
    
    /**
     * The angle measure here *does not* wind up, so 0 and 2pi are the same.
     * 
     * The measurements here are output measurements, e.g. shaft radians, not motor
     * radians.
     * 
     * @param goal              radians
     * @param feedForwardTorque used for gravity compensation
     */
    void setPositionWithVelocity(double goal, double goalVelocity, double feedForwardTorqueNm);

    /**
     * @return Current position measurement, radians.
     */
    OptionalDouble getPosition();

    OptionalDouble getVelocity();

    boolean atSetpoint();

    boolean atGoal();

    double getGoal();

    void stop();

    void close();

    State100 getSetpoint();

    @Override
    default String getGlassName() {
        return "AngularPositionServo";
    }

}