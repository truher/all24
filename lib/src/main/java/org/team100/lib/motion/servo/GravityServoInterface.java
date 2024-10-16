package org.team100.lib.motion.servo;

import java.util.OptionalDouble;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.profile.Profile100;
import org.team100.lib.state.State100;

// this is for refactoring the gravity servo
public interface GravityServoInterface extends Glassy {

    /** Zeros controller errors, sets setpoint to current position. */
    void reset();

    OptionalDouble getPositionRad();

    /** set position with zero velocity */
    default void setPosition(double goalRad) {
        setState(new State100(goalRad, 0));
    }

    /**
     * Resets the encoder position, is very slow, so 
     * only do this on startup
     * 
     * @param positionRad The position of the encoder
     */
    void setEncoderPosition(double positionRad);

    /** allow moving end-state */
    void setState(State100 goal);

    void stop();

    void setProfile(Profile100 profile);

    void setTorqueLimit(double torqueNm);

    void periodic();
}