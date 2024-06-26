package org.team100.lib.sensors;

import org.team100.lib.async.AsyncFactory;
import org.team100.lib.config.Identity;

public class GyroFactory {
    private final Identity m_identity;
    private final AsyncFactory m_asyncFactory;

    public GyroFactory(Identity identity, AsyncFactory asyncFactory) {
        m_identity = identity;
        m_asyncFactory = asyncFactory;
    }

    public Gyro100 get() {
        switch (m_identity) {
            case COMP_BOT:
            case SWERVE_ONE:
            case SWERVE_TWO:
            case BETA_BOT:
                return new SingleNavXGyro(m_asyncFactory.get());
            default:
                return new NullGyro();
        }
    }
}
