package org.team100.lib.telemetry;

import org.team100.lib.logging.DummySender;
import org.team100.lib.logging.NTPrimitiveLogger2;
import org.team100.lib.logging.PrimitiveLogger2;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.UdpPrimitiveLogger2;
import org.team100.lib.logging.UdpSender;
import org.team100.lib.util.Util;

import com.ctre.phoenix6.SignalLogger;

/**
 * Simple logging wrapper.
 * 
 * Use keys of the form "/foo/bar"; the slashes separate levels in the tree.
 * 
 * Don't use a slash for any other reason, e.g. for "meters per second" don't
 * say "m/s" say "m_s"
 * 
 * Logged items are "retained" which means they persist even after the logging
 * stops; this means you should see the latest values after disabling the robot.
 */
public class Telemetry {
    private static final boolean USE_UDP_LOGGING = false;
    private static final boolean USE_REAL_UDP = false;

    public enum Level {
        /**
         * Minimal, curated set of measurements for competition matches. Comp mode
         * overrides root logger enablement.
         */
        COMP(1),
        /**
         * Keep this set small enough to avoid overrunning. This should only contain
         * things you're actively working on.
         */
        DEBUG(2),
        /**
         * Slow, shows absolutely everything. Overruns. Don't expect normal robot
         * behavior in this mode.
         */
        TRACE(3);

        private int priority;

        private Level(int priority) {
            this.priority = priority;
        }

        public boolean admit(Level other) {
            return this.priority >= other.priority;
        }
    }

    private static final Telemetry instance = new Telemetry();

    private UdpPrimitiveLogger2 udpLogger;
    private PrimitiveLogger2 ntLogger;
    private Level m_level;

    /**
     * root is "field", with a ".type"->"Field2d" entry as required by glass.
     */
    public final SupplierLogger2 fieldLogger;
    public final SupplierLogger2 rootLogger;

    /**
     * Uses the default network table instance.
     * Clients should use the static instance, not the constructor.
     */
    private Telemetry() {
        // this will be overridden by {@link TelemetryLevelPoller}
        m_level = Level.TRACE;

        if (USE_UDP_LOGGING) {
            Util.warn("=======================================");
            Util.warn("Using UDP network logging!");
            Util.warn("You must have a log listener connected!");
            Util.warn("=======================================");
            if (USE_REAL_UDP) {
                udpLogger = new UdpPrimitiveLogger2(
                        UdpSender.data(),
                        UdpSender.meta());
            } else {
                udpLogger = new UdpPrimitiveLogger2(
                        new DummySender(),
                        new DummySender());
            }
        } else {
            ntLogger = new NTPrimitiveLogger2();
        }

        if (USE_UDP_LOGGING) {
            fieldLogger = new SupplierLogger2(() -> m_level, "field", udpLogger);
            rootLogger = new SupplierLogger2(() -> m_level, "log", udpLogger);
        } else {
            fieldLogger = new SupplierLogger2(() -> m_level, "field", ntLogger);
            rootLogger = new SupplierLogger2(() -> m_level, "log", ntLogger);
        }
        fieldLogger.stringLogger(Level.COMP, ".type").log(() -> "Field2d");

        // turn off the CTRE log we never use
        SignalLogger.enableAutoLogging(false);
    }

    public int keyCount() {
        if (udpLogger != null)
            return udpLogger.keyCount();
        if (ntLogger != null)
            return ntLogger.keyCount();
        return 0;
    }

    public void periodic() {
        if (udpLogger != null)
            udpLogger.periodic();
    }

    public void setLevel(Level level) {
        m_level = level;
    }

    public static Telemetry instance() {
        return instance;
    }
}