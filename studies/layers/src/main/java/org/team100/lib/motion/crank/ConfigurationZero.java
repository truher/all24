package org.team100.lib.motion.crank;

/** Always produces zero actuation. */
public class ConfigurationZero implements Actuations {

    @Override
    public Actuation get() {
        return new Actuation(0.0);
    }

    @Override
    public void accept(Indicator indicator) {
        indicator.indicate(this);
    }
}
