package org.team100.lib.commands.drivetrain;

import java.util.function.Supplier;

import org.team100.lib.telemetry.NamedChooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ManualMode implements Supplier<ManualMode.Mode> {
    public enum Mode {
        /** Control module speed and direction directly */
        MODULE_STATE,
        /** Robot-relative dx, dy, and omega */
        ROBOT_RELATIVE_CHASSIS_SPEED,
        /** Field-relative dx, dy, and omega */
        FIELD_RELATIVE_TWIST,
        /** Field-relative dx and dy, rotational feedback control */
        SNAPS
    }
    private final SendableChooser<Mode> m_manualModeChooser;

    public ManualMode() {
        m_manualModeChooser = new NamedChooser<>("Manual Drive Mode");
        for (Mode mode : Mode.values()) {
            m_manualModeChooser.addOption(mode.name(), mode);
        }
        m_manualModeChooser.setDefaultOption(
                Mode.SNAPS.name(),
                Mode.SNAPS);
        SmartDashboard.putData(m_manualModeChooser);
    }

    public Mode getSelected() {
        return m_manualModeChooser.getSelected();
    }

    @Override
    public Mode get() {
        return getSelected();
    }
    
}
