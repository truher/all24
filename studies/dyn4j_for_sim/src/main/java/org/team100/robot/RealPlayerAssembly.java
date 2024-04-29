package org.team100.robot;

import java.util.function.BooleanSupplier;

import org.team100.commands.PlayerDefaultDrive;
import org.team100.control.Pilot;
import org.team100.sim.RobotBody;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** This robot is controlled by a human. */
public class RealPlayerAssembly extends RobotAssembly {

    private final Pilot m_control;

    public RealPlayerAssembly(Pilot pilot, RobotBody robotBody, Translation2d speakerPosition) {
        super(robotBody, speakerPosition);
        m_control = pilot;
        m_drive.setDefaultCommand(new PlayerDefaultDrive(m_drive, m_control));
        whileTrue(m_control::intake, m_indexer.run(m_indexer::intake));
        whileTrue(m_control::outtake, m_indexer.run(m_indexer::outtake));
        whileTrue(m_control::shoot, Commands.parallel(
                m_indexer.run(m_indexer::towardsShooter),
                m_shooter.run(m_shooter::shoot)));
        whileTrue(m_control::lob, Commands.parallel(
                m_indexer.run(m_indexer::towardsShooter),
                m_shooter.run(m_shooter::lob)));
        whileTrue(m_control::amp, Commands.parallel(
                m_indexer.run(m_indexer::towardsShooter),
                m_shooter.run(m_shooter::amp)));
        whileTrue(m_control::rotateToShoot,
                m_drive.run(m_drive::rotateToShoot).finallyDo(x -> System.out.println("done " + x)));
    }

    @Override
    public boolean isNPC() {
        return false;
    }

    ///////////////////////////////////////////

    private void whileTrue(BooleanSupplier condition, Command command) {
        new Trigger(condition).whileTrue(command);
    }

}
