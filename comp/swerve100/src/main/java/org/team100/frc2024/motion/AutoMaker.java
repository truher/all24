package org.team100.frc2024.motion;

import java.lang.reflect.Field;
import java.util.List;

import org.team100.frc2024.motion.drivetrain.DriveToWithAutoStart;
import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.lib.commands.drivetrain.DriveToWaypoint100;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.timing.CentripetalAccelerationConstraint;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoMaker {
    public SwerveDriveSubsystem m_swerve;
    public TrajectoryPlanner m_planner;
    public DriveMotionController m_controller;
    public List<TimingConstraint> m_constraints;
    private final double kMaxVelM_S = 4;
    private final double kMaxAccelM_S_S = 5;
    private final double kShooterScale;
    private final Alliance m_alliance;
    private final double kIntakeOffset = .381;

    public enum FieldPoint {
        NOTE1, NOTE2, NOTE3, NOTE4, NOTE5, NOTE6, NOTE7, NOTE8, CLOSEWINGSHOT, FARWINGSHOT, STAGESHOT,
        CENTRALSTAGEOPENING,
        STAGEADJACENT, FARSTAGEOPENING, DROPSHOT
    }

    private Translation2d getTranslation(FieldPoint point) {
        switch (point) {
            case NOTE1:
                return forAlliance(new Translation2d(2.8956, 7.0061), m_alliance);
            case NOTE2:
                return forAlliance(new Translation2d(2.8956, 5.5583), m_alliance);
            case NOTE3:
                return forAlliance(new Translation2d(2.8956, 4.1105), m_alliance);
            case NOTE4:
                return forAlliance(new Translation2d(8.271, 0.7577), m_alliance);
            case NOTE5:
                return forAlliance(new Translation2d(8.271, 2.4341), m_alliance);
            case NOTE6:
                return forAlliance(new Translation2d(8.271, 4.1105), m_alliance);
            case NOTE7:
                return forAlliance(new Translation2d(8.271, 5.7869), m_alliance);
            case NOTE8:
                return forAlliance(new Translation2d(8.271, 7.4633), m_alliance);
            case CLOSEWINGSHOT:
                return forAlliance(new Translation2d(5.87248, 6.4), m_alliance);
            case FARWINGSHOT:
                return forAlliance(new Translation2d(4, 1.5), m_alliance);
            case STAGESHOT:
                return forAlliance(new Translation2d(4.25, 5), m_alliance);
            case CENTRALSTAGEOPENING:
                return forAlliance(new Translation2d(5.87248, 4.1105), m_alliance);
            case FARSTAGEOPENING:
                return forAlliance(new Translation2d(4.3, 3.3), m_alliance);
            case STAGEADJACENT:
                return forAlliance(new Translation2d(5.87248, 1.9), m_alliance);
            case DROPSHOT:
                return forAlliance(new Translation2d(.5, 1.8), m_alliance);
            default:
                return new Translation2d();
        }
    }

    private Pose2d getPose(FieldPoint point) {
        Translation2d translation = getTranslation(point);
        Rotation2d heading = new Rotation2d(Math.PI);
        switch (point) {
            case NOTE1, NOTE2, NOTE3:
                heading = ShooterUtil.getRobotRotationToSpeaker(translation, kShooterScale);
                Translation2d offset = new Translation2d(kIntakeOffset * heading.getCos(), kIntakeOffset *
                        heading.getSin());
                return new Pose2d(translation.plus(offset), heading);
            case CLOSEWINGSHOT, FARWINGSHOT, STAGESHOT, DROPSHOT:
                heading = ShooterUtil.getRobotRotationToSpeaker(translation, kShooterScale);
                return new Pose2d(translation, heading);
            case CENTRALSTAGEOPENING, FARSTAGEOPENING, STAGEADJACENT:
                return new Pose2d(translation, heading);
            default:
                return new Pose2d(translation.plus(new Translation2d(-kIntakeOffset, 0)), heading);
        }
    }

    public AutoMaker(SwerveDriveSubsystem swerve, TrajectoryPlanner planner, DriveMotionController controller,
            SwerveKinodynamics limits, double shooterScale, Alliance alliance) {
        m_swerve = swerve;
        m_planner = planner;
        m_controller = controller;
        m_constraints = List.of(
                new CentripetalAccelerationConstraint(limits));
        kShooterScale = shooterScale;
        m_alliance = alliance;
    }

    public Translation2d forAlliance(Translation2d translation, Alliance alliance) {
        if (alliance == Alliance.Blue) {
            return translation;
        }
        return new Translation2d(translation.getX(), 8.221 - translation.getY());
    }

    public Rotation2d forAlliance(Rotation2d rotation, Alliance alliance) {
        return rotation.times(-1);
    }

    public Pose2d forAlliance(Pose2d pose, Alliance alliance) {
        return new Pose2d(forAlliance(pose.getTranslation(), alliance), forAlliance(pose.getRotation(), alliance));
    }

    public TrajectoryCommand100 adjacentWithShooterAngle(FieldPoint noteA, FieldPoint noteB) {
        Pose2d startPose = getPose(noteA);
        Pose2d endPose = getPose(noteB);
        Rotation2d angleToGoal = endPose.getTranslation().minus(startPose.getTranslation()).getAngle();
        Pose2d startWaypoint = new Pose2d(startPose.getTranslation(), angleToGoal.times(1.25));
        Pose2d endWaypoint = new Pose2d(endPose.getTranslation(), endPose.getRotation().plus(new Rotation2d(Math.PI)));
        List<Pose2d> waypointsM = List.of(startWaypoint, endWaypoint);
        List<Rotation2d> headings = List.of(startPose.getRotation(), endPose.getRotation());
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints, kMaxVelM_S,
                kMaxAccelM_S_S);
        return new TrajectoryCommand100(m_swerve, trajectory, m_controller);
    }

    public DriveToWithAutoStart startToNote(FieldPoint note) {
        Pose2d endPose = getPose(note);
        Rotation2d endRotation = endPose.getRotation().plus(new Rotation2d(Math.PI));
        Pose2d endWaypoint = new Pose2d(endPose.getTranslation(), endRotation);
        return new DriveToWithAutoStart(m_swerve, endWaypoint, endPose.getRotation(), m_planner, m_controller,
                m_constraints);
    }

    public TrajectoryCommand100 tuningTrajectory1() {
        List<Pose2d> waypointsM = List.of(new Pose2d(2, 2, new Rotation2d()),
                new Pose2d(5, 2, new Rotation2d()));
        List<Rotation2d> headings = List.of(new Rotation2d(Math.PI), new Rotation2d(Math.PI));
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints, kMaxVelM_S,
                kMaxAccelM_S_S);
        return new TrajectoryCommand100(m_swerve, trajectory, m_controller);
    }

    public TrajectoryCommand100 tuningTrajectory2() {
        List<Pose2d> waypointsM = List.of(new Pose2d(5, 2, new Rotation2d(Math.PI)),
                new Pose2d(2, 2, new Rotation2d(Math.PI)));
        List<Rotation2d> headings = List.of(new Rotation2d(Math.PI), new Rotation2d(Math.PI));
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints, kMaxVelM_S,
                kMaxAccelM_S_S);
        return new TrajectoryCommand100(m_swerve, trajectory, m_controller);
    }

    public TrajectoryCommand100 tuningTrajectory3() {
        List<Pose2d> waypointsM = List.of(new Pose2d(), new Pose2d());
        List<Rotation2d> headings = List.of(new Rotation2d(Math.PI), new Rotation2d());
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints, kMaxVelM_S,
                kMaxAccelM_S_S);
        return new TrajectoryCommand100(m_swerve, trajectory, m_controller);
    }

    public TrajectoryCommand100 tuningTrajectory4() {
        List<Pose2d> waypointsM = List.of(new Pose2d(), new Pose2d());
        List<Rotation2d> headings = List.of(new Rotation2d(), new Rotation2d(Math.PI));
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints, kMaxVelM_S,
                kMaxAccelM_S_S);
        return new TrajectoryCommand100(m_swerve, trajectory, m_controller);
    }

    public TrajectoryCommand100 stageManeuver(FieldPoint start, FieldPoint between, FieldPoint end) {
        Pose2d startPose = getPose(start);
        Pose2d betweenPose = getPose(between);
        Pose2d endPose = getPose(end);

        Rotation2d startRotationToOpening = betweenPose.getTranslation().minus(startPose.getTranslation()).getAngle();
        Rotation2d betweenRotation = endPose.getTranslation().minus(startPose.getTranslation()).getAngle();
        Rotation2d endRotationFromOpening = endPose.getTranslation().minus(betweenPose.getTranslation()).getAngle();
        Rotation2d startRotation = startRotationToOpening.minus(betweenRotation).times(1.5).plus(betweenRotation);
        Rotation2d endRotation = endRotationFromOpening.minus(betweenRotation).times(1.5).plus(betweenRotation);
        Pose2d startWaypoint = new Pose2d(startPose.getTranslation(), startRotation);
        Pose2d betweenWaypoint = new Pose2d(betweenPose.getTranslation(), betweenRotation);
        Pose2d endWaypoint = new Pose2d(endPose.getTranslation(), endRotation);
        List<Pose2d> waypointsM = List.of(startWaypoint, betweenWaypoint, endWaypoint);
        List<Rotation2d> headings = List.of(startPose.getRotation(), betweenPose.getRotation(), endPose.getRotation());
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints, kMaxVelM_S,
                kMaxAccelM_S_S);
        return new TrajectoryCommand100(m_swerve, trajectory, m_controller);
    }

    public TrajectoryCommand100 throughCentralStageOpening(FieldPoint start, FieldPoint end) {
        return stageManeuver(start, FieldPoint.CENTRALSTAGEOPENING, end);
    }

    public TrajectoryCommand100 throughFarStageOpening(FieldPoint start, FieldPoint end) {
        return stageManeuver(start, FieldPoint.FARSTAGEOPENING, end);
    }

    public TrajectoryCommand100 aroundStage(FieldPoint start, FieldPoint end) {
        return stageManeuver(start, FieldPoint.STAGEADJACENT, end);
    }

    public DriveToWaypoint100 driveToStraight(FieldPoint point) {
        return new DriveToWaypoint100(getPose(point), m_swerve, m_planner, m_controller, m_constraints);
    }

    public SequentialCommandGroup eightNoteAuto() {
        return new SequentialCommandGroup(startToNote(FieldPoint.NOTE3),
                adjacentWithShooterAngle(FieldPoint.NOTE3, FieldPoint.NOTE2),
                adjacentWithShooterAngle(FieldPoint.NOTE2, FieldPoint.NOTE1), driveToStraight(FieldPoint.NOTE8),
                driveToStraight(FieldPoint.CLOSEWINGSHOT), driveToStraight(FieldPoint.NOTE7),
                throughCentralStageOpening(FieldPoint.NOTE7, FieldPoint.STAGESHOT),
                throughCentralStageOpening(FieldPoint.STAGESHOT, FieldPoint.NOTE6),
                throughCentralStageOpening(FieldPoint.NOTE6, FieldPoint.STAGESHOT),
                throughCentralStageOpening(FieldPoint.NOTE5, FieldPoint.STAGESHOT),
                throughCentralStageOpening(FieldPoint.STAGESHOT, FieldPoint.NOTE5));
    }

    public SequentialCommandGroup complementAuto() {
        return new SequentialCommandGroup(driveToStraight(FieldPoint.NOTE4), driveToStraight(FieldPoint.FARWINGSHOT),
                aroundStage(FieldPoint.FARWINGSHOT, FieldPoint.NOTE5),
                throughCentralStageOpening(FieldPoint.NOTE5, FieldPoint.STAGESHOT), throughFarStageOpening(FieldPoint.STAGESHOT, FieldPoint.DROPSHOT));
    }

    public SequentialCommandGroup fourNoteAuto() {
        return new SequentialCommandGroup(
                startToNote(FieldPoint.NOTE1),
                adjacentWithShooterAngle(FieldPoint.NOTE1, FieldPoint.NOTE2),
                adjacentWithShooterAngle(FieldPoint.NOTE2, FieldPoint.NOTE3)
        );
    }

    public SequentialCommandGroup fiveNoteAuto() {
        return new SequentialCommandGroup(startToNote(FieldPoint.NOTE3),
                adjacentWithShooterAngle(FieldPoint.NOTE3, FieldPoint.NOTE2),
                adjacentWithShooterAngle(FieldPoint.NOTE2, FieldPoint.NOTE1), driveToStraight(FieldPoint.NOTE8),
                driveToStraight(FieldPoint.CLOSEWINGSHOT));
    }

    public SequentialCommandGroup tuning() {
        return new SequentialCommandGroup(tuningTrajectory1(), new WaitCommand(1), tuningTrajectory2());
    }
}
