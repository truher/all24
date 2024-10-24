""" Run a GTSAM model that uses constant camera calibration.

The general idea is to run a free-running loop, polling for inputs.
Those inputs are asserted using one of the input methods here.
"""

# pylint: disable=C0301,E0611,E1101,R0903

import gtsam
import numpy as np
from gtsam import noiseModel
from gtsam.symbol_shorthand import X
from wpimath.geometry import Rotation2d, Translation2d, Twist2d

import app.pose_estimator.odometry as odometry
from app.pose_estimator.drive_util import DriveUtil
from app.pose_estimator.swerve_drive_kinematics import SwerveDriveKinematics100
from app.pose_estimator.swerve_module_delta import SwerveModuleDelta
from app.pose_estimator.swerve_module_position import (OptionalRotation2d,
                                                       SwerveModulePosition100)

# odometry noise.  TODO: real noise estimate.
NOISE3 = noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))


class Estimate:
    def __init__(self) -> None:
        """Initialize the model"""
        self.isam: gtsam.FixedLagSmoother = self.make_smoother()
        self.result: gtsam.Values
        # between updates we accumulate inputs here
        self.new_factors = gtsam.NonlinearFactorGraph()
        self.new_values = gtsam.Values()
        self.new_timestamps = {}
        # TODO: correct wheelbase etc
        self.kinematics = SwerveDriveKinematics100(
            [
                Translation2d(0.5, 0.5),
                Translation2d(0.5, -0.5),
                Translation2d(-0.5, 0.5),
                Translation2d(-0.5, -0.5),
            ]
        )
        # TODO: reset position
        self.positions = [
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
        ]
        self.timestamp: int = 0

    def init(self) -> None:
        """Add a state at zero"""
        prior_mean = gtsam.Pose2(0, 0, 0)
        prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))
        self.new_factors.push_back(
            gtsam.PriorFactorPose2(X(0), prior_mean, prior_noise)
        )
        self.new_values.insert(X(0), prior_mean)
        self.new_timestamps[X(0)] = 0.0

    def odometry(self, time_us: int, positions: list[SwerveModulePosition100]) -> None:
        """Add an odometry measurement

        time_us: network tables timestamp in integer microseconds.
        """
        # odometry is not guaranteed to arrive in order, but for now, it is.
        # TODO: out-of-order odometry
        # each odometry update maps exactly to a "between" factor
        # remember a "twist" is a robot-relative concept
        deltas: list[SwerveModuleDelta] = DriveUtil.module_position_delta(
            self.positions, positions
        )
        # this is the tangent-space (twist) measurement
        measurement: Twist2d = self.kinematics.to_twist_2d(deltas)
        # each odometry update makes a new state
        # if you're using the batch smoother, the initial value almost doesn't matter:
        # TODO: use the previous pose as the initial value
        initial_value = gtsam.Pose2()
        self.new_values.insert(X(time_us), initial_value)
        self.new_timestamps[X(time_us)] = time_us

        self.new_factors.add(
            odometry.factor(
                measurement,
                NOISE3,
                X(self.timestamp),
                X(time_us),
            )
        )

        self.positions = positions
        self.timestamp = time_us

    def update(self) -> None:
        """Run the solver"""
        self.isam.update(self.new_factors, self.new_values, self.new_timestamps)
        self.result: Values = self.isam.calculateEstimate()  # type: ignore
        # reset the accumulators
        self.new_factors.resize(0)
        self.new_values.clear()
        self.new_timestamps.clear()

    def make_smoother(self) -> gtsam.FixedLagSmoother:
        lag_s = 10
        lm_params = gtsam.LevenbergMarquardtParams()
        return gtsam.BatchFixedLagSmoother(lag_s, lm_params)
