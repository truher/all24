""" This is the coprocessor main loop.

Each task is run by its own Looper, in its own thread.

You can't run this from the command line.  To run the app,
use the script called "runapp.py" in the raspberry_pi directory
(one level above this one).
"""

# pylint: disable=R0914

from threading import Event, Thread

from app.camera.camera_factory import CameraFactory
from app.camera.camera_loop import CameraLoop
from app.config.identity import Identity
from app.dashboard.real_display import RealDisplay
from app.framework.looper import Looper
from app.network.real_network import RealNetwork
from app.localization.note_detector import NoteDetector
from app.localization.tag_detector import TagDetector
from app.sensors.gyro_factory import GyroFactory
from app.sensors.gyro_loop import GyroLoop


def main() -> None:
    print("main")
    identity: Identity = Identity.get()
    network = RealNetwork(identity)

    done = Event()
    try:
        loops: list[Looper] = []

        camera0 = CameraFactory.get(identity, 0, network)
        size0 = camera0.get_size()
        display0 = RealDisplay(size0.width // 2, size0.height // 2, "tag0")
        # display01 = RealDisplay(size0.width, size0.height, "note0")
        detector0 = TagDetector(identity, camera0, 0, display0, network)
        # detector01 = NoteDetector(identity, camera0, 1, display01, network)
        # loops.append(CameraLoop(camera0, [detector0, detector01], done))
        loops.append(CameraLoop(camera0, [detector0], done))

        # TODO: a better way to associate cameras and detectors
        #
        if CameraFactory.get_num_cameras(identity) > 1:
            camera1 = CameraFactory.get(identity, 1, network)
            size1 = camera1.get_size()
            display1 = RealDisplay(size1.width // 2, size1.height // 2, "note1")
            # display11 = RealDisplay(size1.width, size1.height, "tag1")
            detector1 = NoteDetector(identity, camera1, 2, display1, network)
            # detector11 = TagDetector(identity, camera1, 3, display11, network)
            # loops.append(CameraLoop(camera1, [detector1, detector11], done))
            loops.append(CameraLoop(camera1, [detector1], done))

        gyro = GyroFactory.get(identity, network)
        loops.append(GyroLoop(gyro, done))

        for loop in loops:
            Thread(target=loop.run).start()

        # Waits forever.  If any thread sets the event, exit.
        done.wait()

    finally:
        done.set()  # exit threads cleanly
