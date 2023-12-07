#!/usr/bin/env python3
import logging
import pathlib
import signal
import sys
import threading

import rospy
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix


from PySide2 import QtCore
from PySide2 import QtGui
from PySide2 import QtQml
from PySide2.QtPositioning import QGeoCoordinate

from PySide2.QtCore import (
    Property,
    QObject,
    Signal,
    Slot,
)


def traceme(func):
    def newfunc(*args, **kwargs):
        print(f"BEGIN: ========== {func.__name__} ============")
        ret = func(*args, **kwargs)
        print(f"DONE ============ {func.__name__} ============")
        return ret

    return newfunc


def get_coord(lat, lon):
    pos = QGeoCoordinate()
    pos.setLatitude(lat)
    pos.setLongitude(lon)
    return pos


class Backend(QObject):
    exitRequested = Signal()
    valueChanged = Signal()
    locationChanged = Signal()
    targetLocationChanged = Signal()
    historyChanged = Signal()

    def __init__(self, parent=None):
        QObject.__init__(self, parent)
        self._shoud_run = True
        self._rate = rospy.Rate(1)
        self._sub_location = rospy.Subscriber(
            "/robot_location", NavSatFix, self.cb_location
        )
        self._pub_target = rospy.Publisher(
            "/target_location", GeoPoint, queue_size=1, latch=True
        )
        self._robot_location = get_coord(32.072734, 34.787465)
        self._target_location = get_coord(32.072734, 34.787465)
        self._robot_route = []
        self.max_tractor_hist = 1000
        self._thread = threading.Thread(target=self.main_loop)
        self._thread.start()

    @Slot(float, float)
    def set_target_point(self, lat, lon):
        self._target_location = get_coord(lat, lon)
        msg = GeoPoint()
        msg.latitude = lat
        msg.longitude = lon
        self._pub_target.publish(msg)
        self.targetLocationChanged.emit()

    @Property(
        QGeoCoordinate,
        constant=False,
        notify=locationChanged,
    )
    def robot_location(self):
        return self._robot_location

    @Property(
        QGeoCoordinate,
        constant=False,
        notify=targetLocationChanged,
    )
    def target_location(self):
        return self._target_location

    # @traceme
    def cb_location(self, msg: NavSatFix):
        self._robot_location = get_coord(msg.latitude, msg.longitude)
        self.locationChanged.emit()

    # @traceme
    def append_location_to_hist(self):
        self._robot_route.append(self._robot_location)
        if len(self._robot_route) > self.max_tractor_hist:
            del self._robot_route[0]
        self.historyChanged.emit()

    @Slot()
    def clearHistory(self):
        self._robot_route.clear()

    @Slot()
    def exit(self):
        self._shoud_run = False
        self._thread.join()

    @traceme
    def main_loop(self):
        while not rospy.is_shutdown() and self._shoud_run:
            self.append_location_to_hist()
            self._rate.sleep()

    @QtCore.Property(list, constant=False, notify=historyChanged)
    def location_history(self):
        logging.warning("Location history requested")
        return self._robot_route


if __name__ == "__main__":
    rospy.init_node("sim_gui", anonymous=True)
    app = QtGui.QGuiApplication(sys.argv)
    app.setApplicationVersion("0.1")
    app.setOrganizationDomain("bw-robotics.com")
    app.setOrganizationName("BWR")
    app.styleHints().setMousePressAndHoldInterval(500)
    backend = Backend(parent=app)
    engine = QtQml.QQmlApplicationEngine()
    engine.rootContext().setContextProperty("backend", backend)
    current_dir = pathlib.Path(__file__).resolve().parent
    ok = engine.load(QtCore.QUrl(str(current_dir / "view.qml")))
    if not engine.rootObjects():
        sys.exit(-1)
    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(100)

    def sigint_handler(sig, frame):
        print(f"\tSIGINT: exiting (signal={sig},frame={frame})", flush=True)
        backend.exitRequested.emit()

    signal.signal(signal.SIGINT, sigint_handler)
    signal.signal(signal.SIGTERM, sigint_handler)
    result = app.exec_()
    backend.exit()
    sys.exit(result)
