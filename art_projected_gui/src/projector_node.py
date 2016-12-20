#!/usr/bin/env python

import sys
import signal
import rospy
from PyQt4 import QtGui, QtCore
from gui import Projector


def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    sys.stderr.write('\r')
    QtGui.QApplication.quit()


def main(args):

    rospy.init_node('projected_gui_projector')

    signal.signal(signal.SIGINT, sigint_handler)

    app = QtGui.QApplication(sys.argv)

    proj = Projector()

    timer = QtCore.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)  # Let the interpreter run each 500 ms.

    sys.exit(app.exec_())

if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
