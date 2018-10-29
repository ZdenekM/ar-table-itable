from PyQt4 import QtCore
import rospy
from typing import Tuple
from art_helpers import ProgramHelper

translate = QtCore.QCoreApplication.translate


class GuiInstruction(QtCore.QObject):

    NAME = ""  # localized name of the instruction

    def __init__(self, ui, block_id, item_id, editable=False, flags=None):

        super(GuiInstruction, self).__init__()

        self.ui = ui
        self.block_id = block_id
        self.instruction_id = item_id
        self.editable = editable
        self.flags = flags
        self.notified = False

    def loginfo(self, msg):
        # type: (str) -> None

        rospy.loginfo(self.NAME + ": " + msg)

    def logerr(self, msg):
        # type: (str) -> None

        rospy.logerr(self.NAME + ": " + msg)

    def logwarn(self, msg):
        # type: (str) -> None

        rospy.logwarn(self.NAME + ": " + msg)

    def logdebug(self, msg):
        # type: (str) -> None

        rospy.logdebug(self.NAME + ": " + msg)

    @staticmethod
    def get_text(ph, block_id, item_id):
        # type: (ProgramHelper, int, int) -> str
        """
        Returns additional (localized) text displayed below instruction name in program visualization
        """

        return ""

    def cleanup(self):
        """
        Removes all previously created scene items (such as dialogs).
        Return scene items to be deleted later in form of ((item1, rospy.Time), (item2, rospy.Time), )...
        """

        return ()

    def object_selected(self, obj, selected, msg):

        return

    @property
    def cid(self):
        # type: () -> Tuple[int, int]
        """Current instruction id"""

        return self.block_id, self.instruction_id

    def learning_done(self):
        # type: () -> None
        """Called when user presses 'Done' button."""

        return

    def detected_objects(self, msg):

        return
