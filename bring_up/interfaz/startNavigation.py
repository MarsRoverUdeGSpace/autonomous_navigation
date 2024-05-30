import sys
from PyQt5 import QtWidgets
import rospy
import subprocess

from navigation import Ui_MainWindow  # Import the converted .ui file

class MyApp(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.btnNavigate.clicked.connect(self.run_ros_node)

    def run_ros_node(self):
        # Get the inputs from the QLineEdit fields
        arg_latitude = self.editLatitude.text()
        arg_longitude = self.editLongitude.text()

        # Ensure ROS master is running
        if not rospy.core.is_initialized():
            rospy.init_node('gui_node', anonymous=True, disable_signals=True)

        # Prepare the ROS node execution command with arguments
        ros_command = ['rosrun', 'bring_up', 'post_goal.py', 
                       arg_latitude, arg_longitude]

        # Execute the ROS node with the arguments
        subprocess.Popen(ros_command)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MyApp()
    window.show()
    sys.exit(app.exec_())
