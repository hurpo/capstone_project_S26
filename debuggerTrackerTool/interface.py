from PySide6 import QtCore, QtWidgets, QtGui
import cv2
import numpy as np

from map_plotting import FieldPlot
from robot_data_display import RobotDataDisplay
from client import disconnect, send_message, get_message, get_latest_frame, send_command, get_latest_position, get_latest_robot_data

class InterfacePage(QtWidgets.QWidget):
    def __init__(self, stack):
        super().__init__()
        self.stack = stack

        label = QtWidgets.QLabel("Robot Interface Loaded", alignment=QtCore.Qt.AlignCenter)

        self.robot_data = {
            "State": "None",
            "RP": "None",
            "LED_Started?": False,
            "In_Cave?": False,
            "Magnetics_Sorted?": 0,
            "dist_in_m1": 0, 
            "dist_in_m2": 0, 
            "dist_in_m3": 0, 
            "dist_in_m4": 0
        }
        self.data_excluded_from_generic = ["State", "dist_in_m1", "dist_in_m2", "dist_in_m3", "dist_in_m4"]

        #--------------------------------------------

        # Field Plot
        self.field = FieldPlot(self)
        self.robot_x, self.robot_y, self.robot_dir = self.field.return_robot_pos()
        print(f"{self.robot_x}, {self.robot_y}, {self.robot_dir}")

        # Video display
        self.video_label = QtWidgets.QLabel("No video")
        self.video_label.setAlignment(QtCore.Qt.AlignCenter)
        self.video_label.setMinimumHeight(300)
        self.video_label.setStyleSheet("background: #111; color: #ddd;")

        # Field and Camera Feed Row
        field_and_cam_row = QtWidgets.QHBoxLayout()
        field_and_cam_row.addWidget(self.field, 5)
        field_and_cam_row.addWidget(self.video_label, 3)

        #--------------------------------------------

        # Robot Position Coordinates

        robot_pos_coords = QtWidgets.QHBoxLayout()
        self.robot_xcoord = QtWidgets.QLabel(f"X: {self.robot_x}", )
        self.robot_ycoord = QtWidgets.QLabel(f"Y: {self.robot_y}")
        self.robot_dircoord = QtWidgets.QLabel(f"Degrees: {self.robot_dir}")
        
        self.robot_xcoord.setAlignment(QtCore.Qt.AlignCenter | QtCore.Qt.AlignVCenter)
        self.robot_ycoord.setAlignment(QtCore.Qt.AlignCenter | QtCore.Qt.AlignVCenter)
        self.robot_dircoord.setAlignment(QtCore.Qt.AlignCenter | QtCore.Qt.AlignVCenter)
        
        coords_style = """
            color: #ffffff;
            font-size: 16px;
            font-weight: bold;
            text-align: center;
            padding: 2px;
        """
        coords_x_style = """background-color: #000080;"""
        coords_y_style = """background-color: #7f0400;"""
        coords_dir_style = """background-color: #007f00;"""

        self.robot_xcoord.setStyleSheet(coords_style + coords_x_style)
        self.robot_ycoord.setStyleSheet(coords_style + coords_y_style)
        self.robot_dircoord.setStyleSheet(coords_style + coords_dir_style)

        robot_pos_coords.addWidget(self.robot_xcoord)
        robot_pos_coords.addWidget(self.robot_ycoord)
        robot_pos_coords.addWidget(self.robot_dircoord)
        
        # Encoder Readings

        m_dist_style = """
            background-color: #393b3d;
            color: #ffffff;
            font-size: 16px;
            font-weight: bold;
            text-align: center;
            padding: 2px;
        """
        m_dist_reading_widget = QtWidgets.QWidget()
        m_dist_reading_container = QtWidgets.QHBoxLayout()
        m_dist_reading_label = QtWidgets.QLabel("Motors\n[m1, m2, m3, m4]:")
        self.m1_dist_reading = QtWidgets.QLabel(f"{self.robot_data["dist_in_m1"]}")
        self.m2_dist_reading = QtWidgets.QLabel(f"{self.robot_data["dist_in_m2"]}")
        self.m3_dist_reading = QtWidgets.QLabel(f"{self.robot_data["dist_in_m3"]}")
        self.m4_dist_reading = QtWidgets.QLabel(f"{self.robot_data["dist_in_m4"]}")
        m_dist_reading_container.addWidget(m_dist_reading_label)
        m_dist_reading_container.addWidget(self.m1_dist_reading)
        m_dist_reading_container.addWidget(self.m2_dist_reading)
        m_dist_reading_container.addWidget(self.m3_dist_reading)
        m_dist_reading_container.addWidget(self.m4_dist_reading)
        m_dist_reading_widget.setLayout(m_dist_reading_container)
        m_dist_reading_widget.setStyleSheet(m_dist_style)
       
        # Robot State

        robot_current_state_label = QtWidgets.QLabel("State: ")
        robot_current_state_label.setAlignment(QtCore.Qt.AlignCenter | QtCore.Qt.AlignVCenter)
        self.robot_current_state = QtWidgets.QLabel(f"{self.robot_data["State"]}")
        self.robot_current_state.setAlignment(QtCore.Qt.AlignCenter | QtCore.Qt.AlignVCenter)

        current_state_style = """
            font-size: 18px;
        """
        self.robot_current_state.setStyleSheet(current_state_style)

        robot_current_state = QtWidgets.QHBoxLayout()
        robot_current_state.addWidget(robot_current_state_label)
        robot_current_state.addWidget(self.robot_current_state)
        robot_current_state_container = QtWidgets.QWidget()
        robot_current_state_container.setLayout(robot_current_state)

        robot_current_state_container_style = """
            background-color: #4f0f56;
            color: #ffffff;
            font-size: 16px;
            font-weight: bold;
            text-align: center;
            padding: 2px;
        """
        robot_current_state_container.setStyleSheet(robot_current_state_container_style)
        

        robot_coords_and_state = QtWidgets.QVBoxLayout()
        robot_coords_and_state.addLayout(robot_pos_coords)
        robot_coords_and_state.addWidget(m_dist_reading_widget)
        robot_coords_and_state.addWidget(robot_current_state_container)

        # Robot Data Display
        self.robot_data_display = RobotDataDisplay(self)
        filtered_robot_data = {k: v for k, v in self.robot_data.items() if k not in self.data_excluded_from_generic}
        self.robot_data_display.updateLabels(filtered_robot_data)

        # Data Row
        data_row = QtWidgets.QHBoxLayout()
        data_row.addLayout(robot_coords_and_state)
        data_row.addWidget(self.robot_data_display)

        #--------------------------------------------

        # Robot Controls
        start_sm = QtWidgets.QPushButton("Start")
        stop_sm = QtWidgets.QPushButton("Stop")
        pause_sm = QtWidgets.QPushButton("Pause")
        resume_sm = QtWidgets.QPushButton("Resume")
        disconnect_btn = QtWidgets.QPushButton("Disconnect")

        start_stop_row = QtWidgets.QHBoxLayout()
        start_stop_row.addWidget(start_sm)
        start_stop_row.addWidget(stop_sm)

        pause_resume_row = QtWidgets.QHBoxLayout()
        pause_resume_row.addWidget(pause_sm)
        pause_resume_row.addWidget(resume_sm)

        control_buttons = QtWidgets.QVBoxLayout()
        control_buttons.addLayout(start_stop_row)
        control_buttons.addLayout(pause_resume_row)

        # Chat Commands
        self.chat_box = QtWidgets.QTextEdit()
        self.chat_box.setReadOnly(True)

        self.message_line = QtWidgets.QLineEdit()
        submit_message = QtWidgets.QPushButton("Send")

        message_row = QtWidgets.QHBoxLayout()
        message_row.addWidget(self.message_line)
        message_row.addWidget(submit_message)

        text_comm = QtWidgets.QVBoxLayout()
        text_comm.addWidget(self.chat_box)
        text_comm.addLayout(message_row)
    
        # Controls Layout
        controls_layout = QtWidgets.QHBoxLayout()
        controls_layout.addLayout(control_buttons)
        controls_layout.addLayout(text_comm)

        #--------------------------------------------

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(label)
        layout.addLayout(field_and_cam_row)
        layout.addLayout(data_row)
        layout.addLayout(controls_layout)
        layout.addWidget(disconnect_btn)

        self._last_frame_ptr = None 
        self._last_position = None
        self._last_robot_data = None

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.poll_updates)
        self.timer.start(30)  # ~33 fps polling 

        start_sm.clicked.connect(lambda: self.send_msg_button("START"))
        stop_sm.clicked.connect(lambda: self.send_msg_button("STOP"))
        pause_sm.clicked.connect(lambda: self.send_msg_button("PAUSE"))
        resume_sm.clicked.connect(lambda: self.send_msg_button("RESUME"))

        submit_message.clicked.connect(self.send_msg)
        disconnect_btn.clicked.connect(self.disconnect)

    def poll_updates(self):
        # Chat messages
        for msg in get_message():
            self.chat_box.append(msg)

        # Position updates
        position = get_latest_position()
        if position and position != self._last_position:
            print(f"position: {position['y']}")
            print("Updating Field POS")
            self._last_position = position.copy()
            self.field.update_robot_pos(position['x'], position['y'], position['degrees'])
            
            self.robot_x, self.robot_y, self.robot_dir = self.field.return_robot_pos()
            print(f"")
            
            self.robot_xcoord.setText(f"X: {self.robot_x}")
            self.robot_ycoord.setText(f"Y: {self.robot_y}")
            self.robot_dircoord.setText(f"Degrees: {self.robot_dir}")

        # Robot Data updates
        new_rdata = get_latest_robot_data()
        if new_rdata and new_rdata != self._last_robot_data:
            self._last_robot_data = new_rdata.copy()
            for key in new_rdata:
                if key == "State":
                    self.robot_data[key] = new_rdata[key]
                    self.robot_current_state.setText(f"{new_rdata[key]}")
                if key == "dist_in_m1":
                    self.robot_data[key] = new_rdata[key]
                    self.m1_dist_reading.setText(f"{new_rdata[key]}")
                if key == "dist_in_m2":
                    self.robot_data[key] = new_rdata[key]
                    self.m2_dist_reading.setText(f"{new_rdata[key]}")
                if key == "dist_in_m3":
                    self.robot_data[key] = new_rdata[key]
                    self.m3_dist_reading.setText(f"{new_rdata[key]}")
                if key == "dist_in_m4":
                    self.robot_data[key] = new_rdata[key]
                    self.m4_dist_reading.setText(f"{new_rdata[key]}")
                elif key in self.robot_data:
                    print(f"Key {key} already in dict")
                    if self.robot_data[key] != new_rdata[key]:
                        print(f"Updated {key} with new value {new_rdata[key]}")
                        self.robot_data[key] = new_rdata[key]
                else:
                    print(f"Creating new key, {key}, with value {new_rdata[key]}")
                    self.robot_data[key] = new_rdata[key]
            filtered_robot_data = {k: v for k, v in self.robot_data.items() if k not in self.data_excluded_from_generic}
            self.robot_data_display.updateLabels(filtered_robot_data)
            print(f"new_rdata: {self.robot_data}")

        # Video frame
        jpg_bytes = get_latest_frame()
        if not jpg_bytes:
            return

        if jpg_bytes is self._last_frame_ptr:
            return
        self._last_frame_ptr = jpg_bytes

        arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        # Convert BGR -> RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = frame_rgb.shape
        bytes_per_line = ch * w

        qimg = QtGui.QImage(frame_rgb.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap.fromImage(qimg)

        # Fit to label
        pix = pix.scaled(self.video_label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        self.video_label.setPixmap(pix)

    def send_msg(self):
        msg = self.message_line.text()
        if not msg.strip():
            return
        send_command(msg)
        self.message_line.clear()
    
    def send_msg_button(self, textin):
        msg = textin
        send_command(msg)

    def disconnect(self):
        disconnect()
        self.stack.setCurrentIndex(0)
