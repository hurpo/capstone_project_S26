import sys
import ipaddress
from PySide6 import QtCore, QtWidgets

from client import connect

testing = False

class LoginPage(QtWidgets.QWidget):
    def __init__(self, stack):
        super().__init__()
        self.stack = stack

        self.text = QtWidgets.QLabel(
            "Enter Robot IP:",
            alignment=QtCore.Qt.AlignCenter
        )
        self.text.setStyleSheet("""font-size: 35px; font-weight: bold;""")

        self.ipenter_container = QtWidgets.QHBoxLayout()
        self.ipenter_container.setSpacing(0)
        self.ipenter_container.setContentsMargins(0, 0, 0, 0)

        self.ip_enter = QtWidgets.QLineEdit()
        self.ip_enter.setPlaceholderText("192.168.1.10")
        # self.ip_enter.setAlignment(QtCore.Qt.AlignCenter | QtCore.Qt.AlignVCenter)
        self.ip_enter.setStyleSheet("""font-size: 25px; margin: 0px;""")

        self.button = QtWidgets.QPushButton("Connect")
        self.button.setStyleSheet("""font-size: 25px; margin: 0px;""")
        self.button.setFixedWidth(120)

        self.ipenter_container.addWidget(self.ip_enter)
        self.ipenter_container.addWidget(self.button)

        self.scroll_error = QtWidgets.QScrollArea()
        self.error = QtWidgets.QLabel(
            "",
            alignment=QtCore.Qt.AlignCenter
        )
        self.error.setStyleSheet("""color: red; font-size: 30px; font-weight: bold;""")
        self.scroll_error.setWidget(self.error)
        self.scroll_error.setWidgetResizable(True)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.text)
        layout.addLayout(self.ipenter_container)
        layout.addWidget(self.scroll_error)

        self.button.clicked.connect(self.connect)

    @QtCore.Slot()
    def connect(self):
        global testing

        ip_text = self.ip_enter.text()

        try:
            ipaddress.ip_address(ip_text)
            connection = connect(ip_text)

            if "Error" in connection:
                self.error.setText(connection)
            if "Connection Successful" in connection or testing == True:
                self.stack.setCurrentIndex(1)
        except ValueError:
            self.error.setText("Invalid IP address")
