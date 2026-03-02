import sys
import random
from PySide6 import QtCore, QtWidgets, QtGui

from login import LoginPage
from interface import InterfacePage

class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        self.stack = QtWidgets.QStackedWidget()


        self.login_page = LoginPage(self.stack)
        self.interface_page = InterfacePage(self.stack)

        self.stack.addWidget(self.login_page)      # index 0
        self.stack.addWidget(self.interface_page)  # index 1

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.stack)

        self.setWindowTitle("IEEE Robot Interface")
        self.resize(800, 600)


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)

    window = MainWindow()
    window.show()

    sys.exit(app.exec())
