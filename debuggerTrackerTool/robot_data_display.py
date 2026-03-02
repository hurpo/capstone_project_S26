from PySide6 import QtCore, QtWidgets

class RobotDataDisplay(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.setAlignment(QtCore.Qt.AlignTop)

        self.label_style = """
            color: #ffffff;
            background-color: #535354;
            font-size: 12px;
            text-align: center;
            padding: 2px;
        """

        self.label_value = """font-weight: bold; """

        self.labels = {}

    def updateLabels(self, robot_data):
        for key, value in robot_data.items():
            if key not in self.labels:
                label = QtWidgets.QLabel(f"{key}:")
                label.setAlignment(QtCore.Qt.AlignCenter | QtCore.Qt.AlignVCenter)
                label_val = QtWidgets.QLabel(f"{value}")
                label_val.setStyleSheet(self.label_value)
                label_val.setAlignment(QtCore.Qt.AlignCenter | QtCore.Qt.AlignVCenter)

                layout = QtWidgets.QHBoxLayout()
                layout.addWidget(label)
                layout.addWidget(label_val)
                container = QtWidgets.QWidget()
                container.setLayout(layout)

                container.setStyleSheet(self.label_style)
                self.labels[key] = container
                self.layout.addWidget(container)
            else:
                print(f"{self.labels[key]}, {key}, {value}")
                container = self.labels[key]
                layout = container.layout()
                label_val = layout.itemAt(1).widget()
                label_val.setText(f"{value}")