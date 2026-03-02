import matplotlib
matplotlib.use('Qt5Agg')

from math import sin, cos, radians, degrees

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from matplotlib.patches import FancyArrow
import matplotlib.pyplot as plt

class FieldPlot(FigureCanvasQTAgg):

    def __init__(self, parent=None, width=8, height=4, dpi=100, image_path="Field.png"):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super().__init__(fig)

        self.img_path = image_path
        self.robot_pos = (32.0, 6.0)
        self.robot_dir = radians(90)
        self.robot_radius = 6.0

        #* Robot Marker
        self.robot_marker = None
        self.robot_arrow = None

        #* Robot Marker Parameters
        self.rmarker_color = 'white'
        self.rarrow_color = 'black'
        self.rarrow_width = 1
        self.rarrow_head_w = 2
        self.rarrow_head_l = 1

        self.setup_plot()
    
    def setup_plot(self):
        img = plt.imread(self.img_path)
        self.axes.imshow(img, extent=[0, 96, 0, 48])

        self.robot_marker = plt.Circle(self.robot_pos, self.robot_radius, color=self.rmarker_color)

        dx = 6*cos(self.robot_dir)
        dy = 6*sin(self.robot_dir)

        self.robot_arrow = FancyArrow(self.robot_pos[0], self.robot_pos[1],
                              dx, dy,
                              width=self.rarrow_width, head_width=self.rarrow_head_w, 
                              head_length=self.rarrow_head_l, color=self.rarrow_color)
        
        self.axes.add_patch(self.robot_marker)
        self.axes.add_patch(self.robot_arrow)

        self.draw()
    
    def update_robot_pos(self, x, y, dir):
        self.robot_pos = (x, y)
        self.robot_dir = radians(dir)

        if self.robot_marker:
            self.robot_marker.center = (x, y)
        
        if self.robot_arrow:
            dx = 6*cos(self.robot_dir)
            dy = 6*sin(self.robot_dir)

            self.robot_arrow.set_data(x=x, y=y, dx=dx, dy=dy)

        self.draw_idle()
    
    def return_robot_pos(self):
        return self.robot_pos[0], self.robot_pos[1], degrees(self.robot_dir)