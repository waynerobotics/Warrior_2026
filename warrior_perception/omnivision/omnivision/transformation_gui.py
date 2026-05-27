##############################################################################
# Transformation Adjustment GUI - QT-based slider for camera-LiDAR alignment
#
# Software License Agreement (GPL-3.0)
# Copyright (C) 2025 Shanti Robot Team
##############################################################################

import sys
import rclpy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from PyQt5.QtWidgets import (QApplication, QMainWindow, QSlider, QVBoxLayout,
                             QHBoxLayout, QLabel, QPushButton, QWidget, QGroupBox,
                             QDoubleSpinBox, QTextEdit)
from PyQt5.QtCore import Qt, QTimer
import numpy as np
import math

class TransformationAdjusterGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        # Initialize ROS node
        rclpy.init(args=None)
        self.node = rclpy.create_node('transformation_adjuster_gui')

        # Create parameter clients - connect to both servers
        self.transform_server = '/transformation_reconfiguration_server'
        self.mask_server = '/mask_to_pointcloud'

        self.transform_param_client = self.node.create_client(
            SetParameters, f'{self.transform_server}/set_parameters')

        self.mask_param_client = self.node.create_client(
            SetParameters, f'{self.mask_server}/set_parameters')

        # Initial values
        self.yaw_angle = 0.0

        # Initialize the UI
        self.init_ui()

        # Set up a timer for spinning the ROS node
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.spin_once)
        self.ros_timer.start(100)  # 10Hz

        # Set up a timer for updating the transformation matrix display
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_matrix_display)
        self.update_timer.start(500)  # 2Hz

        # Track if we've connected to the parameter servers
        self.param_servers_connected = False

    def init_ui(self):
        # Main window setup
        self.setWindowTitle("Transformation Adjuster")
        self.setGeometry(100, 100, 800, 500)

        # Main layout
        main_layout = QVBoxLayout()

        # Yaw adjustment group
        yaw_group = QGroupBox("Yaw Angle Adjustment")
        yaw_layout = QVBoxLayout()

        # Yaw slider
        slider_layout = QHBoxLayout()
        self.yaw_slider = QSlider(Qt.Horizontal)
        self.yaw_slider.setMinimum(-1800)  # -180.0 degrees
        self.yaw_slider.setMaximum(1800)   # 180.0 degrees
        self.yaw_slider.setValue(0)        # 0.0 degrees
        self.yaw_slider.setTickPosition(QSlider.TicksBelow)
        self.yaw_slider.setTickInterval(300)  # 30.0 degrees
        self.yaw_slider.valueChanged.connect(self.update_yaw_from_slider)

        self.yaw_label = QLabel("Yaw: 0.0°")
        slider_layout.addWidget(QLabel("-180°"))
        slider_layout.addWidget(self.yaw_slider)
        slider_layout.addWidget(QLabel("180°"))
        yaw_layout.addLayout(slider_layout)

        # Precise adjustment
        precise_layout = QHBoxLayout()
        precise_layout.addWidget(QLabel("Precise Adjustment:"))

        self.yaw_spin = QDoubleSpinBox()
        self.yaw_spin.setRange(-180.0, 180.0)
        self.yaw_spin.setSingleStep(0.1)
        self.yaw_spin.setDecimals(1)
        self.yaw_spin.setValue(0.0)
        self.yaw_spin.valueChanged.connect(self.update_yaw_from_spin)

        precise_layout.addWidget(self.yaw_spin)
        precise_layout.addWidget(QLabel("degrees"))
        precise_layout.addStretch()

        # Add current value label
        precise_layout.addWidget(self.yaw_label)

        yaw_layout.addLayout(precise_layout)
        yaw_group.setLayout(yaw_layout)
        main_layout.addWidget(yaw_group)

        # Transformation matrix display
        matrix_group = QGroupBox("Transformation Matrix")
        matrix_layout = QVBoxLayout()

        self.matrix_display = QTextEdit()
        self.matrix_display.setReadOnly(True)
        self.matrix_display.setMinimumHeight(200)
        matrix_layout.addWidget(self.matrix_display)

        matrix_group.setLayout(matrix_layout)
        main_layout.addWidget(matrix_group)

        # Action buttons
        button_layout = QHBoxLayout()

        self.apply_button = QPushButton("Apply")
        self.apply_button.clicked.connect(self.apply_settings)

        self.reset_button = QPushButton("Reset")
        self.reset_button.clicked.connect(self.reset_settings)

        button_layout.addStretch()
        button_layout.addWidget(self.apply_button)
        button_layout.addWidget(self.reset_button)

        main_layout.addLayout(button_layout)

        # Create central widget and set the layout
        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        # Initial matrix display
        self.update_matrix_display()

    def update_yaw_from_slider(self):
        value = self.yaw_slider.value() / 10.0  # Convert from int to float with precision
        self.yaw_angle = value
        self.yaw_spin.blockSignals(True)
        self.yaw_spin.setValue(value)
        self.yaw_spin.blockSignals(False)
        self.yaw_label.setText(f"Yaw: {value:.1f}°")
        self.update_matrix_display()

    def update_yaw_from_spin(self):
        value = self.yaw_spin.value()
        self.yaw_angle = value
        self.yaw_slider.blockSignals(True)
        self.yaw_slider.setValue(int(value * 10))
        self.yaw_slider.blockSignals(False)
        self.yaw_label.setText(f"Yaw: {value:.1f}°")
        self.update_matrix_display()

    def calculate_transformation_matrix(self):
        # Calculate rotation matrix for the current yaw angle
        angle_rad = math.radians(self.yaw_angle)
        cos_angle = math.cos(angle_rad)
        sin_angle = math.sin(angle_rad)

        # Create the rotation matrix around Z axis (yaw)
        rotation_matrix = np.identity(4)
        rotation_matrix[0, 0] = cos_angle
        rotation_matrix[0, 1] = -sin_angle
        rotation_matrix[1, 0] = sin_angle
        rotation_matrix[1, 1] = cos_angle

        # For simplicity, we're applying this to the identity matrix as base
        # In a real application, you'd apply it to your base transformation
        base_transformation = np.identity(4)
        return np.matmul(rotation_matrix, base_transformation)

    def update_matrix_display(self):
        matrix = self.calculate_transformation_matrix()

        # Format the matrix for display
        display_text = "Current Transformation Matrix:\n\n"
        for i in range(4):
            row = " ".join([f"{matrix[i, j]:8.4f}" for j in range(4)])
            display_text += f"[{row}]\n"

        # Add flattened format for copy-paste
        display_text += "\nFlattened format for parameters:\n"
        flat_array = matrix.flatten().tolist()
        matrix_str = '[' + ', '.join([f"{val:.6f}" for val in flat_array]) + ']'
        display_text += matrix_str

        self.matrix_display.setText(display_text)

    def apply_settings(self):
        # Wait for service if not already available
        if not self.transform_param_client.wait_for_service(timeout_sec=1.0) or not self.mask_param_client.wait_for_service(timeout_sec=1.0):
            self.matrix_display.append("\nWarning: Parameter server not available. Start it with:\nros2 launch omnivision transform_reconfigure.launch.py")
            self.param_servers_connected = False
            return

        # Create parameter message for transformation server
        transform_param_value = ParameterValue()
        transform_param_value.type = ParameterType.PARAMETER_DOUBLE
        transform_param_value.double_value = self.yaw_angle

        transform_param = Parameter()
        transform_param.name = 'yaw_angle'
        transform_param.value = transform_param_value

        transform_request = SetParameters.Request()
        transform_request.parameters = [transform_param]

        # Send request to transformation server
        self.transform_param_client.call_async(transform_request)

        # Create parameter message for mask server
        mask_param_value = ParameterValue()
        mask_param_value.type = ParameterType.PARAMETER_DOUBLE
        mask_param_value.double_value = self.yaw_angle  # Assuming same parameter for simplicity

        mask_param = Parameter()
        mask_param.name = 'yaw_angle'
        mask_param.value = mask_param_value

        mask_request = SetParameters.Request()
        mask_request.parameters = [mask_param]

        # Send request to mask server
        self.mask_param_client.call_async(mask_request)

        self.param_servers_connected = True

        # Update status
        self.matrix_display.append(f"\nApplied yaw angle: {self.yaw_angle:.1f}° to both servers")

    def reset_settings(self):
        self.yaw_angle = 0.0
        self.yaw_slider.setValue(0)
        self.yaw_spin.setValue(0.0)
        self.yaw_label.setText("Yaw: 0.0°")
        self.update_matrix_display()

        # Apply the reset if connected to parameter servers
        if self.param_servers_connected:
            self.apply_settings()

    def spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0.01)

    def closeEvent(self, event):
        # Cleanup ROS node when closing the window
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()

def main():
    app = QApplication(sys.argv)
    gui = TransformationAdjusterGUI()
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
