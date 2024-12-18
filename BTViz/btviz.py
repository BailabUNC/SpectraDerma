
import sys
import logging
import asyncio
import argparse
from collections import deque
from datetime import datetime

from bleak import BleakScanner, BleakClient
from PyQt5 import QtWidgets, QtCore
from pyqtgraph import PlotWidget, mkPen
import pyqtgraph as pg
from qasync import QEventLoop, asyncSlot

from datastore.SQLiteDatabase import SQLiteDatabase
from datastore.DBHandler import DBHandler

logger = logging.getLogger(__name__)

MAX_RETRIES = 5  # number of reconn retries before give up 

class BTVizApp(QtWidgets.QMainWindow):
    
    def __init__(self, no_recording: bool):
        super().__init__()
        self.setWindowTitle('BTViz')
        self.resize(800, 600)

        # BLE and UI states
        self.device_name = ''
        self.client = None
        self.characteristic_uuid = None
        self.data_buffer = []
        self.plots = []
        self.curves = []
        self.timestamps = []
        self.connected = False
        self.no_recording = no_recording

        # initialize UI and database
        self.init_ui()

        if not self.no_recording:
            current_time = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
            self.db = SQLiteDatabase(f"{current_time}.db")
            self.dbHandler = DBHandler(database=self.db)
        else:
            self.db = None
            self.dbHandler = None

    def init_ui(self):
        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout_ = QtWidgets.QVBoxLayout(self.central_widget)

        # Top input section
        self.top_layout = QtWidgets.QHBoxLayout()
        self.layout_.addLayout(self.top_layout)

        # Device name input
        self.device_name_input = QtWidgets.QLineEdit()
        self.device_name_input.setPlaceholderText("Enter BLE Device Name")
        self.device_name_input.setText("SpectraDerma")
        self.top_layout.addWidget(QtWidgets.QLabel('Device Name:'))
        self.top_layout.addWidget(self.device_name_input)

        # Connect button
        self.connect_button = QtWidgets.QPushButton('Connect')
        self.connect_button.clicked.connect(self.on_connect_button_clicked)
        self.top_layout.addWidget(self.connect_button)

        # Status Label
        self.status_label = QtWidgets.QLabel('Not Connected')
        self.layout_.addWidget(self.status_label)

        # Visualization Area (Disabled initially)
        self.plot_area = QtWidgets.QWidget()
        self.plot_layout = QtWidgets.QVBoxLayout(self.plot_area)
        self.layout_.addWidget(self.plot_area)
        self.plot_area.setEnabled(False)  # Disable until connected

    def on_connect_button_clicked(self):
        """
        Handle the connect button click event
        """
        self.device_name = self.device_name_input.text().strip()
        if not self.device_name:
            QtWidgets.QMessageBox.warning(self, 'Input Error', 'Please enter a BLE device name.')
            return
        
        # Disable input during connection
        self.connect_button.setEnabled(False)
        self.device_name_input.setEnabled(False)
        self.status_label.setText('Connecting...')
        asyncio.ensure_future(self.start_ble())

    @asyncSlot()
    async def start_ble(self):
        num_retry = 0

        while num_retry < MAX_RETRIES and not self.connected:
            try:
                self.status_label.setText(f'Scanning for {self.device_name}... (Attempt {num_retry + 1} / {MAX_RETRIES})')
                devices = await BleakScanner.discover()
                target_device = next((d for d in devices if d.name == self.device_name), None)

                if not target_device:
                    raise Exception(f'Device {self.device_name} not found.')

                self.status_label.setText(f'Connecting to {self.device_name}')
                self.client = BleakClient(target_device, winrt=dict(use_cached_service=False))

                # Attempt connection
                await self.client.connect()
                self.connected = True
                logger.info('Connect to device.')

                # Setup notifications
                await self.setup_notifications()
                self.status_label.setText('Connected and receiving data.')
                await self.client.start_notify(self.characteristic_uuid, self.notification_handler)

                # Monitor connection
                while self.client.is_connected:
                    await asyncio.sleep(1)  # keep the task alive while connected
                logger.warning('Connection lost. Reconnecting...')
                self.connected = False

            except Exception as e:
                num_retry += 1
                logger.error(f'Connection failed: {e}')
                self.status_label.setText(f'Connection failed: {e}. Retrying...')
                await asyncio.sleep(2)

        if not self.connected:
            QtWidgets.QMessageBox.critical(self, 'Error', 'Failed to connect after multiple attempts.')

    async def setup_notifications(self):
        try:
            services = self.client.services
            for service in services:
                for char in service.characteristics:
                    if 'notify' in char.properties:
                        self.characteristic_uuid = char.uuid
                        return
            raise Exception('No notifiable characteristic found.')
        except Exception as e:
            logger.error(f'Failed to set up notifications: {e}')
            raise

    def notification_handler(self, sender, data):
        """
        Handle incoming data from the BLE device.
        """
        try:
            if not self.no_recording:
                self.dbHandler.notification_handler(sender=sender, data=data)
        except Exception as e:
            raise Exception(f"There was an error in inserting the Data into the database: {str(e)}") from e

        try:
            text = data.decode('utf-8').strip()
            values = [float(v) for v in text.split(',')]
        except Exception as e:
            print(f"Failed to parse data: {e}")
            return
        
        if not self.plots:
            # Initialize plots based on number of channels
            num_channels = len(values)
            for i in range(num_channels):
                plot_widget = PlotWidget()
                plot_widget.enableAutoRange(axis='y')
                plot_widget.showGrid(x=True, y=True)
                plot_widget.setTitle(f"Channel {i+1}")
                pen = mkPen(color=pg.intColor(i))
                curve = plot_widget.plot(pen=pen)
                self.layout.addWidget(plot_widget)
                self.plots.append(plot_widget)
                self.curves.append(curve)

                self.data_buffer.append(deque(maxlen=100))
                self.timestamps.append(deque(maxlen=100))

        # Append data and update plots
        timestamp = QtCore.QTime.currentTime().msecsSinceStartOfDay() / 1000.0
        for i, value in enumerate(values):
            self.data_buffer[i].append(value)
            self.timestamps[i].append(timestamp)
            self.curves[i].setData(list(self.timestamps[i]), list(self.data_buffer[i]))

def main():
    parser = argparse.ArgumentParser(description='BTViz - BLE Visulization Tool')
    parser.add_argument(
        '--no-recording',
        action='store_true',
        help='Disable data recording and database saving.'
    )
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    loop = QEventLoop(app)
    asyncio.set_event_loop(loop)

    window = BTVizApp(no_recording=args.no_recording)
    window.show()
    with loop:
        loop.run_forever()

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    main()