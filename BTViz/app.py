import sys
import logging
import asyncio
from collections import deque
from datetime import datetime

from bleak import BleakScanner, BleakClient
from PyQt5 import QtWidgets, QtCore
from pyqtgraph import PlotWidget, mkPen
import pyqtgraph as pg
from qasync import QEventLoop, asyncSlot

#SQLite3 imports
from datastore.SQLiteDatabase import SQLiteDatabase
from datastore.DBHandler import DBHandler
logger = logging.getLogger(__name__)

class BTVizApp(QtWidgets.QMainWindow):

    # SQLite3 Database instaniation
    # REPLACE DB WITH REAL DB LOCATIONs
    
    

    def __init__(self):
        super().__init__()
        self.setWindowTitle('BTViz')
        self.device_name = ''
        self.client = None
        self.characteristic_uuid = None
        self.data_buffer = []
        self.plots = []
        self.curves = []
        self.timestamps = []
        self.init_ui()
        
        # get current time when the app is initated:
        current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        #db handler!!!
        self.db: SQLiteDatabase = SQLiteDatabase(f"{current_time}.db")
        self.dbHandler: DBHandler = DBHandler(database=self.db)

    def init_ui(self):
        # Create input dialog to get BLE device name
        self.device_name, ok = QtWidgets.QInputDialog.getText(
            self, 'Device Name Input', 'Enter BLE Device Name:', QtWidgets.QLineEdit.EchoMode.Normal, 'SpectraDerma')
        if not ok or not self.device_name:
            QtWidgets.QMessageBox.critical(
                self, 'Error', 'No device name entered. Exiting.')
            sys.exit(1)

        # Set up central widget and layout
        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QtWidgets.QVBoxLayout(self.central_widget)

        # Status label
        self.status_label = QtWidgets.QLabel('Connecting to device...')
        self.layout.addWidget(self.status_label)

        # Start BLE connection
        asyncio.ensure_future(self.start_ble())

    @asyncSlot()
    async def start_ble(self):
        # Scan for devices
        devices = await BleakScanner.discover()
        target_device = None
        for d in devices:
            if d.name == self.device_name:
                target_device = d
                break

        if not target_device:
            QtWidgets.QMessageBox.critical(
                self, 'Error', f'Device {self.device_name} not found.')
            sys.exit(1)

        self.status_label.setText(f'Connecting to {self.device_name}...')

        # Connect to device
        try:
            self.client = BleakClient(target_device, winrt=dict(use_cached_services=False))
            await self.client.connect()

            services = self.client.services
            # Assuming only one custom service and characteristic
            for service in services:
                for char in service.characteristics:
                    if 'notify' in char.properties or 'indicate' in char.properties:
                        self.characteristic_uuid = char.uuid
                        break

            if not self.characteristic_uuid:
                QtWidgets.QMessageBox.critical(
                    self, 'Error', 'No notifiable characteristic found.')
                sys.exit(1)

            self.status_label.setText('Connected. Starting notifications...')
            await asyncio.sleep(1.0)
            await self.client.start_notify(self.characteristic_uuid, self.notification_handler)

        except Exception as e:
            QtWidgets.QMessageBox.critical(
                self, 'Error', f'Failed to connect: {e}')
            print(e)
            sys.exit(1)

    def notification_handler(self, sender, data):
        """
        Handle incoming data from the BLE device.
        """

        # print(data)
        # DB Handler
        try: 
            self.dbHandler.notification_handler(sender=sender, data=data)
        except Exception as e:
            raise Exception("There was an error in inserting the Data into the database: " + str(e))

        # Decode and parse the data
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
    app = QtWidgets.QApplication(sys.argv)
    loop = QEventLoop(app)
    asyncio.set_event_loop(loop)
    window = BTVizApp()
    window.show()
    with loop:
        loop.run_forever()

if __name__ == "__main__":
    main()