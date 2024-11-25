from datetime import datetime
from datastore.SQLiteDatabase import SQLiteDatabase

class DBHandler:
    def __init__(self, database: SQLiteDatabase):
        """
        Initialize with a database instance.
        :param database: An object providing `insert_data` and `query_data` methods.
        """
        self.database = database

    def notification_handler(self, sender, data):
        """Handle incoming BLE data and store it in the database."""
        try:
            text = data.decode('utf-8').strip()
            values = [float(v) for v in text.split(',')]
            self.database.insert_data(values)
        except Exception as e:
            print(f"Failed to handle notification: {e}")
            raise Exception