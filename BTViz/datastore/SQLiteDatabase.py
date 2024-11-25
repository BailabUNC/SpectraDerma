
"""
WRITTEN BY MIHIR SHARMA - NOV 23RD 2024

/datastore is an alternate notification handler function, with the primary focus of storing data.

"""

import sqlite3
from sqlite3 import Error

class SQLiteDatabase:

    # static num_of_channels:
    num_of_channels: int = 12

    def __init__(self, db_path: str):
        if not db_path: 
            raise Exception("DB_Path cannot be null")
        self.connection = None
        self._connect(db_path)

    def _connect(self, db_path):
        """Connect to the SQLite database."""
        try:
            self.connection = sqlite3.connect(db_path)
            cursor = self.connection.cursor()
            # Create a table optimized for time series data
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS data (
                    time DATETIME DEFAULT CURRENT_TIMESTAMP,
                    channel_1 FLOAT,
                    channel_2 FLOAT,
                    channel_3 FLOAT,
                    channel_4 FLOAT,
                    channel_5 FLOAT,
                    channel_6 FLOAT,
                    channel_7 FLOAT,
                    channel_8 FLOAT,
                    channel_9 FLOAT,
                    channel_10 FLOAT,
                    channel_11 FLOAT,
                    channel_12 FLOAT
                )
            """)
            self.connection.commit()
        except Error as e:
            print(f"Failed to connect to database: {e}")

    def insert_data(self, values):
        """Insert a row of data into the database."""
        if len(values) != self.num_of_channels:
            raise ValueError("Exactly 6 channel values are required.")
        try:
            cursor = self.connection.cursor()
            cursor.execute("""
                INSERT INTO data (channel_1, channel_2, channel_3, channel_4, channel_5, channel_6, channel_7, channel_8, channel_9, channel_10, channel_11, channel_12)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, values)
            self.connection.commit()
        except Error as e:
            print(f"Failed to insert data: {e}")

    def query_data(self, start_time=None, end_time=None):
        """Query data within a time range."""
        try:
            cursor = self.connection.cursor()
            query = "SELECT * FROM data WHERE 1=1"
            params = []
            if start_time:
                query += " AND time >= ?"
                params.append(start_time)
            if end_time:
                query += " AND time <= ?"
                params.append(end_time)
            cursor.execute(query, params)
            return cursor.fetchall()
        except Error as e:
            print(f"Failed to query data: {e}")
            return []

    def close_connection(self):
        """Close the database connection."""
        if self.connection:
            self.connection.close()
            self.connection = None

    def __del__(self):
        self.close_connection()