from SQLiteDatabase import SQLiteDatabase
from DBHandler import DBHandler

def main():
    # Dependency Injection
    db = SQLiteDatabase(db_path="time_series_data.db")
    data_handler = DBHandler(database=db)

    # Example usage
    # Simulate BLE notification
    fake_data = b"0.1,0.2,0.3,0.4,0.5,0.6"  # Simulating incoming BLE data
    data_handler.notification_handler(sender=None, data=fake_data)

    # Query and print data for verification
    results = db.query_data()
    for row in results:
        print(row)

    # Cleanup
    db.close_connection()

if __name__ == "__main__":
    main()