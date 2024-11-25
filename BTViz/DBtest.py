from datastore.SQLiteDatabase import SQLiteDatabase
from datastore.DBHandler import DBHandler

def main():
    # Dependency Injection
    file_path = input("Type in the file path: (including .db)")
    db = SQLiteDatabase(file_path)
    data_handler = DBHandler(database=db)

    # Query and print data for verification
    results = db.query_data()
    for row in results:
        print(row)

    # Cleanup
    db.close_connection()

if __name__ == "__main__":
    main()