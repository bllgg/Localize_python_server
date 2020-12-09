import mysql.connector

ils_db = mysql.connector.connect(
  host="localhost",
  user="root",
  password="",
  database="Localization"
)

ils_cursor = ils_db.cursor(buffered=True)

ils_cursor.execute("SHOW TABLES")

for x in ils_cursor:
    print(x)

sql = "UPDATE position SET building_id = %s, x_position = %s, y_position = %s WHERE device_id = %s"
val = ('1', '2.0', '2.0', '255')
ils_cursor.execute(sql, val)

ils_db.commit()

print(ils_cursor.rowcount, "record(s) affected")

# sql = "SELECT * FROM ESP_device where esp_device = {}".format("24:6F:28:A9:64:C8")
# dev_id = ('24:6F:28:A9:64:C8')
ils_cursor.execute("SELECT x_position, y_position FROM ESP_device where esp_device = \"%s\"" %("24:6F:28:A9:64:C8"))

coords = ils_cursor.fetchone()
x = coords[0]
y = coords[1]
print(x, y)
# for x in myresult:
#   print(x)