import paho.mqtt.client as mqttClient
import time
import json
 
def on_connect(client, userdata, flags, rc):
 
    if rc == 0:
 
        print("Connected to broker")
 
        global Connected                #Use global variable
        Connected = True                #Signal connection 
 
    else:
 
        print("Connection failed")
 
Connected = False   #global variable for the state of the connection
'''
broker_address= "m11.cloudmqtt.com"
port = 17595
user = "twhkvnkt"
password = "0qLBb25EOa3T"
'''
broker_address= "192.168.8.100"  #Broker address
port = 1883                         #Broker port
user = "twhkvnkt"
password = "0qLBb25EOa3T"            #Connection password
 
client = mqttClient.Client("Python_pub")               #create new instance
client.username_pw_set(user, password=password)    #set username and password
client.on_connect= on_connect                      #attach function to callback
client.connect(broker_address, port=port)          #connect to broker
 
client.loop_start()        #start the loop
 
while Connected != True:    #Wait for connection
    time.sleep(0.1)
 
try:
    i = 0
    while True:
        #value = input('Enter the message:')
        value = x = {"seq_num": i, "dev_id": 30, "tx_pow": 0, "RSSI": -37, "MAC": 5008, "acc_x": i+1, "acc_y": i+2, "acc_z": i+3, "gyro_x": i+1, "gyro_y": i+2, "gyro_z": i+3, "mag_x": i+0.1, "mag_y": i+0.2, "mag_z": i+0.3 }
        y = json.dumps(x)
        client.publish("python/test",y,2)
        i += 1
        time.sleep(2)
 
except KeyboardInterrupt:
 
    client.disconnect()
    client.loop_stop()
