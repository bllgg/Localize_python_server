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
 
def on_message(client, userdata, message):
    print ("Message received: "  + message.payload.decode())
    value = json.loads(message.payload.decode())
    '''
    print (value["seq_num"])
    print (value["dev_id"])
    print (value["tx_pow"])
    print (value["RSSI"])
    print (value["MAC"])
    print (value["acc_x"])
    print (value["gyro_y"])
    print (value["mag_z"])
    '''

 
Connected = False   #global variable for the state of the connection
'''
broker_address= "m11.cloudmqtt.com"  #Broker address
port = 17595                         #Broker port
user = "twhkvnkt"
password = "0qLBb25EOa3T"            #Connection password
'''

broker_address= "192.168.8.100"  #Broker address
port = 1883                        #Broker port
user = "twhkvnkt"
password = "0qLBb25EOa3T"            #Connection password
 
client = mqttClient.Client("Python_sub")               #create new instance
client.username_pw_set(user, password=password)    #set username and password
client.on_connect= on_connect                      #attach function to callback
client.on_message= on_message                      #attach function to callback
 
client.connect(broker_address, port=port)          #connect to broker
 
client.loop_start()        #start the loop
 
while Connected != True:    #Wait for connection
    time.sleep(0.1)
 
client.subscribe("python/test",2)
 
try:
    while True:
        time.sleep(1)
 
except KeyboardInterrupt:
    print ("exiting")
    client.disconnect()
    client.loop_stop()
