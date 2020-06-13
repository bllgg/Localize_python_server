import paho.mqtt.client as mqttClient
import time
import json
import pandas as pd

def on_connect(client, userdata, flags, rc):
 
    if rc == 0:
 
        print("Connected to broker")
 
        global Connected                #Use global variable
        Connected = True                #Signal connection 
 
    else:
 
        print("Connection failed")
 
def on_message(client, userdata, message):
    msg = message.payload.decode()
    print ("Message received: "  + msg)
    f = open("demofile2.txt", "a")
    f.write("Now the file has more content!")
    f.close()
    
def write_to_csv(data):
    df = pd.DataFrame(data)
    df.to_csv('sensor_data.csv', mode='a', header=False)


 
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

sensor_data=[]
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
