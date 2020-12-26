import paho.mqtt.client as mqttClient
import time
import json
import pandas as pd
import csv
k = 0

def on_connect(client, userdata, flags, rc):
 
    if rc == 0:
 
        print("Connected to broker")
 
        global Connected                #Use global variable
        Connected = True                #Signal connection 
 
    else:
 
        print("Connection failed")
 
def on_message(client, userdata, message):
    msg = message.payload.decode()
    #print ("Msg received")#, k)
    print (msg)
    # k+=1
    value = json.loads(message.payload.decode())
    # print(type(value))
    with open('data.csv', 'a', newline='') as f:  # Just use 'w' mode in 3.x
        w = csv.writer(f)
        w.writerow(value.values())

 
Connected = False   #global variable for the state of the connection

# broker_address= "m11.cloudmqtt.com"  #Broker address
# port = 17595                         #Broker port
# user = "twhkvnkt"
# password = "0qLBb25EOa3T"            #Connection password

broker_address= "192.168.9.230"  #Broker address
port = 1883                         #Broker port
user = "twhkvnkt"
password = "0qLBb25EOa3T"            #Connection password


# broker_address= "192.168.43.125"  #Broker address
# port = 1883                         #Broker port
# user = "mob"
# password = "asdfghjk"            #Connection password
 
client = mqttClient.Client("Python_sub")               #create new instance
client.username_pw_set(user, password=password)    #set username and password
client.on_connect= on_connect                      #attach function to callback
client.on_message= on_message                      #attach function to callback
 
client.connect(broker_address, port=port)          #connect to broker
 
client.loop_start()        #start the loop


while Connected != True:    #Wait for connection
    time.sleep(0.1)
 
client.subscribe("/topic/esp1",2)
client.subscribe("/topic/esp2",2)
client.subscribe("/topic/esp3",2)
# client.subscribe("/topic/qos1",2)

try:
    while True:
        time.sleep(1)
 
except KeyboardInterrupt:
    print ("exiting")
    client.disconnect()
    client.loop_stop()
