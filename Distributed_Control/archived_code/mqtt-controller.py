import sys
import paho.mqtt.client as mqtt
import time
import datetime

next_pos = 0

# def controller_algo():
#     next_pos = next_pos+1

#     #TODO: control algorithm goes here

#     return next_pos

with open("output_mqtt_controller.txt", 'w') as f:
    sys.stdout = f

    def on_log(client, userdata, level, buf):
        now = datetime.datetime.now()
        f.write(str(now.time()))
        f.write(" log: "+buf+'\n')

    def on_connect(client, userdata, flags, rc):
        if rc==0:
            now = datetime.datetime.now()
            f.write(str(now.time()))
            f.write(" Connection ok"+'\n')
        else:
            now = datetime.datetime.now()
            f.write(str(now.time()))
            f.write(" Bad connection, exited with code "+str(rc)+'\n')

    def on_message(client, userdata, msg):
        topic = msg.topic
        msg_decode = str(msg.payload.decode("utf-8"))
        now = datetime.datetime.now()
        f.write(str(now.time()))
        f.write(" Message received: "+msg_decode+'\n')

    # def on_disconnect(client, userdata, flags, rc=0):
    #     now = datetime.datetime.now()
    #     f.write(str(now.time()))
    #     f.write(" Disconnection with code "+str(rc))+'\n'


    broker = "test.mosquitto.org"

    client = mqtt.Client("controller1")

    client.on_connect = on_connect
    client.on_log = on_log
    client.on_message = on_message
    # client.on_disconnect = on_disconnect

    now = datetime.datetime.now()
    f.write(str(now.time()))
    f.write(" Connecting to broker "+broker+'\n')

    client.connect(broker)
    client.loop_start()
    while True:
       
        client.subscribe("/robot/position")
        
        #TEST - Uncomment to debug
        # next_pos = next_pos + 1
        # ctrl_cmd = next_pos #controller_algo()

        client.publish("/robot/command", "Control command received: "+str(ctrl_cmd))
        
        time.sleep(4)
        f.flush()

            # client.loop_stop()
            # client.disconnect()
            
            # f.close()