#-*- coding:utf-8 -*-
import paho.mqtt.client as mqtt
from mqtt.WatchMileMQTT import WatchMileMQTT
from mqtt.Login_MQTT import login_mqtt
from mqtt.Service_MQTT import mqtt_data_client
import time

#MQTT 클라이언트가 MQTT 서버에 정상 접속된 후 CONNACK 응답을 받음.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    #client.subscribe("$SYS/#")
    client.subscribe("wlogs/device/service/wlogs:kor:wlogsORG:embedded-sg20:137fcf3c-70b7-4b81-835e-c64518dab3fc:1658896328.073243/lidar/sktv1/inout" + "/result")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
	print(msg.topic+" "+str(msg.payload))

location = "skv1"
topic_class='lidar'
logins = login_mqtt()
service_mqtt_client = mqtt_data_client(logins, topic_class)

client_id = logins.clientID
service_topic = "wlogs/device/service/" +client_id+ "/" + topic_class + "/" + location + "/inout"
#service_mqtt_client._paho_mqtt.on_connect = on_connect
#service_mqtt_client._paho_mqtt.on_message = on_message

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
#service_mqtt_client._paho_mqtt.loop_forever()
while True :
    service_mqtt_client.publish(
        topic=service_topic,
        msg={"temp" : 1}
    )
    time.sleep(1)