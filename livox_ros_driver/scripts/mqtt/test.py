from mqtt.WatchMileMQTT import WatchMileMQTT
from mqtt.Login_MQTT import login_mqtt
from mqtt.Service_MQTT import mqtt_data_client
import time

topic_class='sensers'

logins = login_mqtt()
service_mqtt_client = mqtt_data_client(logins, topic_class)

client_id = logins.clientID

service_topic = "wlogs/device/service/" +client_id+ "/" + topic_class # /wlogs/device/service/{device_id}/{version}/api/{parkingLotId}/log
i = 0
while i<100 :
    service_mqtt_client.publish(
        topic=service_topic,
        msg={"temp" : i}
    )
    i+=1
    time.sleep(1)
    service_mqtt_client.subscribe(service_topic)

while True :
    service_mqtt_client.publish(
        topic=service_topic,
        msg={"temp" : 1}
    )
    time.sleep(1)



