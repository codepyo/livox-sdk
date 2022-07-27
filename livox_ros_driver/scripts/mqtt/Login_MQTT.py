from mqtt.WatchMileMQTT import *
from mqtt.MQTTConfigBuilder import *
from mqtt.mqtt_config.MQTTPath import MQTTPath
import time, datetime

def timeout(i) :
    if i <= 1:
        print("Wait for Response : " + str(i) + " second")
    else:
        print("Wait for Response : " + str(i) + " seconds")
    time.sleep(1)


def returnMQTTClient(topic_class,client_id,user_auth,ssl_ft=False) :
    mqtt_client = WatchMileMQTT(
        clientID=client_id,
        broker=user_auth['host'],
        port=user_auth['port'],
        user=user_auth['username'],
        pwd=user_auth['password'],
        ssl_check= ssl_ft
    )
    return mqtt_client

def login_mqtt ():
    mqttPath = MQTTPath()

    topic_class = "login"

    config = MQTTConfigBuilder()
    payload = config.getUUIDCode()
    user_auth = config.getUserConfig(topic_class)
    client_id = "wlogs:kor:wlogsORG:embedded-sg20:" + payload['device_id'] + ":" + str(datetime.datetime.now().timestamp())

    mqtt_client = returnMQTTClient(topic_class, client_id, user_auth,ssl_ft=True)

    mqtt_client.setUser()
    mqtt_client.setssl()
    mqtt_client.start()

    login_topic = "wlogs/device/auth/" + client_id + "/" + topic_class
    mqtt_client.publish(
        topic=login_topic,
        msg=payload
    )

    mqtt_client.subscribe(topic=login_topic + "/result")

    i = 0
    while len(mqtt_client.received_msg.keys()) < 1:
        if i > 20 :
            break
        timeout(i)
        i += 1

    mqtt_client.stop()
    return mqtt_client



