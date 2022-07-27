from sklearn.model_selection import StratifiedShuffleSplit
from mqtt.WatchMileMQTT import *
from mqtt.MQTTConfigBuilder import MQTTConfigBuilder
from mqtt.mqtt_config.MQTTPath import MQTTPath
import time, uuid

def timeout(i) :
    if i <= 1:
        print("Wait for Response : " + str(i) + " second")
    else:
        print("Wait for Response : " + str(i) + " seconds")
    time.sleep(1)


def returnMQTTClient(topic_class,client_id,user_auth, ssl_ft = False) :
    mqtt_client = WatchMileMQTT(
        clientID=client_id,
        broker=user_auth['host'],
        port=user_auth['port'],
        user=user_auth['username'],
        pwd=user_auth['password'],
        ssl_check= ssl_ft
    )
    print(ssl_ft)
    return mqtt_client



def registration_mqtt() :
    topic_class = "registration"
    config = MQTTConfigBuilder()
    payload = config.getUUIDCode()
    user_auth = config.getUserConfig(topic_class)
    client_id = "wlogs:kor:lidarORG:lidar-horizon:" + payload['device_id']


    mqtt_client = returnMQTTClient(topic_class,client_id,user_auth,ssl_ft=True)

    mqtt_client.setUser()
    mqtt_client.start()

    reg_topic = "wlogs/device/reg/" + client_id + "/" + topic_class

    mqtt_client.publish(
        topic= reg_topic,
        msg = payload
    )
    mqtt_client.subscribe(topic = reg_topic + "/result")

    print("\nbonfire check 'connected_on'\n")

    ssl_topic = "wlogs/device/reg/" + client_id + "/ssl"
    mqtt_client.publish(
        topic =ssl_topic,
        msg = payload
    )
    mqtt_client.subscribe(ssl_topic + "/result")

    i = 0
    while len(mqtt_client.received_msg.keys()) < 2 :
        if i > 10 :
            break
        timeout(i)
        i+=1

    mqtt_client.stop()
    return mqtt_client


def registration_pem() :
    mqttPath = MQTTPath()

    mqtt_reg = registration_mqtt()

    ssls = json.loads(mqtt_reg.received_msg['ssl'].decode('utf-8'))

    for ssl in ssls.keys() :
        if ssl == "ca_cert" :
            file_name = "ca_certificate_wlogs.pem"
        elif ssl == "client_cert" :
            file_name = "client_mqtt.wlogs.watchmile.com_certificate_wlogs.pem"
        elif ssl == "client_key" :
            file_name = "client_mqtt.wlogs.watchmile.com_key_wlogs.pem"
        else :
            print("Invalid SSL Keys..")
            break
        f = open(mqttPath.get_file_path(file_name),"w") # file_name -> mqttPath.get
        f.write(ssls[ssl].replace("\r",""))
        f.close()


if __name__ == "__main__" :
    #builder = MQTTConfigBuilder()
    #builder.setAndGetUUIDCode(uuid.uuid1())
    registration_pem()


    # bonfire link : https://api.wlogs.watchmile.com/wlogs_devices/index
