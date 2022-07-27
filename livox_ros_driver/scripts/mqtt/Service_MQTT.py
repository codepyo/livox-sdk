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

def mqtt_data_client(login_mqtt_f, topic_class) : # login_mqtt_f = login_mqtt()

    mqtt_login = login_mqtt_f
    mqttPath = MQTTPath()

    config = MQTTConfigBuilder()
    payload = config.getUUIDCode()
    user_auth = config.getUserConfig(topic_class)

    user_auth['password'] = json.loads(list(mqtt_login.received_msg.values())[0].decode('utf-8'))['device_token']

    client_id = mqtt_login.clientID

    mqtt_client = returnMQTTClient(topic_class, client_id, user_auth,ssl_ft=True)

    mqtt_client.setUser()
    mqtt_client.setssl()
    mqtt_client.start()

    return mqtt_client


def publish_pks_data(client, topic, result, parkingFloor) :
    count = len(result['pks-id-list'])
    result_list= []
    for i in range(count):
        created = datetime.datetime.now().timestamp()
        updated = datetime.datetime.now().timestamp()
        return_params = {
            "region": result['region'],
            "pkName": result['pk-name'],
            "pkFloor" : parkingFloor,
            "cctvId": result['cctv-id'],
            "pksId": result['pks-id-list'][i],
            "ROIBoxCoord": ",".join(map(str, result['roi-box-coord-list'][i])),
            "predictedByPerson": "",
            "modelName": result['model-name'],
            "modelVersion": result['model-version'],
            "created": created,
            "updated": updated,
            "indoorDivision": "",
            "weatherForecast": ""
        }
        if result['img-path']:
            return_params['imgPath'] = result['img-path']
        if result['img-base64'] :
            return_params['imgBase64'] = str(result['img-base64'])
        if result['predicted-by-model-list']:
            if result['predicted-by-model-list'][i] == 'free':
                return_params['predictedByModel'] = 0
            else:
                return_params['predictedByModel'] = 1
        if result['predicted-by-model-list-v1']:
            if result['predicted-by-model-list-v1'][i] == 'free':
                return_params['predictedByModelV1'] = 0
            else:
                return_params['predictedByModelV1'] = 1
        if result['predicted-by-model-list-v2']:
            if result['predicted-by-model-list-v2'][i] == 'free':
                return_params['predictedByModelV2'] = 0
            else:
                return_params['predictedByModelV2'] = 1
        if result['bounding-box-coord-list']:
            return_params['boundingBoxCoordList'] = ".".join([",".join(map(str, s)) for s in result['bounding-box-coord-list']])
        result_list.append(return_params)
    client.publish(topic, {"pkslog" : result_list})

def check_token_is_expired(client) :
    #client = service_mqtt_client
    service_topic = "wlogs/device/auth/" + client.clientID + "/reflash"
    client.publish(
        topic=service_topic,
        msg={"msg": "hello 3"}
    )
    client.received_msg = {}
    client.subscribe(service_topic + "/result")

    i = 0
    while len(client.received_msg.keys()) < 1:
        if i > 20 :
            break
        timeout(i)
        i += 1
    result_pwd = json.loads(list(client.received_msg.values())[0].decode('utf-8'))['device_token']
    return result_pwd
