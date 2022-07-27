import json, os
from mqtt.mqtt_config.MQTTPath import MQTTPath

class MQTTConfigBuilder:
    def __init__(self):
        mqttPath = MQTTPath()

        self._src = mqttPath.get_file_path("mqttUUID.json")
        self._user_src = mqttPath.get_file_path("watchmileMQTTUser.json")

    def setAndGetUUIDCode(self, uuid):

        new_json = {"device_id": str(uuid), "device_secret": str(uuid),"product_key": str(uuid) }

        with open(self._src, "w") as json_file:
            json.dump(new_json, json_file)

        return new_json

    def getUUIDCode(self):

        with open(self._src, "r") as json_file:
            auth_json = json.load(json_file)
            json_file.close()

        return auth_json

    def getUserConfig(self, param):

        with open(self._user_src, "r") as json_file:
            user_json = json.load(json_file)
            json_file.close()

        return user_json[param]
