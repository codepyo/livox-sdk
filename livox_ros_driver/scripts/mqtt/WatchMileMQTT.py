import paho.mqtt.client as mqtt
import json, os, uuid
from mqtt.mqtt_config.MQTTPath import MQTTPath


class WatchMileMQTT:
    def __init__(self, clientID, user, pwd, broker, port, ssl_check=False, notifier=None, protocol=5):
        mqttPath = MQTTPath()

        self.broker = broker  # broker == host
        self.port = port
        self.notifier = notifier
        self.clientID = clientID
        self._topic = ""
        self._isSubscriber = False
        self._paho_mqtt = mqtt.Client(clientID, protocol)
        self.username = user
        self.password = pwd
        self.ssl_check = ssl_check
        if ssl_check:
            self.tls_set = {
                'ca_certs': mqttPath.get_file_path("ca_certificate_wlogs.pem"),
                'certfile': mqttPath.get_file_path("client_mqtt.wlogs.watchmile.com_certificate_wlogs.pem"),
                'keyfile': mqttPath.get_file_path("client_mqtt.wlogs.watchmile.com_key_wlogs.pem")
            }
        self._paho_mqtt.on_connect = self.myOnConnect
        self._paho_mqtt.on_message = self.myOnMessageReceived
        self._paho_mqtt.on_subscribe = self.onSubscribe

        self.received_msg = {}

    def myOnConnect(self, paho_mqtt, userdata, flags, rc):
        print("Connected to %s with result code: %d" % (self.broker, rc))

    def myOnMessageReceived(self, paho_mqtt, userdata, msg):
        # A new message is received

        self.received_msg[msg.topic.split("/")[-2]] = msg.payload
        pay = json.loads(msg.payload.decode('utf-8'))
        if "device_token" in pay :
            print(pay['device_token'])
        # self.notifier(msg.topic, msg.payload)

    def onSubscribe(self, userdata, mid, reasonCodes, properties):
        print("Subscribed to MQTT Topic")

    def publish(self, topic, msg):
        # if needed, you can do some computation or error-check before publishing
        print("publishing '%s' with topic '%s'" % (msg, topic))
        # publish a message with a certain topic
        self._paho_mqtt.publish(topic=topic, payload=json.dumps(msg), qos=0)

    def subscribe(self, topic):
        # if needed, you can do some computation or error-check before subscribing
        print("subscribing to %s" % (topic))
        # subscribe for a topic
        self._paho_mqtt.subscribe(topic, 0)
        # just to remember that it works also as a subscriber
        self._isSubscriber = True
        self._topic = topic

    def setUser(self, pwd = None):
        if pwd is None :
            self._paho_mqtt.username_pw_set(self.username, self.password)
        else:
            self._paho_mqtt.username_pw_set(self.username, pwd)

    def setssl(self):
        self._paho_mqtt.tls_set(self.tls_set['ca_certs'], self.tls_set['certfile'], self.tls_set['keyfile'])

    def start(self):
        # manage connection to broker
        self._paho_mqtt.connect(self.broker, self.port)
        self._paho_mqtt.loop_start()

    def stop(self):
        self._paho_mqtt.loop_stop()
        self._paho_mqtt.disconnect()

