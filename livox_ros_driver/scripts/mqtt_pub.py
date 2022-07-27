#-*- coding:utf-8 -*-
import paho.mqtt.client as mqtt

#MQTT 클라이언트 생성, 이름은 mypub
mqtt = mqtt.Client("mypub")
#로컬호스트에 있는 MQTT 서버에 TCP 1883 포트로 접속
mqtt.connect("localhost", 1883)

#토픽에 메세지 발행
mqtt.publish("mqtt/myiot/paho", "This is Paho Eclipse Pub client.")
print("The message is published.")
mqtt.loop(2)