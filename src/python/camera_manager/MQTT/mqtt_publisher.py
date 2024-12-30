import paho.mqtt.client as mqtt


class MQTTClient:
    def __init__(self, broker, port, topics):
        self.broker = broker
        self.port = port
        self.topics = topics
        self.client = mqtt.Client()
        self.client.connect(self.broker, self.port)

    def publish(self, topic, message):
        self.client.publish(topic, message)

    def subscribe(self, topic):
        self.client.subscribe(topic)

    def loop(self):
        self.client.loop_start()
