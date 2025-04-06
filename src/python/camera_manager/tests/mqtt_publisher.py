import paho.mqtt.client as mqtt


def on_disconnect(client, userdata, rc):
    print(f"Disconnected from broker. Reason: {rc}")


def get_connect_error_message(rc):
    error_messages = {
        0: "Connection successful.",
        1: "Incorrect protocol version.",
        2: "Invalid client identifier.",
        3: "Server unavailable.",
        4: "Bad username or password.",
        5: "Not authorized."
    }
    return error_messages.get(rc, "Unknown error.")


def on_message(client, userdata, msg):
    print(f"Received message on {msg.topic}: {msg.payload.decode()}")


class MQTTClient:
    def __init__(self, broker, port, client_name, topics):
        self.broker = broker
        self.port = int(port)
        self.client_name = client_name
        self.topics = topics
        self.client = mqtt.Client(client_name)

        # Set callbacks for connect, disconnect, and message
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = on_disconnect
        self.client.on_message = on_message

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print(f"Connected to MQTT broker: {self.broker}")
            # Subscribe to topics after connection
            for topic in self.topics:
                self.client.subscribe(topic)
        else:
            print(f"Failed to connect with result code {rc}. Error description: {get_connect_error_message(rc)}")

    def connect(self):
        try:
            # Attempt to connect to the MQTT broker
            self.client.connect(self.broker, self.port, 60)
            # Start a loop to handle incoming messages and ensure a successful connection
            self.client.loop_start()
        except Exception as e:
            print(f"Error connecting to MQTT broker: {e}")

    async def publish(self, topic, message):
        if self.client.is_connected():
            # Publish a message to the given topic
            self.client.publish(topic, message)
        else:
            print("Cannot publish. Client is not connected to the broker.")

    def disconnect(self):
        self.client.disconnect()
        self.client.loop_stop()
