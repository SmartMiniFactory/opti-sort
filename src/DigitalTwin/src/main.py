import threading
import time
import requests
import json
import paho.mqtt.client as mqtt
import websockets
import queue
import numpy as np
from utils.MQTT_broker_address import MQTT_broker_data
import websockets.sync.client

class IPhysics():
    def __init__(self, uri):
        self.session = requests.Session()  # Define an HTML session
        self.uri = uri
        self.ws = None
        self.running = True
        self.joint_positions = [0, 0, 0, 0]

        # Queue to handle incoming WebSocket requests from MQTT thread and Main
        self.ws_queue = queue.Queue()

        # Start WebScoket thread
        self.ws_thread = threading.Thread(target=self.webSocketWorker, daemon=True)
        self.ws_thread.start()

        # Initialise and start MQTT on another thread
        self.client = mqtt.Client()
        self.client.on_connect = self.onConnect
        self.client.on_message = self.onMessage
        self.client.connect(MQTT_broker_data.HOST, MQTT_broker_data.PORT)
        self.client.subscribe("DT_BROADCAST", 0)
        self.client.loop_start()  # Start MQTT thread in background

    def webSocketWorker(self):
        while self.running:
            try:
                print("🔗 Trying WebSocket connection...")
                with websockets.sync.client.connect(self.uri) as ws:

                    self.ws = ws
                    print("✅ WebSocket successfully connected!")

                    while self.running:
                        try:
                            request = self.ws_queue.get(timeout=1)  # Wait for new requests
                            ws.send(json.dumps(request))  # Send request to WebSocket server
                            print(f"📤 Request {request['id']} sent: {request}")
                        except queue.Empty:
                            continue  # If no requests, move on
            except websockets.exceptions.ConnectionClosed:
                print("⚠️ Lost WebSocket connection, reconnection in 5 seconds...")
                time.sleep(5)  # Wait before reconnecting

    def sendRequest(self, request_id, method, url, body=None, headers=None):
        request = {
            "id": request_id,
            "method": method,
            "url": url,
            "body": json.dumps(body) if body else None,
            "headers": headers or {}
        }
        self.ws_queue.put(request)  # Add the request to the queue

    def onConnect(self, client, userdata, flags, reason_code):
        if reason_code == 0:
            print("🔗 Connected to MQTT broker")
        else:
            print(f"⚠️ Error MQTT: {reason_code}")

    def onMessage(self, client, userdata, message):
        msg = json.loads(message.payload)

        if msg["receiver"] == "IPHYSICS" and msg["command"] == 10:
            self.joint_positions = [
                msg["payload"]["q1"],
                msg["payload"]["q2"],
                msg["payload"]["q3"],
                msg["payload"]["q4"]
            ]

    def stop(self):
        self.running = False
        self.ws_thread.join()
        self.client.loop_stop()
        print("❌ WebSocket e MQTT threads stopped.")

def main():
    IPH = IPhysics("ws://localhost:8180")

    try:
        while True:
            IPH.sendRequest(10, "PUT", "/io/COBRA 600 CAD MODEL/BASE ASSY-1/tQ1",
                            {"value": np.deg2rad(IPH.joint_positions[0])}) # Shoulder
            IPH.sendRequest(20, "PUT", "/io/COBRA 600 CAD MODEL/BASE ASSY-1/tQ2",
                            {"value": np.deg2rad(IPH.joint_positions[1])}) # Wrist
            IPH.sendRequest(30, "PUT", "/io/COBRA 600 CAD MODEL/BASE ASSY-1/tQ3",
                            {"value": IPH.joint_positions[2]}) # Arm
            IPH.sendRequest(40, "PUT", "/io/COBRA 600 CAD MODEL/BASE ASSY-1/tQ4",
                            {"value": np.deg2rad(IPH.joint_positions[3])}) # Hand

            time.sleep(0.05)  # Wait before sending new requests
    except KeyboardInterrupt:
        print("\n🛑 Stopping...")
        IPH.stop()

if __name__ == '__main__':
    main()
