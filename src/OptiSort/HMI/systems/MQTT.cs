using MQTTnet.Client;
using System;
using System.Collections.Concurrent;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Text.Json;
using System.Collections.Generic;
using System.Windows.Forms;

namespace OptiSort
{
    public class MQTT
    {
        public string Broker { get; set; }
        public int Port { get; set; }

        private ConcurrentDictionary<string, IMqttClient> _clients = new ConcurrentDictionary<string, IMqttClient>(); // Use ConcurrentDictionary for thread-safe access to MQTT clients
        public event Action<string, JsonElement> MessageReceived; // topic, message
        private byte[] _byteReceived = new byte[1];
        private string _msgReceived = string.Empty;

        public MQTT(string broker, string port) 
        { 
            Broker = broker;
            Port = int.Parse(port);
        }


        /// <summary>
        /// Creates a new MQTT client with the specified ClientID.
        /// </summary>
        /// <param name="clientId">The ID of the client to create.</param>
        /// <returns>True if the client was successfully created; otherwise, false.</returns>
        public async Task<bool> CreateClient(string clientId)
        {
            try
            {
                // Check if the client already exists
                if (_clients.ContainsKey(clientId))
                {
                    Console.WriteLine($"Client '{clientId}' already exists.");
                    return false;
                }

                var mqttFactory = new MQTTnet.MqttFactory();
                var mqttClient = mqttFactory.CreateMqttClient();
                var options = new MqttClientOptionsBuilder()
                    .WithClientId(clientId)
                    .WithTcpServer(Broker, Port)
                    .WithProtocolVersion(MQTTnet.Formatter.MqttProtocolVersion.V500)
                    .WithCleanSession()
                    .Build();

                // Attach event handler for incoming messages
                mqttClient.ApplicationMessageReceivedAsync += (arg) => Client_ApplicationMessageReceivedAsync(clientId, arg);

                await mqttClient.ConnectAsync(options);

                // Thread-safe addition to the dictionary
                if (_clients.TryAdd(clientId, mqttClient))
                {
                    Console.WriteLine($"Client '{clientId}' successfully created and connected.");
                    return true;
                }

                Console.WriteLine($"Failed to add client '{clientId}' to the dictionary.");
                return false;
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error creating client '{clientId}': {ex.Message}");
                return false;
            }
        }


        /// <summary>
        /// Checks if there is a connected client and returns its client ID.
        /// </summary>
        /// <returns>The client ID if a connected client is found; otherwise, null.</returns>
        public string GetConnectedClientName()
        {
            foreach (var client in _clients)
            {
                if (client.Value.IsConnected)
                {
                    return client.Key;
                }
            }
            return null;
        }


        /// <summary>
        /// Handles the message received by a specified MQTT client.
        /// </summary>
        /// <param name="clientId">The ID of the client receiving the message.</param>
        /// <param name="arg">The MQTT application message received event arguments.</param>
        /// <returns>A completed task once the message is processed.</returns>
        private Task Client_ApplicationMessageReceivedAsync(string clientId, MqttApplicationMessageReceivedEventArgs arg)
        {
            try
            {
                if (arg.ReasonCode == MqttApplicationMessageReceivedReasonCode.Success && arg.ApplicationMessage.PayloadSegment.Count > 0)
                {
                    // Extract the topic and message payload
                    string topic = arg.ApplicationMessage.Topic;
                    string msgReceived = Encoding.UTF8.GetString(arg.ApplicationMessage.PayloadSegment.ToArray());

                    // Parse the message as JSON
                    var jsonMessage = JsonDocument.Parse(msgReceived).RootElement;

                    // Raise the event to notify subscribers with the JSON formatted message
                    MessageReceived?.Invoke(topic, jsonMessage);
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error processing message for client '{clientId}': {ex.Message}");
            }

            return Task.CompletedTask;
        }



        /// <summary>
        /// Destroys an MQTT client with the specified ClientID.
        /// </summary>
        /// <param name="clientId">The ID of the client to destroy.</param>
        /// <returns>True if the client was successfully destroyed; otherwise, false.</returns>
        public async Task<bool> DestroyClient(string clientId)
        {
            try
            {
                if (!_clients.ContainsKey(clientId))
                {
                    Console.WriteLine($"Client '{clientId}' does not exist.");
                    return false;
                }

                var client = _clients[clientId];

                // Disconnect and dispose of the client
                await client.DisconnectAsync();
                client.Dispose();

                // Thread-safe removal from the dictionary
                if (_clients.TryRemove(clientId, out _))
                {
                    Console.WriteLine($"Client '{clientId}' successfully disconnected and removed.");
                    return true;
                }

                Console.WriteLine($"Failed to remove client '{clientId}'.");
                return false;
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error destroying client '{clientId}': {ex.Message}");
                return false;
            }
        }



        /// <summary>
        /// Subscribes the specified client to a given topic.
        /// </summary>
        /// <param name="clientId">The ID of the client to subscribe.</param>
        /// <param name="topic">The topic to subscribe to.</param>
        /// <returns>True if the subscription was successful; otherwise, false.</returns>
        public async Task<bool> SubscribeClientToTopic(string clientId, string topic)
        {
            try
            {
                if (!_clients.ContainsKey(clientId))
                {
                    Console.WriteLine($"Client '{clientId}' does not exist.");
                    return false;
                }

                var client = _clients[clientId];
                var topicFilter = new MQTTnet.MqttTopicFilterBuilder().WithTopic(topic).Build();
                var subscribeOptions = new MqttClientSubscribeOptions();
                subscribeOptions.TopicFilters.Add(topicFilter);

                await client.SubscribeAsync(subscribeOptions);
                Console.WriteLine($"Client '{clientId}' successfully subscribed to topic '{topic}'.");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error subscribing client '{clientId}' to topic '{topic}': {ex.Message}");
                return false;
            }
        }



        /// <summary>
        /// Unsubscribes the specified client from a given topic.
        /// </summary>
        /// <param name="clientId">The ID of the client to unsubscribe.</param>
        /// <param name="topic">The topic to unsubscribe from.</param>
        /// <returns>True if the unsubscription was successful; otherwise, false.</returns>
        public async Task<bool> UnsubscribeClientFromTopic(string clientId, string topic)
        {
            try
            {
                if (!_clients.ContainsKey(clientId))
                {
                    Console.WriteLine($"Client '{clientId}' does not exist.");
                    return false;
                }

                var client = _clients[clientId];
                await client.UnsubscribeAsync(topic);
                Console.WriteLine($"Client '{clientId}' successfully unsubscribed from topic '{topic}'.");
                return true;
            }
            catch (Exception ex)
            {
                // Enhanced error logging
                Console.WriteLine($"Error unsubscribing client '{clientId}' from topic '{topic}': {ex.Message}");
                return false;
            }
        }

    }
}
