using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.Sockets;
using System.Net;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.Remoting.Messaging;

namespace FlexibowlLibrary
{
    public class Flexibowl
    {

        public static UdpClient UdpClient {  get; private set; }
        public static IPEndPoint Endpoint { get; private set; }
        public IPAddress IP { get; set; }


        public Flexibowl(string flexibowlIp)
        {
            UdpClient = new UdpClient(5001);
            IP = IPAddress.Parse(flexibowlIp);
        }


        /// <summary>
        /// Connecting the flexibowl through UDP connection
        /// </summary>
        /// <param name="ipAddress"></param>
        /// <returns></returns>
        public bool Connect()
        {
            Endpoint = new IPEndPoint(IP, 5001);

            try
            {
                UdpClient.Connect(Endpoint);
                UdpClient.Client.SendTimeout = 500;
                UdpClient.Client.ReceiveTimeout = 500;
                Console.Write("Connected to flexibowl");
                return true;
            }
            catch (ArgumentNullException ex)
            {
                Console.Write($"ArgumentNullException: {ex}");
                return false;
            }
        }


        /// <summary>
        /// Disconnect flexibowl from UDP connection
        /// </summary>
        /// <param name="client"></param>
        public bool Disconnect()
        {
            Endpoint = null;

            try
            {
                UdpClient.Dispose();
                UdpClient.Close();
                Endpoint = null;
                Console.Write("Disconnected from FlexiBowl");
                return true;
            }
            catch (Exception ex)
            {
                Console.Write($"Unable to disconnect from FlexiBowl: {ex}");
                return false;
            }
        }

        // ----------------------------------------------------------------------------------
        // --------------------------------- SUPPORT ----------------------------------------
        // ----------------------------------------------------------------------------------


        /// <summary>
        /// Check Flexible availability to receive commands
        /// </summary>
        /// <param name="client"></param>
        /// <param name="endpoint"></param>
        /// <returns></returns>
        public static bool isBusy(UdpClient client, IPEndPoint endpoint) 
        {

            // TODO: is this repeating the "send command" method??

            string receiveString = "";
            int byteSent = 0;
            bool available = false;
            do
            {
                // Convert the command to bytes
                Byte[] SCLstring = Encoding.ASCII.GetBytes("ob[4]");
                Byte[] sendBytes = new Byte[SCLstring.Length + 1];

                Array.Copy(SCLstring, 0, sendBytes, 0, SCLstring.Length);
                sendBytes[sendBytes.Length - 1] = 13; // CR

                // Send the command to the server
                byteSent = client.Send(sendBytes, sendBytes.Length);

                // Receive data from the server
                Byte[] receivedData = client.Receive(ref endpoint);
                receiveString = Encoding.ASCII.GetString(receivedData);

                // Define a separator byte
                byte separator = 13;

                // Convert byte array to a list of bytes
                List<byte> byteList = new List<byte>(receivedData);

                // Create a list to hold the divided byte arrays
                List<byte[]> dividedByteArrays = new List<byte[]>();

                int lastSeparatorIndex = 0;
                for (int i = 0; i < byteList.Count; i++)
                {
                    if (byteList[i] == separator)
                    {
                        // Get the range of bytes from the last separator index to the current index
                        byte[] dividedArray = byteList.GetRange(lastSeparatorIndex, i - lastSeparatorIndex).ToArray();
                        dividedByteArrays.Add(dividedArray);

                        // Update the last separator index
                        lastSeparatorIndex = i + 1;
                    }
                }

                // Handle the case where the byte array does not end with a separator
                if (lastSeparatorIndex != byteList.Count)
                {
                    byte[] dividedArray = byteList.GetRange(lastSeparatorIndex, byteList.Count - lastSeparatorIndex).ToArray();
                    dividedByteArrays.Add(dividedArray);
                }

                // Convert the second byte array to a string
                string answer = Encoding.ASCII.GetString(dividedByteArrays[1]);

                bool.TryParse(answer, out available);

            } while (available == true);

            return available;
        }


        /// <summary>
        /// Send a command to the Flexibowl over UDP protocol
        /// </summary>
        /// <param name="client"></param>
        /// <param name="endpoint"></param>
        /// <param name="command"></param>
        /// <returns></returns>
        public static string SendCommand(string command) 
        {
            string receiveString = "";
            int byteSent = 0;

            // Convert the command to bytes
            Byte[] SCLstring = Encoding.ASCII.GetBytes(command);
            Byte[] sendBytes = new Byte[SCLstring.Length + 1];

            System.Array.Copy(SCLstring, 0, sendBytes, 0, SCLstring.Length);
            sendBytes[sendBytes.Length - 1] = 13; // CR

            // Send the command to the server
            byteSent = UdpClient.Send(sendBytes, sendBytes.Length);
            //Console.WriteLine("\nCommand sent");

            // Receive data from the server
            IPEndPoint ep = Endpoint;
            Byte[] receivedData = UdpClient.Receive(ref ep);

            receiveString = Encoding.ASCII.GetString(receivedData);

            return receiveString;
        }

        // ----------------------------------------------------------------------------------
        // --------------------------------- SUPPORT ----------------------------------------
        // ----------------------------------------------------------------------------------

        // TODO: improve with return strings instead of voids

        public static class Set
        {
            /// <summary>
            /// Enable or disable servo
            /// </summary>
            /// <param name="status">true = on; false = off</param>
            public static void Servo(bool status)
            {
                string cmd;

                if (status == true)
                    cmd = "servo=1";
                else
                    cmd = "servo=0";

                string rsp = SendCommand(cmd);
                Console.WriteLine("\nFlexibowl: setting servo to " + status.ToString());
                
                while (isBusy(UdpClient, Endpoint))
                {
                    System.Threading.Thread.Sleep(50);
                }
            }

            /// <summary>
            /// Turn on or off the backlight for contrast enhancement
            /// </summary>
            /// <param name="status">true = on; false = off</param>
            public static void Light(bool status) 
            {
                string cmd;

                if (status == true)
                    cmd = "light=1";
                else
                    cmd = "light=0";

                string rsp = SendCommand(cmd);
                Console.WriteLine("\nFlexibowl: setting light to " + status.ToString());
                
                while (isBusy(UdpClient, Endpoint))
                {
                    System.Threading.Thread.Sleep(50);
                }
            }


            public static class Rotation
            {
                /// <summary>
                /// Set the rotational speed in RPM, at which the feeder will advance at each subsequent “forward=1” instruction.
                /// </summary>
                /// <param name="speed">RPM, between 1 and 130</param>
                public static void Speed(int speed) 
                {
                    if ((speed < 1) && (speed > 130))
                        Console.WriteLine("Error: rotational speed out of range (1-130)");
                    else
                    {
                        string cmd = "speed=" + speed.ToString();
                        string rsp = SendCommand(cmd);
                        Console.WriteLine("\nFlexibowl: setting rotational speed");
                    }
                    System.Threading.Thread.Sleep(50);
                }


                /// <summary>
                /// Set the rotational acceleration used for each subsequent “forward=1” instruction
                /// </summary>
                /// <param name="acc">Between 10 and 10000</param>
                public static void Acceleration(int acc) 
                {
                    if ((acc < 10) && (acc > 10000))
                        Console.WriteLine("Error: rotational acceleration out of range (10-10000)");
                    else
                    {
                        string cmd = "acc=" + acc.ToString();
                        string rsp = SendCommand(cmd);
                        Console.WriteLine("\nFlexibowl: setting rotational acceleration");
                    }
                    System.Threading.Thread.Sleep(50);
                }


                /// <summary>
                /// Set the rotational deceleration used for each subsequent “forward=1” instruction
                /// </summary>
                /// <param name="dec">Between 10 and 10000</param>
                public static void Deceleration(int dec)
                {
                    if ((dec < 10) && (dec > 10000))
                        Console.WriteLine("Error: rotational deceleration out of range (10-10000)");
                    else
                    {
                        string cmd = "dec=" + dec.ToString();
                        string rsp = SendCommand(cmd);
                        Console.WriteLine("\nFlexibowl: setting rotational deceleration");
                    }
                    System.Threading.Thread.Sleep(50);
                }


                /// <summary>
                /// Set the angular step by which the feeder will advance at each subsequent “forward=1” instruction.
                /// </summary>
                /// <param name="angle">Positive values => clockwise rotation; Negative values => counter-clockwise rotation</param>
                public static void Angle(int angle)
                {
                    string cmd = "angle=" + angle.ToString();
                    string rsp = SendCommand(cmd);
                    Console.WriteLine("\nFlexibowl: setting rotational angle");
                    System.Threading.Thread.Sleep(50);
                }
            }

            public static class Shake
            {
                /// <summary>
                /// Set the shaking speed at which to shake the feeder at each subsequent “shake = 1” instruction
                /// </summary>
                /// <param name="speed">In RPM, between 1 and 130</param>
                public static void Speed(int speed)
                {
                    if ((speed < 1) && (speed > 130))
                        Console.WriteLine("Error: shaking speed out of range (1-130)");
                    else
                    {
                        string cmd = "sh_speed=" + speed.ToString();
                        string rsp = SendCommand(cmd);
                        Console.WriteLine("\nFlexibowl: setting shaking speed");
                        System.Threading.Thread.Sleep(50);
                    }
                }


                /// <summary>
                /// Set the shaking acceleration used for each subsequent movement of the “shake=1” instruction
                /// </summary>
                /// <param name="acc">Between 10 and 10000</param>
                public static void Acceleration(int acc)
                {
                    if ((acc < 10) && (acc > 10000))
                        Console.WriteLine("Error: shaking acceleration out of range (10-10000)");
                    else
                    {
                        string cmd = "acc=" + acc.ToString();
                        string rsp = SendCommand(cmd);
                        Console.WriteLine("\nFlexibowl: setting shaking acceleration");
                    }
                    System.Threading.Thread.Sleep(50);
                }


                /// <summary>
                /// Set the shaking deceleration used for each subsequent movement of the “shake=1” instruction
                /// </summary>
                /// <param name="dec">Between 10 and 10000</param>
                public static void Deceleration(int dec) 
                {
                    if ((dec < 10) && (dec > 10000))
                        Console.WriteLine("Error: shaking deceleration out of range (10-10000)");
                    else
                    {
                        string cmd = "dec=" + dec.ToString();
                        string rsp = SendCommand(cmd);
                        Console.WriteLine("\nFlexibowl: setting shaking deceleration");
                    }
                    System.Threading.Thread.Sleep(50);
                }


                /// <summary>
                /// Set the number of shaking movements in alternate directions to be performed at subsequent “shake=1” instructions
                /// </summary>
                /// <param name="cnt">must be positive</param>
                public static void Count(int cnt) 
                {
                    if (cnt <= 0)
                        Console.WriteLine("Error: shake count must be positive");
                    else
                    {
                        string cmd = "sh_count=" + cnt.ToString();
                        string rsp = SendCommand(cmd);
                        Console.WriteLine("\nFlexibowl: setting shake count");
                    }
                    System.Threading.Thread.Sleep(50);
                }


                /// <summary>
                /// Set the shaking angular step the feeder moves for each subsequent “shake=1” instructions
                /// </summary>
                /// <param name="angle">Positive values => clockwise; Negative values => counter-clockwise</param>
                public static void Angle(int angle)
                {
                    string cmd = "sh_angle=" + angle.ToString();
                    string rsp = SendCommand(cmd);
                    Console.WriteLine("\nFlexibowl: setting shake angle");
                    System.Threading.Thread.Sleep(50);
                }
            }

            public static class Blow
            {
                /// <summary>
                /// Set the blowing time in milliseconds
                /// </summary>
                /// <param name="blw_time">must be positive [ms]</param>
                public static void Time(int blw_time) 
                {
                    if (blw_time <= 0)
                        Console.WriteLine("Error: blow time must be positive");
                    else
                    {
                        string cmd = "Blow_time=" + blw_time.ToString();
                        string rsp = SendCommand(cmd);
                        Console.WriteLine("\nFlexibowl: setting blow time");
                    }
                    System.Threading.Thread.Sleep(50);
                }
            }

            public static class Flip
            {
                /// <summary>
                /// Set the number of flipping movements (ON/OFF cycles that the piston will perform) at each subsequent “flip=1” instruction.
                /// </summary>
                /// <param name="cnt">Must be positive</param>
                public static void Count(int cnt) 
                {
                    if (cnt <= 0)
                        Console.WriteLine("Error: flip count must be positive");
                    else
                    {
                        string cmd = "fl_count=" + cnt.ToString();
                        string rsp = SendCommand(cmd);
                        Console.WriteLine("\nFlexibowl: setting flip count");
                    }
                    System.Threading.Thread.Sleep(50);
                }


                /// <summary>
                /// Set the delay between flipping movements (between an ON and an OFF of the piston) at each subsequent “flip=1” instruction
                /// </summary>
                /// <param name="delay">Must be positive [ms]</param>
                public static void Delay(int delay)
                {
                    if (delay <= 0)
                        Console.WriteLine("Error: flip delay must be positive");
                    else
                    {
                        string cmd = "fl_delayt=" + delay.ToString();
                        string rsp = SendCommand(cmd);
                        Console.WriteLine("\nFlexibowl: setting flip delay");
                    }
                    System.Threading.Thread.Sleep(50);
                }
            }
        }

        public static class Move
        {
            /// <summary>
            /// Moves the feeder forward with the current parameters
            /// </summary>
            public static void Forward() 
            {
                string cmd = "forward=1";
                string rsp = SendCommand(cmd);
                Console.WriteLine("\nFlexibowl: moving forward");
                while (isBusy(UdpClient, Endpoint))
                {
                    System.Threading.Thread.Sleep(50);
                }
            }


            /// <summary>
            /// Moves the feeder backward with the current parameters
            /// </summary>
            public static void Backward()
            {
                string cmd = "backward=1";
                string rsp = SendCommand(cmd);
                Console.WriteLine("\nFlexibowl: moving backward");
                while (isBusy(UdpClient, Endpoint))
                {
                    System.Threading.Thread.Sleep(50);
                }
            }

            /// <summary>
            /// Shakes the feeder with the current parameters
            /// </summary>
            public static void Shake()
            {
                string cmd = "shake=1";
                string rsp = SendCommand(cmd);
                Console.WriteLine("\nFlexibowl: shaking");
                while (isBusy(UdpClient, Endpoint))
                {
                    System.Threading.Thread.Sleep(50);
                }
            }


            /// <summary>
            /// Activates the flipping unit
            /// </summary>
            /// <param name="piston">select piston 1 or 2</param>
            public static void Flip(int piston) 
            {
                string cmd;
                if (piston == 1)
                {
                    cmd = "flip=1";
                    string rsp = SendCommand(cmd);
                    Console.WriteLine("\nFlexibowl: flipping piston 1");
                }
                else if (piston == 2)
                {
                    cmd = "flip2=1";
                    string rsp = SendCommand(cmd);
                    Console.WriteLine("\nFlexibowl: flipping piston 2");
                }
                else
                {
                    Console.WriteLine("\nError: piston number out of range (1,2)");
                }
                while (isBusy(UdpClient, Endpoint))
                {
                    System.Threading.Thread.Sleep(50);
                }

            }


            /// <summary>
            /// Activates the blowing unit
            /// </summary>
            public static void Blow() 
            {
                string cmd = "Blow=1";
                string rsp = SendCommand(cmd);
                Console.WriteLine("Flexibowl: blowing");
                while (isBusy(UdpClient, Endpoint))
                {
                    System.Threading.Thread.Sleep(50);
                }
            }


            /// <summary>
            /// Activates the flipping and blowing units simultaneously; activates valves 1 and 2 simultaneously
            /// </summary>
            public static void FlipBlow() 
            {
                string cmd = "flip_Blow=1";
                string rsp = SendCommand(cmd);
                Console.WriteLine("\nFlexibowl: flipping piston 1 and blowing");
                while (isBusy(UdpClient, Endpoint))
                {
                    System.Threading.Thread.Sleep(50);
                }
            }


            /// <summary>
            /// Moves the feeder forward and activates the flipping and blowing units simultaneously
            /// </summary>
            public static void ForwardFlipBlow() 
            {
                string cmd = "fwd_fl_bw=1";
                string rsp = SendCommand(cmd);
                Console.WriteLine("\nFlexibowl: moving forward, flipping piston 1 and 2, blowing");
                while (isBusy(UdpClient, Endpoint))
                {
                    System.Threading.Thread.Sleep(50);
                }
            }


            /// <summary>
            /// Moves the feeder forward and activates the blowing unit
            /// </summary>
            public static void ForwardBlow() 
            {
                string cmd = "fwd_blw=1";
                string rsp = SendCommand(cmd);
                Console.WriteLine("\nFlexibowl: moving forward and blowing");
                while (isBusy(UdpClient, Endpoint))
                {
                    System.Threading.Thread.Sleep(50);
                }
            }
        }
    }
}
