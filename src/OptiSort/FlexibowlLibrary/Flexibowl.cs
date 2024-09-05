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
        public static UdpClient Connect(IPEndPoint ep) // Connect to the Flexibowl via UDP
        {
            Console.WriteLine("\nConnecting to the UDP server ...");
            // Create a new UDP client
            UdpClient client = new UdpClient();

            try
            {   // Connect to the UDP server
                client.Connect(ep);

                // Set send and receive timeouts
                client.Client.SendTimeout = 500;
                client.Client.ReceiveTimeout = 500;
            }
            catch (ArgumentNullException ex)
            {
                Console.WriteLine("\nArgumentNullException: {0}", ex);
            }

            Console.WriteLine("\nConnected to FlexiBowl");

            return client;
        }
        public static void Disconnect(UdpClient client)
        {
            // Dispose the client
            client.Dispose();

            // Close the client
            client.Close();

            Console.WriteLine("\nDisconnected from FlexiBowl");
        } // Disconnect from the Flexibowl
        public static bool isBusy(UdpClient client, IPEndPoint ep) // Check wheter the Flexibowl is available or not
        {
            string receiveString = "";
            int byteSent = 0;
            bool available = false;
            do
            {
                // Convert the command to bytes
                Byte[] SCLstring = Encoding.ASCII.GetBytes("ob[4]");
                Byte[] sendBytes = new Byte[SCLstring.Length + 1];

                System.Array.Copy(SCLstring, 0, sendBytes, 0, SCLstring.Length);
                sendBytes[sendBytes.Length - 1] = 13; // CR

                // Send the command to the server
                byteSent = client.Send(sendBytes, sendBytes.Length);

                // Receive data from the server
                Byte[] receivedData = client.Receive(ref ep);
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
        public static string SendCommand(UdpClient client, IPEndPoint ep, string command) // Send a command to the Flexibowl over UDP
        {
            string receiveString = "";
            int byteSent = 0;

            // Convert the command to bytes
            Byte[] SCLstring = Encoding.ASCII.GetBytes(command);
            Byte[] sendBytes = new Byte[SCLstring.Length + 1];

            System.Array.Copy(SCLstring, 0, sendBytes, 0, SCLstring.Length);
            sendBytes[sendBytes.Length - 1] = 13; // CR

            // Send the command to the server
            byteSent = client.Send(sendBytes, sendBytes.Length);
            //Console.WriteLine("\nCommand sent");

            // Receive data from the server
            Byte[] receivedData = client.Receive(ref ep);

            receiveString = Encoding.ASCII.GetString(receivedData);

            return receiveString;
        }
        public static class Set
        {
            public static void Servo(UdpClient client, IPEndPoint ep, bool status) // Enable/disable the servo
            {    
                string cmd;
                if (status == true)
                {
                    cmd = "servo=1";
                }
                else
                {
                    cmd = "servo=0";
                }
                string rsp = SendCommand(client, ep, cmd);
                Console.WriteLine("\nFlexibowl: setting servo to " + status.ToString());
                while (isBusy(client, ep))
                {
                    System.Threading.Thread.Sleep(50);
                }
            }
            public static void Light(UdpClient client, IPEndPoint ep, bool status) // Turn on/off the light
            {
                string cmd;

                if (status == true)
                {
                    cmd = "light=1";
                }
                else
                {
                    cmd = "light=0";
                }
                string rsp = SendCommand(client, ep, cmd);
                Console.WriteLine("\nFlexibowl: setting light to " + status.ToString());
                while (isBusy(client, ep))
                {
                    System.Threading.Thread.Sleep(50);
                }
            }
            public static class Rotation
            {
                public static void Speed(UdpClient client, IPEndPoint ep, int speed) // Set the rotational speed
                {
                    // Set the rotational speed in RPM, at which the feeder will
                    // advance at each subsequent “forward=1” instruction.
                    // Between 1 and 130
                    if ((speed < 1) && (speed > 130))
                    {
                        Console.WriteLine("Error: rotational speed out of range (1-130)");
                    }
                    else
                    {
                        string cmd = "speed=" + speed.ToString();
                        string rsp = SendCommand(client, ep, cmd);
                        Console.WriteLine("\nFlexibowl: setting rotational speed");
                    }

                    System.Threading.Thread.Sleep(50);
                }
                public static void Acceleration(UdpClient client, IPEndPoint ep, int acc) // Set the rotational acceleration
                {
                    // Acceleration used for each subsequent “forward=1”
                    // instruction.
                    // Between 10 and 10000
                    if ((acc < 10) && (acc > 10000))
                    {
                        Console.WriteLine("Error: rotational acceleration out of range (10-10000)");
                    }
                    else
                    {
                        string cmd = "acc=" + acc.ToString();
                        string rsp = SendCommand(client, ep, cmd);

                        Console.WriteLine("\nFlexibowl: setting rotational acceleration");

                    }
                    System.Threading.Thread.Sleep(50);
                }
                public static void Deceleration(UdpClient client, IPEndPoint ep, int dec) // Set the rotational deceleration
                {
                    // Deceleration used for each subsequent “forward=1”
                    // instruction.
                    // Between 10 and 10000
                    if ((dec < 10) && (dec > 10000))
                    {
                        Console.WriteLine("Error: rotational deceleration out of range (10-10000)");
                    }
                    else
                    {
                        string cmd = "dec=" + dec.ToString();
                        string rsp = SendCommand(client, ep, cmd);

                        Console.WriteLine("\nFlexibowl: setting rotational deceleration");

                    }
                    System.Threading.Thread.Sleep(50);
                }
                public static void Angle(UdpClient client, IPEndPoint ep, int angle) // Set the angular step
                {
                    // Angle by which the feeder will advance at each subsequent
                    // “forward=1” instruction.
                    // Positive values => clockwise rotation
                    // Negative values => counter-clockwise rotation
                    string cmd = "angle=" + angle.ToString();
                    string rsp = SendCommand(client, ep, cmd);

                    Console.WriteLine("\nFlexibowl: setting rotational angle");

                    System.Threading.Thread.Sleep(50);

                }
            }

            public static class Shake
            {
                public static void Speed(UdpClient client, IPEndPoint ep, int speed) // Set the shaking speed
                {
                    // Speed, in RPM, at which to shake the feeder at each subsequent
                    // “shake = 1” instruction.
                    // Between 1 and 130
                    if ((speed < 1) && (speed > 130))
                    {
                        Console.WriteLine("Error: shaking speed out of range (1-130)");
                    }
                    else
                    {
                        string cmd = "sh_speed=" + speed.ToString();
                        string rsp = SendCommand(client, ep, cmd);
                        Console.WriteLine("\nFlexibowl: setting shaking speed");
                        System.Threading.Thread.Sleep(50);
                    }
                }
                public static void Acceleration(UdpClient client, IPEndPoint ep, int acc) // Set the shaking acceleration
                {
                    // Acceleration used for each subsequent movement of the “shake=1”
                    // instruction.
                    // Between 10 and 10000.
                    if ((acc < 10) && (acc > 10000))
                    {
                        Console.WriteLine("Error: shaking acceleration out of range (10-10000)");
                    }
                    else
                    {
                        string cmd = "acc=" + acc.ToString();
                        string rsp = SendCommand(client, ep, cmd);
                        Console.WriteLine("\nFlexibowl: setting shaking acceleration");
                    }
                    System.Threading.Thread.Sleep(50);
                }
                public static void Deceleration(UdpClient client, IPEndPoint ep, int dec) // Set the shaking deceleration
                {
                    // Decceleration used for each subsequent movement of the “shake=1”
                    // instruction.
                    // Between 10 and 10000.
                    if ((dec < 10) && (dec > 10000))
                    {
                        Console.WriteLine("Error: shaking deceleration out of range (10-10000)");
                    }
                    else
                    {
                        string cmd = "dec=" + dec.ToString();
                        string rsp = SendCommand(client, ep, cmd);

                        Console.WriteLine("\nFlexibowl: setting shaking deceleration");

                    }
                    System.Threading.Thread.Sleep(50);
                }
                public static void Count(UdpClient client, IPEndPoint ep, int cnt) // Set the number of shaking movements
                {
                    // Number of movements, in alternate directions,
                    // to be performed at subsequent “shake=1” instructions.
                    if (cnt <= 0)
                    {
                        Console.WriteLine("Error: shake count must be positive");
                    }
                    else
                    {
                        string cmd = "sh_count=" + cnt.ToString();
                        string rsp = SendCommand(client, ep, cmd);

                        Console.WriteLine("\nFlexibowl: setting shake count");
                    }
                    System.Threading.Thread.Sleep(50);
                }
                public static void Angle(UdpClient client, IPEndPoint ep, int angle) // Set the shaking angular step
                {
                    // Angle the feeder moves for each shake
                    // for subsequent “shake=1” instructions.
                    // Positive values => clockwise
                    // Negative values => counter-clockwise
                    string cmd = "sh_angle=" + angle.ToString();
                    string rsp = SendCommand(client, ep, cmd);

                    Console.WriteLine("\nFlexibowl: setting shake angle");

                    System.Threading.Thread.Sleep(50);
                }
            }

            public static class Blow
            {
                public static void Time(UdpClient client, IPEndPoint ep, int blw_time) // Set the blowing time
                {
                    // Blowing time, in milliseconds
                    if (blw_time <= 0)
                    {
                        Console.WriteLine("Error: blow time must be positive");
                    }
                    else
                    {
                        string cmd = "Blow_time=" + blw_time.ToString();
                        string rsp = SendCommand(client, ep, cmd);

                        Console.WriteLine("\nFlexibowl: setting blow time");

                    }
                    System.Threading.Thread.Sleep(50);
                }
            }

            public static class Flip
            {
                public static void Count(UdpClient client, IPEndPoint ep, int cnt) // Set the number of flipping movements
                {
                    // Number of ON/OFF cycles that the piston will perform
                    // at each subsequent “flip=1” instruction.
                    // Must be positive
                    if (cnt <= 0)
                    {
                        Console.WriteLine("Error: flip count must be positive");
                    }
                    else
                    {
                        string cmd = "fl_count=" + cnt.ToString();
                        string rsp = SendCommand(client, ep, cmd);

                        Console.WriteLine("\nFlexibowl: setting flip count");
                    }
                    System.Threading.Thread.Sleep(50);
                }
                public static void Delay(UdpClient client, IPEndPoint ep, int delay) // Set the delay between flipping movements
                {
                    // Time, in milliseconds, between an ON and an OFF
                    // of the piston at each subsequent “flip=1” instruction.
                    // Must be positive
                    if (delay <= 0)
                    {
                        Console.WriteLine("Error: flip delay must be positive");
                    }
                    else
                    {
                        string cmd = "fl_delayt=" + delay.ToString();
                        string rsp = SendCommand(client, ep, cmd);

                        Console.WriteLine("\nFlexibowl: setting flip delay");

                    }
                    System.Threading.Thread.Sleep(50);
                }
            }
        }
        public static class Move
        {
            public static void Forward(UdpClient client, IPEndPoint ep) // Moves the feeder forward with the current parameters
            {                
                string cmd = "forward=1";
                string rsp = SendCommand(client, ep, cmd);
                Console.WriteLine("\nFlexibowl: moving forward");
                while (isBusy(client, ep))
                {
                    System.Threading.Thread.Sleep(50);
                }
            }
            public static void Shake(UdpClient client, IPEndPoint ep) // Shakes the feeder with the current parameters
            {                
                string cmd = "shake=1";
                string rsp = SendCommand(client, ep, cmd);

                Console.WriteLine("\nFlexibowl: shaking");

                while (isBusy(client, ep))
                {
                    System.Threading.Thread.Sleep(50);
                }
            }
            public static void Flip(UdpClient client, IPEndPoint ep, int piston) // Activates the flipping unit
            {
                // Activate piston 1 or 2 with current parameters
                string cmd;
                if (piston == 1)
                {
                    cmd = "flip=1";
                    string rsp = SendCommand(client, ep, cmd);

                    Console.WriteLine("\nFlexibowl: flipping piston 1");
                }
                else if (piston == 2)
                {
                    cmd = "flip2=1";
                    string rsp = SendCommand(client, ep, cmd);

                    Console.WriteLine("\nFlexibowl: flipping piston 2");
                }
                else
                {
                    Console.WriteLine("\nError: piston number out of range (1,2)");
                }
                while (isBusy(client, ep))
                {
                    System.Threading.Thread.Sleep(50);
                }

            }
            public static void Blow(UdpClient client, IPEndPoint ep) // Activates the blowing unit
            {
                // Activates the Blow unit for Blow_time milliseconds
                string cmd = "Blow=1";
                string rsp = SendCommand(client, ep, cmd);

                Console.WriteLine("Flexibowl: blowing");

                while (isBusy(client, ep))
                {
                    System.Threading.Thread.Sleep(50);
                }
            }
            public static void FlipBlow(UdpClient client, IPEndPoint ep) // Activates the flipping and blowing units simultaneously
            {
                // Activates valves 1 and 2 simultaneously
                string cmd = "flip_Blow=1";
                string rsp = SendCommand(client, ep, cmd);
                Console.WriteLine("\nFlexibowl: flipping piston 1 and blowing");
                while (isBusy(client, ep))
                {
                    System.Threading.Thread.Sleep(50);
                }
            }
            public static void ForwardFlipBlow(UdpClient client, IPEndPoint ep) // Moves the feeder forward and activates the flipping and blowing units simultaneously
            {
                // Moves the feeder forward and activates Flip 1, Flip2 and Blow simultaneously
                string cmd = "fwd_fl_bw=1";
                string rsp = SendCommand(client, ep, cmd);
                Console.WriteLine("\nFlexibowl: moving forward, flipping piston 1 and 2, blowing");
                while (isBusy(client, ep))
                {
                    System.Threading.Thread.Sleep(50);
                }
            }
            public static void ForwardBlow(UdpClient client, IPEndPoint ep) // Moves the feeder forward and activates the blowing unit
            {
                // Moves the feeder and activates the Blow unit simultaneously
                string cmd = "fwd_blw=1";
                string rsp = SendCommand(client, ep, cmd);
                Console.WriteLine("\nFlexibowl: moving forward and blowing");

                while (isBusy(client, ep))
                {
                    System.Threading.Thread.Sleep(50);
                }
            }
        }
    }
}
