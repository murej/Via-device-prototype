using System;
using System.Collections;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.Presentation;
using Microsoft.SPOT.Presentation.Controls;
using Microsoft.SPOT.Presentation.Media;
using Microsoft.SPOT.Touch;

using Gadgeteer.Networking;
using GT = Gadgeteer;
using GTM = Gadgeteer.Modules;
using Gadgeteer.Modules.GHIElectronics;

using GHI.Premium.System;
using GHI.Premium.Net;
using Gadgeteer.Modules.Seeed;


namespace Via_prototype
{
    public partial class Program
    {

        GT.Interfaces.PWMOutput servo;
        GT.Networking.WebEvent collectData;

        double currentLatitude = 45.01;
        double currentLongitude = 13.02;

        // set destination (Bavarski dvor)
        double destinationLatitude = 46.056726;
        double destinationLongitude = 14.505598;

        /*double compassX;
        double compassY;
        double compassZ;
        double accelerometerX;
        double accelerometerY;
        double compassXmax = 381;
        double compassXmin = -269;
        double compassYmax = 0;
        double compassYmin = -793;*/

        double heading;
        double bearing;
        double relativeBearing;

        /*
        bool direction = true;
        double servoValue;
        */

        // This method is run when the mainboard is powered up or reset.   
        void ProgramStarted()
        {
            /*******************************************************************************************
            Modules added in the Program.gadgeteer designer view are used by typing 
            their name followed by a period, e.g.  button.  or  camera.
            
            Many modules generate useful events. Type +=<tab><tab> to add a handler to an event, e.g.:
                button.ButtonPressed +=<tab><tab>
            
            If you want to do something periodically, use a GT.Timer and handle its Tick event, e.g.:
                GT.Timer timer = new GT.Timer(1000); // every second (1000ms)
                timer.Tick +=<tab><tab>
                timer.Start();
            *******************************************************************************************/

            // setup a servo on pin 7 of the extender and turn it on
            //servo = this.extender.SetupPWMOutput(GT.Socket.Pin.Seven);
            //servo.Active = true;

            // setup compass
            //compass.SetGain(Compass.Gain.Gain2);
            compass.MeasurementComplete += new GTM.Seeed.Compass.MeasurementCompleteEventHandler(compass_MeasurementComplete);
            
            // setup accelerometer
            //accelerometer.Calibrate();
            //accelerometer.SaveCalibration();
            /*
            Debug.Print(accelerometer.LoadCalibration().ToString());
            accelerometer.MeasurementComplete += new Accelerometer.MeasurementCompleteEventHandler(accelerometer_MeasurementComplete);
            */


            
            // setup wifi
            if (!wifi.Interface.IsOpen)
                wifi.Interface.Open();

            //if (!wifi.Interface.NetworkInterface.IsDhcpEnabled)
            //    wifi.Interface.NetworkInterface.EnableDhcp();
            wifi.Interface.NetworkInterface.EnableStaticIP("172.20.10.5", "255.255.255.240", "172.20.10.1");

            NetworkInterfaceExtension.AssignNetworkingStackTo(wifi.Interface);

            //wifi.Interface.WirelessConnectivityChanged += new WiFiRS9110.WirelessConnectivityChangedEventHandler(wifi_WirelessConnectivityChanged);
            wifi.Interface.NetworkAddressChanged += new NetworkInterfaceExtension.NetworkAddressChangedEventHandler(wifi_NetworkAddressChanged);

            // create an ad hoc connection for the smartphone to connect to
            wifi.Interface.StartAdHocHost("Via", SecurityMode.WEP, "123tolmin", 11);

            
            Debug.Print("------------------------------------------------");
            Debug.Print("IP Address:   " + wifi.Interface.NetworkInterface.IPAddress);
            Debug.Print("DHCP Enabled: " + wifi.Interface.NetworkInterface.IsDhcpEnabled);
            Debug.Print("Subnet Mask:  " + wifi.Interface.NetworkInterface.SubnetMask);
            Debug.Print("Gateway:      " + wifi.Interface.NetworkInterface.GatewayAddress);
            Debug.Print("DNS server:   " + wifi.Interface.NetworkInterface.DnsAddresses[0]);
            Debug.Print("------------------------------------------------");
            Debug.Print("Actvated: " + wifi.Interface.IsActivated);
            Debug.Print("Link connected: " + wifi.Interface.IsLinkConnected);
            Debug.Print("------------------------------------------------");
            
            
            /*
            // setup web server
            WebServer.StartLocalServer(wifi.Interface.NetworkInterface.IPAddress, 80);
            collectData = WebServer.SetupWebEvent("data"); // SEND DATA TO: http://{ip}/data
            collectData.WebEventReceived += new WebEvent.ReceivedWebEventHandler(collectData_WebEventReceived);
            */

            // puts servo to default position for 5 seconds to sync it with the arrow piece
            //servo.SetPulse(20 * 1000 * 1000, 150 * 10 * 1000);
            //Mainboard.SetDebugLED(true);
            //Thread.Sleep(5000);
            //Mainboard.SetDebugLED(false);

            // create a timer robot
            GT.Timer timer = new GT.Timer(100);
            timer.Tick += new GT.Timer.TickEventHandler(timer_Tick);
            timer.Start();

            // convert coordinates to radians
            currentLatitude = currentLatitude * MathEx.PI / 180;
            currentLongitude = currentLongitude * MathEx.PI / 180;
            destinationLatitude = destinationLatitude * MathEx.PI / 180;
            destinationLongitude = destinationLongitude * MathEx.PI / 180;

            // tell everyone that we have started
            Debug.Print("Program Started");

        }

        void wifi_NetworkAddressChanged(object sender, EventArgs e)
        {
            Debug.Print("NETWORK ADDRESS CHANGED: " + wifi.Interface.NetworkInterface.IPAddress);
        }

        void collectData_WebEventReceived(string path, WebServer.HttpMethod method, Responder responder)
        {
            /*EXAMPLE: path = "data?currentLatitude=XX.XXXXXXX&currentLongitude=XX.XXXXXXX&destinationLatitude=XX.XXXXXXX&destinationLongitude=XX.XXXXXXX";*/

            Debug.Print(path.ToString());

            //Debug.Print(receivedContent.ToString());

            // process data
            /*currentLatitude = path;
            currentLongitude = path;
            destinationLatitude = path;
            destinationLongitude = path;*/

            // send information back
            string respondContent = "HTTP/1.1 200 OK"; // this can be anything (HTML etc..)
            byte[] HeaderData = new System.Text.UTF8Encoding().GetBytes(respondContent);
            responder.Respond(HeaderData, "text/plain");
        }

        void timer_Tick(GT.Timer timer)
        {
            compass.RequestMeasurement();
            //accelerometer.RequestMeasurement();

            //gps.PositionReceived += new GPS.PositionReceivedHandler(gps_PositionReceived);

            // if current location was found
            if (currentLatitude != 0 && currentLongitude != 0)
            {

                // CALCULATE BEARING
                double y = MathEx.Sin(destinationLongitude - currentLongitude) * MathEx.Cos(destinationLatitude);
                double x = MathEx.Cos(currentLatitude) * MathEx.Sin(destinationLatitude) - MathEx.Sin(currentLatitude) * MathEx.Cos(destinationLatitude) * MathEx.Cos(destinationLongitude - currentLongitude);
                bearing = MathEx.Atan2(y, x);
                // convert radian value to degrees (-180 to 180 deg)
                bearing = bearing * (180.0 / MathEx.PI);
                // normalise the angle (0 to 360 deg)
                bearing = (bearing + 360) % 360;

                /*
                // CALCULATE HEADING

                // nearby ferrous effect compensation calculations
                double Xsf = (compassYmax - compassYmin) / (compassXmax - compassXmin);
                if (Xsf < 1)
                    Xsf = 1;
                double Ysf = (compassXmax - compassXmin) / (compassYmax - compassYmin);
                if (Ysf < 1)
                    Ysf = 1;
                double Xoff = (((compassXmax - compassXmin) / 2) - compassXmax) * Xsf;
                double Yoff = (((compassYmax - compassYmin) / 2) - compassYmax) * Ysf;
            
                // tilt compensation calculations
                double pitchRadians = MathEx.Asin(accelerometerX);
                double rollRadians = MathEx.Asin(accelerometerY);

                // we cannot correct for tilt over 40 degrees with this algorthem, if the board is tilted as such, return 0.
                if (rollRadians > 0.78 || rollRadians < -0.78 || pitchRadians > 0.78 || pitchRadians < -0.78)
                {
                    heading = 0;
                }

                else
                {

                    // tilt compensation algorithm
                    double Xh = compassX * MathEx.Cos(pitchRadians) + compassY * MathEx.Sin(rollRadians) * MathEx.Sin(pitchRadians) - compassZ * MathEx.Cos(rollRadians) * MathEx.Sin(pitchRadians);
                    double Yh = compassY * MathEx.Cos(rollRadians) + compassZ * MathEx.Sin(rollRadians);
                
                    // calculate final compass X and Y coordinates
                    double Xvalue = Xsf * Xh + Xoff;
                    double Yvalue = Ysf * Yh + Yoff;

                    // calculate heading
                    if (Xvalue == 0 && Yvalue < 0)
                        heading = 90;
                    else if (Xvalue == 0 && Yvalue > 0)
                        heading = 270;
                    else if (Xvalue < 0)
                        heading = 180 - ( MathEx.Atan( Yvalue / Xvalue ) * 180 / MathEx.PI );
                    else if (Xvalue > 0 && Yvalue < 0)
                        heading = - ( MathEx.Atan( Yvalue / Xvalue ) * 180 / MathEx.PI );
                    else if (Xvalue > 0 && Yvalue > 0)
                        heading = 360 - ( MathEx.Atan(Yvalue / Xvalue) * 180 / MathEx.PI );

                    /*if (compassX == 0 && compassY < 0)
                        heading = 90;
                    else if (compassX == 0 && compassY > 0)
                        heading = 270;
                    else if (compassX < 0)
                        heading = 180 - (MathEx.Atan(compassY / compassX) * 180 / MathEx.PI);
                    else if (compassX > 0 && compassY < 0)
                        heading = -(MathEx.Atan(compassY / compassX) * 180 / MathEx.PI);
                    else if (compassX > 0 && compassY > 0)
                        heading = 360 - (MathEx.Atan(compassY / compassX) * 180 / MathEx.PI);*/
                /*
                }
                */
                

                // CALCULATE RELATIVE BEARING
                if (heading <= bearing)
                    relativeBearing = bearing - heading;
                else if (heading > bearing)
                    relativeBearing = 360 - (heading - bearing);

       
                // MOVE ARROW
                /*
                if (relativeBearing <= 180)
                    servoValue = map(relativeBearing, 0, 180, 150, 0);
                else if (relativeBearing > 180)
                    servoValue = map(relativeBearing, 180, 360, 300, 150);

                servo.SetPulse(20 * 1000 * 1000, (uint)servoValue * 10 * 1000);
                */

            }
            // if no gps coordinates found
            else
            {

                /*
                // set direction
                if (servoValue == 180)
                    direction = false;
                else if (servoValue == 120)
                    direction = true;

                // calculate servo position
                if (direction == false)
                    servoValue -= 10;
                else if (direction == true)
                    servoValue += 10;

                servo.SetPulse(20 * 1000 * 1000, (uint)servoValue * 10 * 1000);
                */

                Debug.Print("No GPS position");

            }
            
            /*z
            // print measured data
            //Debug.Print("lat: " + currentLatitude.ToString());
            //Debug.Print("lon: " + currentLongitude.ToString());
            Debug.Print("---".ToString());
            Debug.Print("heading: " + heading.ToString());
            Debug.Print("bearing: " + bearing.ToString());
            Debug.Print("relative bearing: " + relativeBearing.ToString());
            Debug.Print("---".ToString());
            */
        }

        void gps_PositionReceived(GPS sender, GPS.Position position)
        {
            currentLatitude = gps.LastPosition.Latitude;
            currentLatitude = gps.LastPosition.Longitude;
        }

        void compass_MeasurementComplete(GTM.Seeed.Compass sender, GTM.Seeed.Compass.SensorData compassSensorData)
        {
            /*
            compassX = compassSensorData.X;
            compassY = compassSensorData.Y;
            compassZ = compassSensorData.Z;
            
            if (compassX > compassXmax)
                compassXmax = compassX;
            if (compassX < compassXmin)
                compassXmin = compassX;
            if (compassY > compassYmax)
                compassYmax = compassY;
            if (compassY < compassYmin)
                compassYmin = compassY;
            */

            heading = compassSensorData.Angle;

        }

        /*
        void accelerometer_MeasurementComplete(Accelerometer sender, Accelerometer.Acceleration acceleration)
        {
            accelerometerX = acceleration.X;
            accelerometerY = acceleration.Y;
        }
        */

        double map(double input, double in_min, double in_max, double out_min, double out_max)
        {
            return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        }
    }
}