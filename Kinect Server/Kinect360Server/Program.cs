using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Drawing;
using Demonixis.Server.Kinect;
using System.Timers;

namespace Demonixis.Kinect.Server
{
    class Program
    {
        // Server attributes
        private static WebSocketServer _server;
        private static KinectSet _kinectSet;
        private static KinectSensor[] _kinectSensorArray;
        private static BodySendType _sendType = BodySendType.AllTracked;

        //Timer parameters
        private static Timer _timer;
        private static bool _timerCondition = false;
        

        //Kinect parameters
        private static int _numberOfKinects;
        private static bool[] _receivingColorData;
        private static bool _allKinectsOpened = false;

        //Image parameters
        private static float _xMin, _xMax, _yMin, _yMax, _zMin, _zMax;
        private static float _cubeSize;

        private static byte[] _raw = null;
        private static int _resX = 320;
        private static int _resY = 240;
        private static int _colorWidth;
        private static int _depthWidth;
        private static int _depthHeight;
        private static int _colorHeight;
        private static byte[] _colorPixels;
        private static DepthImagePixel[] _depthPixels;
        private static ColorImageFormat _ColorFormat = ColorImageFormat.YuvResolution640x480Fps15;
        private static DepthImageFormat _DepthFormat = DepthImageFormat.Resolution320x240Fps30;

        public static void Main(string[] args)
        {
            _timer = new Timer();
            _timer.Elapsed += new ElapsedEventHandler(TimerCallback);
            _timer.Interval = 500;
            _timer.Start();
            if (InitilizeKinect())
            {
                //Ranges in global axis system
                _xMin = (float)-0.25;  _xMax = (float)0.25;
                _yMin = (float)-1; _yMax = (float)0.5;
                _zMin = (float)-2; _zMax = (float)-0.1;
                //_xMin = (float)-1;  _xMax = (float)2;
                //_yMin = (float)-1; _yMax = (float)1;
                //_zMin = (float)-1; _zMax = (float)1;
                _cubeSize = (float)0.03;
                _kinectSet = new Demonixis.Server.Kinect.KinectSet(_xMin, _xMax, _yMin, _yMax, _zMin, _zMax, _cubeSize);

                //First Kinect
                if (_numberOfKinects > 0)
                {
                    Vector3 position = new Vector3((float)-1.5, (float)0, (float)0);
                    Vector3 rotation = new Vector3((float)0, (float)0, (float)0);

                    //Vector3 position = new Vector3((float)60, (float)0,(float)0);
                    //Vector3 rotation = new Vector3((float)0, (float)0, (float)0);
                    Pose pose = new Pose(position, rotation);
                    Resolution resolution = new Resolution(_resX, _resY);
                    Calibration calibration = new Calibration(335, 243, 521, 514);
                    _kinectSet.addKinect(new KinectData(0, pose, resolution, calibration, _xMin, _xMax, _yMin, _yMax, _zMin, _zMax, _cubeSize));
                }

                //Second Kinect
                if (_numberOfKinects > 1)
                {
                    //Vector3 position = new Vector3((float)2.66, (float)1.24,(float)-1.61);
                    //Vector3 rotation = new Vector3((float)-2.52, (float)-0.35, (float)0);
                    Vector3 position = new Vector3((float)60, (float)0,(float)0);
                    Vector3 rotation = new Vector3((float)0, (float)0, (float)0);
                    Pose pose = new Pose(position, rotation);
                    Resolution resolution = new Resolution(_resX, _resY);
                    Calibration calibration = new Calibration(335, 243, 521, 514);
                    _kinectSet.addKinect(new KinectData(1, pose, resolution, calibration, _xMin, _xMax, _yMin, _yMax, _zMin, _zMax, _cubeSize));
                }

                //Second Kinect
                if (_numberOfKinects > 2)
                {
                    Vector3 position = new Vector3((float)0.12, (float)1.34, (float)-1.61);
                    Vector3 rotation = new Vector3((float)-1.7, (float)-0.35, (float)0);
                    //Vector3 position = new Vector3((float)60, (float)0,(float)0);
                    //Vector3 rotation = new Vector3((float)0, (float)0, (float)0);
                    Pose pose = new Pose(position, rotation);
                    Resolution resolution = new Resolution(_resX, _resY);
                    Calibration calibration = new Calibration(521, 260, 331, 519);
                    _kinectSet.addKinect(new KinectData(1, pose, resolution, calibration, _xMin, _xMax, _yMin, _yMax, _zMin, _zMax, _cubeSize));
                }


                //Starting the server
                var parameters = new ServerParameters(args);
                _sendType = parameters.SendType;
                _server = new WebSocketServer();
                _server.PrintServerHeader(_sendType, false);
                for(int i = 0; i < _numberOfKinects; i++)
                {
                    _kinectSensorArray[i].Start();
                }
                _server.Start(parameters);
            }
            else
            {
                Console.WriteLine("The Kinect sensor is not connected. Please connect it and restart the server.");
                Console.ReadKey();
            }
        }

        private static void DisplayTimeEvent(object sender, ElapsedEventArgs e)
        {
            throw new NotImplementedException();
        }

        private static bool InitilizeKinect()
        //Returns true if kinects connected, and start them ; false else
        {
            _numberOfKinects = KinectSensor.KinectSensors.Count;
            _kinectSensorArray = new KinectSensor[_numberOfKinects];
            _receivingColorData = new bool[_numberOfKinects];
            if (_numberOfKinects > 0)
            {
                for (int i = 0; i < _numberOfKinects; i++)
                {
                    _kinectSensorArray[i] = KinectSensor.KinectSensors[i];
                    _kinectSensorArray[i].DepthStream.Enable(_DepthFormat);
                    _kinectSensorArray[i].ColorStream.Enable(_ColorFormat);
                    if (i == 0)
                    {
                        _kinectSensorArray[i].ColorFrameReady += new EventHandler<ColorImageFrameReadyEventArgs>(OnColorFrameReady0);
                        _kinectSensorArray[i].DepthFrameReady += new EventHandler<DepthImageFrameReadyEventArgs>(OnDepthFrameReady0);
                    }
                    if (i == 1)
                    {
                        _kinectSensorArray[i].ColorFrameReady += new EventHandler<ColorImageFrameReadyEventArgs>(OnColorFrameReady1);
                        _kinectSensorArray[i].DepthFrameReady += new EventHandler<DepthImageFrameReadyEventArgs>(OnDepthFrameReady1);
                    }
                    if (i == 2)
                    {
                        _kinectSensorArray[i].ColorFrameReady += new EventHandler<ColorImageFrameReadyEventArgs>(OnColorFrameReady2);
                        _kinectSensorArray[i].DepthFrameReady += new EventHandler<DepthImageFrameReadyEventArgs>(OnDepthFrameReady2);
                    }
                }
                return true;
            }
            return false;
        }

        private static void OnColorFrameReady0(object sender, ColorImageFrameReadyEventArgs e)
        {
            using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
            {
                if (e != null && colorFrame.PixelDataLength > 1)
                {
                    _kinectSet.getKinectData(0).updateColorData(colorFrame);
                    _receivingColorData[0] = true;
                }
            }
        }

        private static void OnDepthFrameReady0(object sender, DepthImageFrameReadyEventArgs e)
        {
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (_receivingColorData[0] == true)
                {
                    _kinectSet.getKinectData(0).updateDepthData(depthFrame);
                    _allKinectsOpened = true;
                    for (int i = 0; i < _numberOfKinects; i++)
                    {
                        if (_receivingColorData[i] != true)
                        {
                            _allKinectsOpened = false;
                        }
                    }
                    if (_allKinectsOpened == true)
                    {
                        for(int i = 0; i < _numberOfKinects; i++)
                        {
                            _kinectSet.getKinectData(i).processing();
                        }
                        _kinectSet.mergeKinects();
                        if (_timerCondition)
                        {
                            Console.WriteLine("Sending {0} points from {1} Kinects", _kinectSet.getNumberOfPoints(), _kinectSet.getNumberOfKinects());
                            _server.SendData(WebSocketServer.Serialize(_kinectSet.ToJsonFormat()));
                            _timerCondition = false;
                        }
                    }
                }
            }
        }
        
        private static void TimerCallback(object source, ElapsedEventArgs e)
        {
            _timerCondition = true;
        }

        private static void OnColorFrameReady1(object sender, ColorImageFrameReadyEventArgs e)
        {
            using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
            {
                if (e != null && colorFrame.PixelDataLength > 1)
                {
                    _kinectSet.getKinectData(1).updateColorData(colorFrame);
                    _receivingColorData[1] = true;
                }
            }
        }

        private static void OnDepthFrameReady1(object sender, DepthImageFrameReadyEventArgs e)
        {
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (_receivingColorData[1] == true)
                {
                    _kinectSet.getKinectData(1).updateDepthData(depthFrame);
                }
            }
        }

        private static void OnColorFrameReady2(object sender, ColorImageFrameReadyEventArgs e)
        {
            using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
            {
                if (e != null && colorFrame.PixelDataLength > 1)
                {
                    _kinectSet.getKinectData(2).updateColorData(colorFrame);
                    _receivingColorData[2] = true;
                }
            }
        }

        private static void OnDepthFrameReady2(object sender, DepthImageFrameReadyEventArgs e)
        {
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (_receivingColorData[2] == true)
                {
                    _kinectSet.getKinectData(2).updateDepthData(depthFrame);
                }
            }
        }
    }
}