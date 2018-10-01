using System;
using System.Collections.Generic;
using System.Text;

//////////////////////////////////////////////////////////////////////////////
//
//      File Name:      Program.cs
//      Description:    Sample program demonstrating how to use the InterSense library from C#
//      Created:        2008-07-01
//      Updated:        2013-02-08
//      Author:         Rand Kmiec
//
//      Copyright:      InterSense 2010 - All rights Reserved.
//
//      Comments:       This file is a C# analog to the main.c example file for C/C++
//
//                       
//////////////////////////////////////////////////////////////////////////////


namespace CSharpSample
{
    class Program
    {
        static void Main(string[] args)
        {
            int handle;
            ISenseLib.ISD_TRACKING_DATA_TYPE data;
            ISenseLib.ISD_STATION_INFO_TYPE[] Station;
            ISenseLib.ISD_TRACKER_INFO_TYPE Tracker;
            ISenseLib.ISD_HARDWARE_INFO_TYPE hwInfo;
            IntPtr handy;
            bool done = false;
            int station = 1;
            uint maxStations = 8;
            float lastTime;

            // Detect first tracker. If you have more than one InterSense device and
            // would like to have a specific tracker, connected to a known port, 
            // initialized first, then enter the port number instead of 0. Otherwise, 
            // tracker connected to the rs232 port with lower number is found first 
            handy = new IntPtr();
            Console.ForegroundColor = ConsoleColor.Cyan;
            Console.Out.WriteLine("Connecting to InterSense tracking device...");
            // handle = ISenseLib.ISD_OpenTracker(IntPtr.Zero, 0, false, true);
            handle = ISenseLib.ISD_OpenAllTrackers(IntPtr.Zero, ref handy, false, true);
            // Check value of handle to see if tracker was located 
            if (handle < 1)
            {
                Console.Out.WriteLine("Failed to detect InterSense tracking device");
            }
            else
            {
                Console.Out.WriteLine("Connected; press 'q' to quit, 'e' for enhancement, 'i' to get info\n");

                if (handle > 0)
                {
                    Tracker = new ISenseLib.ISD_TRACKER_INFO_TYPE();
                    hwInfo = new ISenseLib.ISD_HARDWARE_INFO_TYPE();
                    Station = new ISenseLib.ISD_STATION_INFO_TYPE[8];

                    // Get tracker configuration info 
                    ISenseLib.ISD_GetTrackerConfig(handle, ref Tracker, true);

                    if (ISenseLib.ISD_GetSystemHardwareInfo(handle, ref hwInfo))
                    {
                        if (hwInfo.Valid)
                        {
                            maxStations = hwInfo.Cap_MaxStations;
                        }
                    }

                    lastTime = ISenseLib.ISD_GetTime();

                    ISenseLib.ISD_GetStationConfig(handle, ref Station[station - 1], station, true);

                    data = new ISenseLib.ISD_TRACKING_DATA_TYPE();

                    while (!done)
                    {
                        ISenseLib.ISD_GetTrackingData(handle, ref data);

                        if (ISenseLib.ISD_GetTime() - lastTime > 0.02f)
                        {
                            lastTime = ISenseLib.ISD_GetTime();

                            showStationData(handle, Tracker,
                                Station[station - 1], data.Station[station - 1]);
                        }

                        if (Console.KeyAvailable)
                        {
                            switch (Console.ReadKey(true).KeyChar)
                            {
                                case '1':
                                    station = 1;
                                    Console.Write("\n>> Current Station is set to {0:d} <<\n", station);
                                    break;
                                case '2':
                                    if (maxStations > 1)
                                    {
                                        station = 2;
                                        Console.Write("\n>> Current Station is set to {0:d} <<\n", station);
                                    }
                                    break;
                                case '3':
                                    if (maxStations > 2)
                                    {
                                        station = 3;
                                        Console.Write("\n>> Current Station is set to {0:d} <<\n", station);
                                    }
                                    break;
                                case '4':
                                    if (maxStations > 3)
                                    {
                                        station = 4;
                                        Console.Write("\n>> Current Station is set to {0:d} <<\n", station);
                                    }
                                    break;

                                case '5':
                                    if (maxStations > 4)
                                    {
                                        station = 5;
                                        Console.Write("\n>> Current Station is set to {0:d} <<\n", station);
                                    }
                                    break;
                                case '6':
                                    if (maxStations > 5)
                                    {
                                        station = 6;
                                        Console.Write("\n>> Current Station is set to {0:d} <<\n", station);
                                    }
                                    break;
                                case '7':
                                    if (maxStations > 6)
                                    {
                                        station = 7;
                                        Console.Write("\n>> Current Station is set to {0:d} <<\n", station);
                                    }
                                    break;
                                case '8':
                                    if (maxStations > 7)
                                    {
                                        station = 8;
                                        Console.Write("\n>> Current Station is set to {0:d} <<\n", station);
                                    }
                                    break;
                                case 'Q':
                                case 'q':
                                    done = true;
                                    break;
                                case 'I':
                                case 'i':
                                    showTrackerStats(handle, ref hwInfo);
                                    break;
                                case 'e': // Set enhancement; IS-x products only, not for InterTrax 
                                case 'E':

                                    // First get current station configuration 
                                    if (ISenseLib.ISD_GetStationConfig(handle,
                                        ref Station[station - 1], station, true))
                                    {
                                        // Cycle enhancement
                                        Station[station - 1].Enhancement =
                                            (Station[station - 1].Enhancement + 1) % 3;

                                        // Send the new configuration to the tracker 
                                        if (ISenseLib.ISD_SetStationConfig(handle,
                                            ref Station[station - 1], station, true))
                                        {
                                            // display the results 
                                            showTrackerStats(handle, ref hwInfo);
                                        }
                                    }
                                    break;

                                case 't':
                                case 'T':
                                    // First get current station configuration
                                    if (ISenseLib.ISD_GetStationConfig(handle,
                                        ref Station[station - 1], station, true))
                                    {
                                        Station[station - 1].TimeStamped = !(Station[station - 1].TimeStamped);

                                        // Send the new configuration to the tracker
                                        if (ISenseLib.ISD_SetStationConfig(handle,
                                            ref Station[station - 1], station, true))
                                        {
                                            // display the results
                                            showTrackerStats(handle, ref hwInfo);
                                        }
                                    }

                                    break;

                                case 'd':
                                case 'D':
                                    showTrackerStats(handle, ref hwInfo);
                                    break;

                                case 'p':
                                case 'P':

                                    // First get current station configuration 
                                    if (ISenseLib.ISD_GetStationConfig(handle,
                                        ref Station[station - 1], station, true))
                                    {
                                        // Cycle enhancement
                                        Station[station - 1].Prediction =
                                            (Station[station - 1].Prediction + 10) % 60;

                                        // Send the new configuration to the tracker 
                                        if (ISenseLib.ISD_SetStationConfig(handle,
                                            ref Station[station - 1], station, true))
                                        {
                                            // display the results 
                                            showTrackerStats(handle, ref hwInfo);
                                        }
                                    }
                                    break;
                                case 's':
                                case 'S':

                                    // First get current station configuration 
                                    if (ISenseLib.ISD_GetStationConfig(handle,
                                        ref Station[station - 1], station, true))
                                    {
                                        // Cycle enhancement
                                        Station[station - 1].Sensitivity =
                                            (Station[station - 1].Sensitivity + 1) % 5;

                                        if (Station[station - 1].Sensitivity == 0)
                                            Station[station - 1].Sensitivity = 1;

                                        // Send the new configuration to the tracker 
                                        if (ISenseLib.ISD_SetStationConfig(handle,
                                            ref Station[station - 1], station, true))
                                        {
                                            // display the results 
                                            showTrackerStats(handle, ref hwInfo);
                                        }
                                    }
                                    break;
                                case 'c':
                                case 'C':

                                    // First get current station configuration 
                                    if (ISenseLib.ISD_GetStationConfig(handle,
                                        ref Station[station - 1], station, true))
                                    {
                                        // Cycle enhancement
                                        Station[station - 1].Compass =
                                            (Station[station - 1].Compass + 1) % 3;

                                        // Send the new configuration to the tracker 
                                        if (ISenseLib.ISD_SetStationConfig(handle,
                                            ref Station[station - 1], station, true))
                                        {
                                            // display the results 
                                            showTrackerStats(handle, ref hwInfo);
                                        }
                                    }
                                    break;
                            }
                        }
                    }

                    ISenseLib.ISD_CloseTracker(handle);
                }
            }
        }

        //==========================================================================================
        //
        //  Display Tracker data
        //
        //==========================================================================================
        static public void showStationData(int handle, ISenseLib.ISD_TRACKER_INFO_TYPE Tracker,
            ISenseLib.ISD_STATION_INFO_TYPE Station, ISenseLib.ISD_STATION_DATA_TYPE data)
        {
            Console.ForegroundColor = ConsoleColor.Yellow;

            // display position only if system supports it
            if (Tracker.TrackerModel == (uint)ISenseLib.ISD_SYSTEM_MODEL.ISD_IS600 ||
                Tracker.TrackerModel == (uint)ISenseLib.ISD_SYSTEM_MODEL.ISD_IS900 ||
                Tracker.TrackerModel == (uint)ISenseLib.ISD_SYSTEM_MODEL.ISD_IS1200 ||
                Tracker.TrackerModel == (uint)ISenseLib.ISD_SYSTEM_MODEL.ISD_IS1500)
            {
                Console.Write("[{0:d2}%] ({1:F2},{2:F2},{3:F2})m ",
                   (int)(data.TrackingStatus / 2.55), data.Position[0], data.Position[1], data.Position[2]);
            }

            // all products can return orientation
            if (Station.AngleFormat == ISenseLib.ISD_QUATERNION)
            {
                Console.Write("{0:F2} {1:F2} {2:F2} {3:F2}   ",
                    data.Quaternion[0], data.Quaternion[1],
                    data.Quaternion[2], data.Quaternion[3]);
                Console.ForegroundColor = ConsoleColor.Red;
                Console.Write("Time: {0:F1}       ", data.TimeStamp);
            }
            else // Euler angles
            {
                Console.Write("({0:F2},{1:F2},{2:F2})deg ",
                    data.Euler[0], data.Euler[1], data.Euler[2]);
                Console.ForegroundColor = ConsoleColor.Red;
                Console.Write("Time: {0:F1}       ", data.TimeStamp);
            }

            // Reset the cursor
            Console.CursorLeft = 0;
        }

        //==========================================================================================
        //
        //  Get and display tracker information
        //
        //==========================================================================================
        static public void showTrackerStats(int handle, ref ISenseLib.ISD_HARDWARE_INFO_TYPE hwInfo)
        {
            ISenseLib.ISD_TRACKER_INFO_TYPE Tracker;
            ISenseLib.ISD_STATION_INFO_TYPE Station;
            
            int i, numStations = 4;
            Tracker = new ISenseLib.ISD_TRACKER_INFO_TYPE();
            Station = new ISenseLib.ISD_STATION_INFO_TYPE();

            Console.ForegroundColor = ConsoleColor.Green;

            if (ISenseLib.ISD_GetTrackerConfig(handle, ref Tracker, true))
            {
                Console.Out.WriteLine("\n\n********** InterSense Tracker Information ***********\n\n");

                Console.Out.WriteLine("DLL Version: " + Tracker.LibVersion);
                Console.Out.WriteLine("Type:        " + systemType((int)Tracker.TrackerType) + " device on port " +
                        Tracker.Port );

                Console.Out.WriteLine("Model:       " + (hwInfo.Valid ?  
                    new String(hwInfo.ModelName) : "Unknown Tracker" ) );

                switch( Tracker.TrackerModel ) 
                {
                    case (int)ISenseLib.ISD_SYSTEM_MODEL.ISD_IS300:
                    case (int)ISenseLib.ISD_SYSTEM_MODEL.ISD_IS1200:
                        numStations = 4;
                        break;
                    case (int)ISenseLib.ISD_SYSTEM_MODEL.ISD_IS600:
                    case (int)ISenseLib.ISD_SYSTEM_MODEL.ISD_IS900:
                        numStations = ISenseLib.ISD_MAX_STATIONS;
                        break;
                    default:
                        numStations = 1;
                        break;
                }
                
                Console.Out.WriteLine( "\nStation\tTime\tState\tCube  Enhancement  Sensitivity  Compass  Prediction" );
                
                for(i = 1; i <= numStations; i++)
                {
                    Console.Out.Write("{0:d}\t", i);

                    if (ISenseLib.ISD_GetStationConfig(handle, ref Station, i, false))
                    {

                        Console.Out.WriteLine("{0:s}\t{1:s}\t{2:s}\t   {3:d}\t\t{4:d}\t  {5:d}\t  {6:d}", 
                            Station.TimeStamped ? "ON" : "OFF", 
                            Station.State ? "ON" : "OFF",
                            Station.InertiaCube == -1 ? "None" : Station.InertiaCube.ToString(), 
                            Station.Enhancement, 
                            Station.Sensitivity, 
                            Station.Compass, 
                            Station.Prediction );
                    }
                    else
                    {
                        Console.Out.WriteLine("ISD_GetStationConfig failed");
                        break;
                    }
                }
                Console.Out.WriteLine();
            }
            else
            {
                Console.Out.WriteLine("ISD_GetTrackerConfig failed");
            }
        }

        // Return a string representing system type
        static public String systemType( int Type ) 
        {
            switch( Type ) 
            {

                case (int)ISenseLib.ISD_SYSTEM_TYPE.ISD_NONE:
                    return "Unknown";
                case (int)ISenseLib.ISD_SYSTEM_TYPE.ISD_PRECISION_SERIES:
                    return "IS Precision Series";
                case (int)ISenseLib.ISD_SYSTEM_TYPE.ISD_INTERTRAX_SERIES:
                    return "InterTrax Series";
            }

            return "Unknown";
        }
    }
}
