using UnityEngine;
using System.Collections;
using ISense;
using System;
using System.Diagnostics;

public class ISenseSample : MonoBehaviour
{
	//	INTERSENSE	  //
	//==========================================================================================
	//
	//  ISenseSample
	//  Copyright Thales Visionix 2017 - All rights Reserved.
	//	Last updated 2017-04-24
	//	
	//  Sample code for connecting InterSense devices to Unity
	//
	//==========================================================================================
	
	
	public 	int 	station 		= 1; 			// Current station number. 1 is the only valid value number on IS-1500
	public 	bool 	sync 			= true; 		// Whether to attempt to sync using prediction
	
	public 	string 	sfHubDirWin 	= "../sfHub"; 	// Windows directory for sfHub
	public 	string 	sfHubNameWin 	= "sfHub.exe"; 	// Windows executable for sfHub
	
	int 	handle 			= 0; 					// Handle for which tracker to use.
	int 	framePredict 	= -1; 					// How many frames to predict to. Negative values are frames in the past.
	
	bool 	sfHubLaunched 	= false; 				// Whether we have tried to launch sfHub
	bool 	isSfAccess 		= false; 				// Whether this is an sfAccess device
	
	sbyte 	trackState 		= 0;					// IS1200 and IS1500 only. Denotes the state the tracker is in.
	// Without NFT:  -1, 0: No comm, 1: Initializing, 2: Tracking without point of reference, 3: tracking with point of reference, 4: tracking but cannot see point of reference
	// With NFT: 0: No comm, 1: Tracking without point of reference, 2: tracking with POR, 3: diverged
	
	Process sfHub;									// Process for launching sfHub
	
	ISenseLib.ISD_TRACKING_DATA_TYPE 	data;			// Storage for tracker data
	ISenseLib.ISD_STATION_INFO_TYPE[] 	stationInfo;	// Storage for station info
	ISenseLib.ISD_TRACKER_INFO_TYPE 	tracker;		// Storage for tracker info
	
	int									bufferSize;		// TEST ring buffer stuff

	ISenseLib.ISD_SYSTEM_MODEL			trackerModel;   // Tracker model
	
	//==========================================================================================
	//
	//  Initializes device
	//
	//==========================================================================================
	void Start()
	{
		// Check if an sfAccess.ini file exists in the local directory
		isSfAccess = System.IO.File.Exists("./sfAccess.ini");
		
		if (!isSfAccess)
		{
			// If sfAccess.ini is not in local directory, check other valid locations
			
			if (Application.platform == RuntimePlatform.WindowsPlayer || Application.platform == RuntimePlatform.WindowsEditor)
			{
				isSfAccess = System.IO.File.Exists("C:/Dev/sfAccess.ini");
				isSfAccess = isSfAccess || System.IO.File.Exists("C:/sfAccess.ini");
			}
			else if (Application.platform == RuntimePlatform.LinuxPlayer)
			{
				isSfAccess = System.IO.File.Exists("/dev/sfAccess.ini");
				isSfAccess = isSfAccess || System.IO.File.Exists("/sfAccess.ini");
			}
		}
		
		/*
         * This section launches sfHub, if using sfAccess
         */
		
		if (isSfAccess)
		{
			if (sfHub != null && !sfHub.HasExited)
			{
				sfHub.Kill();
				sfHub.WaitForExit(); // If we already launched sfHub through unity and are still running it, kill it
			}
			int sfHubs = System.Diagnostics.Process.GetProcessesByName("sfHub").Length; // Check if sfHub is already running
			
			if (sfHubs == 0) // If sfHub is not already running, launch it
			{
				sfHub = new Process();
				string file = "";
				string dir = "";
				
				// Get sfHub path based on OS
				if (Application.platform == RuntimePlatform.WindowsPlayer || Application.platform == RuntimePlatform.WindowsEditor)
				{
					file = sfHubDirWin + "/" + sfHubNameWin;
					dir = sfHubDirWin;

					// Get full sfHub path
					sfHub.StartInfo.FileName = System.IO.Path.GetFullPath(System.IO.Path.Combine(System.IO.Directory.GetCurrentDirectory(), @file));
					sfHub.StartInfo.WorkingDirectory = System.IO.Path.GetFullPath(System.IO.Path.Combine(System.IO.Directory.GetCurrentDirectory(), @dir));
					
					sfHub.EnableRaisingEvents = true;
					
					// Launch sfHub if it actually exists
					if (System.IO.File.Exists(sfHub.StartInfo.FileName))
					{
						sfHub.Start();
						sfHubLaunched = true;
						System.Threading.Thread.Sleep(5000);
					}
					else
					{
						sfHubLaunched = false;
					}
				}
				else
				{
					sfHubLaunched = false;
				}

			}
		}
		AttemptConnection ();
	}
	IEnumerator Sleep(float time)
	{
		yield return new WaitForSeconds(time);
	}
	
	//==========================================================================================
	//
	//  Attempts to initialize connection to tracker
	//
	//==========================================================================================
	void AttemptConnection()
	{
		DateTime maxWait = System.DateTime.Now;
		DateTime curTime = System.DateTime.Now;
		maxWait = maxWait.AddSeconds(10.0);
		
		handle = 0;
		
		while (maxWait.CompareTo(curTime) > 0 && handle < 1) //check for tracker for 10 seconds
		{
			handle = ISenseLib.ISD_OpenTracker(IntPtr.Zero, 0, false, true);
			if(handle < 1)
			{
				Sleep (.1f);
			}
			curTime = System.DateTime.Now;
		}

		// Check value of handle to see if tracker was located 
		if (handle < 1)
		{
			print("Failed to detect");
			print(handle);
		}
		else
		{
			print("Connected to InterSense tracking device!");
			tracker 	= new ISenseLib.ISD_TRACKER_INFO_TYPE();
			stationInfo = new ISenseLib.ISD_STATION_INFO_TYPE[8];
			data 		= new ISenseLib.ISD_TRACKING_DATA_TYPE(); 
			
			// Get tracker configuration info 
			ISenseLib.ISD_GetTrackerConfig(handle, ref tracker, true);
			ISenseLib.ISD_GetStationConfig(handle, ref stationInfo[station - 1], station, true);
			ISenseLib.ISD_SetStationConfig(handle, ref stationInfo[station - 1], station, true);


			trackerModel = (ISenseLib.ISD_SYSTEM_MODEL)tracker.TrackerModel;
			bufferSize	= 180;
			
			ISenseLib.ISD_RingBufferSetup(handle, station, null, bufferSize);
			
			ISenseLib.ISD_RingBufferStart(handle, station);
		}
	}
	
	//==========================================================================================
	//
	//  Called once per frame. If we've connected to a tracker, get its data and move attached object with device
	//
	//==========================================================================================
	void Update()
	{
		if (handle > 0)
		{
			
			if (sync)
			{ // Try to predict
				ISenseLib.ISD_STATION_DATA_TYPE bufferData = new ISenseLib.ISD_STATION_DATA_TYPE();
				ISenseLib.ISD_RingBufferQuery(handle, station, ref bufferData, new IntPtr(), new IntPtr());
				
				float dT = (float)Time.deltaTime;
				float curTime = bufferData.TimeStamp; //TODO: is this right?
				float timeToPredict = curTime + (framePredict * dT);
				
				
				GetRingBufferDataAtTime(timeToPredict);
			}
			else
			{
				ISenseLib.ISD_GetTrackingData(handle, ref data); // Get the tracking data
			}
			
			ISenseLib.ISD_STATION_DATA_TYPE dataSet = data.Station[station - 1]; // Specify the dataset
			
			// Only IS-1200 and IS-1500 have trackState
			if (trackerModel == ISenseLib.ISD_SYSTEM_MODEL.ISD_IS1200 || trackerModel == ISenseLib.ISD_SYSTEM_MODEL.ISD_IS1500)
			{
				trackState = dataSet.TrackingState;
			}

			Vector3 pos;
			
			// If device can control position, control position!
			
			if (trackerModel == ISenseLib.ISD_SYSTEM_MODEL.ISD_IS600 ||
			    trackerModel == ISenseLib.ISD_SYSTEM_MODEL.ISD_IS900 ||
			    trackerModel == ISenseLib.ISD_SYSTEM_MODEL.ISD_IS1200 ||
			    trackerModel == ISenseLib.ISD_SYSTEM_MODEL.ISD_IS1500)
			{
				pos = new Vector3(dataSet.Position[1], -dataSet.Position[2], dataSet.Position[0]); // Their y is up/down while that's our z
				
				transform.localPosition = pos;
			}
			
			// Time to deal with rotation!
			
			float[] cbn = dataSet.Cbn;
			
			float[] eulers;
			
			cbn = AngleUtil.ConvertUnityCbn (cbn); // Converts our right-handed rotation frame to unity's left-handed one.
			
			eulers = AngleUtil.Cbn2Euler(cbn);
			
			transform.localRotation = Quaternion.Euler(eulers[1], eulers[2], eulers[0]); // Pitch, Yaw, Roll
		} 
	}
	
	
	
	//==========================================================================================
	//
	//  Runs when the object closes, either by turning it off within Unity or closing the program
	//
	//==========================================================================================
	private void OnDisable()
	{
		UnityEngine.Debug.Log("closing dll");
		ISenseLib.ISD_CloseTracker(handle); // Severs the connection to the tracker
		
		// If sfHub is open, we should close it
		if (sfHub != null && isSfAccess)
		{
			if(!sfHub.HasExited)
			{
				UnityEngine.Debug.Log("closing sfHub");
				sfHub.Kill();
				
				sfHub.WaitForExit();
				UnityEngine.Debug.Log("sfHub closed");
			}
			
			sfHubLaunched = false;
			sfHub = null;
		}
		UnityEngine.Debug.Log("done with isense sample");
	}
	//==========================================================================================
	//
	//  Stops and restarts streaming
	//
	//==========================================================================================
	public void SoftReset()
	{
		ISenseLib.ISD_GetStationConfig(handle, ref stationInfo[station - 1], station, true); // Get station config so we don't change anything accidentally

		stationInfo[station - 1].State = false; // Setting state to false stops streaming
		ISenseLib.ISD_SetStationConfig(handle, ref stationInfo[station - 1], station, true); // Send station config back to tracker
		stationInfo[station - 1].State = true; // Re-enable streaming
		ISenseLib.ISD_SetStationConfig(handle, ref stationInfo[station - 1], station, true); // Send station config back to tracker
	}

	//==========================================================================================
	//
	//  Closes, then re-launches sfHub and reconnects to tracker.
	//
	//==========================================================================================
	public void Reset()
	{
		OnDisable(); // Close sfHub
		
		if(!sfHubLaunched) // If there are sfHubs we didn't launch ourselves, close them too
		{
			Process[] sfHubs = System.Diagnostics.Process.GetProcessesByName("sfHub");
			
			for(int i = 0; i < sfHubs.Length; i = i + 1)
			{
				if(!sfHubs[i].HasExited)
				{
					sfHubs[i].Kill();
					sfHubs[i].WaitForExit();
				}
			}
		}
		
		Start(); // Reconnect to tracker
	}
	
	//==========================================================================================
	//
	//  Approximates data at time using closest data in ring buffer
	//
	//==========================================================================================
	
	void GetRingBufferDataAtTime(float time)
	{
		ISenseLib.ISD_GetTrackingData (handle, ref data);
		float[] lastPos = new float[3];
		float[] lastCbn = new float[9];
		float lastTime = -1;
		
		// Search for first data point after time
		while(data.Station[station - 1].NewData == 1 && data.Station[station - 1].TimeStamp < time)
		{
			// Store old data
			for(int i = 0; i < 3; i = i + 1)
			{
				lastPos[i] = data.Station[station - 1].Position[i];
			}
			for(int i = 0; i < 9; i = i + 1)
			{
				lastCbn[i] = data.Station[station - 1].Cbn[i];
			}
			lastTime = data.Station[station - 1].TimeStamp;
			ISenseLib.ISD_GetTrackingData (handle, ref data);
		}
		// If last data was saved
		if (lastTime > 0) 
		{
			float postTime = data.Station[station - 1].TimeStamp - time;
			float prevTime = time - lastTime;
			
			float postWeight = 1 - (postTime / (postTime + prevTime));
			float prevWeight = 1 - (prevTime / (postTime + prevTime));
			
			// Approximate data at time by weighing data from immediately before and immediately after
			
			for(int i = 0; i < 3; i = i + 1)
			{
				data.Station[station - 1].Position[i] = (data.Station[station - 1].Position[i] * postWeight) + (lastPos[i] * prevWeight);
			}
			for(int i = 0; i < 9; i = i + 1)
			{
				data.Station[station - 1].Cbn[i] = (data.Station[station - 1].Cbn[i] * postWeight) + (lastCbn[i] * prevWeight);
			}
			data.Station[station - 1].TimeStamp = time;
		}
	}
	
	//==========================================================================================
	//
	//  Mutators
	//
	//==========================================================================================
	
	public void ToggleSync(GameObject target)
	{
		ToggleSync(target.GetComponent<UnityEngine.UI.Toggle> ().isOn); //Sets sync to toggle value
	}
	
	public void ToggleSync()
	{
		ToggleSync (!this.sync);
	}
	
	public void ToggleSync(bool value)
	{
		this.sync = value;
		//If we are syncing, turn buffer on.
		if (this.sync) 
		{
			ISenseLib.ISD_RingBufferStart(handle, station);
		}
		else 
		{
			ISenseLib.ISD_RingBufferStop(handle, station);
		}
	}
	
	public void SetPrediction(int pred)
	{
		ISenseLib.ISD_GetStationConfig(handle, ref stationInfo[station - 1], station, true);
		stationInfo[station - 1].Prediction = pred;
		ISenseLib.ISD_SetStationConfig(handle, ref stationInfo[station - 1], station, true);
	}
	
	public bool BoresightReferenced(float yaw, float pitch, float roll)
	{
		return ISenseLib.ISD_BoresightReferenced (handle, station, yaw, pitch, roll);
	}
	
	public void setFramePredict(int predict)
	{
		this.framePredict = predict;
	}
	
	//==========================================================================================
	//
	//  Accessors
	//
	//==========================================================================================
	
	public int GetHandle()
	{
		return this.handle;
	}
	
	public int GetTrackState()
	{
		return this.trackState;
	}
	
	public ISenseLib.ISD_TRACKER_INFO_TYPE getTracker()
	{
		return this.tracker;
	}
	
	public ISenseLib.ISD_TRACKING_DATA_TYPE getData()
	{
		return this.data;
	}
	
	public int GetStation()
	{
		return this.station;
	}
	
	public bool DidsfHubLaunch()
	{
		return sfHubLaunched;
	}

	public ISenseLib.ISD_SYSTEM_MODEL GetModel()
	{
		return trackerModel;
	}
	
	//==========================================================================================
	//
	//  Display Tracker data in Unity console
	//
	//==========================================================================================
	static public void ShowStationData(int handle, ISenseLib.ISD_TRACKER_INFO_TYPE Tracker,
	                                   ISenseLib.ISD_STATION_INFO_TYPE Station, ISenseLib.ISD_STATION_DATA_TYPE data)
	{
		
		// Display position only if system supports it
		if (Tracker.TrackerModel == (uint)ISenseLib.ISD_SYSTEM_MODEL.ISD_IS600 ||
		    Tracker.TrackerModel == (uint)ISenseLib.ISD_SYSTEM_MODEL.ISD_IS900 ||
		    Tracker.TrackerModel == (uint)ISenseLib.ISD_SYSTEM_MODEL.ISD_IS1200 ||
		    Tracker.TrackerModel == (uint)ISenseLib.ISD_SYSTEM_MODEL.ISD_IS1500)
		{
			print("[" + (int)(data.TrackingStatus / 2.55) + "%] (" + data.Position[0] + "," + data.Position[1] + "," + data.Position[2] + ")m ");
		}
		
		// All products can return orientation
		if (Station.AngleFormat == ISenseLib.ISD_QUATERNION)
		{
			print(data.Quaternion[0] + " " + data.Quaternion[1] + " " + data.Quaternion[2] + " " + data.Quaternion[3] + " Time: " + data.TimeStamp);
		}
		else // Euler angles
		{
			print("(" + data.Euler[0] + "," + data.Euler[1] + "," + data.Euler[2] + ")deg Time: " + data.TimeStamp);
		}
	}
	
	//==========================================================================================
	//
	//  Prints pretty much everything not in ShowStationData
	//
	//==========================================================================================
	static public void DebugPrintEverything(int handle, ISenseLib.ISD_TRACKER_INFO_TYPE Tracker,
	                                        ISenseLib.ISD_STATION_INFO_TYPE Station, ISenseLib.ISD_STATION_DATA_TYPE data)
	{
		String s;
		s = "isense Lib Version: " + Tracker.LibVersion;
		s = s + " Tracker Type: " + Tracker.TrackerType;
		s = s + " Tracker Model: " + Tracker.TrackerModel;
		s = s + " Port: " + Tracker.Port;
		s = s + " Records Per Second: " + Tracker.RecordsPerSec;
		s = s + " Kbits Per Second: " + Tracker.KBitsPerSec;
		s = s + " Sync State: " + Tracker.SyncState;
		s = s + " Sync Rate: " + Tracker.SyncRate;
		s = s + " Sync Phase: " + Tracker.SyncPhase;
		s = s + " Interface: " + Tracker.Interface;
		s = s + " Ult Timeout: " + Tracker.UltTimeout;
		s = s + " Ult Volume: " + Tracker.UltVolume;
		s = s + " Firmware Revision: " + Tracker.FirmwareRev;
		s = s + " LED Enable: " + Tracker.LedEnable;
		
		print("ISD_TRACKER_INFO_TYPE");
		print(s);
		
		s = "ID: " + Station.ID;
		s = s + " State: " + Station.State;
		s = s + " Compass: " + Station.Compass;
		s = s + " InertiaCube: " + Station.InertiaCube;
		s = s + " Enhancement: " + Station.Enhancement;
		s = s + " Sensitivity: " + Station.Sensitivity;
		s = s + " Prediction: " + Station.Prediction;
		s = s + " AngleFormat: " + Station.AngleFormat;
		s = s + " TimeStamped: " + Station.TimeStamped;
		s = s + " GetInputs: " + Station.GetInputs;
		s = s + " GetEncoderData: " + Station.GetEncoderData;
		s = s + " Compass Compensation: " + Station.CompassCompensation;
		s = s + " ImuShockSuppression: " + Station.ImuShockSuppression;
		s = s + " UrmRejectionFactor: " + Station.UrmRejectionFactor;
		s = s + " GetAHRSData: " + Station.GetAHRSData;
		s = s + " CoordFrame: " + Station.CoordFrame;
		s = s + " AccelSensitivity: " + Station.AccelSensitivity;
		for (int i = 0; i < 3; i = i + 1)
		{
			s = s + " TipOffset[" + i + "]: " + Station.TipOffset[i];
		}
		s = s + " GetCameraData: " + Station.GetCameraData;
		s = s + " GetAuxInputs: " + Station.GetAuxInputs;
		s = s + " GetCovarianceData: " + Station.GetCovarianceData;
		s = s + " GetExtendedData: " + Station.GetExtendedData;
		
		print("ISD_STATION_INFO_TYPE");
		print(s);
		
		ISenseLib.ISD_HARDWARE_INFO_TYPE hwInfo = new ISenseLib.ISD_HARDWARE_INFO_TYPE();
		ISenseLib.ISD_GetSystemHardwareInfo(handle, ref hwInfo);
		
		s = "Valid: " + hwInfo.Valid;
		s = s + " TrackerType: " + hwInfo.TrackerType;
		s = s + " TrackerModel: " + hwInfo.TrackerModel;
		s = s + " Port: " + hwInfo.Port;
		s = s + " Interface: " + hwInfo.Interface;
		s = s + " OnHost: " + hwInfo.OnHost;
		s = s + " AuxSystem: " + hwInfo.AuxSystem;
		s = s + " FirmwareRev: " + hwInfo.FirmwareRev;
		s = s + " Model Name: " + hwInfo.ModelName;
		s = s + " Cap_Position: " + hwInfo.Cap_Position;
		s = s + " Cap_Orientation: " + hwInfo.Cap_Orientation;
		s = s + " Cap_Encoders: " + hwInfo.Cap_Encoders;
		s = s + " Cap_Prediction: " + hwInfo.Cap_Prediction;
		s = s + " Cap_Compass: " + hwInfo.Cap_Compass;
		s = s + " Cap_SelfTest: " + hwInfo.Cap_SelfTest;
		s = s + " Cap_ErrorLog: " + hwInfo.Cap_ErrorLog;
		s = s + " Cap_UltVolume: " + hwInfo.Cap_UltVolume;
		s = s + " Cap_UltGain: " + hwInfo.Cap_UltGain;
		s = s + " Cap_UltTimeout: " + hwInfo.Cap_UltTimeout;
		s = s + " Cap_PhotoDiode: " + hwInfo.Cap_PhotoDiode;
		s = s + " Cap_MaxStations: " + hwInfo.Cap_MaxStations;
		s = s + " Cap_MaxImus: " + hwInfo.Cap_MaxImus;
		s = s + " Cap_MaxFPses: " + hwInfo.Cap_MaxFPses;
		s = s + " Cap_MaxChannels: " + hwInfo.Cap_MaxChannels;
		s = s + " Cap_MaxButtons: " + hwInfo.Cap_MaxButtons;
		s = s + " Cap_MeasData: " + hwInfo.Cap_MeasData;
		s = s + " Cap_DiagData: " + hwInfo.Cap_DiagData;
		s = s + " Cap_PseConfig: " + hwInfo.Cap_PseConfig;
		s = s + " Cap_ConfigLock: " + hwInfo.Cap_ConfigLock;
		s = s + " Cap_UltMaxRange: " + hwInfo.Cap_UltMaxRange;
		s = s + " Cap_CompassCal: " + hwInfo.Cap_CompassCal;
		s = s + " BaudRate: " + hwInfo.BaudRate;
		s = s + " NumTestLevels: " + hwInfo.NumTestLevels;
		
		print("ISD_HARDWARE_INFO_TYPE");
		print(s);
		
		ISenseLib.ISD_STATION_HARDWARE_INFO_TYPE sh = new ISenseLib.ISD_STATION_HARDWARE_INFO_TYPE();
		ISenseLib.ISD_GetStationHardwareInfo(handle, ref sh, 0);
		
		s = "Valid: " + sh.Valid;
		s = s + " ID: " + sh.ID;
		s = s + " DescVersion: " + sh.DescVersion;
		s = s + " FirmwareRev: " + sh.FirmwareRev;
		s = s + " Serial Number: " + sh.SerialNum;
		s = s + " CalDate: " + sh.CalDate;
		s = s + " Port: " + sh.Port;
		s = s + " Type: " + sh.Type;
		
		print("ISD_STATION_HARDWARE_INFO_TYPE");
		print(s);
		
		s = "TrackingStatus: " + data.TrackingStatus;
		s = s + " NewData: " + data.NewData;
		s = s + " CommIntegrity: " + data.CommIntegrity;
		s = s + " BatteryState: " + data.BatteryState;
		s = s + " TimeStamp: " + data.TimeStamp;
		s = s + " StillTime: " + data.StillTime;
		s = s + " BatteryLevel: " + data.BatteryLevel;
		s = s + " CompassYaw: " + data.CompassYaw;
		s = s + " MeasQuality: " + data.MeasQuality;
		s = s + " HardIronCal: " + data.HardIronCal;
		s = s + " SoftIronCal: " + data.SoftIronCal;
		s = s + " EnvironmentalCal: " + data.EnvironmentalCal;
		s = s + " TimeSTampSeconds: " + data.TimeStampSeconds;
		s = s + " TimeStampMicroSec: " + data.TimeStampMicroSec;
		s = s + " OSTimeStampSeconds: " + data.OSTimeStampSeconds;
		s = s + " OSTimeSTampMicroSec: " + data.OSTimeStampMicroSec;
		s = s + " CompassQuality: " + data.CompassQuality;
		s = s + " Temperature: " + data.Temperature;
		for (int i = 0; i < 3; i = i + 1)
		{
			s = s + " MagBodyFrame[" + i + "]: " + data.MagBodyFrame[i];
		}
		s = s + " TrackingState: " + data.TrackingState;
		
		print("ISD_STATION_DATA_TYPE");
		print(s);
	}
}
