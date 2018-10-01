using UnityEngine;
using System.Collections;
using System;

public class AngleUtil : MonoBehaviour {
	
	//==========================================================================================
	//
	//  Converts our Rotation Matrix to Unity's
	//
	//==========================================================================================
	public static float[] ConvertUnityCbn(float[] dataCbn)
	{
		float[] usToUnity;
		float[] unityToUs;
		usToUnity = new float[9] { 1, 0, 0, 0, 1, 0, 0, 0, -1 }; // Swap to left-handed
		unityToUs = Inverse(usToUnity);// Inverse
		
		float[] result = MultiplyCbn(unityToUs, dataCbn);
		
		
		result = MultiplyCbn(result, usToUnity);
		
		
		
		return result; // inverse(m) * X * m
	}
	
	//==========================================================================================
	//
	//  Multiplies two rotation matrices
	//
	//==========================================================================================
	public static float[] MultiplyCbn(float[] cbn1, float[] cbn2)
	{
		float[] resultCbn = new float[9];
		
		
		for (int i = 0; i < 3; i = i + 1)
		{
			for (int j = 0; j < 3; j = j + 1)
			{
				float num;
				num = cbn1[3 * i] * cbn2[j];
				
				num = num + cbn1[3 * i + 1] * cbn2[j + 3];
				num = num + cbn1[3 * i + 2] * cbn2[j + 6];
				resultCbn[3 * i + j] = num;
			}
		}
		
		return resultCbn;
	}
	
	//==========================================================================================
	//
	//  Converts rotation matrix to euler array
	//
	//==========================================================================================
	public static float[] Cbn2Euler(float[] dataCbn)
	{
		float[] eul = new float[3];
		
		
		
		if(dataCbn[6] <= 1.0f)
		{       
			// Pitch  = - asin( Cbn[2][0] ) 
			eul[1] = -(float)Math.Asin(dataCbn[6]);
		}
		else
		{
			eul[1] = 0;
		}
		// Roll = atan2( Cbn[2][1] , Cbn[2][2] )
		eul[0] = (float)Math.Atan2(dataCbn[7], dataCbn[8]);
		
		// Yaw = atan2( Cbn[1][0], Cbn[[0][0] )
		eul[2] = (float)Math.Atan2(dataCbn[3], dataCbn[0]);
		for (int i = 0; i < 3; i = i + 1)
		{
			eul[i] = RadianToDegree(eul[i]);
		}
		
		return eul;
		
	}
	
	//==========================================================================================
	//
	//  Converts rotation matrix to quaternion array
	//
	//==========================================================================================
	public static Quaternion Cbn2Quaternion(float[] dataCbn)
	{
		float 	t;
		float[] quat = new float[4];
		
		t = dataCbn[0] + dataCbn[4] + dataCbn[8];
		
		if (t > 0.0f)
		{
			//print("use w form");
			t = 0.5f / (float)Math.Sqrt(1.0f + t);
			quat[0] = t * (dataCbn[5] - dataCbn[7]);
			quat[1] = t * (dataCbn[6] - dataCbn[2]);
			quat[2] = t * (dataCbn[1] - dataCbn[3]);
			quat[3] = 0.25f / t;
		}
		else if ((dataCbn[0] > dataCbn[4]) && (dataCbn[0] > dataCbn[8]))
		{
			//print("use x form");
			t = 0.5f / (float)Math.Sqrt(1.0f + dataCbn[0] - dataCbn[4] - dataCbn[8]);
			quat[0] = 0.25f / t;
			quat[1] = t * (dataCbn[3] + dataCbn[1]);
			quat[2] = t * (dataCbn[6] + dataCbn[2]);
			quat[3] = t * (dataCbn[5] - dataCbn[7]);
		}
		else if (dataCbn[4] > dataCbn[8])
		{
			//print("use y form");              
			t = 0.5f / (float)Math.Sqrt(1.0f + dataCbn[4] - dataCbn[0] - dataCbn[8]);
			quat[0] = t * (dataCbn[3] + dataCbn[1]);
			quat[1] = 0.25f / t;
			quat[2] = t * (dataCbn[7] + dataCbn[5]);
			quat[3] = t * (dataCbn[6] - dataCbn[2]);
		}
		else
		{
			//print("use z form");
			t = -0.5f / (float)Math.Sqrt(1.0f + dataCbn[8] - dataCbn[0] - dataCbn[4]);
			quat[0] = t * (dataCbn[6] + dataCbn[2]);
			quat[1] = t * (dataCbn[7] + dataCbn[5]);
			quat[2] = 0.25f / t;
			quat[3] = t * (dataCbn[1] - dataCbn[3]);
		}
		
		for (int i = 0; i < 4; i = i + 1)
		{
			quat[i] = RadianToDegree(quat[i]);
		}
		return new Quaternion(quat[1], quat[2], quat[3], quat[0]); //Unity's 
	}
	
	//==========================================================================================
	//
	//  Converts from radian to degrees
	//
	//==========================================================================================
	public static float RadianToDegree(float angle)
	{
		return (float)(angle * (180.0 / Math.PI));
	}
	
	//==========================================================================================
	//
	//  Inverts rotation matrix
	//
	//==========================================================================================
	public static float[] Inverse(float[] cbn)
	{
		float[] result = new float[9];
		
		for (int i = 0; i < 3; i = i + 1)
		{
			for (int j = 0; j < 3; j = j + 1)
			{
				result[(3 * i) + j] = cbn[i + (3 * j)];
			}
		}
		
		return result;
	}
}
