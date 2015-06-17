/**
 * Cubulus Serial Interface Controller
 * @author Norman Liebich
 * @year 2011/2012
 */ 

using UnityEngine;
using System;
using System.Collections;
using System.IO.Ports;
using System.Threading;


public class SerialController : MonoBehaviour{
	public const byte 
		RQS_STATE = 0x10,
		RQS_CALIBRATE = 0x16,
		RQS_VIBRATE = 0x20,
		RQS_SEND_DATA = 0x21;
	
	public string ComPort = "COM2";
	public int ComSpeed = 57600;
	
	static bool port_open=false;
	
	static SerialPort _SerialPort;
	static Quaternion SEq;
	static bool batteryEmpty;
	static float sensorTemperature;
	static byte[] writeBuffer = new byte[2] {0x00, 0x00};
	static byte writeBufferLength = 0;
	
	//private String guiString="";
	private static Thread readThread;
		
	public static void ThreadRead() {
		int sendCounter=0;
		float x, y, z, w;
		bool extSent = false;
		while (port_open) {
			// one Quaternion = 4x float(4B)
			byte[] rxBuffer = new byte[16]; 
			byte pType=0x00;
					
			try {
				if (sendCounter==0 || sendCounter > 10) {
					if (writeBufferLength>0 && !extSent) {
						_SerialPort.Write(writeBuffer, 0, writeBufferLength);
						extSent = true;
					}
					_SerialPort.Write("!");
					sendCounter=1;
				}
				else if (++sendCounter>1) {
					try {
						pType = (byte)_SerialPort.ReadByte();
						// Reset writeBuffer length
						if (extSent) {
							writeBufferLength=0;
							extSent = false;
						}
					}
					catch(TimeoutException) {
						sendCounter=0;
					}
					// Data-Packet
					if (pType == 0x01) {
						try {
							_SerialPort.Read(rxBuffer, 0, 16);
						}
						catch(TimeoutException) {
							sendCounter=0;
						}
						if (sendCounter>0) {
							w = BitConverter.ToSingle(rxBuffer, 0);
							x = BitConverter.ToSingle(rxBuffer, 4); // Gyro X
							// different assignment, here Y-axis is up
							y = -BitConverter.ToSingle(rxBuffer, 12); // Gyro Z
							z = BitConverter.ToSingle(rxBuffer, 8);	// Gyro Y
							if (x*x + y*y + z*z + w*w > 0.0) {
								SEq.w = w;
								SEq.x = x;
								SEq.y = y;
								SEq.z = z;
							}
						}
					}
					// Status-Packet
					else if (pType== 0x02) {
						_SerialPort.Read(rxBuffer, 0, 5);
						batteryEmpty = rxBuffer[0] == 0x01 ? true : false;
						sensorTemperature = BitConverter.ToSingle(rxBuffer, 1);
					}
					
					sendCounter=0;
				}
				
				
			}
			catch(Exception error) {
				Debug.Log(error);
			}
		}
	}
	
	void OnDestroy() {
		OnApplicationQuit();
	}
	
	void OnApplicationQuit() {
		if (port_open) {
			port_open=false;
			readThread.Abort();
			readThread.Join();
			// send standby
			_SerialPort.Write("#");
			_SerialPort.Close();
			Debug.Log("serial port closed");
		}
		//_instance = null;
	}
	
	public void Start() {
		/*
		string[] ports = SerialPort.GetPortNames();
		foreach(string portname in ports) {
			Debug.Log(portname);
		}
		*/
		
		SEq = new Quaternion(0,0,0,1);
		
		
		if (ComPort != CubeController.CTRL_KEYBOARD) {
			_SerialPort = new SerialPort(ComPort, ComSpeed, Parity.None, 8, StopBits.One);
			readThread = new Thread(ThreadRead);
			
			if (_SerialPort != null) {
				if (_SerialPort.IsOpen) {
					_SerialPort.Close();
				}
			}
			try {
				_SerialPort.Open();
				port_open=true;
			}
			catch(Exception) {
				port_open=false;
				Debug.LogWarning("Serial port could not be opened.");
			}
		}
		
		if (port_open) {
			// wirklich nötig?
			_SerialPort.ReadTimeout = 400;
			
			Debug.Log("Serial Port open");
			
			byte[] cmd = new byte[1] {RQS_CALIBRATE};
			
			_SerialPort.Write(cmd, 0, 1);
			
			readThread.Start();
		}
	}
	
	
	public Quaternion SensorEarthQuaternion {
		get {
			return new Quaternion(SEq.x, SEq.y, SEq.z, SEq.w);
		}
	}
	/*public void OnGUI() {
		String battState = batteryEmpty ? "critical!" : "okay";
		GUI.Label(new Rect(10, 10, 100, 20), "Battery: "+battState);
	}
	*/
	
	public void Recalibrate() {
		if (port_open) {
			writeBuffer[0] = RQS_CALIBRATE;
			writeBufferLength=1;
		}
	}
	
	public bool IsConnected {
		get {
			return port_open;
		}
	}
	
	// send a 
	public void Vibrate(int mSeconds) {
		if (port_open) {
			byte val = (byte)(mSeconds/10);

			writeBuffer[0] = RQS_VIBRATE;
			writeBuffer[1] = val;
			writeBufferLength=2;
			
		}
	}
	
	public Boolean batteryOK {
		get {
			return !batteryEmpty;
		}
	}
}
