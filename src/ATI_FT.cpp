//==========================================================================================================
// 					 This code is a part of the robot steerable catheter project 
//  		   		Canadian Surgical Technologies and Advanced Robotics (CSTAR). 
// 							Copyright (C) 2022 Navid Feizi <nfeizi@uwo.ca>
//					     Copyright (C) 2022 Filipe C. Pedrosa <fpedrosa@uwo.ca>
// 
//       Project developed under the supervision of Dr Jayender Jagadeesan (❖) Dr Rajni Patel (◈)	            
//  ========================================================================================================  
//   (◈) CSTAR (Canadian Surgical Technologies & Advanced Robotics) @ Western University, London, ON  Canada   
//             (❖) Surgical Planning Lab @ Brigham and Women's Hospital / Harvard Medical School				
//==========================================================================================================

/* This library is for reading data from NIDAQ ATI FT sensor and converting to a standatd units */

#include "ATI_FT.hpp"

/* Class contrsuctor 
	-> Channel: physical channel of the NI DAQ - example: {"Dev1/ai0:6"}
	-> calibrationAddress: address to the "***.cal" file with respect to the build location
	-> forceUnit: force unit must be selected from: lbf, Klbf, N, kN, g, kg
	-> torqueUnit: torque unit must be selected from: lbf-in, lbf-ft, N-m, k-mm, kg-cm, kN-m		*/
AtiSensor::AtiSensor(char* Channel, char* calibrationAddress,  char* forceUnit,  char* torqueUnit)
: address_cal(calibrationAddress), physicalChannel(Channel), unit_force(forceUnit), unit_torque(torqueUnit)
{
	// char physicalChannel[20] = {"Dev1/ai0:6"};		/* !!! MAY NEED TO CHANGE THIS!!!! MEASUREMENT & Automation Software !!! */
	// this->CalAddress_ptr = calAddress; //"../../../lib_ati/FT11712.cal";
	this->initialize_NIcard(physicalChannel);
	this->initialize_ATI_forcesensor(address_cal, unit_force, unit_torque);
}


/* class destructor: */
AtiSensor::~AtiSensor()
{
  std::cout << "ATI FT Closing...!!" << std::endl;
}


/* initilize NI DAQ and set physical channels and max voltages 
	-> channel: physical channel of the NI DAQ - example: {"Dev1/ai0:6"}		*/
void AtiSensor::initialize_NIcard(char* channel)
{
	// this is a sample of the channel: char physicalChannel[20] = {"Dev1/ai0:6"};		
	DAQmxCreateTask("", &this->taskHandle);
	DAQmxCreateAIVoltageChan(this->taskHandle, channel, "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, "");
	DAQmxStartTask(this->taskHandle);
}


/* initilize ATI force sensor (load .cal file and create calibration object) 
	-> force unit must be selected from: lbf, Klbf, N, kN, g, kg
	-> torque unit must be selected from: lbf-in, lbf-ft, N-m, k-mm, kg-cm, kN-m 	  */
int AtiSensor::initialize_ATI_forcesensor(char* address_cal, char* unit_force,  char* unit_torque)
{
	unsigned short index = 1;   // index of calibration in file (second parameter; default = 1)
	this->cal = createCalibration(address_cal, index);
	if (cal==NULL) {
		std::cout << "Specified calibration could not be loaded" << std::endl;
		return 0;
	}

	// set force unit - could be selected from: lbf, Klbf, N, kN, g, kg
	SetForceUnits(cal, unit_force); 
	// set torque unit - could be selected from: lbf-in, lbf-ft, N-m, k-mm, kg-cm, kN-m
	SetTorqueUnits(cal, unit_torque);
	return 1;
}


/* get unbiased force and torque 
	-> forceUB_reading: Fx, Fy, Fz, Tx, Ty, Tz	*/
void AtiSensor::GetForceUnBias(std::vector<double>& forceUB_reading)
{
	std::vector<double> force_reading = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
	this->GetForceBias(force_reading);
	for (int i=0; i<6; i++)
	{
		forceUB_reading[i] = force_reading[i] - this->bias[i];
		// ati_force[i] = (ati_force[i] + dt*w_cutoff_force*force_unbias[i]) / (1 + w_cutoff_force*dt);
	}	
}


/* get biased force and toruqe 
	-> force_reading: Fx, Fy, Fz, Tx, Ty, Tz	*/
void AtiSensor::GetForceBias(std::vector<double>& force_reading)
{
	float voltage[7];			// This array will hold the resultant force/torque vector.
    float force[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	force_reading = {{0,0,0,0,0,0}};

	GetVoltage(voltage);
	ConvertToFT(this->cal, voltage, force);		// convert voltage to force & torque
	for (int i=0; i<6; i++)
	{
		force_reading[i] = force[i];	// type cast
	}
	// std::cout << std::fixed;
	// std::cout << std::setprecision(4);
	// std::cout << "Fx: " << force_reading[0] << "   Fy: " << force_reading[1] << "   Fz: " << force_reading[2] << "   Tx: " << force_reading[3] << "   Ty: " << force_reading[4] << "   Tz: " << force_reading[5] << std::endl;
}


/* get gage voltage from the NI DAQcard */
void AtiSensor::GetVoltage(float* voltage_reading)
{
	float64     value[7];
	long int no_bytes_ni;
	DAQmxReadAnalogF64(this->taskHandle, 1, 10.0, DAQmx_Val_GroupByChannel, value, 7, &no_bytes_ni, 0);
	for(int i=0; i<7; i++)
		voltage_reading[i] = value[i];		// Typecasting
		// std::cout << "Acquired reading: " <<  value[0] << value[1] << value[2] << value[3] << value[4] << value[5] << value[6] << std::endl;
}


/* calcualted the offsets and update it. 
	This function should be called one time before data recording for proper offset cancelation using GetForceUnBias method */
void AtiSensor::NullOffsets()
{
	int numRecords = 100;
	int N_total[] = {0, 0, 0, 0, 0, 0};	
	std::vector<double> bias_temp = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
	std::vector<double> force_reading = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
	
	for(int i=0; i<numRecords; i++)
	{
		this->GetForceBias(force_reading);
		for(int j=0; j<6; j++)
		{
			if(force_reading[j] < 10000 && force_reading[j] > -10000)
			{
				bias_temp[j] += force_reading[j];
				N_total[j] += 1;
			}
		}
		Sleep(1);
	}

	for(int j=0; j<6; j++)
		this->bias[j] = bias_temp[j]/N_total[j];
}


/* close NI DAQ card */
void AtiSensor::CloseNICard(void)
{
	DAQmxStopTask(taskHandle);
	DAQmxClearTask(taskHandle);
	destroyCalibration(cal);
}