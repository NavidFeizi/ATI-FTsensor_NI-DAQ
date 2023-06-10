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

#pragma once

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include "ftconfig.h"
#include "NIDAQmx.h"
#include "dom.h"

#include <iostream>
#include <vector>
#include <iomanip>

#define DAQmxErrChk(functionCall) { if( DAQmxFailed(error=(functionCall)) ) { goto Error; } }


class AtiSensor
{

public:

    /* Class contrsuctor 
	-> Channel: physical channel of the NI DAQ - example: {"Dev1/ai0:6"}
	-> calibrationAddress: address to the "***.cal" file with respect to the build location
	-> forceUnit: force unit must be selected from: lbf, Klbf, N, kN, g, kg
	-> torqueUnit: torque unit must be selected from: lbf-in, lbf-ft, N-m, k-mm, kg-cm, kN-m		*/
    AtiSensor(char* Channel, char* calibrationAddress,  char* forceUnit,  char* torqueUnit);

    /* class destructor: */
    ~AtiSensor();

    /* get biased force and toruqe 
	-> force_reading: Fx, Fy, Fz, Tx, Ty, Tz	*/
    void GetForceBias(std::vector<double>& force_reading);
    
    /* get unbiased force and torque 
	-> forceUB_reading: Fx, Fy, Fz, Tx, Ty, Tz	*/
    void GetForceUnBias(std::vector<double>& forceUB_reading);

    /* calcualted the offsets and update it. 
	This function should be called one time before data recording for proper offset cancelation using GetForceUnBias method */
    void NullOffsets();

    /* close NI DAQ card */
    void CloseNICard(void);


private:

    char* address_cal;
    char* physicalChannel;
    char* unit_force;
    char* unit_torque;
    Calibration *cal;		    // struct containing calibration information
    TaskHandle	taskHandle;
    std::vector<double> bias = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

    /* get gage voltage from the NI DAQcard */
    void GetVoltage(float* voltage_reading);

    /* initilize NI DAQ and set physical channels and max voltages 
	-> channel: physical channel of the NI DAQ - example: {"Dev1/ai0:6"}		*/
    void initialize_NIcard(char *channel);

    /* initilize ATI force sensor (load .cal file and create calibration object) 
	-> force unit must be selected from: lbf, Klbf, N, kN, g, kg
	-> torque unit must be selected from: lbf-in, lbf-ft, N-m, k-mm, kg-cm, kN-m 	  */
    int initialize_ATI_forcesensor(char* address_cal, char* unit_force,  char* unit_torque);

};

