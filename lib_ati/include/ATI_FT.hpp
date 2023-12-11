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

#ifdef _WIN32
#include <windows.h>
#elif __linux__
#include <unistd.h>
#define Sleep(x) usleep((x) * 1000)
#endif

#include "ftconfig.h"
#include "NIDAQmx.h"
#include "dom.h"

#include <iostream>
#include <vector>
#include <iomanip>
#include <thread>

#include <blaze/Blaze.h>
#include <blaze/Math.h>
#include <blaze/math/DenseMatrix.h>

#define DAQmxErrChk(functionCall)                \
    {                                            \
        if (DAQmxFailed(error = (functionCall))) \
        {                                        \
            goto Error;                          \
        }                                        \
    }

class AtiSensor
{

public:
    /* Class contrsuctor
    -> Channel: physical channel of the NI DAQ - example: {"Dev1/ai0:6"}
    -> calibration_filename: file name of the "***.cal" file. It must be in "calibration" folder
    -> forceUnit: force unit must be selected from: lbf, Klbf, N, kN, g, kg
    -> torqueUnit: torque unit must be selected from: lbf-in, lbf-ft, N-m, k-mm, kg-cm, kN-m		*/
    AtiSensor(std::string Channel, std::string clibration_filename, std::string forceUnit, std::string torqueUnit);

    AtiSensor();

    /* Copy constructor */
    AtiSensor(const AtiSensor &rhs);

    /* move constructor */
    AtiSensor(AtiSensor &&rhs) noexcept;

    /* class destructor: */
    ~AtiSensor();

    void initialize();

    /* get biased force and toruqe
    -> force_reading: Fx, Fy, Fz, Tx, Ty, Tz	*/
    void ReadForce(blaze::StaticVector<double, 6> &force_reading);

    /* get unbiased force and torque
    -> forceUB_reading: Fx, Fy, Fz, Tx, Ty, Tz	*/
    void UnBias(blaze::StaticVector<double, 6> &force_reading, blaze::StaticVector<double, 6> &bias, blaze::StaticVector<double, 6> &forceUB_reading);

    /* calcualted the offsets and update it.
        This function should be called one time before data recording for proper offset cancelation using GetForceUnBias method */
    void NullOffsets(blaze::StaticVector<double, 6> &bias);

    /* close NI DAQ card */
    void CloseNICard(void);

    void Start_Read_Thread();

    void Read_Loop();

    void GetForce(blaze::StaticVector<double, 6> &force_reading);

private:
    char *address_cal;
	char *physical_channel;
	char *unit_force;
	char *unit_torque;
    Calibration *cal; // struct containing calibration information
    TaskHandle taskHandle;
    std::thread emThread;
    blaze::StaticVector<double, 6> bias = blaze::StaticVector<double, 6>(0.0);
    blaze::StaticVector<double, 6> force_unbiased = blaze::StaticVector<double, 6>(0.0);

    /* get gage voltage from the NI DAQcard */
    void ReadVoltage(float *voltage_reading);

    /* initilize NI DAQ and set physical channels and max voltages
    -> channel: physical channel of the NI DAQ - example: {"Dev1/ai0:6"}		*/
    void initialize_NIcard(char *channel);

    /* initilize ATI force sensor (load .cal file and create calibration object)
    -> force unit must be selected from: lbf, Klbf, N, kN, g, kg
    -> torque unit must be selected from: lbf-in, lbf-ft, N-m, k-mm, kg-cm, kN-m 	  */
    int initialize_ATI_forcesensor(char *address_cal, char *unit_force, char *unit_torque);
};
