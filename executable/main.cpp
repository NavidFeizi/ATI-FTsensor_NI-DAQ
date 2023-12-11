
//==========================================================================================================
// 					 This code is a part of the robot actuated catheter project
//  		   		Canadian Surgical Technologies and Advanced Robotics (CSTAR).
// 							Copyright (C) 2022 Navid Feizi <nfeizi@uwo.ca>
//					     Copyright (C) 2022 Filipe C. Pedrosa <fpedrosa@uwo.ca>
//
//       Project developed under the supervision of Dr Jayender Jagadeesan (❖) Dr Rajni Patel (◈)
//  ========================================================================================================
//   (◈) CSTAR (Canadian Surgical Technologies & Advanced Robotics) @ Western University, London, ON  Canada
//             (❖) Surgical Planning Lab @ Brigham and Women's Hospital / Harvard Medical School
//==========================================================================================================

#include "ATI_FT.hpp"
#include <signal.h>
#include <random>

#ifdef _WINDOWS
#include <windows.h>
#else
#include <unistd.h>
#define Sleep(x) usleep((x) * 1000)
#endif

#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <thread>

#include <blaze/Blaze.h>
#include <blaze/Math.h>
#include <blaze/math/DenseMatrix.h>

void print_FT(double t, blaze::StaticVector<double, 6> FT_reading);

int main()
{
	blaze::StaticVector<double, 6> force_reading = blaze::StaticVector<double, 6>(0.0);

	std::cout << std::fixed;
	std::cout << std::setprecision(4);

	/* open recording file */
	std::ofstream file;
	file.open("../../Output.csv");

	/* initialize ATI FT */
	std::string physicalChannel = "dev1/ai0:6";
	std::string clibration_filename = "FT35366.cal";
	std::string forecUnit = "N";
	std::string torqueUnit = "N-mm";
	AtiSensor AtiFT(physicalChannel, clibration_filename, forecUnit, torqueUnit);

	AtiFT.NullOffsets();

	file << "Fx" << ',' << "Fy" << ',' << "Fz"
		 << "Tx" << ',' << "Ty" << ',' << "Tz" << '\n';

	double t = 0;
	auto t0 = std::chrono::high_resolution_clock::now();

	for (int i = 0; i < 10000; i++)
	{
		AtiFT.GetForceUnBias(force_reading);
		file << force_reading[0] << ','
			 << force_reading[1] << ','
			 << force_reading[2] << ','
			 << force_reading[3] << ','
			 << force_reading[4] << ','
			 << force_reading[5] << '\n';

		auto currentTime = std::chrono::high_resolution_clock::now();
		auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - t0);
		t = static_cast<double>(elapsedTime.count()) / 1e3;

		print_FT(t, force_reading);

		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	/* close recording file */
	file.close();
	Sleep(1000);

	return 0;
}

void print_FT(double t, blaze::StaticVector<double, 6> FT_reading)
{
	auto printWithSpaceIfPositive = [](double value)
	{
		if (value >= 0)
		{
			std::cout << " " << value;
		}
		else
		{
			std::cout << value;
		}
	};

	std::cout << "\r" << std::dec << std::fixed << std::setprecision(2) << "Time: " << t << " [s]"
			  << "    ";

	std::cout << "Fx: ";
	std::cout << std::fixed << std::setprecision(5);
	printWithSpaceIfPositive(FT_reading[0]);
	std::cout << " [N]    ";
	std::cout << "Fy: ";
	std::cout << std::fixed << std::setprecision(5);
	printWithSpaceIfPositive(FT_reading[1]);
	std::cout << " [N]    ";
	std::cout << "Fz: ";
	std::cout << std::fixed << std::setprecision(5);
	printWithSpaceIfPositive(FT_reading[2]);
	std::cout << " [N]    ";
	std::cout << "Tx: ";
	std::cout << std::fixed << std::setprecision(5);
	printWithSpaceIfPositive(FT_reading[3]);
	std::cout << " [Nmm]    ";
	std::cout << "Ty: ";
	std::cout << std::fixed << std::setprecision(5);
	printWithSpaceIfPositive(FT_reading[4]);
	std::cout << " [Nmm]    ";
	std::cout << "Tz: ";
	std::cout << std::fixed << std::setprecision(5);
	printWithSpaceIfPositive(FT_reading[5]);
	std::cout << " [Nmm]    ";

	std::cout << " \n";

	// std::cout << "\r"
	//   << "----------------------------------------------------------------------------------" << std::endl;
}
