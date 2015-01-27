/*  TORCSLink, an interface between TORCS and MATLAB/Simulink
	Copyright(C) 2014 Owen McAree

	This program is free software : you can redistribute it and / or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef TORCSLINK_H
#define TORCSLINK_H

#ifdef _WIN32
	#include <windows.h>
#elif __linux__
	#include <unistd.h>
	#include <sys/mman.h>
	#include <sys/types.h>
       	#include <sys/stat.h>
       	#include <fcntl.h>
       	#include <errno.h>
#endif

/* Enable this if you have modified your TORCS sourcecode to perform integration of 
position data in doule precision rather than the default float. See <Insert link here> */
#define TL_USE_DOUBLE_POSITION

/* Enale this if you have modified your TORCS sourcecode to allow robots to restart races
See here: http://torcs.sourceforge.net/index.php?name=Sections&op=viewarticle&artid=30#c6_8 */
#define TL_ENABLE_RESTARTS


/* Number of vehicles 
10 seems to be the maximum values here */
#define N_VEHICLES 10

/* Define some control flags for the robot */
#define TL_NO_DATA 0			/* There is no new data available */
#define TL_NEW_DATA 1			/* There is new data available */
#define TL_RESTART_RACE 2		/* Robot should restart the race (if available) */
#define TL_READY 255			/* Robot is ready */

/* Window resolution of TORCS for screen grabbing*/
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

/* Structures for vehicle data */
/* Use double for all floating point values for easy compatibility with Simulink */

/* Vehicle Control Data*/
typedef struct vehicleControlStruct {
	double throttle;			/* Throttle value, (0-1) */
	double brake;				/* Brake value, (0-1) */
	double clutch;				/* Clutch value, (0-1) */
	double steering;			/* Steering value (-1-1) */
	int gear;				/* Gear (-1-6) */
} vehicleControl_t;

/* Vehicle Data */
typedef struct vehicleDataStruct {
	double position[3];			/* Global position [m] */
	double velocity[3];			/* Global velocity [m/s] */
	double acceleration[3];			/* Global acceleration [m/s/s] */
	double angle[3];			/* Roll/Pitch/Yaw [rad] */
	double angularVelocity[3];		/* Roll/Pitch/Yaw rates [rad/s] */
	double headingError;			/* Error between vehicle heading and track heading (at current location) [rad] */
	double lateralError;			/* Lateral error between car (CoG) and track centreline (at current location) [m] */
	double roadDistance;			/* Distance travelled along track from start/finish line [m] */
	double roadCurvature;			/* Curvature of track (at current location), left turns = +ve curvature, right turns = -ve curvature */
	double engineRPM;			/* Engine RPM */
} vehicleData_t;

/* Total vehicle data */
typedef struct vehicleStruct {
	vehicleControl_t control;
	vehicleData_t data;
} vehicle_t;

/* Structure to be shared between processes */
typedef struct TOCSDataStruct {
	volatile int enable;
	volatile int controlFlag;
	volatile unsigned char imgBuffer[SCREEN_WIDTH*SCREEN_HEIGHT*3];
	vehicle_t vehicle[N_VEHICLES];
} TORCSData_t;

/* Location of shared memory */
#ifdef _WIN32
	TCHAR FileMapName[] = TEXT("Local\\TORCSDataStore");
	HANDLE hMapFile;
#elif __linux__
	const char * FileMapName = "/TORCSDataStore";
	int hMapFile;
#endif

volatile TORCSData_t * tlData;

/* Initialise shared memory */
int tlInitSharedMemory() {
#ifdef _WIN32
	hMapFile = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0,	sizeof(TORCSData_t), FileMapName);
	tlData = (TORCSData_t*)MapViewOfFile(hMapFile, FILE_MAP_ALL_ACCESS,	0, 0, sizeof(TORCSData_t));
	if (tlData == NULL) {
		return GetLastError();
	}
#elif __linux__
	hMapFile = shm_open(FileMapName, O_CREAT | O_RDWR | O_SYNC, S_IRUSR | S_IWUSR);
	ftruncate(hMapFile, sizeof(TORCSData_t));
	tlData = (TORCSData_t*)mmap(NULL, sizeof(TORCSData_t), PROT_READ | PROT_WRITE, MAP_SHARED, hMapFile, 0);
	if (tlData == MAP_FAILED) {
		return errno;
	}
#endif

	return 0;
}

/* Clean up shared memory */
int tlCloseSharedMemory() {
#ifdef _WIN32
	UnmapViewOfFile((void*)tlData);
	CloseHandle(hMapFile);
#elif __linux__
	munmap((void*)tlData, sizeof(TORCSData_t));
#endif
	return 0;
}

#endif
