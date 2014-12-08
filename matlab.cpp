/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
#ifdef _WIN32
	#include <ctime>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>

#include "TORCSLink.h"

TORCSData_t *data;

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s);
static void newRace(int index, tCarElt* car, tSituation *s);
static void drive(int index, tCarElt* car, tSituation *s);
static void shutdown(int index);
static int InitFuncPt(int index, void *pt);
static void endRace(int index, tCarElt *car, tSituation *s);

HANDLE hMapFile;
bool inRace[N_VEHICLES];
int step[N_VEHICLES];

// Module entry point.
extern "C" int matlab(tModInfo *modInfo)
{
	// Testing IPC
	hMapFile = CreateFileMapping(
                 INVALID_HANDLE_VALUE,			// use paging file
                 NULL,							// default security
                 PAGE_READWRITE,				// read/write access
                 0,								// maximum object size (high-order DWORD)
                 sizeof(TORCSData_t),			// maximum object size (low-order DWORD)
				 FileMapName);						// name of mapping object
	data = (TORCSData_t*) MapViewOfFile(hMapFile,		// handle to map object
                        FILE_MAP_ALL_ACCESS,					// read/write permission
                        0,
                        0,
                        sizeof(TORCSData_t));

	// Clear all structures.
	memset(modInfo, 0, N_VEHICLES*sizeof(tModInfo));

	for (int i = 0; i < N_VEHICLES; i++) {
		char name[64];
		sprintf(name, "matlab %d", i);
		modInfo[i].name    = strdup(name);	// name of the module (short).
		modInfo[i].desc	   = strdup(name);	// Description of the module (can be long).
		modInfo[i].fctInit = InitFuncPt;			// Init function.
		modInfo[i].gfId    = ROB_IDENT;				// Supported framework version.
		modInfo[i].index   = i;						// Indices from 0 to 9.
		inRace[i] = false;
		step[i] = 0;
	}
	return 0;
}


// Module interface initialization.
static int InitFuncPt(int index, void *pt)
{
	tRobotItf *itf = (tRobotItf *)pt;

	// Create robot instance for index.
//	driver[index] = new Driver(index);
	itf->rbNewTrack = initTrack;	// Give the robot the track view called.
	itf->rbNewRace  = newRace;		// Start a new race.
	itf->rbDrive    = drive;		// Drive during race.
	itf->rbPitCmd   = NULL;			// Pit commands.
	itf->rbEndRace  = endRace;		// End of the current race.
	itf->rbShutdown = shutdown;		// Called before the module is unloaded.
	itf->index      = index;		// Index used if multiple interfaces.
	return 0;
}


// Called for every track change or new race.
static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s)
{
	// Don't load any particular car set up
	*carParmHandle = NULL;
}

// Start a new race.
static void newRace(int index, tCarElt* car, tSituation *sit)
{
	data->controlFlag = TL_NO_DATA;
	inRace[index] = true;
	step[index] = 0;
	// Zero local data store
	memset(data,0,sizeof(TORCSData_t));
	printf("matlab %d: Online\n",index);
}

int lastEnable = 0;
// Drive during race.
static void drive(int index, tCarElt* car, tSituation *s)
{
	/* We don't want control before the race starts */
	if (s->currentTime < 0) {
		return;
	}

	if (lastEnable != data->enable) {
		if (data->enable) {
			printf("matlab robots: Enabled!\n");
		} else {
			printf("matlab robots: Disabled!\n");
		}
	}
	lastEnable = data->enable;

#ifdef TL_ENABLE_RESTARTS
	/* If Simulink has requested a restart process this before anything else */
	if (data->controlFlag == TL_RESTART_RACE) {
		car->ctrl.askRestart = true;
		return;
	}
#endif

	/* If we are disabled, occasionally let the user know we're still alive, but otherwise do nothing */
	if (data->enable < 1) {
		data->controlFlag = TL_READY;	// Tell Simulink we're ready and waiting...
		if (index == 0 && fmod(s->currentTime, 10) <= RCM_MAX_DT_SIMU) {
			printf("matlab robots: Disabled (t=%.0fs)\n", s->currentTime);
		}
		return;
	}

	/* We want to block TORCS until we have an update from Simulink this ensures the two remain in sync
		Ensure we timeout after 10s so we don't block forever if Simulink dies on us
	*/
	int startT = std::time(0);
	int timeout = 0;
	while (data->enable == 1 && data->controlFlag == TL_NO_DATA) {
		if ((std::time(0) - startT) > 5) {
			startT = std::time(0);
			switch (timeout) {
				case 0:
					printf("matlab robots: No data from Simulink in the last 5 seconds... waiting 5 more...\n");
					timeout++;
					break;
				case 1:
					printf("matlab robots: No data from Simulink in the last 10 seconds... disabling robot...\n");
					data->enable = 0;
					return;
			}
		}
	}

	/* If we've got this far then we have new data from Simulink, update our control */
	car->ctrl.accelCmd = data->vehicle[index].control.throttle;
	car->ctrl.brakeCmd = data->vehicle[index].control.brake;
	car->ctrl.clutchCmd = data->vehicle[index].control.clutch;
	car->ctrl.gear = data->vehicle[index].control.gear;
	car->ctrl.steer = data->vehicle[index].control.steering;

	/* Update the outputs of our vehicle */
#ifdef TL_USE_DOUBLE_POSITION
	data->vehicle[index].data.position[0] = car->pub.DynGCg.posD.x;
	data->vehicle[index].data.position[1] = car->pub.DynGCg.posD.y;
	data->vehicle[index].data.position[2] = car->pub.DynGCg.posD.z;
#else
	data->vehicleData[index].position[0] = car->pub.DynGCg.pos.x;
	data->vehicleData[index].position[1] = car->pub.DynGCg.pos.y;
	data->vehicleData[index].position[2] = car->pub.DynGCg.pos.z;
#endif
	data->vehicle[index].data.velocity[0] = car->pub.DynGCg.vel.x;
	data->vehicle[index].data.velocity[1] = car->pub.DynGCg.vel.y;
	data->vehicle[index].data.velocity[2] = car->pub.DynGCg.vel.z;
	data->vehicle[index].data.acceleration[0] = car->pub.DynGCg.acc.x;
	data->vehicle[index].data.acceleration[1] = car->pub.DynGCg.acc.y;
	data->vehicle[index].data.acceleration[2] = car->pub.DynGCg.acc.z;
	data->vehicle[index].data.angle[0] = car->pub.DynGCg.pos.ax;
	data->vehicle[index].data.angle[1] = car->pub.DynGCg.pos.ay;
	data->vehicle[index].data.angle[2] = car->pub.DynGCg.pos.az;
	data->vehicle[index].data.angularVelocity[0] = car->pub.DynGC.vel.ax;
	data->vehicle[index].data.angularVelocity[1] = car->pub.DynGC.vel.ay;
	data->vehicle[index].data.angularVelocity[2] = car->pub.DynGC.vel.az;
	
	double error = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
	error = fmod(error + 2*PI,2*PI);	// Normalise heading angle
	if (error > PI) {
		error = error - 2*PI;
	}
	data->vehicle[index].data.headingError = error;
	data->vehicle[index].data.lateralError = car->_trkPos.toMiddle;
	data->vehicle[index].data.roadDistance = RtGetDistFromStart(car);
	double curvature = 0.0;
	if (car->_trkPos.seg->radius > 0) {
		curvature = 1.0/car->_trkPos.seg->radius;
		if (car->_trkPos.seg->type == TR_RGT) {
			curvature *= -1;
		}
	}
	data->vehicle[index].data.roadCurvature = curvature;
	data->vehicle[index].data.engineRPM = car->_enginerpm * 9.54929659; // For some reason RPM is actually rad/s!

	/* Keep track of when all vehicles have been updated */
	step[index]++;
	bool update = true;
	for (int i = 0; i<N_VEHICLES; i++) {
		if (inRace[i] && step[i] != step[index]) {
			update = false;
		}
	}

	/* When all vehicles are updated inform Simulink there is data to be read */
	if (update) {
		data->controlFlag = TL_NO_DATA;
	}
}


// End of the current race.
static void endRace(int index, tCarElt *car, tSituation *s)
{
	inRace[index] = 0;
	step[index] = 0;
	printf("matlab %d: Offline\n", index);
}


// Called before the module is unloaded.
static void shutdown(int index)
{
	UnmapViewOfFile(data);
    CloseHandle(hMapFile);
}