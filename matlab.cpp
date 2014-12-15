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

#include <ctime>
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

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s);
static void newRace(int index, tCarElt* car, tSituation *s);
static void drive(int index, tCarElt* car, tSituation *s);
static void shutdown(int index);
static int InitFuncPt(int index, void *pt);
static void endRace(int index, tCarElt *car, tSituation *s);


/* Module entry point */
extern "C" int matlab(tModInfo *modInfo) {
	
	/* Set up shared memory */
	if (tlInitSharedMemory() != 0) {
		printf("matlab robots: Error initialising shared memory!\n");
	}
	
	/* Clear all structures */
	memset(modInfo, 0, N_VEHICLES*sizeof(tModInfo));

	for (int i = 0; i < N_VEHICLES; i++) {
		char name[64];
		sprintf(name, "matlab %d", i);
		modInfo[i].name    = strdup(name);
		modInfo[i].desc	   = strdup(name);
		modInfo[i].fctInit = InitFuncPt;
		modInfo[i].gfId    = ROB_IDENT;	
		modInfo[i].index   = i;
	}
	return 0;
}


/* Module interface initialization */
static int InitFuncPt(int index, void *pt) {
	tRobotItf *itf = (tRobotItf *)pt;

	itf->rbNewTrack = initTrack;	/* Give the robot the track view called */
	itf->rbNewRace  = newRace;		/* Start a new race */
	itf->rbDrive    = drive;		/* Drive during race */
	itf->rbPitCmd   = NULL;			/* Pit commands */
	itf->rbEndRace  = endRace;		/* End of the current race */
	itf->rbShutdown = shutdown;		/* Called before the module is unloaded */
	itf->index      = index;		/* Index used if multiple interfaces */
	return 0;
}


/* Called for every track change or new race */
static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s) {
	/* Don't load any particular car set up */
	*carParmHandle = NULL;
}

/* Start a new race  */
static void newRace(int index, tCarElt* car, tSituation *sit) {
	// Zero data store
	memset(tlData,0,sizeof(TORCSData_t));
	tlData->controlFlag = TL_NO_DATA;
	printf("matlab %d: Online\n",index);
}

int lastEnable = 0;
/* Called once per robot update */
static int oncePerCycle(double t) {

	/* Inform user when enabling/disabling */
	if (lastEnable != tlData->enable) {
		if (tlData->enable) {
			printf("matlab robots: Enabled!\n");
		}
		else {
			printf("matlab robots: Disabled!\n");
		}
	}
	lastEnable = tlData->enable;

	/* If we are disabled, occasionally let the user know we're still alive, but otherwise do nothing */
	if (tlData->enable < 1) {
		tlData->controlFlag = TL_READY;	/* Tell Simulink we're ready and waiting... */
		if (fmod(t, 10) <= RCM_MAX_DT_SIMU) {
			printf("matlab robots: Disabled (t=%.0fs)\n", t);
		}
		return -1;
	} else {
		tlData->controlFlag = TL_NO_DATA;
	}

	/* We want to block TORCS until we have an update from Simulink this ensures the two remain in sync
	Ensure we timeout after 10s so we don't block forever if Simulink dies on us */
	int startT = std::time(0);
	int timeout = 0;
	while (tlData->enable == 1 && tlData->controlFlag == TL_NO_DATA) {
		if ((std::time(0) - startT) > 5) {
			startT = std::time(0);
			switch (timeout) {
			case 0:
				printf("matlab robots: No data from Simulink in the last 5 seconds... waiting 5 more...\n");
				timeout++;
				break;
			case 1:
				printf("matlab robots: No data from Simulink in the last 10 seconds... disabling robot...\n");
				tlData->enable = 0;
				return -1;
			}
		}
	}
	return 0;
}

double lastTime = 0.0;
/* Drive during race */
static void drive(int index, tCarElt* car, tSituation *s) {
	/* We don't want control before the race starts */
	if (s->currentTime < 0) {
		return;
	}

#ifdef TL_ENABLE_RESTARTS
	/* If Simulink has requested a restart process this before anything else */
	if (tlData->controlFlag == TL_RESTART_RACE) {
		car->ctrl.askRestart = true;
		return;
	}
#endif

	if ((s->currentTime - lastTime) > RCM_MAX_DT_ROBOTS / 2) {
		if (oncePerCycle(s->currentTime) < 0) {
			return;
		} else {
			lastTime = s->currentTime;
		}
	}

	/* If we've got this far then we have new data from Simulink, update our control */
	car->ctrl.accelCmd = tlData->vehicle[index].control.throttle;
	car->ctrl.brakeCmd = tlData->vehicle[index].control.brake;
	car->ctrl.clutchCmd = tlData->vehicle[index].control.clutch;
	car->ctrl.gear = tlData->vehicle[index].control.gear;
	car->ctrl.steer = tlData->vehicle[index].control.steering;

	/* Update the outputs of our vehicle */
#ifdef TL_USE_DOUBLE_POSITION
	tlData->vehicle[index].data.position[0] = car->pub.DynGCg.posD.x;
	tlData->vehicle[index].data.position[1] = car->pub.DynGCg.posD.y;
	tlData->vehicle[index].data.position[2] = car->pub.DynGCg.posD.z;
#else
	tlData->vehicle[index].data.position[0] = car->pub.DynGCg.pos.x;
	tlData->vehicle[index].data.position[1] = car->pub.DynGCg.pos.y;
	tlData->vehicle[index].data.position[2] = car->pub.DynGCg.pos.z;
#endif
	tlData->vehicle[index].data.velocity[0] = car->pub.DynGCg.vel.x;
	tlData->vehicle[index].data.velocity[1] = car->pub.DynGCg.vel.y;
	tlData->vehicle[index].data.velocity[2] = car->pub.DynGCg.vel.z;
	tlData->vehicle[index].data.acceleration[0] = car->pub.DynGCg.acc.x;
	tlData->vehicle[index].data.acceleration[1] = car->pub.DynGCg.acc.y;
	tlData->vehicle[index].data.acceleration[2] = car->pub.DynGCg.acc.z;
	tlData->vehicle[index].data.angle[0] = car->pub.DynGCg.pos.ax;
	tlData->vehicle[index].data.angle[1] = car->pub.DynGCg.pos.ay;
	tlData->vehicle[index].data.angle[2] = car->pub.DynGCg.pos.az;
	tlData->vehicle[index].data.angularVelocity[0] = car->pub.DynGC.vel.ax;
	tlData->vehicle[index].data.angularVelocity[1] = car->pub.DynGC.vel.ay;
	tlData->vehicle[index].data.angularVelocity[2] = car->pub.DynGC.vel.az;
	
	double error = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
	error = fmod(error + 2*PI,2*PI);	// Normalise heading angle
	if (error > PI) {
		error = error - 2*PI;
	}
	tlData->vehicle[index].data.headingError = error;
	tlData->vehicle[index].data.lateralError = car->_trkPos.toMiddle;
	tlData->vehicle[index].data.roadDistance = RtGetDistFromStart(car);
	double curvature = 0.0;
	if (car->_trkPos.seg->radius > 0) {
		curvature = 1.0/car->_trkPos.seg->radius;
		if (car->_trkPos.seg->type == TR_RGT) {
			curvature *= -1;
		}
	}
	tlData->vehicle[index].data.roadCurvature = curvature;
	tlData->vehicle[index].data.engineRPM = car->_enginerpm * 9.54929659; // For some reason RPM is actually rad/s!

}


// End of the current race.
static void endRace(int index, tCarElt *car, tSituation *s) {
	printf("matlab %d: Offline\n", index);
}


// Called before the module is unloaded.
static void shutdown(int index) {
	tlCloseSharedMemory();
}