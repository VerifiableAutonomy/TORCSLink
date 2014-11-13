/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
#ifdef _WIN32
	#include<winsock2.h>
	#include <windows.h>
#endif

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define NBBOTS 10
#define BASE_PORT 8000   //The port on which the first vehicle to listen for incoming data

typedef struct vehicleDataStruct {
	float throttle;
	float brake;
	float clutch;
	float steering;
	int gear;
	float dist;
	float speed;
	float accel;
	float angle;
	float latErr;
	float radius;
	float rpm;
} vehicleData_t;
vehicleData_t* vehicleData;

static const char* botname[NBBOTS] = {
	"matlab 1", "matlab 2", "matlab 3", "matlab 4", "matlab 5",
	"matlab 6", "matlab 7", "matlab 8", "matlab 9", "matlab 10"
};

static const char* botdesc[NBBOTS] = {
	"matlab 1", "matlab 2", "matlab 3", "matlab 4", "matlab 5",
	"matlab 6", "matlab 7", "matlab 8", "matlab 9", "matlab 10"
};

struct sockaddr_in client[NBBOTS];
SOCKET sock[NBBOTS];
WSADATA wsa;
pollfd fds[NBBOTS];

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s);
static void newRace(int index, tCarElt* car, tSituation *s);
static void drive(int index, tCarElt* car, tSituation *s);
static void shutdown(int index);
static int InitFuncPt(int index, void *pt);
static void endRace(int index, tCarElt *car, tSituation *s);

TCHAR szName[]=TEXT("Global\\TORCSDataStore");
HANDLE hMapFile;

// Module entry point.
extern "C" int matlab(tModInfo *modInfo)
{
     
    //Initialise winsock
    WSAStartup(MAKEWORD(2,2),&wsa);
	
	int i;

	// Testing IPC
	hMapFile = CreateFileMapping(
                 INVALID_HANDLE_VALUE,			// use paging file
                 NULL,							// default security
                 PAGE_READWRITE,				// read/write access
                 0,								// maximum object size (high-order DWORD)
                 sizeof(vehicleData_t)*NBBOTS,	// maximum object size (low-order DWORD)
                 szName);						// name of mapping object
	vehicleData = (vehicleData_t*) MapViewOfFile(hMapFile,		// handle to map object
                        FILE_MAP_ALL_ACCESS,					// read/write permission
                        0,
                        0,
                        sizeof(vehicleData_t)*NBBOTS);

	// Clear all structures.
	memset(modInfo, 0, 10*sizeof(tModInfo));

	for (i = 0; i < NBBOTS; i++) {
		modInfo[i].name    = strdup(botname[i]);	// name of the module (short).
		modInfo[i].desc    = strdup(botdesc[i]);	// Description of the module (can be long).
		modInfo[i].fctInit = InitFuncPt;			// Init function.
		modInfo[i].gfId    = ROB_IDENT;				// Supported framework version.
		modInfo[i].index   = i;						// Indices from 0 to 9.
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
	itf->rbPitCmd   = NULL;		// Pit commands.
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
	struct sockaddr_in server;

    // Create a socket
	sock[index] = socket(AF_INET, SOCK_DGRAM, 0 );
     
    // Prepare the sockaddr_in structure
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons(BASE_PORT + index);
    
	// Set tx and rx buffers so we only retain one frame
	int buf = 20;
	setsockopt(sock[index],SOL_SOCKET,SO_RCVBUF,(const char*)&buf,sizeof(int));
	buf = 24;
	setsockopt(sock[index],SOL_SOCKET,SO_SNDBUF,(const char*)&buf,sizeof(int));

    // Bind to port
    bind(sock[index],(struct sockaddr *)&server, sizeof(server));

	// Update poll structure
	fds[index].fd = sock[index];
	fds[index].events = POLLIN;
	fds[index].revents = -1;

	// Zero local data store
	memset(vehicleData,0,sizeof(vehicleData_t)*NBBOTS);


	printf("matlab %d: Bound to %d (fd %d)\n",index,BASE_PORT + index, sock[index]);
}

static void udpRXTX(int index, tCarElt* car) {
	int slen = sizeof(client[index]);
	char recvBuf[20];

	// Read any pending data (all vehicles) and save to local storage
	if (WSAPoll(fds,NBBOTS,0)) {
		for (int i = 0; i<NBBOTS; i++) {
			if (fds[i].revents & POLLIN) {		
//				printf("Data for #%d: ", i);
				int len = recvfrom(fds[i].fd, recvBuf, 20, 0, (struct sockaddr *) &client[i], &slen);
				if (len == 20) {	// Ignore the spurious data we get on end of simulation
					vehicleData[i].throttle = *(float*)(recvBuf);
					vehicleData[i].brake = *(float*)(recvBuf+4);
					vehicleData[i].clutch = *(float*)(recvBuf+8);
					vehicleData[i].gear = *(int*)(recvBuf+12);
					vehicleData[i].steering = *(float*)(recvBuf+16);
				}
//				printf("%f %f %f %d %f",throttle[i],brake[i],clutch[i],gear[i],steering[i]);
			}
		}
//		printf("\n");
	}
	// Update current vehicle from local storage
	car->ctrl.accelCmd = vehicleData[index].throttle;
	car->ctrl.brakeCmd = vehicleData[index].brake;
	car->ctrl.clutchCmd = vehicleData[index].clutch;
	car->ctrl.gear = vehicleData[index].gear;
	car->ctrl.steer = vehicleData[index].steering;

	// Output current vehicle to client (if we have one)
	if (true) {		// TODO: Figure out how to test if client structure is empty
		vehicleData[index].dist = RtGetDistFromStart(car);
		vehicleData[index].speed = car->_speed_x;
		vehicleData[index].accel = car->_accel_x;
		vehicleData[index].angle = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
		vehicleData[index].angle = fmod(vehicleData[index].angle + 2*PI,2*PI);	// Normalise heading angle
		if (vehicleData[index].angle > PI) {
			vehicleData[index].angle = vehicleData[index].angle - 2*PI;
		}
		vehicleData[index].latErr = car->_trkPos.toMiddle;
		vehicleData[index].radius = car->_trkPos.seg->radius;
		vehicleData[index].rpm = car->_enginerpm * 9.54929659; // For some reason RPM is actually rad/s!


		float f = 3.14159;
		int ii = 54321;

		//CopyMemory((PVOID)(pBuf + sizeof(vehicleData_t)*index), &vehicleData[index], sizeof(vehicleData_t));
		//CopyMemory((PVOID)(pBuf), &f, sizeof(f));
//		void* pp = (void*)(pBuf) + index;
//		memcpy(pp+int(sizeof(vehicleData_t)*index),&(vehicleData[index]),sizeof(vehicleData_t));

		char sendBuf[28];
		memcpy(sendBuf,&vehicleData[index].dist,4);
		memcpy(sendBuf+4,&vehicleData[index].speed,4);
		memcpy(sendBuf+8,&vehicleData[index].accel,4);
		memcpy(sendBuf+12,&vehicleData[index].angle,4);
		memcpy(sendBuf+16,&vehicleData[index].latErr,4);
		memcpy(sendBuf+20,&vehicleData[index].radius,4);
		memcpy(sendBuf+24,&vehicleData[index].rpm,4);

		sendto(sock[index],sendBuf,28,0,(const sockaddr *)&client[index],slen);
	}
}


// Drive during race.
static void drive(int index, tCarElt* car, tSituation *s)
{
	udpRXTX(index, car);
}


// End of the current race.
static void endRace(int index, tCarElt *car, tSituation *s)
{
	closesocket(sock[index]);
	// TODO: Clear out sock, fds and client
}


// Called before the module is unloaded.
static void shutdown(int index)
{
	WSACleanup();
	UnmapViewOfFile(vehicleData);
    CloseHandle(hMapFile);
}