#include "TORCSLink.h"

HANDLE hMapFile;
TORCSData_t *data;

int initTORCSLink() {
	/* Connect to TORCS Data Store*/
   hMapFile = OpenFileMapping(
                   FILE_MAP_ALL_ACCESS,   // read/write access
                   FALSE,                 // do not inherit the name
				   FileMapName);               // name of mapping object


   data = (TORCSData_t*)MapViewOfFile(hMapFile, // handle to map object
               FILE_MAP_ALL_ACCESS,  // read/write permission
               0,
               0,
			   sizeof(TORCSData_t));
   /* If it doesn't exist we should exit with the error */
   if (data == NULL) {
	return GetLastError();
   }

   /* If robot restarting is enabled, restart the race before doing anything else */
#ifdef TL_ENABLE_RESTARTS
   int timeout = 0, maxTimeout = 10000;
   
   /* Set the restart flag and wait for robots to be ready */
   data->controlFlag = TL_RESTART_RACE;
   while (data->controlFlag != TL_READY && timeout++<maxTimeout) {
	   Sleep(100);
   }
   if (timeout >= maxTimeout) {
	   return -255;
   }
#endif

   return 0;
}

/* Simulink doesn't like passing arrays of structure (No idea why...), so had to split these three up*/

/* Set vehicle controls */
int setVehicleControl(int i, vehicleControl_t con) {
	/* If we can't read any data then something went wrong with the initialise
	Easiest way to inform user is by returning -1 and letting Simulink deal with it */
	if (data == NULL) {
		return -1;	// Bad connection
	}
	data->enable = 1;
	data->vehicle[i].control = con;
	return 0;
}

/* Update TORCS by informing it that it has new data for controls then wait until it updates outputs */
int updateTORCS() {
	int itr = 0, maxItr = 1E9;
	/* If we can't read any data then something went wrong with the initialise
	Easiest way to inform user is by returning -1 and letting Simulink deal with it */
	if (data == NULL) {
		return -1;	// Bad connection
	}

	/* Inform robot is has new data */
	data->controlFlag = TL_NEW_DATA;

	/* Wait until robots have updated themselves */
	while (data->controlFlag == TL_NEW_DATA && ++itr<maxItr) {
		// TODO: This should probably be a time based timeout?
	}
	if (itr >= maxItr) {
		return -2;	// No new data in time
	}
	return itr;
}

/* Read out vehicle data */
int getVehicleData(int i, vehicleData_t *veh) {
	/* If we can't read any data then something went wrong with the initialise
	Easiest way to inform user is by returning -1 and letting Simulink deal with it */
	if (data == NULL) {
		return -1;	// Bad connection
	}
	(*veh) = data->vehicle[i].data;
	return 0; // veh[0].control.gear;	// All is well
}

int terminateTORCSLink() {
	data->enable = 0;
	UnmapViewOfFile(data);
	CloseHandle(hMapFile);
	return 1;
}