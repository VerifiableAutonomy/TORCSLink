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

#include "TORCSLink.h"

int initTORCSLink() {

	/* Set up shared memory */
	if (tlInitSharedMemory() != 0) {
		return -1;
	}

   /* If robot restarting is enabled, restart the race before doing anything else */
#ifdef TL_ENABLE_RESTARTS
   int timeout = 0, maxTimeout = 300;
   
   /* Set the restart flag and wait for robots to be ready */
   tlData->controlFlag = TL_RESTART_RACE;
   while (tlData->controlFlag != TL_READY && timeout++<maxTimeout) {
	   Sleep(100);
   }
   if (timeout >= maxTimeout) {
	   return -2;
   }
#endif
   return 0;
}

/* Simulink doesn't like passing arrays of structure (No idea why...), so had to split these three up*/

/* Set vehicle controls */
int setVehicleControl(int i, vehicleControl_t con) {
	/* If we can't read any data then something went wrong with the initialise
	Easiest way to inform user is by returning -1 and letting Simulink deal with it */
	if (tlData == NULL) {
		return -1;	// Bad connection
	}
	tlData->enable = 1;
	tlData->vehicle[i].control = con;
	return 0;
}

/* Update TORCS by informing it that it has new data for controls then wait until it updates outputs */
int updateTORCS() {
	int itr = 0, maxItr = 1E9;
	/* If we can't read any data then something went wrong with the initialise
	Easiest way to inform user is by returning -1 and letting Simulink deal with it */
	if (tlData == NULL) {
		return -1;	// Bad connection
	}

	/* Inform robot is has new data */
	tlData->controlFlag = TL_NEW_DATA;

	/* Wait until robots have updated themselves */
	while (tlData->controlFlag == TL_NEW_DATA && ++itr<maxItr) {
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
	if (tlData == NULL) {
		return -1;	// Bad connection
	}
	(*veh) = tlData->vehicle[i].data;
	return 0; // veh[0].control.gear;	// All is well
}

int terminateTORCSLink() {
	tlData->enable = 0;
	tlCloseSharedMemory();
	return 0;
}