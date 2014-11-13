#include "windows.h"

#define NBBOTS 10
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

TCHAR szName[]=TEXT("Global\\TORCSDataStore");
HANDLE hMapFile;

int initTORCSLink()
{
   hMapFile = OpenFileMapping(
                   FILE_MAP_ALL_ACCESS,   // read/write access
                   FALSE,                 // do not inherit the name
                   szName);               // name of mapping object


   vehicleData = (vehicleData_t*)MapViewOfFile(hMapFile, // handle to map object
               FILE_MAP_ALL_ACCESS,  // read/write permission
               0,
               0,
               sizeof(vehicleData_t)*NBBOTS);
   if (vehicleData == NULL) {
    return GetLastError();
   }

   return 0;
}

int updateVehicle(int index, vehicleData_t *veh) {
    //vehicleData_t v = *(vehicleData_t*)pBuf;
    //return v.latErr;
    if (vehicleData == NULL) {
        initTORCSLink();
    }
    vehicleData[index].throttle = veh->throttle;
    vehicleData[index].brake = veh->brake;
    vehicleData[index].clutch = veh->clutch;
    vehicleData[index].steering = veh->steering;
    vehicleData[index].gear = veh->gear;
	veh->dist = vehicleData[index].dist;
    veh->speed = vehicleData[index].speed;
    veh->accel = vehicleData[index].accel;
    veh->angle = vehicleData[index].angle;
    veh->latErr = vehicleData[index].latErr;
    veh->radius = vehicleData[index].radius;
    veh->rpm = vehicleData[index].rpm;
    
    return 0;
}



int terminateTORCSLink() {
   UnmapViewOfFile(vehicleData);
   CloseHandle(hMapFile);
   return 1;
}
