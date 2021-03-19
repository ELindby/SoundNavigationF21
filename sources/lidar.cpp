#include "../includes/LIDAR.h"

LIDAR::LIDAR()
{
	printf("Ultra simple LIDAR data grabber for RPLIDAR.\n"
		"Version: " RPLIDAR_SDK_VERSION "\n");

	drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
	if (!drv) {
		fprintf(stderr, "Insufficent memory, exit\n");
		exit(-2);
	}

	size_t baudRateArraySize = (sizeof(baudrateArray)) / (sizeof(baudrateArray[0]));
	for (size_t i = 0; i < baudRateArraySize; ++i) {
		if (!drv)
			drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);

		if (IS_OK(drv->connect(opt_com_path, baudrateArray[i]))) {

			op_result = drv->getDeviceInfo(devinfo);

			if (IS_OK(op_result)) {
				connectSuccess = true;
				break;
			}
			else {
				delete drv;
				drv = NULL;
			}
		}
	}

	if (!connectSuccess) {

		fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
			, opt_com_path);
		RPlidarDriver::DisposeDriver(drv);
		drv = NULL;
		exit(-2);
	}

	// print out the device serial number, firmware and hardware version number..
	printf("RPLIDAR S/N: ");
	for (int pos = 0; pos < 16; ++pos) {
		printf("%02X", devinfo.serialnum[pos]);
	}

	printf("\n"
		"Firmware Ver: %d.%02d\n"
		"Hardware Rev: %d\n"
		, devinfo.firmware_version >> 8
		, devinfo.firmware_version & 0xFF
		, (int)devinfo.hardware_version);

	// check health...
	if (!checkRPLIDARHealth(drv)) {
		RPlidarDriver::DisposeDriver(drv);
		drv = NULL;
		exit(-2);
	}

}

LIDAR::~LIDAR()
{

}

void LIDAR::scanLIDAR()
{
	drv->startMotor();
	// start scan...
	drv->startScan(0, 1);

	// fetech result and print it out...
	while (1) {

		size_t count = _countof(nodes);

		op_result = drv->grabScanDataHq(nodes, count);

		if (IS_OK(op_result)) {

			drv->ascendScanData(nodes, count);

			//           for (int pos = 0; pos < (int)count; ++pos) {
			//
			//                printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
			//                    (nodes[pos].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ? "S " : "  ",
			//                    (nodes[pos].angle_z_q14 * 90.f / (1 << 14)),
			//                    nodes[pos].dist_mm_q2 / 4.0f,
			//                    nodes[pos].quality);
			//            }
			writeScan(); //Copies nodes to dataNodes for safe access.

		}

		if (ctrl_c_pressed) {
			break;
		}

	} //End of while

	//Stop motor and delete driver
	drv->stop();
	drv->stopMotor();

	RPlidarDriver::DisposeDriver(drv);
	drv = NULL;
}


bool LIDAR::checkRPLIDARHealth(RPlidarDriver* drv)
{
	op_result = drv->getHealth(healthinfo);
	if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
		printf("RPLidar health status : %d\n", healthinfo.status);
		if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
			fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
			// enable the following code if you want rplidar to be reboot by software
			// drv->reset();
			return false;
		}
		else {
			return true;
		}

	}
	else {
		fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
		return false;
	}
}

static inline void delay(_word_size_t ms) {
	while (ms >= 1000) {
		usleep(1000 * 1000);
		ms -= 1000;
	};
	if (ms != 0)
		usleep(ms * 1000);
}

void LIDAR::ctrlc(int) {
	ctrl_c_pressed = true;
}

void LIDAR::writeScan()
{
	std::lock_guard<std::mutex> guard(LIDARMutex);
	memcpy(dataNodes, nodes, sizeof(dataNodes));

}

rplidar_response_measurement_node_hq_t LIDAR::readScan()
{
	rplidar_response_measurement_node_hq_t tempNodes[8192];
	while (true) {

		if (LIDARMutex.try_lock()) {
			memcpy(tempNodes, dataNodes, sizeof(tempNodes));
			LIDARMutex.unlock();
			break;
		}
	}
	_u32 compDist = 9999;
	int nodeIndex = 0;
	size_t count = _countof(tempNodes);

	for (int i = 0; i < (int)count; i++)
	{
		//                printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
		//                    (tempNodes[i].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ? "S " : "  ",
		//                    (tempNodes[i].angle_z_q14 * 90.f / (1 << 14)),
		//                    tempNodes[i].dist_mm_q2 / 4.0f,
		//                    tempNodes[i].quality);
		if (compDist > tempNodes[i].dist_mm_q2 && (tempNodes[i].quality != 0)) {
			compDist = tempNodes[i].dist_mm_q2;
			nodeIndex = i;
		}
	}
	return tempNodes[nodeIndex];
}