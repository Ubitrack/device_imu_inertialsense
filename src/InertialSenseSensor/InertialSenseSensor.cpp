/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2006, Technische Universitaet Muenchen, and individual
 3* contributors as indicated by the @authors tag. See the
 * copyright.txt in the distribution for a full listing of individual
 * contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */

/**
 * @ingroup vision_components
 * @file
 * Reads IMU data
 *
 * @author Adnane Jadid <jadid@in.tum.de>
 */


// change these include paths to the correct paths for your project
#include "ISComm.h"
#include "serialPortPlatform.h"

#include <string>
#include <list>
#include <iostream>
#include <iomanip>
#include <strstream>
#include <log4cpp/Category.hh>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/scoped_array.hpp>

#include <utDataflow/PushSupplier.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utMeasurement/TimestampSync.h>
#include <utUtil/OS.h>
#include <utUtil/TracingProvider.h>


// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.InertialSenseSensor" ) );

using namespace Ubitrack;


namespace Ubitrack { namespace Drivers {

/**
 * @ingroup components
 *
 * @par Input Ports
 * None.
 *
 * @par Output Ports
 * \c Output push port of type Ubitrack::Measurement::IMUMeasurement.
 *
 * @par Configuration
 * The configuration tag contains a \c <dsvl_input> configuration.
 * For details, see the InertialSense Docs
 *
 */
class InertialSenseSensor
	: public Dataflow::Component
{
public:

	/** constructor */
	InertialSenseSensor( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph >  );

	/** destructor, waits until thread stops */
	~InertialSenseSensor();

	/** starts the IMU */
	void start();

	/** starts the capturing */
	void startCapturing();

	/** stops the IMU */
	void stop();

protected:


	
	void handleInsMessage(ins_1_t* ins);
	void handleGpsMessage(gps_pos_t* gps);
	void handleImuMessage(dual_imu_t* imu);
	void handleMAGMessage(magnetometer_t* mag);

	// shift timestamps (ms)
	int m_timeOffset;

	//serial port for thr communication with IMU
	serial_port_t serialPort;
	
	is_comm_instance_t comm;

	//params
	int freq;
	std::string portName;
	int baudRate;


	/** thread is running?*/
	bool m_bStop;

	// pointer to the thread
	boost::shared_ptr< boost::thread > m_pThread;

	int count;

	uint8_t inByte;

	int messageSize;
	uint8_t buffer[2048];

	/** timestamp of last frame */
	double m_lastTime;


	// the ports
	Dataflow::PushSupplier< Measurement::Vector3D > m_acc_OutPort;
	Dataflow::PushSupplier< Measurement::RotationVelocity> m_gyro_OutPort;
	Dataflow::PushSupplier< Measurement::Vector3D > m_mag_OutPort;

	Dataflow::PushSupplier< Measurement::Rotation > m_gyro_outPortRAW;
  
};



InertialSenseSensor::InertialSenseSensor(const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph)
	: Dataflow::Component(sName)
	, m_timeOffset(0)
	, m_lastTime(-1e10)
	, m_bStop(true)
	, m_acc_OutPort("acc_OutPort", *this)
	, m_gyro_OutPort("gyro_OutPort", *this)
	, m_mag_OutPort("mag_OutPort", *this)
	, m_gyro_outPortRAW("gyro_outPortRAW", *this)
{
	subgraph->m_DataflowAttributes.getAttributeData("timeOffset", m_timeOffset);
	subgraph->m_DataflowAttributes.getAttributeData("frequency", freq);
	subgraph->m_DataflowAttributes.getAttributeData("baudrate", baudRate);
	portName = subgraph->m_DataflowAttributes.getAttributeString("port");
	
	// very important - the serial port must be initialized to zeros
	memset(&serialPort, 0, sizeof(serialPort));
	

	// make sure to assign a valid buffer and buffer size to the comm instance
	comm.buffer = buffer;
	comm.bufferSize = sizeof(buffer);

	// initialize the comm instance, sets up state tracking, packet parsing, etc.
	is_comm_init(&comm);

	// initialize the serial port (Windows, MAC or Linux) - if using an embedded system like Arduino,
	//  you will need to handle the serial port creation, open and reads yourself. In this
	//  case, you do not need to include serialPort.h/.c and serialPortPlatform.h/.c in your project.
	serialPortPlatformInit(&serialPort);
				
}



InertialSenseSensor::~InertialSenseSensor()
{
	
}


void InertialSenseSensor::start()
{
	if ( !m_running ) {
		// open serial, last parameter is a 1 which means a blocking read, you can set as 0 for non-blocking
		// you can change the baudrate to a supported baud rate (IS_BAUDRATE_*), make sure to reboot the uINS
		//  if you are changing baud rates, you only need to do this when you are changing baud rates.
		if (!serialPortOpen(&serialPort, portName.c_str(), baudRate, 1))
		{
			LOG4CPP_ERROR(logger, "Failed to open serial port on com port COM3");
		}
		// stop all broadcasts on the device
		messageSize = is_comm_stop_broadcasts_all_ports(&comm);
		if (messageSize < 1)
		{
			LOG4CPP_ERROR(logger, "Failed to encode stop broadcasts message");
		
		}
		serialPortWrite(&serialPort, buffer, messageSize);
#if 0
		// ask for INS message 20 times a second (period of 50 milliseconds).  Max rate is 500 times a second (2ms period).
		messageSize = is_comm_get_data(&comm, _DID_INS_LLA_EULER_NED, 0, 0, 50);
		if (messageSize < 1)
		{
			LOG4CPP_ERROR(logger, "Failed to encode get INS message");
		}
		serialPortWrite(&serialPort, buffer, messageSize);

		// ask for gps message 5 times a second (period of 200 milliseconds) - offset and size can be left at 0 unless you want to just pull a specific field from a data set
		messageSize = is_comm_get_data(&comm, _DID_GPS_NAV, 0, 0, 200);
		if (messageSize < 1)
		{
			LOG4CPP_ERROR(logger, "Failed to encode get GPS message");
		}
		serialPortWrite(&serialPort, buffer, messageSize);
#endif
		// ask for IMU data 10 times a second - this could be as high as 1000 times a second (a period of 1)
		messageSize = is_comm_get_data(&comm, _DID_IMU_DUAL, 0, 0, (unsigned int)(1.f/freq*1000.f) );
		if (messageSize < 1)
		{
			LOG4CPP_ERROR(logger, "Failed to encode get IMU message");
		}
		serialPortWrite(&serialPort, buffer, messageSize);

		// ask for MAG message 10 times a second.  this could be as high as 1000 times a second (a period of 1)
		messageSize = is_comm_get_data(&comm, _DID_MAGNETOMETER_1, 0, 0, (unsigned int)(1.f / freq * 1000.f));
		if (messageSize < 1)
		{
			LOG4CPP_ERROR(logger, "Failed to encode get MAG message");
		}
		serialPortWrite(&serialPort, buffer, messageSize);
		
		m_running = true;
		m_bStop = false;
		m_pThread.reset(new boost::thread(boost::bind(&InertialSenseSensor::startCapturing, this)));
	}
	Component::start();
}

void InertialSenseSensor::startCapturing()
{
	
	while ( (count = serialPortReadCharTimeout(&serialPort, &inByte, -1)) > 0)
	{
		switch (is_comm_parse(&comm, inByte))
		{
		case _DID_IMU_DUAL:
			handleImuMessage((dual_imu_t*)buffer);
			break;
		case _DID_GPS1_POS:
			handleGpsMessage((gps_pos_t*)buffer);
			break;
		case _DID_MAGNETOMETER_1:
			handleMAGMessage((magnetometer_t*)buffer);
			break;
		case _DID_INS_LLA_EULER_NED:
			handleInsMessage((ins_1_t*)buffer);
			break;
		
			// TODO: add other cases for other data ids that you care about
		}
	}
}

void InertialSenseSensor::stop()
{
	if (m_running)
	{
		m_running = false;
		m_bStop = true;
		if (m_pThread)
		{
			m_pThread->join();
		}
	}
	Component::stop();
}

void InertialSenseSensor::handleInsMessage(ins_1_t* ins){
/*Measurement::Timestamp ts = Measurement::now();// ins->timeOfWeek;
	Measurement::Vector3D macc(ts, Math::Vector3d(ins->lla[0], ins->lla[1], ins->lla[2]));
	m_acc_OutPort.send(macc);
	m_gyro_OutPort.send(Measurement::Rotation(ts, Math::Quaternion(ins->theta[0] * C_RAD2DEG_F, ins->theta[1] * C_RAD2DEG_F, ins->theta[2] * C_RAD2DEG_F)));
*/
}
void InertialSenseSensor::handleGpsMessage(/*Measurement::Timestamp utTime,*/gps_pos_t* gps){

}

void InertialSenseSensor::handleImuMessage(/*Measurement::Timestamp utTime,*/ dual_imu_t* imu){
	Measurement::Timestamp ts = ceil(imu->time * 1000000000); //Measurement::now();// ins->timeOfWeek;
	Measurement::RotationVelocity mgyro(ts, Math::RotationVelocity(imu->I[0].pqr[0] * C_RAD2DEG_F, imu->I[0].pqr[1] * C_RAD2DEG_F, imu->I[0].pqr[2] * C_RAD2DEG_F));
	Measurement::Vector3D macc(ts, Math::Vector3d(imu->I[0].acc[0], imu->I[0].acc[1], imu->I[0].acc[2]));
	m_acc_OutPort.send(macc);
	m_gyro_OutPort.send(mgyro);

}
void InertialSenseSensor::handleMAGMessage(magnetometer_t* mag){

	Measurement::Timestamp ts = Measurement::now();// ins->timeOfWeek;

	m_mag_OutPort.send(Measurement::Vector3D(ts, Math::Vector3d(mag->mag[0], mag->mag[1], mag->mag[2])));

}
} } // namespace Ubitrack::Components

UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< Ubitrack::Drivers::InertialSenseSensor > ( "InertialSenseSensor" );

}