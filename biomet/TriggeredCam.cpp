#include "stdafx.h"

#include "TriggeredCam.h"
#include <iostream>
#include <exception>

using namespace std;
using namespace FlyCapture2;
using namespace cv;

/*
callback for status messages for XC cams
*/
ErrCode statusCheck(void *v_pUserParm, int iMsg, unsigned long ulP,
	unsigned long ulT) {
	switch (iMsg) {
	case XSMessage: {
		uint64_t ptr = (uint64_t)ulT << 32 | ulP;
		char *msg = reinterpret_cast<char *>(ptr);
		cout << msg << endl;
		break;
	}
	case XSDeviceInformation: {
		uint64_t ptr = (uint64_t)ulT << 32 | ulP;
		XDeviceInformation *dev = reinterpret_cast<XDeviceInformation *>(ptr);
		cout << dev->address << endl;
		cout << dev->name << endl;
		cout << dev->pid << endl;
		cout << dev->serial << endl;
		cout << dev->size << endl;
		cout << dev->state << endl;
		cout << dev->transport << endl;
		cout << dev->url << endl;
		break;
	}
	default:
		break;
	}
	return I_OK;
}

//XCTriggeredCam::XCTriggeredCam(const uint32_t serial, const uint32_t pktDelay) :
//TriggeredCam(serial) {
XCTriggeredCam::XCTriggeredCam(const uint32_t serial) : TriggeredCam(serial){
	try{
		DBG(cerr << "construct " << serial << endl);

		// find device from serial
		uint32_t deviceCount = 0;
		XC_Call(XCD_EnumerateDevices(NULL, &deviceCount, XEF_GigEVision), 1, 0, 0);
		assert_throw(deviceCount > 0);

		XDeviceInformation *devices = new XDeviceInformation[deviceCount];
		XC_Call(XCD_EnumerateDevices(devices, &deviceCount, XEF_UseCached), 1, 0, 0);

		XDeviceInformation *dev;
		for (dev = devices; dev != devices + deviceCount; dev++) {
			if (dev->serial == serial) {
				break;
			}
		}
		assert_throw(dev != devices + deviceCount);
		XC_Call(XCD_EnumerateDevices(NULL, NULL, XEF_ReleaseCache), 1, 0, 0);

		// init gige camera
		stringstream ss;
		ss << "gev://" << dev->address;
		assert_throw(
			(cam = XCamera::Create(ss.str().c_str(), DBGIFELSE(statusCheck, (XStatus)0))) != NULL);
		assert_throw(cam->IsInitialised());

		long pValue;

		//ensure properties are set
		XC_Call(cam->GetPropertyValueL("TriggerDirection", &pValue), 1, 0, 0);
		if (pValue != 1)
			XC_Call(cam->SetPropertyValueL("TriggerDirection", 1), 1, 100, 0);

		XC_Call(cam->GetPropertyValueL("TriggerInMode", &pValue), 1, 0, 0);
		if (pValue != 0)
			XC_Call(cam->SetPropertyValueL("TriggerInMode", 0), 1, 100, 0);

		XC_Call(cam->GetPropertyValueL("TriggerInEnable", &pValue), 1, 0, 0);
		if (pValue != 1)
			XC_Call(cam->SetPropertyValueL("TriggerInEnable", 1), 1, 100, 0);

		if (cam->GetHeight() != cam->GetMaxHeight())
			XC_Call(cam->SetPropertyValueL("Height", cam->GetMaxHeight()), 1, 100, 0);

		if (cam->GetWidth() != cam->GetMaxWidth())
			XC_Call(cam->SetPropertyValueL("Width", cam->GetMaxWidth()), 1, 100, 0);

		XC_Call(cam->GetPropertyValueL("XGIGEV_PacketDelay", &pValue), 1, 0, 0);
		if (pValue != 1000)
			XC_Call(cam->SetPropertyValueL("XGIGEV_PacketDelay", 1000), 1, 100, 0);

		XC_Call(cam->GetPropertyValueL("_API_FPC_DROPS", &pValue), 1, 0, 0);
		if (pValue != 1)
			XC_Call(cam->SetPropertyValueL("_API_FPC_DROPS", 1), 1, 100, 0);

		frameSize = cam->GetFrameSize();
		frameWidth = cam->GetWidth();
		frameHeight = cam->GetHeight();
		DBG(assert_throw(frameSize == (frameWidth * frameHeight * sizeof(word))));

		camsleep(3000);

		XC_Call(cam->StartCapture(), 1, 1000, 0);
		assert_throw(cam->IsCapturing());
	}
	catch (const runtime_error& e){
		throw TriggeredCamError(serial, e.what());
	}
}

XCTriggeredCam::~XCTriggeredCam() {
	try{
		DBG(cerr << "destroy " << serial << endl);
		XC_Call(cam->StopCapture(), 1, 0, 0);
	}
	catch (const runtime_error& e){
		throw TriggeredCamError(serial, e.what());
	}
}

void XCTriggeredCam::trigger() {
	try{
		DBG(cerr << "trigger " << serial << endl);
#if defined(_DEBUG)
		assert_throw(cam != NULL);
		assert_throw(cam->IsInitialised());
		assert_throw(cam->IsCapturing());
#endif
		XC_Call(cam->SetPropertyValueL("SoftwareTrigger", 1), 1, 0, 0);
	}
	catch (const runtime_error& e){
		throw TriggeredCamError(serial, e.what());
	}
}

Mat XCTriggeredCam::read() {
	try{
		DBG(cerr << "read " << serial << endl);
#if defined(_DEBUG)
		assert_throw(cam != NULL);
		assert_throw(cam->IsInitialised());
		assert_throw(cam->IsCapturing());
#endif
		Mat m;
		word *frameBuffer = new word[frameSize / sizeof(word)];
		try {
			XC_Call(cam->GetFrame(FT_NATIVE, 0, frameBuffer, frameSize), 40, 0, 1);
			m = Mat(frameHeight, frameWidth, CV_16UC1, frameBuffer).clone();
		}
		catch (...) {
			delete frameBuffer;
			throw;
		}
		delete frameBuffer;
		return m;
	}
	catch (const runtime_error& e){
		throw TriggeredCamError(serial, e.what());
	}
}

//PG1394TriggeredCam::PG1394TriggeredCam(const uint32_t serial,
//	const float& shutterSpeed,
//	const bool &broadcast) : broadcast(broadcast), TriggeredCam(serial){
PG1394TriggeredCam::PG1394TriggeredCam(const uint32_t serial, const bool broadcast) : broadcast(broadcast), TriggeredCam(serial){
	try{
		DBG(cerr << "construct " << serial << endl);

		// init gige cam from serial
		PGRGuid guid;
		BusManager bm;
		PG_Call(bm.GetCameraFromSerialNumber(this->serial, &guid), 1, 0, 0);
		PG_Call(cam.Connect(&guid), 1, 0, 0);
		assert_throw(cam.IsConnected());
		try {
			// ensure properties are set

			uint32_t regVal;
			PG_Call(cam.ReadRegister(REG_CAM_POWER, &regVal), 1, 0, 0);
			assert_throw(!(regVal >> 31 == 0));
			/*if (regVal >> 31 == 0) {
			DBG(cerr << regVal << endl);
			regVal = 1 << 31;
			PG_Call(cam.WriteRegister(REG_CAM_POWER, regVal), 1, 1000, 0);
			PG_Call(cam.ReadRegister(REG_CAM_POWER, &regVal), 1, 0, 0);
			assert_throw(regVal >> 31 != 0);
			}*/

			TriggerMode tm;
			PG_Call(cam.GetTriggerMode(&tm), 1, 0, 0);
			assert_throw(!(tm.onOff != true || tm.mode != 14 || tm.parameter != 0 || tm.source != 4));
			/*if (tm.onOff != true || tm.mode != 14 || tm.parameter != 0
			|| tm.source != 4) {
			DBG(cerr << tm.onOff << " " << tm.mode << " " << tm.parameter << " " << tm.source << endl);
			tm.onOff = true;
			tm.mode = 14;
			tm.parameter = 0;
			tm.source = 4;//7; // A source of 7 means software trigger
			PG_Call(cam.SetTriggerMode(&tm), 1, 100, 0);
			}*/

			Property prop;
			prop.type = SHUTTER;
			PG_Call(cam.GetProperty(&prop), 1, 0, 0);
			assert_throw(!(prop.autoManualMode != false || round(prop.absValue) != 10));
			/*if (prop.autoManualMode != false || prop.absValue != shutterSpeed) {
			DBG(cerr << prop.autoManualMode << " " << prop.absValue << endl);
			//prop.onOff = true;
			prop.autoManualMode = false;
			prop.absControl = true;
			prop.absValue = shutterSpeed;
			PG_Call(cam.SetProperty(&prop), 1, 100, 0);
			}*/

			// i dont understand exactly
			Format7Info fm7Info;
			bool supported;
			fm7Info.mode = MODE_0;
			PG_Call(cam.GetFormat7Info(&fm7Info, &supported), 1, 0, 0);
			assert_throw(supported);

			Format7ImageSettings fm7ImSett;
			Format7PacketInfo fm7PktInfo;
			float pktPct;
			PG_Call(cam.GetFormat7Configuration(&fm7ImSett, &fm7PktInfo.unitBytesPerPacket, &pktPct), 1, 0, 0);
			assert_throw(!(fm7ImSett.height != 960 || fm7ImSett.width != 1280
				|| fm7ImSett.offsetX != (fm7Info.maxWidth - fm7ImSett.width) / 2
				|| fm7ImSett.offsetY
				!= (fm7Info.maxHeight - fm7ImSett.height) / 2));
			/*if (gigeSett.height != 960 || gigeSett.width != 1280
			|| gigeSett.offsetX != (gigeInfo.maxWidth - gigeSett.width) / 2
			|| gigeSett.offsetY
			!= (gigeInfo.maxHeight - gigeSett.height) / 2) {
			DBG(cerr << gigeSett.height << " " << gigeSett.width << " " << gigeSett.offsetY << " " << gigeSett.offsetX << endl);
			gigeSett.height = 960;
			gigeSett.width = 1280;
			gigeSett.offsetX = (gigeInfo.maxWidth - gigeSett.width) / 2;
			gigeSett.offsetY = (gigeInfo.maxHeight - gigeSett.height) / 2;
			PG_Call(cam.SetGigEImageSettings(&gigeSett), 1, 100, 0);
			}*/

			/*GigEStreamChannel channel;
			PG_Call(cam.GetGigEStreamChannelInfo(0, &channel), 1, 0, 0);
			assert_throw(!(channel.interPacketDelay != pktDelay));*/
			/*if (channel.interPacketDelay != pktDelay) {
			DBG(cerr << channel.interPacketDelay << endl);
			channel.interPacketDelay = pktDelay;
			PG_Call(cam.SetGigEStreamChannelInfo(0, &channel), 1, 100, 0);
			}*/

			FC2Config config;
			PG_Call(cam.GetConfiguration(&config), 1, 0, 0);
			if (config.grabMode != DROP_FRAMES) {
				DBG(cerr << config.grabMode << endl);
				//config.grabTimeout = 250;
				config.grabMode = DROP_FRAMES;
				PG_Call(cam.SetConfiguration(&config), 1, 100, 0);
			}

			camsleep(3000);

			PG_Call(cam.StartCapture(), 1, 1000, 0);
		}
		catch (...) {
			PG_Call(cam.Disconnect(), 1, 0, 0);
			throw;
		}
	}
	catch (const runtime_error& e){
		throw TriggeredCamError(serial, e.what());
	}
}

PG1394TriggeredCam::~PG1394TriggeredCam() {
	try{
		DBG(cerr << "destroy " << serial << endl);
		PG_Call(cam.StopCapture(), 1, 0, 0);
		PG_Call(cam.Disconnect(), 1, 0, 0);
	}
	catch (const runtime_error& e){
		throw TriggeredCamError(serial, e.what());
	}
}

void PG1394TriggeredCam::trigger() {
	try{
		DBG(cerr << "trigger " << serial << endl);
		DBG(assert_throw(cam.IsConnected()));
		if (broadcast){
			PG_Call(cam.FireSoftwareTrigger(broadcast), 1, 0, 0);
		}
	}
	catch (const runtime_error& e){
		throw TriggeredCamError(serial, e.what());
	}
}

Mat PG1394TriggeredCam::read() {
	try{
		DBG(cerr << "read " << serial << endl);
		DBG(assert_throw(cam.IsConnected()));
		Image image, convImage;
		PG_Call(cam.RetrieveBuffer(&image), 1, 0, 0);
		PG_Call(image.Convert(PIXEL_FORMAT_BGR, &convImage), 1, 0, 0);
		return Mat(convImage.GetRows(), convImage.GetCols(), CV_8UC3,
			convImage.GetData()).clone();
	}
	catch (const runtime_error& e){
		throw TriggeredCamError(serial, e.what());
	}
}

//PGTriggeredCam::PGTriggeredCam(const uint32_t serial,
//	const float& shutterSpeed, const uint32_t pktDelay) :
//	TriggeredCam(serial) {
PGTriggeredCam::PGTriggeredCam(const uint32_t serial) : TriggeredCam(serial){
	try{
		DBG(cerr << "construct " << serial << endl);

		// init gige cam from serial
		PGRGuid guid;
		BusManager bm;
		PG_Call(bm.GetCameraFromSerialNumber(this->serial, &guid), 1, 0, 0);
		PG_Call(cam.Connect(&guid), 1, 0, 0);
		assert_throw(cam.IsConnected());
		try {
			// ensure properties are set

			uint32_t regVal;
			PG_Call(cam.ReadRegister(REG_CAM_POWER, &regVal), 1, 0, 0);
			assert_throw(!(regVal >> 31 == 0));
			/*if (regVal >> 31 == 0) {
				DBG(cerr << regVal << endl);
				regVal = 1 << 31;
				PG_Call(cam.WriteRegister(REG_CAM_POWER, regVal), 1, 1000, 0);
				PG_Call(cam.ReadRegister(REG_CAM_POWER, &regVal), 1, 0, 0);
				assert_throw(regVal >> 31 != 0);
				}*/

			TriggerMode tm;
			PG_Call(cam.GetTriggerMode(&tm), 1, 0, 0);
			assert_throw(!(tm.onOff != true || tm.mode != 14 || tm.parameter != 0 || tm.source != 4));
			/*if (tm.onOff != true || tm.mode != 14 || tm.parameter != 0
				|| tm.source != 4) {
				DBG(cerr << tm.onOff << " " << tm.mode << " " << tm.parameter << " " << tm.source << endl);
				tm.onOff = true;
				tm.mode = 14;
				tm.parameter = 0;
				tm.source = 4;//7; // A source of 7 means software trigger
				PG_Call(cam.SetTriggerMode(&tm), 1, 100, 0);
				}*/

			Property prop;
			prop.type = SHUTTER;
			PG_Call(cam.GetProperty(&prop), 1, 0, 0);
			assert_throw(!(prop.autoManualMode != false || prop.absValue != 10));
			/*if (prop.autoManualMode != false || prop.absValue != shutterSpeed) {
				DBG(cerr << prop.autoManualMode << " " << prop.absValue << endl);
				//prop.onOff = true;
				prop.autoManualMode = false;
				prop.absControl = true;
				prop.absValue = shutterSpeed;
				PG_Call(cam.SetProperty(&prop), 1, 100, 0);
				}*/

			GigEImageSettings gigeSett;
			GigEImageSettingsInfo gigeInfo;
			PG_Call(cam.GetGigEImageSettings(&gigeSett), 1, 0, 0);
			PG_Call(cam.GetGigEImageSettingsInfo(&gigeInfo), 1, 0, 0);
			assert_throw(!(gigeSett.height != 960 || gigeSett.width != 1280
				|| gigeSett.offsetX != (gigeInfo.maxWidth - gigeSett.width) / 2
				|| gigeSett.offsetY
				!= (gigeInfo.maxHeight - gigeSett.height) / 2));
			/*if (gigeSett.height != 960 || gigeSett.width != 1280
				|| gigeSett.offsetX != (gigeInfo.maxWidth - gigeSett.width) / 2
				|| gigeSett.offsetY
				!= (gigeInfo.maxHeight - gigeSett.height) / 2) {
				DBG(cerr << gigeSett.height << " " << gigeSett.width << " " << gigeSett.offsetY << " " << gigeSett.offsetX << endl);
				gigeSett.height = 960;
				gigeSett.width = 1280;
				gigeSett.offsetX = (gigeInfo.maxWidth - gigeSett.width) / 2;
				gigeSett.offsetY = (gigeInfo.maxHeight - gigeSett.height) / 2;
				PG_Call(cam.SetGigEImageSettings(&gigeSett), 1, 100, 0);
				}*/

			GigEStreamChannel channel;
			PG_Call(cam.GetGigEStreamChannelInfo(0, &channel), 1, 0, 0);
			assert_throw(!(channel.interPacketDelay != 1000));
			/*if (channel.interPacketDelay != pktDelay) {
				DBG(cerr << channel.interPacketDelay << endl);
				channel.interPacketDelay = pktDelay;
				PG_Call(cam.SetGigEStreamChannelInfo(0, &channel), 1, 100, 0);
				}*/

			FC2Config config;
			PG_Call(cam.GetConfiguration(&config), 1, 0, 0);
			if (config.grabMode != DROP_FRAMES) {
				DBG(cerr << config.grabMode << endl);
				//config.grabTimeout = 250;
				config.grabMode = DROP_FRAMES;
				PG_Call(cam.SetConfiguration(&config), 1, 100, 0);
			}

			camsleep(3000);

			PG_Call(cam.StartCapture(), 1, 1000, 0);
		}
		catch (...) {
			PG_Call(cam.Disconnect(), 1, 0, 0);
			throw;
		}
	}
	catch (const runtime_error& e){
		throw TriggeredCamError(serial, e.what());
	}
}

PGTriggeredCam::~PGTriggeredCam() {
	try{
		DBG(cerr << "destroy " << serial << endl);
		PG_Call(cam.StopCapture(), 1, 0, 0);
		PG_Call(cam.Disconnect(), 1, 0, 0);
	}
	catch (const runtime_error& e){
		throw TriggeredCamError(serial, e.what());
	}
}

void PGTriggeredCam::trigger() {
	try{
		DBG(cerr << "trigger " << serial << endl);
		DBG(assert_throw(cam.IsConnected()));
		PG_Call(cam.FireSoftwareTrigger(), 1, 0, 0);
	}
	catch (const runtime_error& e){
		throw TriggeredCamError(serial, e.what());
	}
}

Mat PGTriggeredCam::read() {
	try{
		DBG(cerr << "read " << serial << endl);
		DBG(assert_throw(cam.IsConnected()));
		Image image, convImage;
		PG_Call(cam.RetrieveBuffer(&image), 1, 0, 0);
		PG_Call(image.Convert(PIXEL_FORMAT_BGR, &convImage), 1, 0, 0);
		return Mat(convImage.GetRows(), convImage.GetCols(), CV_8UC3,
			convImage.GetData()).clone();
	}
	catch (const runtime_error& e){
		throw TriggeredCamError(serial, e.what());
	}
}
