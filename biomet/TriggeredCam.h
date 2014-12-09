#include "stdafx.h"

#ifndef TRIGGEREDCAM_H_
#define TRIGGEREDCAM_H_

/*
an attempt at making this platform independent
*/
#if defined(_MSC_VER) || defined(__MINGW32__) || defined(__MINGW64__)
#include "windows.h"
#include <FlyCapture2.h>
#define camsleep(ms) (Sleep(ms))
#define __STRING(x) #x
#elif defined(__GNUC__)
#include "unistd.h"
#include <flycapture/FlyCapture2.h>
#define camsleep(ms) (usleep(ms * 1000))
#endif

#include <XCamera.h>
#include <opencv2/core/core.hpp>
#include <stdint.h>
#include <stdexcept>

#ifndef _DEBUG                  /* For RELEASE builds */
#define DBG(expr)  do {;} while (0)
#else                           /* For DEBUG builds   */
#define  DBG(expr) do { expr; } while (0)
#endif

/*
base for all triggered cameras
*/
class TriggeredCam {
public:
	TriggeredCam(const uint32_t serial) :
		serial(serial) {
	}
	virtual ~TriggeredCam() {
	}
	virtual void
		trigger() = 0;
	virtual cv::Mat
		read() = 0;
	const uint32_t serial;
};

/*
GigE point grey triggered camera
*/
class PGTriggeredCam : public TriggeredCam {
public:
	//PGTriggeredCam(const uint32_t serial, const float& shutterSpeed,
	//	const uint32_t pktDelay);
	PGTriggeredCam(const uint32_t serial);
	virtual
		~PGTriggeredCam();
	virtual void
		trigger();
	virtual cv::Mat
		read();
private:
	static const uint32_t REG_CAM_POWER = 0x610;
	FlyCapture2::GigECamera cam;
};

class PG1394TriggeredCam : public TriggeredCam{
public:
	//PG1394TriggeredCam(const uint32_t serial, const float& shutterSpeed, const bool &broadcast);
	PG1394TriggeredCam(const uint32_t serial, const bool broadcast);
	virtual ~PG1394TriggeredCam();
	virtual void trigger();
	virtual cv::Mat read();
private:
	static const uint32_t REG_CAM_POWER = 0x610;
	FlyCapture2::Camera cam;
	bool broadcast;
};

/*
GigE xenics triggered camera
*/
class XCTriggeredCam : public TriggeredCam {
public:
	//XCTriggeredCam(const uint32_t serial, const uint32_t pktDelay);
	XCTriggeredCam(const uint32_t serial);
	virtual
		~XCTriggeredCam();
	virtual void
		trigger();
	virtual cv::Mat
		read();
private:
	cv::Ptr<XCamera> cam;
	dword frameSize;
	dword frameWidth;
	dword frameHeight;
};

/*
checks for errors in PG or XC function calls
*/
#define PG_CheckError(expr) \
{ \
	FlyCapture2::Error error = (expr); \
	if (error != PGRERROR_OK) \
			{ \
		throw PGError(__FILE__, __LINE__, __STRING(expr), error); \
			} \
} \

#define XC_CheckError(expr) \
{ \
	ErrCode ec = (expr); \
	if (ec != I_OK) \
			{ \
		throw XCError(__FILE__, __LINE__, __STRING((expr)), ec); \
			} \
} \

/*
hack for if-else evaluation depending on debug or release configuration
*/
#if defined(_DEBUG)
#define DBGIFELSE(t,f) (t)
#else
#define DBGIFELSE(t,f) (f)
#endif

/*
assert that throws an exception
*/
#define assert_throw(expr)                                             \
	((expr)                                                               \
	? (static_cast<void>(0))                                             \
	: throw AssertionError(__FILE__, __LINE__, __STRING(expr)))

/*
calls PG or XC functions with multiple "tries" and sleeps for "passms" or "failms" depending on success of call
*/
#define PG_Call(expr, tries, passms, failms) \
{ \
	DBG(cerr << __STRING(expr) << endl); \
	assert_throw((tries) > 0); \
	int i; \
	for (i = 0; i < (tries); i++){ \
		try{ \
			PG_CheckError(expr); \
			camsleep((passms)); \
			break; \
		} \
		catch (const PGError& pge){ \
			DBG(cerr << "try " << i + 1 << ": " << pge.what() << endl); \
			if( i + 1 >= (tries)) \
				throw; \
												else \
				camsleep((failms)); \
		} \
			} \
} \

#define XC_Call(expr, tries, passms, failms) \
{ \
	DBG(cerr << __STRING(expr) << endl); \
	assert_throw((tries) > 0); \
	int i; \
	for (i = 0; i < (tries); i++){ \
		try{ \
			XC_CheckError(expr); \
			camsleep((passms)); \
			break; \
		} \
		catch (const XCError& xce){ \
			DBG(cerr << "try " << i + 1 << ": " << xce.what() << endl); \
			if( i + 1 >= (tries)) \
				throw; \
												else \
				camsleep((failms)); \
		} \
			} \
} \


class AssertionError : public std::runtime_error {
public:
	AssertionError(const std::string& file, const int& line,
		const std::string& expr) :
		std::runtime_error(AssertionError::createMsg(file, line, expr)) {
	}
	std::string createMsg(const std::string& file, const int& line,
		const std::string& expr) {
		std::stringstream ss;
		ss << file << "[" << line << "] " << expr << " AssertionError: assertion failed";
		return ss.str();
	}
};

/*
various runtime_error subclasses
*/

class TriggeredCamError : public std::runtime_error{
public:
	TriggeredCamError(const uint32_t serial, const std::string& what) : std::runtime_error(TriggeredCamError::createMsg(serial, what)){
	}
	static std::string createMsg(const uint32_t serial, const std::string& what){
		std::stringstream ss;
		ss << "TriggeredCamError(" << serial << "): " << what;
		return ss.str();
	}
};

class XCError : public std::runtime_error {
public:
	XCError(const std::string& file, const int& line, const std::string& expr,
		const ErrCode& errCode) :
		std::runtime_error(XCError::createMsg(file, line, expr, errCode)), errCode(
		errCode) {
	}
	static std::string createMsg(const std::string& file, const int& line,
		const std::string& expr, const ErrCode& errCode) {
		std::stringstream ss;
		char errCodeStr[256];
		assert_throw(XC_ErrorToString(errCode, errCodeStr, 256) > 0);
		ss << file << "[" << line << "] " << expr << " XCError: " << errCodeStr
			<< "(" << errCode << ")";
		return ss.str();
	}
	const ErrCode errCode;
};

class PGError : public std::runtime_error {
public:
	PGError(const std::string& file, const int& line, const std::string& expr,
		const FlyCapture2::Error& error) :
		std::runtime_error(PGError::createMsg(file, line, expr, error)), error(
		error) {
	}
	~PGError() throw () {
	}
	static std::string createMsg(const std::string& file, const int& line,
		const std::string& expr, const FlyCapture2::Error& error) {
		std::stringstream ss;
		const char * errorStr = error.GetDescription();
		ss << file << "[" << line << "] " << expr << " PGError: " << errorStr
			<< "(" << error.GetType() << ")";
		return ss.str();
	}
	const FlyCapture2::Error error;
};

#endif /* TRIGGEREDCAM_H_ */
