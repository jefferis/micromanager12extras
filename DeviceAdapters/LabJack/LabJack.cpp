///////////////////////////////////////////////////////////////////////////////
// FILE:          LabJack.cpp
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   LabJack USB I/O data acquistion adapter
// COPYRIGHT:     Gregory Jefferis, 2008
// LICENSE:       This file is distributed under the BSD license.
//                License text is included with the source distribution.
//
//                This file is distributed in the hope that it will be useful,
//                but WITHOUT ANY WARRANTY; without even the implied warranty
//                of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
//                IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//                CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//                INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES.
// AUTHOR:        Gregory Jefferis, jefferis@gmail.com, 04/27/2008
//                Adapted from DTOpenLayer adapter by Nenad Amodaj

#ifdef WIN32
   #define WIN32_LEAN_AND_MEAN
   #include <windows.h>
   #define snprintf _snprintf 
#endif

#include "LabJack.h"
#include "../../MMDevice/ModuleInterface.h"
#include "u3.h"

const char* g_DeviceNameLJSwitch = "LJ-Switch";
const char* g_DeviceNameLJShutter = "LJ-Shutter";
const char* g_DeviceNameLJDA0 = "LJ-DAC-0";
const char* g_DeviceNameLJDA1 = "LJ-DAC-1";

const char* g_volts = "Volts";
const char* g_channel = "Channel";

// Global state of the LJ switch to enable simulation of the shutter device.
// The virtual shutter device uses this global variable to restore state of the switch
unsigned g_switchState = 0;
unsigned g_shutterState = 0;

using namespace std;

// global constants

#ifdef WIN32
   BOOL APIENTRY DllMain( HANDLE /*hModule*/, 
                          DWORD  ul_reason_for_call, 
                          LPVOID /*lpReserved*/
		   			 )
   {
   	switch (ul_reason_for_call)
   	{
   	case DLL_PROCESS_ATTACH:
   	case DLL_THREAD_ATTACH:
   	case DLL_THREAD_DETACH:
   	case DLL_PROCESS_DETACH:
   		break;
   	}
       return TRUE;
   }
#endif


HANDLE hDevice;
u3CalibrationInfo caliInfo;

/*
 * Initilize the global board handle
 */
int InitializeTheBoard()
{
   //Open first found U3 over USB
   int localID = -1;
   if( (hDevice = openUSBConnection(localID)) == NULL)
      return DEVICE_ERR;

   //Get calibration information from U3
   if(getCalibrationInfo(hDevice, &caliInfo) < 0){
      closeUSBConnection(hDevice);
      return DEVICE_ERR;
   }
   return DEVICE_OK;
}


///////////////////////////////////////////////////////////////////////////////
// Exported MMDevice API
///////////////////////////////////////////////////////////////////////////////
MODULE_API void InitializeModuleData()
{
   AddAvailableDeviceName(g_DeviceNameLJSwitch);
   AddAvailableDeviceName(g_DeviceNameLJShutter);
   AddAvailableDeviceName(g_DeviceNameLJDA0);
   AddAvailableDeviceName(g_DeviceNameLJDA1);
}

MODULE_API MM::Device* CreateDevice(const char* deviceName)
{
   if (deviceName == 0)
      return 0;

   if (strcmp(deviceName, g_DeviceNameLJSwitch) == 0)
   {
      return new CLJSwitch;
   }
   else if (strcmp(deviceName, g_DeviceNameLJShutter) == 0)
   {
      return new CLJShutter;
   }
   else if (strcmp(deviceName, g_DeviceNameLJDA0) == 0)
   {
      return new CLJDA(0, g_DeviceNameLJDA0);
   }
   else if (strcmp(deviceName, g_DeviceNameLJDA1) == 0)
   {
      return new CLJDA(1, g_DeviceNameLJDA1);
   }


   return 0;
}

MODULE_API void DeleteDevice(MM::Device* pDevice)
{
   delete pDevice;
}

///////////////////////////////////////////////////////////////////////////////
// CLJSwitch implementation
// ~~~~~~~~~~~~~~~~~~~~~~~~~~

CLJSwitch::CLJSwitch() : numPos_(256), busy_(false)
{
   InitializeDefaultErrorMessages();

   // add custom error messages
   SetErrorText(ERR_UNKNOWN_POSITION, "Invalid position (state) specified");
   SetErrorText(ERR_INITIALIZE_FAILED, "Initialization of the device failed");
   SetErrorText(ERR_WRITE_FAILED, "Failed to write data to the device");
   SetErrorText(ERR_CLOSE_FAILED, "Failed closing the device");
}

CLJSwitch::~CLJSwitch()
{
   Shutdown();
}

void CLJSwitch::GetName(char* name) const
{
   CDeviceUtils::CopyLimitedString(name, g_DeviceNameLJSwitch);
}


int CLJSwitch::Initialize()
{
   int ret = InitializeTheBoard();
   if (ret != DEVICE_OK)
      return ret;

   // set property list
   // -----------------
   
   // Name
   int nRet = CreateProperty(MM::g_Keyword_Name, g_DeviceNameLJSwitch, MM::String, true);
   if (DEVICE_OK != nRet)
      return nRet;

   // Description
   nRet = CreateProperty(MM::g_Keyword_Description, "LJ digital output driver", MM::String, true);
   if (DEVICE_OK != nRet)
      return nRet;

   // create positions and labels
   const int bufSize = 1024;
   char buf[bufSize];
   for (long i=0; i<numPos_; i++)
   {
      snprintf(buf, bufSize, "%d", (unsigned)i);
      SetPositionLabel(i, buf);
   }

   // State
   // -----
   CPropertyAction* pAct = new CPropertyAction (this, &CLJSwitch::OnState);
   nRet = CreateProperty(MM::g_Keyword_State, "0", MM::Integer, false, pAct);
   if (nRet != DEVICE_OK)
      return nRet;

   // Label
   // -----
   pAct = new CPropertyAction (this, &CStateBase::OnLabel);
   nRet = CreateProperty(MM::g_Keyword_Label, "", MM::String, false, pAct);
   if (nRet != DEVICE_OK)
      return nRet;

   nRet = UpdateStatus();
   if (nRet != DEVICE_OK)
      return nRet;

   initialized_ = true;

   return DEVICE_OK;
}

int CLJSwitch::Shutdown()
{
   closeUSBConnection(hDevice);
   initialized_ = false;
   return DEVICE_OK;
}

int CLJSwitch::WriteToPort(long value)
{
   long error = eDO(hDevice, 1, 0 /*channel*/, value);
   
   if (error != 0)
      return error;

   return DEVICE_OK;
}

///////////////////////////////////////////////////////////////////////////////
// Action handlers
///////////////////////////////////////////////////////////////////////////////

int CLJSwitch::OnState(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      // nothing to do, let the caller to use cached property
   }
   else if (eAct == MM::AfterSet)
   {
      long pos;
      pProp->Get(pos);
      g_switchState = pos;
      if (g_shutterState > 0)
         return WriteToPort(pos);
   }

   return DEVICE_OK;
}



///////////////////////////////////////////////////////////////////////////////
// CLJDA implementation
// ~~~~~~~~~~~~~~~~~~~~~~

CLJDA::CLJDA(unsigned channel, const char* name) :
      busy_(false), minV_(0.0), maxV_(0.0), channel_(channel), name_(name)
{
   InitializeDefaultErrorMessages();

   // add custom error messages
   SetErrorText(ERR_UNKNOWN_POSITION, "Invalid position (state) specified");
   SetErrorText(ERR_INITIALIZE_FAILED, "Initialization of the device failed");
   SetErrorText(ERR_WRITE_FAILED, "Failed to write data to the device");
   SetErrorText(ERR_CLOSE_FAILED, "Failed closing the device");
}

CLJDA::~CLJDA()
{
   Shutdown();
}

void CLJDA::GetName(char* name) const
{
   CDeviceUtils::CopyLimitedString(name, name_.c_str());
}


int CLJDA::Initialize()
{
   int ret = InitializeTheBoard();
   if (ret != DEVICE_OK)
      return ret;
   
   // Standard for LabJack
   // TODO: Can we query these?
   maxV_=5.0;minV_=0.0;

   // set property list
   // -----------------
   
   // Name
   int nRet = CreateProperty(MM::g_Keyword_Name, name_.c_str(), MM::String, true);
   if (DEVICE_OK != nRet)
      return nRet;

   // Description
   nRet = CreateProperty(MM::g_Keyword_Description, "LJ DAC driver", MM::String, true);
   if (DEVICE_OK != nRet)
      return nRet;


   // State
   // -----
   CPropertyAction* pAct = new CPropertyAction (this, &CLJDA::OnVolts);
   nRet = CreateProperty(g_volts, "0.0", MM::Float, false, pAct);
   if (nRet != DEVICE_OK)
      return nRet;
   nRet = SetPropertyLimits(g_volts, minV_, maxV_);
   if (nRet != DEVICE_OK) return nRet;

   // Channel
   // -----
   pAct = new CPropertyAction (this, &CLJDA::OnChannel);
   nRet = CreateProperty(g_channel, "0", MM::Integer, false, pAct);
   if (nRet != DEVICE_OK)
      return nRet;
   // TODO: can we query these?
   nRet = SetPropertyLimits(g_channel, 0, 1);
   if (nRet != DEVICE_OK) return nRet;
      

   nRet = UpdateStatus();
   if (nRet != DEVICE_OK)
      return nRet;

   initialized_ = true;

   return DEVICE_OK;
}

int CLJDA::Shutdown()
{
   closeUSBConnection(hDevice);
   initialized_ = false;
   return DEVICE_OK;
}

int CLJDA::SetSignal(double volts)
{
   return SetProperty(g_volts, CDeviceUtils::ConvertToString(volts));
}

int CLJDA::SetVolts(double volts)
{
   long error = eDAC(hDevice, &caliInfo, 0, channel_, volts, 0, 0, 0);

   if (error != 0)
      return error;

   return DEVICE_OK;
}

int CLJDA::OnChannel(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set((long int)channel_);
   }
   else if (eAct == MM::AfterSet)
   {
      long channel;
      pProp->Get(channel);
      if (channel >=0 && channel <=1)
         channel_ = channel;
   }
   return DEVICE_OK;
}


///////////////////////////////////////////////////////////////////////////////
// Action handlers
///////////////////////////////////////////////////////////////////////////////

int CLJDA::OnVolts(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      // nothing to do, let the caller to use cached property
   }
   else if (eAct == MM::AfterSet)
   {
      double volts;
      pProp->Get(volts);
      return SetVolts(volts);
   }

   return DEVICE_OK;
}


///////////////////////////////////////////////////////////////////////////////
// CLJShutter implementation
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~

CLJShutter::CLJShutter() : initialized_(false), name_(g_DeviceNameLJShutter), openTimeUs_(0)
{
   InitializeDefaultErrorMessages();
   EnableDelay();
}

CLJShutter::~CLJShutter()
{
   Shutdown();
}

void CLJShutter::GetName(char* name) const
{
   assert(name_.length() < CDeviceUtils::GetMaxStringLength());
   CDeviceUtils::CopyLimitedString(name, name_.c_str());
}

bool CLJShutter::Busy()
{
   long interval = GetClockTicksUs() - openTimeUs_;

   if (interval/1000.0 < GetDelayMs() && interval > 0)
   {
      return true;
   }
   else
   {
       return false;
   }
}

int CLJShutter::Initialize()
{
   int ret = InitializeTheBoard();
   if (ret != DEVICE_OK)
      return ret;

   // set property list
   // -----------------
   
   // Name
   ret = CreateProperty(MM::g_Keyword_Name, name_.c_str(), MM::String, true);
   if (DEVICE_OK != ret)
      return ret;

   // Description
   ret = CreateProperty(MM::g_Keyword_Description, "LJ shutter driver (LPT)", MM::String, true);
   if (DEVICE_OK != ret)
      return ret;

   // OnOff
   // ------
   CPropertyAction* pAct = new CPropertyAction (this, &CLJShutter::OnOnOff);
   ret = CreateProperty("OnOff", "0", MM::Integer, false, pAct);
   if (ret != DEVICE_OK)
      return ret;

   // set shutter into the off state
   WriteToPort(0);

   vector<string> vals;
   vals.push_back("0");
   vals.push_back("1");
   ret = SetAllowedValues("OnOff", vals);
   if (ret != DEVICE_OK)
      return ret;

   ret = UpdateStatus();
   if (ret != DEVICE_OK)
      return ret;

   initialized_ = true;
   openTimeUs_ = GetClockTicksUs();

   return DEVICE_OK;
}

int CLJShutter::Shutdown()
{
   if (initialized_)
   {
      initialized_ = false;
      // GJ: Do I need this?
      closeUSBConnection(hDevice);
   }
   return DEVICE_OK;
}

int CLJShutter::SetOpen(bool open)
{
   if (open)
      return SetProperty("OnOff", "1");
   else
      return SetProperty("OnOff", "0");
}

int CLJShutter::GetOpen(bool& open)
{
   char buf[MM::MaxStrLength];
   int ret = GetProperty("OnOff", buf);
   if (ret != DEVICE_OK)
      return ret;
   long pos = atol(buf);
   pos > 0 ? open = true : open = false;

   return DEVICE_OK;
}

int CLJShutter::Fire(double /*deltaT*/)
{
   return DEVICE_UNSUPPORTED_COMMAND;
}

int CLJShutter::WriteToPort(long value)
{
   long error = eDO(hDevice, 1, 0 /*channel*/, value);
   
   if (error != 0)
      return error;

   return DEVICE_OK;
}

///////////////////////////////////////////////////////////////////////////////
// Action handlers
///////////////////////////////////////////////////////////////////////////////

int CLJShutter::OnOnOff(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      // use cached state
   }
   else if (eAct == MM::AfterSet)
   {
      long pos;
      pProp->Get(pos);
      int ret;
      if (pos == 0)
         ret = WriteToPort(0); // turn everything off
      else
         ret = WriteToPort(g_switchState); // restore old setting
      if (ret != DEVICE_OK)
         return ret;
      g_shutterState = pos;
      openTimeUs_ = GetClockTicksUs();
   }

   return DEVICE_OK;
}

