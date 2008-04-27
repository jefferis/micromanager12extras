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

const char* g_DeviceNameLJSwitch = "LJ-Switch";
const char* g_DeviceNameLJShutter = "LJ-Shutter";
const char* g_DeviceNameLJDA0 = "LJ-DAC-0";
const char* g_DeviceNameLJDA1 = "LJ-DAC-1";

const char* g_volts = "Volts";

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

#include "../../../3rdparty/DataTranslation/SDK/include/olmem.h"
#include "../../../3rdparty/DataTranslation/SDK/include/olerrors.h"
#include "../../../3rdparty/DataTranslation/SDK/include/oldaapi.h"

/* simple structure used with board */

typedef struct tag_board {
   HDEV hdrvr;         /* device handle            */
   HDASS hdass_do;        /* sub system handle digital out*/
   HDASS hdass_da;        /* sub system handle DAC */
   ECODE status;       /* board error status       */
   HBUF  hbuf;         /* sub system buffer handle */
   PWORD lpbuf;        /* buffer pointer           */
   char name[MAX_BOARD_NAME_LENGTH];  /* string for board name    */
   char entry[MAX_BOARD_NAME_LENGTH]; /* string for board name    */
} BOARD;

typedef BOARD* LPBOARD;
static BOARD board;


/*
this is a callback function of olDaEnumBoards, it gets the 
strings of the Open Layers board and attempts to initialize
the board.  If successful, enumeration is halted.
*/
BOOL CALLBACK GetDriver( LPSTR lpszName, LPSTR lpszEntry, LPARAM lParam )   

{
   LPBOARD lpboard = (LPBOARD)(LPVOID)lParam;
   
   /* fill in board strings */

#ifdef WIN32
   strncpy(lpboard->name,lpszName,MAX_BOARD_NAME_LENGTH-1);
   strncpy(lpboard->entry,lpszEntry,MAX_BOARD_NAME_LENGTH-1);
#else
   lstrcpyn(lpboard->name,lpszName,MAX_BOARD_NAME_LENGTH-1);
   lstrcpyn(lpboard->entry,lpszEntry,MAX_BOARD_NAME_LENGTH-1);
#endif

   /* try to open board */

   lpboard->status = olDaInitialize(lpszName,&lpboard->hdrvr);
   if   (lpboard->hdrvr != NULL)
      return FALSE;          /* false to stop enumerating */
   else                      
      return TRUE;           /* true to continue          */
}

/*
 * Initilize the global board handle
 */
int InitializeTheBoard()
{
   if (board.hdrvr != 0)
      return DEVICE_OK;

   // initialize the board
   /* Get first available Open Layers board */
   board.hdrvr = NULL;
   int ret = olDaEnumBoards(GetDriver,(LPARAM)(LPBOARD)&board);
   if (ret != OLNOERROR)
      return ret;

   /* check for error within callback function */
   if (board.status != OLNOERROR)
      return board.status;

   /* check for NULL driver handle - means no boards */
   if (board.hdrvr == NULL)
      return ERR_BOARD_NOT_FOUND;

   /* get handle to DOUT sub system */
   ret = olDaGetDASS(board.hdrvr, OLSS_DOUT, 0, &board.hdass_do);
   if (ret != OLNOERROR)
      return ret;

   ret = olDaGetDASS(board.hdrvr, OLSS_DA, 0, &board.hdass_da);
   if (ret != OLNOERROR)
      return ret;

   /* set subsystem for single value operation */
   ret = olDaSetDataFlow(board.hdass_do, OL_DF_SINGLEVALUE);
   if (ret != OLNOERROR)
      return ret;
   ret = olDaSetDataFlow(board.hdass_da, OL_DF_SINGLEVALUE);
   if (ret != OLNOERROR)
      return ret;

   ret = olDaConfig(board.hdass_do);
   if (ret != OLNOERROR)
      return ret;
   ret = olDaConfig(board.hdass_da);
   if (ret != OLNOERROR)
      return ret;

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
   olDaReleaseDASS(board.hdass_do);
   olDaReleaseDASS(board.hdass_da);
   olDaTerminate(board.hdrvr);
   board.hdrvr = 0;
   initialized_ = false;
   return DEVICE_OK;
}

int CLJSwitch::WriteToPort(long value)
{
   //Out32(g_addrLPT1, buf);
   int ret = olDaPutSingleValue(board.hdass_do, value, 0 /* channel */, 1.0 /*gain*/);
   if (ret != OLNOERROR)
      return ret;

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
      busy_(false), minV_(0.0), maxV_(0.0), encoding_(0), resolution_(0), channel_(channel), name_(name)
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

   // obtain scaling info

   ret = olDaGetRange(board.hdass_da, &maxV_, &minV_);
   if (ret != OLNOERROR)
      return ret;

   ret = olDaGetEncoding(board.hdass_da, &encoding_);
   if (ret != OLNOERROR)
      return ret;

   ret = olDaGetResolution(board.hdass_da, &resolution_);
   if (ret != OLNOERROR)
      return ret;


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

   nRet = UpdateStatus();
   if (nRet != DEVICE_OK)
      return nRet;

   initialized_ = true;

   return DEVICE_OK;
}

int CLJDA::Shutdown()
{
   olDaReleaseDASS(board.hdass_do);
   olDaReleaseDASS(board.hdass_da);
   olDaTerminate(board.hdrvr);
   board.hdrvr = 0;
   initialized_ = false;
   return DEVICE_OK;
}

int CLJDA::SetSignal(double volts)
{
   return SetProperty(g_volts, CDeviceUtils::ConvertToString(volts));
}

int CLJDA::WriteToPort(long value)
{
   int ret = olDaPutSingleValue(board.hdass_da, value, channel_, 1.0 /*gain*/);
   if (ret != OLNOERROR)
      return ret;

   return DEVICE_OK;
}

int CLJDA::SetVolts(double volts)
{
   long value = (long) ((1L<<resolution_)/((float)maxV_ - (float)minV_) * (volts - (float)minV_));
   value = min((1L<<resolution_)-1,value);

   if (encoding_ != OL_ENC_BINARY) {
      // convert to 2's comp by inverting the sign bit
      long sign = 1L << (resolution_ - 1);
      value ^= sign;
      if (value & sign)           //sign extend
         value |= 0xffffffffL << resolution_;
   }
   return WriteToPort(value);
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
   int ret = olDaPutSingleValue(board.hdass_do, value, 0 /* channel */, 1.0 /*gain*/);
   if (ret != OLNOERROR)
      return ret;
   //Out32(g_addrLPT1, buf);
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
