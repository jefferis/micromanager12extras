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
   long error;
   
   //Open first found U3 over USB
   int localID = -1;
   if( (hDevice = openUSBConnection(localID)) == NULL)
      goto done;

   //Get calibration information from U3
   if(getCalibrationInfo(hDevice, &caliInfo) < 0)
      goto close;
   close:
      if(error > 0)
      printf("Received an error code of %ld\n", error);
      closeUSBConnection(hDevice);
   done:
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
   // TOFIX: GJ need to get these values 
   // ret = olDaGetRange(board.hdass_da, &maxV_, &minV_);
   // if (ret != OLNOERROR)
   //    return ret;
   // 
   // ret = olDaGetEncoding(board.hdass_da, &encoding_);
   // if (ret != OLNOERROR)
   //    return ret;
   // 
   // ret = olDaGetResolution(board.hdass_da, &resolution_);
   // if (ret != OLNOERROR)
   //    return ret;


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
      if (channel >=1 && channel <=8)
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

//Author: LabJack
//April 25, 2008
//Example U3 helper functions.  Function descriptions are in the u3.h file.


void normalChecksum(uint8 *b, int n)
{
  b[0] = normalChecksum8(b,n);
}


void extendedChecksum(uint8 *b, int n)
{
  uint16 a;

  a = extendedChecksum16(b,n);
  b[4] = (uint8)(a & 0xff);
  b[5] = (uint8)((a/256) & 0xff);
  b[0] = extendedChecksum8(b);
}


uint8 normalChecksum8(uint8 *b, int n)
{
  int i;
  uint16 a, bb;

  //Sums bytes 1 to n-1 unsigned to a 2 byte value. Sums quotient and
  //remainder of 256 division.  Again, sums quotient and remainder of
  //256 division.
  for(i = 1, a = 0; i < n; i++)
    a+=(uint16)b[i];

  bb = a/256;
  a = (a-256*bb)+bb;
  bb = a/256;

  return (uint8)((a-256*bb)+bb);

}


uint16 extendedChecksum16(uint8 *b, int n)
{
  int i, a = 0;

  //Sums bytes 6 to n-1 to a unsigned 2 byte value
  for(i = 6; i < n; i++)
    a += (uint16)b[i];

  return a;
}


uint8 extendedChecksum8(uint8 *b)
{
  int i, a, bb;

  //Sums bytes 1 to 5. Sums quotient and remainder of 256 division. Again, sums
  //quotient and remainder of 256 division.
  for(i = 1, a = 0; i < 6; i++)
    a+=(uint16)b[i];

  bb=a/256;
  a=(a-256*bb)+bb;
  bb=a/256;

  return (uint8)((a-256*bb)+bb);
}


HANDLE openUSBConnection(int localID)
{
  BYTE sendBuffer[26], recBuffer[38];
  uint16 checksumTotal = 0;
  HANDLE hDevice = 0;
  uint32 numDevices = 0;
  uint32 dev;
  int i;

  numDevices = LJUSB_GetDevCount(U3_PRODUCT_ID);
  if(numDevices == 0)
  {
    printf("Open error: No U3 devices could be found\n");
    return NULL;
  }

  for(dev = 1;  dev <= numDevices; dev++)
  {
    hDevice = LJUSB_OpenDevice(dev, 0, U3_PRODUCT_ID);
    if(hDevice != NULL)
    {
      if(localID < 0)
      {
        return hDevice;
      }
      else
      {
        checksumTotal = 0;

        //setting up a CommConfig command
        sendBuffer[1] = (uint8)(0xF8);
        sendBuffer[2] = (uint8)(0x0A);
        sendBuffer[3] = (uint8)(0x08);

        for(i = 6; i < 26; i++)
          sendBuffer[i] = (uint8)(0x00);

        extendedChecksum(sendBuffer, 26);

        if(LJUSB_BulkWrite(hDevice, U3_PIPE_EP1_OUT, sendBuffer, 26) != 26)
          goto locid_error;

        if(LJUSB_BulkRead(hDevice, U3_PIPE_EP1_IN, recBuffer, 38) != 38)
          goto locid_error;

        checksumTotal = extendedChecksum16(recBuffer, 38);
        if( (uint8)((checksumTotal / 256) & 0xff) != recBuffer[5])
          goto locid_error;

        if( (uint8)(checksumTotal & 0xff) != recBuffer[4])
          goto locid_error;

        if( extendedChecksum8(recBuffer) != recBuffer[0])
          goto locid_error;

        if( recBuffer[1] != (uint8)(0xF8) || recBuffer[2] != (uint8)(0x10) ||
            recBuffer[3] != (uint8)(0x08) )
          goto locid_error;

        if( recBuffer[6] != 0)
		  goto locid_error;

        if( (int)recBuffer[21] == localID)
          return hDevice;
        else
          LJUSB_CloseDevice(hDevice);

      } //else localID >= 0 end
    } //if hDevice != NULL end
  } //for end

  printf("Open error: could not find a U3 with a local ID of %d\n", localID);
  return NULL;

locid_error:
  printf("Open error: problem when checking local ID\n");
  return NULL;
}


void closeUSBConnection(HANDLE hDevice)
{
  LJUSB_CloseDevice(hDevice);
}


long getTickCount()
{
  struct timeval tv;

  gettimeofday(&tv, NULL);

  return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}


long isCalibrationInfoValid(u3CalibrationInfo *caliInfo)
{
  if(caliInfo == NULL)
    goto invalid;
  if(caliInfo->prodID != 3)
    goto invalid;
  return 1;
invalid:
  printf("Error: Invalid calibration info.\n");
  return -1;
}


long isLJTDACCalibrationInfoValid(u3LJTDACCalibrationInfo *caliInfo)
{
  if(caliInfo == NULL)
    goto invalid;
  if(caliInfo->prodID != 3)
    goto invalid;
  return 1;
invalid:
  printf("Error: Invalid LJTDAC calibration info.\n");
  return -1;
}


long getCalibrationInfo(HANDLE hDevice, u3CalibrationInfo *caliInfo)
{
  uint8 sendBuffer[8], recBuffer[40];
  uint8 cU3SendBuffer[26], cU3RecBuffer[38];
  int sentRec = 0;
  int i = 0;

  /* sending ConfigU3 command to get hardware version and see if HV */
  cU3SendBuffer[1] = (uint8)(0xF8);  //command byte
  cU3SendBuffer[2] = (uint8)(0x0A);  //number of data words
  cU3SendBuffer[3] = (uint8)(0x08);  //extended command number

  //setting WriteMask0 and all other bytes to 0 since we only want to read the response
  for(i = 6; i < 26; i++)
    cU3SendBuffer[i] = 0;

  extendedChecksum(cU3SendBuffer, 26);

  sentRec = LJUSB_BulkWrite(hDevice, U3_PIPE_EP1_OUT, cU3SendBuffer, 26);
  if(sentRec < 26)
  {
    if(sentRec == 0)
      goto writeError0;
    else
      goto writeError1;
  }

  sentRec = LJUSB_BulkRead(hDevice, U3_PIPE_EP1_IN, cU3RecBuffer, 38);
  if(sentRec < 38)
  {
    if(sentRec == 0)
      goto readError0;
    else
      goto readError1;
  }

  if(cU3RecBuffer[1] != (uint8)(0xF8) || cU3RecBuffer[2] != (uint8)(0x10) || cU3RecBuffer[3] != (uint8)(0x08))
    goto commandByteError;

  caliInfo->hardwareVersion = cU3RecBuffer[14] + cU3RecBuffer[13]/100.0;
  caliInfo->highVoltage = (((cU3RecBuffer[37]&18) == 18)?1:0);

  /* reading block 0 from memory */
  sendBuffer[1] = (uint8)(0xF8);  //command byte
  sendBuffer[2] = (uint8)(0x01);  //number of data words
  sendBuffer[3] = (uint8)(0x2D);  //extended command number
  sendBuffer[6] = 0;
  sendBuffer[7] = 0;              //Blocknum = 0
  extendedChecksum(sendBuffer, 8);

  sentRec = LJUSB_BulkWrite(hDevice, U3_PIPE_EP1_OUT, sendBuffer, 8);
  if(sentRec < 8)
  {
    if(sentRec == 0)
      goto writeError0;
    else
      goto writeError1;
  }

  sentRec = LJUSB_BulkRead(hDevice, U3_PIPE_EP1_IN, recBuffer, 40);
  if(sentRec < 40)
  {
    if(sentRec == 0)
      goto readError0;
    else
      goto readError1;
  }

  if(recBuffer[1] != (uint8)(0xF8) || recBuffer[2] != (uint8)(0x11) || recBuffer[3] != (uint8)(0x2D))
    goto commandByteError;

  //block data starts on byte 8 of the buffer
  caliInfo->ainSESlope = FPuint8ArrayToFPDouble(recBuffer + 8, 0);
  caliInfo->ainSEOffset = FPuint8ArrayToFPDouble(recBuffer + 8, 8);
  caliInfo->ainDiffSlope = FPuint8ArrayToFPDouble(recBuffer + 8, 16);
  caliInfo->ainDiffOffset = FPuint8ArrayToFPDouble(recBuffer + 8, 24);

  /* reading block 1 from memory */
  sendBuffer[7] = 1;    //Blocknum = 1
  extendedChecksum(sendBuffer, 8);

  sentRec = LJUSB_BulkWrite(hDevice, U3_PIPE_EP1_OUT, sendBuffer, 8);
  if(sentRec < 8)
  {
    if(sentRec == 0)
      goto writeError0;
    else
      goto writeError1;
  }

  sentRec = LJUSB_BulkRead(hDevice, U3_PIPE_EP1_IN, recBuffer, 40);;

  if(sentRec < 40)
  {
    if(sentRec == 0)
      goto readError0;
    else
      goto readError1;
  }

  if(recBuffer[1] != (uint8)(0xF8) || recBuffer[2] != (uint8)(0x11) || recBuffer[3] != (uint8)(0x2D))
    goto commandByteError;

  caliInfo->dacSlope[0] = FPuint8ArrayToFPDouble(recBuffer + 8, 0);
  caliInfo->dacOffset[0] = FPuint8ArrayToFPDouble(recBuffer + 8, 8);
  caliInfo->dacSlope[1] = FPuint8ArrayToFPDouble(recBuffer + 8, 16);
  caliInfo->dacOffset[1] = FPuint8ArrayToFPDouble(recBuffer + 8, 24);

  /* reading block 2 from memory */
  sendBuffer[7] = 2;    //Blocknum = 2
  extendedChecksum(sendBuffer, 8);

  sentRec = LJUSB_BulkWrite(hDevice, U3_PIPE_EP1_OUT, sendBuffer, 8);
  if(sentRec < 8)
  {
    if(sentRec == 0)
      goto writeError0;
    else
      goto writeError1;
  }

  sentRec = LJUSB_BulkRead(hDevice, U3_PIPE_EP1_IN, recBuffer, 40);
  if(sentRec < 40)
  {
    if(sentRec == 0)
      goto readError0;
    else
      goto readError1;
  }

  if(recBuffer[1] != (uint8)(0xF8) || recBuffer[2] != (uint8)(0x11) || recBuffer[3] != (uint8)(0x2D))
    goto commandByteError;

  caliInfo->tempSlope = FPuint8ArrayToFPDouble(recBuffer + 8, 0);
  caliInfo->vref = FPuint8ArrayToFPDouble(recBuffer + 8, 8);
  caliInfo->vref15 = FPuint8ArrayToFPDouble(recBuffer + 8, 16);
  caliInfo->vreg = FPuint8ArrayToFPDouble(recBuffer + 8, 24);

  /* reading block 3 from memory */
  sendBuffer[7] = 3;    //Blocknum = 3
  extendedChecksum(sendBuffer, 8);

  sentRec = LJUSB_BulkWrite(hDevice, U3_PIPE_EP1_OUT, sendBuffer, 8);
  if(sentRec < 8)
  {
    if(sentRec == 0)
      goto writeError0;
    else
      goto writeError1;
  }

  sentRec = LJUSB_BulkRead(hDevice, U3_PIPE_EP1_IN, recBuffer, 40);
  if(sentRec < 40)
  {
    if(sentRec == 0)
      goto readError0;
    else
      goto readError1;
  }

  if(recBuffer[1] != (uint8)(0xF8) || recBuffer[2] != (uint8)(0x11) || recBuffer[3] != (uint8)(0x2D))
    goto commandByteError;

  for(i = 0; i < 4; i++)
    caliInfo->hvAINSlope[i] = FPuint8ArrayToFPDouble(recBuffer + 8, i*8);

  /* reading block 4 from memory */
  sendBuffer[7] = 4;    //Blocknum = 4
  extendedChecksum(sendBuffer, 8);

  sentRec = LJUSB_BulkWrite(hDevice, U3_PIPE_EP1_OUT, sendBuffer, 8);
  if(sentRec < 8)
  {
    if(sentRec == 0)
      goto writeError0;
    else
      goto writeError1;
  }

  sentRec = LJUSB_BulkRead(hDevice, U3_PIPE_EP1_IN, recBuffer, 40);
  if(sentRec < 40)
  {
    if(sentRec == 0)
      goto readError0;
    else
      goto readError1;
  }

  if(recBuffer[1] != (uint8)(0xF8) || recBuffer[2] != (uint8)(0x11) || recBuffer[3] != (uint8)(0x2D))
    goto commandByteError;

  for(i = 0; i < 4; i++)
    caliInfo->hvAINOffset[i] = FPuint8ArrayToFPDouble(recBuffer + 8, i*8);

  caliInfo->prodID = 3;

  return 0;

writeError0:
  printf("Error : getCalibrationInfo write failed\n");
  return -1;

writeError1:
  printf("Error : getCalibrationInfo did not write all of the buffer\n");
  return -1;

readError0:
  printf("Error : getCalibrationInfo read failed\n");
  return -1;

readError1:
  printf("Error : getCalibrationInfo did not read all of the buffer\n");
  return -1;

commandByteError:
  printf("Error : getCalibrationInfo received wrong command bytes for ReadMem\n");
  return -1;
}


long getLJTDACCalibrationInfo(HANDLE hDevice, u3LJTDACCalibrationInfo *caliInfo, uint8 DIOAPinNum) {
  int err;
  uint8 options, speedAdjust, sdaPinNum, sclPinNum, address, numByteToSend, numBytesToReceive, errorcode;
  uint8 bytesCommand[1];
  uint8 bytesResponse[32];
  uint8 ackArray[4];

  err = 0;

  //Setting up I2C command for LJTDAC
  options = 0;                //I2COptions : 0
  speedAdjust = 0;            //SpeedAdjust : 0 (for max communication speed of about 130 kHz)
  sdaPinNum = DIOAPinNum+1;   //SDAPinNum : FIO channel connected to pin DIOB
  sclPinNum = DIOAPinNum;     //SCLPinNum : FIO channel connected to pin DIOA
  address = (uint8)(0xA0);    //Address : h0xA0 is the address for EEPROM
  numByteToSend = 1;          //NumI2CByteToSend : 1 byte for the EEPROM address
  numBytesToReceive = 32;     //NumI2CBytesToReceive : getting 32 bytes starting at EEPROM address specified in I2CByte0

  bytesCommand[0] = 64;       //I2CByte0 : Memory Address (starting at address 64 (DACA Slope)

  //Performing I2C low-level call
  err = I2C(hDevice, options, speedAdjust, sdaPinNum, sclPinNum, address, numByteToSend, numBytesToReceive, bytesCommand, &errorcode, ackArray, bytesResponse);

  if(errorcode != 0)
  {
    printf("Getting LJTDAC calibration info error : received errorcode %d in response\n", errorcode);
    err = -1;
  }

  if(err == -1)
    return err;

  caliInfo->DACSlopeA = FPuint8ArrayToFPDouble(bytesResponse, 0);
  caliInfo->DACOffsetA = FPuint8ArrayToFPDouble(bytesResponse, 8);
  caliInfo->DACSlopeB = FPuint8ArrayToFPDouble(bytesResponse, 16);
  caliInfo->DACOffsetB = FPuint8ArrayToFPDouble(bytesResponse, 24);
  caliInfo->prodID = 3;

  return err;
}


double FPuint8ArrayToFPDouble(uint8 *buffer, int startIndex)
{
  uint32 resultDec = 0;
  uint32 resultWh = 0;
  int i;

  for(i = 0; i < 4; i++)
  {
    resultDec += (uint32)buffer[startIndex + i] * pow(2, i*8);
    resultWh += (uint32)buffer[startIndex + i + 4] * pow(2, i*8);
  }

  return ( (double)((int)resultWh) + (double)(resultDec)/4294967296.0 );
}


long binaryToCalibratedAnalogVoltage(u3CalibrationInfo *caliInfo, int dacEnabled, uint8 negChannel, uint16 bytesVoltage, double *analogVoltage)
{
  if(isCalibrationInfoValid(caliInfo) == -1)
    return -1;

  if(caliInfo->hardwareVersion >= 1.30)
  {
    if(caliInfo->highVoltage == 1)
    {
      printf("binaryToCalibratedAnalogVoltage error: cannot handle U3-HV device.  Please use binaryToCalibratedAnalogVoltage_hw130 function.\n");
      return -1;
    }
    else
      return binaryToCalibratedAnalogVoltage_hw130(caliInfo, 0, negChannel, bytesVoltage, analogVoltage);
  }

  if((negChannel >= 0 && negChannel <= 15) || negChannel == 30)
  {
    if(dacEnabled == 0)
      *analogVoltage = caliInfo->ainDiffSlope*((double)bytesVoltage) + caliInfo->ainDiffOffset;
    else
      *analogVoltage = (bytesVoltage/65536.0)*caliInfo->vreg*2.0 - caliInfo->vreg;
  }
  else if(negChannel == 31)
  {
    if(dacEnabled == 0)
      *analogVoltage = caliInfo->ainSESlope*((double)bytesVoltage) + caliInfo->ainSEOffset;
    else
      *analogVoltage = (bytesVoltage/65536.0)*caliInfo->vreg;
  }
  else
  {
    printf("binaryToCalibratedAnalogVoltage error: invalid negative channel.\n");
    return -1;
  }

  return 0;
}


long binaryToCalibratedAnalogVoltage_hw130(u3CalibrationInfo *caliInfo, uint8 positiveChannel, uint8 negChannel, uint16 bytesVoltage, double *analogVoltage)
{
  if(isCalibrationInfoValid(caliInfo) == -1)
    return -1;

  if(caliInfo->hardwareVersion < 1.30)
  {
    printf("binaryToCalibratedAnalogVoltage_hw130 error: cannot handle U3 hardware versions < 1.30 .  Please use binaryToCalibratedAnalogVoltage function.\n");
    return -1;
  }

  if((negChannel >= 0 && negChannel <= 15) || negChannel == 30)
  {
    if(caliInfo->highVoltage == 0
       || (caliInfo->highVoltage == 1 && positiveChannel >= 4 && negChannel >= 4))
      *analogVoltage = caliInfo->ainDiffSlope*((double)bytesVoltage) + caliInfo->ainDiffOffset;
    else if(caliInfo->hardwareVersion >= 1.30 && caliInfo->highVoltage == 1)
    {
      printf("binaryToCalibratedAnalogVoltage_hw130 error: invalid negative channel for U3-HV.\n");
      return -1;
    }
  }
  else if(negChannel == 31)
  {
    if(caliInfo->highVoltage == 1 && positiveChannel >= 0 && positiveChannel < 4)
      *analogVoltage = caliInfo->hvAINSlope[positiveChannel]*((double)bytesVoltage) + caliInfo->hvAINOffset[positiveChannel];
    else
      *analogVoltage = caliInfo->ainSESlope*((double)bytesVoltage) + caliInfo->ainSEOffset;
  }
  else
  {
    printf("binaryToCalibratedAnalogVoltage_hw130 error: invalid negative channel.\n");
    return -1;
  }

  return 0;
}


long analogToCalibratedBinaryVoltage(u3CalibrationInfo *caliInfo, int DACNumber, double analogVoltage, uint8 *bytesVoltage)
{
  return analogToCalibratedBinary8BitVoltage(caliInfo, DACNumber, analogVoltage, bytesVoltage);
}


long analogToCalibratedBinary8BitVoltage(u3CalibrationInfo *caliInfo, int DACNumber, double analogVoltage, uint8 *bytesVoltage8)
{
  double tempBytesVoltage;

  if(isCalibrationInfoValid(caliInfo) == -1)
    return -1;

  switch(DACNumber)
  {
    case 0:
      tempBytesVoltage = analogVoltage*caliInfo->dacSlope[0] + caliInfo->dacOffset[0];
      break;
    case 1:
      tempBytesVoltage = analogVoltage*caliInfo->dacSlope[1] + caliInfo->dacOffset[1];
      break;
    default:
      printf("analogToCalibratedBinaryVoltage error: invalid channelNumber.\n");
      return -1;
  }

  //Checking to make sure bytesVoltage will be a value between 0 and 255.  Too
  //high of an analogVoltage (about 4.95 and above volts) or too low (below 0
  //volts) will cause a value not between 0 and 255.
  if(tempBytesVoltage < 0)
    tempBytesVoltage = 0;
  else if(tempBytesVoltage > 255 && caliInfo->hardwareVersion < 1.30)
    tempBytesVoltage = 255;

  *bytesVoltage8 = (uint8)tempBytesVoltage;

  return 0;
}


long analogToCalibratedBinary16BitVoltage(u3CalibrationInfo *caliInfo, int DACNumber, double analogVoltage, uint16 *bytesVoltage16)
{
  double tempBytesVoltage;

  if(isCalibrationInfoValid(caliInfo) == -1)
    return -1;

  switch(DACNumber)
  {
    case 0:
      if(caliInfo->hardwareVersion < 1.30)
        tempBytesVoltage = analogVoltage*caliInfo->dacSlope[0] + caliInfo->dacOffset[0];
      else
        tempBytesVoltage = analogVoltage*caliInfo->dacSlope[0]*256 + caliInfo->dacOffset[0]*256;
      break;
    case 1:
      if(caliInfo->hardwareVersion < 1.30)
        tempBytesVoltage = analogVoltage*caliInfo->dacSlope[1] + caliInfo->dacOffset[1];
      else
        tempBytesVoltage = analogVoltage*caliInfo->dacSlope[1]*256 + caliInfo->dacOffset[1]*256;
      break;
    default:
      printf("analogToCalibratedBinaryVoltage error: invalid channelNumber.\n");
      return -1;
  }

  //Checking to make sure bytesVoltage will be a value between 0 and 255/65535.  Too
  //high of an analogVoltage (about 4.95 and above volts) or too low (below 0
  //volts) will cause a value not between 0 and 255/65535.
  if(tempBytesVoltage < 0)
    tempBytesVoltage = 0;
  if(tempBytesVoltage > 65535 && caliInfo->hardwareVersion >= 1.30)
    tempBytesVoltage = 65535;
  else if(tempBytesVoltage > 255 && caliInfo->hardwareVersion < 1.30)
    tempBytesVoltage = 255;

  *bytesVoltage16 = (uint16)tempBytesVoltage;

  return 0;
}


long LJTDACAnalogToCalibratedBinaryVoltage(u3LJTDACCalibrationInfo *caliInfo, int DACNumber, double analogVoltage, uint16 *bytesVoltage)
{
  double slope;
  double offset;
  double tempBytesVoltage;

  if(isLJTDACCalibrationInfoValid(caliInfo) == -1)
    return -1;

  switch(DACNumber)
  {
    case 0:
      slope = caliInfo->DACSlopeA;
      offset = caliInfo->DACOffsetA;
      break;
    case 1:
      slope = caliInfo->DACSlopeB;
      offset = caliInfo->DACOffsetB;
      break;
    default:
      printf("LJTDACAnalogToCalibratedBinaryVoltage error: invalid DACNumber.\n");
      return -1;
  }

  tempBytesVoltage = slope*analogVoltage + offset;

  //Checking to make sure bytesVoltage will be a value between 0 and 65535.  A
  //too high analogVoltage (above 10 volts) or too low analogVoltage (below
  //-10 volts) will create a value not between 0 and 65535.
  if(tempBytesVoltage < 0)
    tempBytesVoltage = 0;
  if(tempBytesVoltage > 65535)
    tempBytesVoltage = 65535;

  *bytesVoltage = (uint16)tempBytesVoltage; 

  return 0;
}


long binaryToCalibratedAnalogTemperature(u3CalibrationInfo *caliInfo, uint16 bytesTemperature, double *analogTemperature)
{
  if(isCalibrationInfoValid(caliInfo) == -1)
    return -1;

  *analogTemperature = caliInfo->tempSlope*((double)bytesTemperature);
  return 0;
}


long binaryToUncalibratedAnalogVoltage(int dac1Enabled, uint8 negChannel, uint16 bytesVoltage, double *analogVoltage)
{
  if( (negChannel >= 0 && negChannel <= 15) || negChannel == 30)
  {
    if(dac1Enabled == 0)
      *analogVoltage = (double)bytesVoltage*0.000074463 - 2.44;
    else
      *analogVoltage = (bytesVoltage/65536.0)*6.6 - 3.3;
  }
  else if(negChannel == 31)
  {
    if(dac1Enabled == 0)
      *analogVoltage = (double)bytesVoltage*0.000037231;
    else
      *analogVoltage = (bytesVoltage/65536.0)*3.3;
  }
  else
  {
    printf("binaryToCalibratedAnalogVoltage error: invalid negative channel.\n");
    return -1;
  }

  return 0;
}


long binaryToUncalibratedAnalogVoltage_hw130(int highVoltage, uint8 positiveChannel, uint8 negChannel, uint16 bytesVoltage, double *analogVoltage)
{
  if( (negChannel >= 0 && negChannel <= 15) || negChannel == 30)
  {
    if(highVoltage == 0
       || (highVoltage == 1 && positiveChannel >= 4 && negChannel >= 4))
      *analogVoltage = (double)bytesVoltage*0.000074463 - 2.44;
    else if(highVoltage == 1)
    {
      printf("binaryToCalibratedAnalogVoltage_hw130 error: invalid negative channel for U3-HV.\n");
      return -1;
    }
  }
  else if(negChannel == 31)
  {
    if(highVoltage == 1)
      *analogVoltage = (double)bytesVoltage*0.000314 - 10.3;
    else
      *analogVoltage = (double)bytesVoltage*0.000037231;
  }
  else
  {
    printf("binaryToCalibratedAnalogVoltage_hw130 error: invalid negative channel.\n");
    return -1;
  }

  return 0;
}


long analogToUncalibratedBinaryVoltage(double analogVoltage, uint8 *bytesVoltage)
{
  return analogToUncalibratedBinary8BitVoltage(analogVoltage, bytesVoltage);
}


long analogToUncalibratedBinary8BitVoltage(double analogVoltage, uint8 *bytesVoltage8)
{
  double tempBytesVoltage;

  tempBytesVoltage = analogVoltage*51.717;

  //Checking to make sure bytesVoltage will be a value between 0 and 255.  Too
  //high of an analogVoltage (about 4.95 and above volts) or too low (below 0
  //volts) will cause a value not between 0 and 255.
  if(tempBytesVoltage < 0)
    tempBytesVoltage = 0;
  else if(tempBytesVoltage > 255)
    tempBytesVoltage = 255;

  *bytesVoltage8 = (uint8)tempBytesVoltage;

  return 0;
}


long analogToUncalibratedBinary16BitVoltage(double analogVoltage, uint16 *bytesVoltage16)
{
  double tempBytesVoltage;

  tempBytesVoltage = analogVoltage*51.717*256;

  //Checking to make sure bytesVoltage will be a value between 0 and 65535.  Too
  //high of an analogVoltage (about 4.95 and above volts) or too low (below 0
  //volts) will cause a value not between 0 and 65535.
  if(tempBytesVoltage < 0)
    tempBytesVoltage = 0;
  if(tempBytesVoltage > 65535)
    tempBytesVoltage = 65535;

  *bytesVoltage16 = (uint16)tempBytesVoltage;

  return 0;
}


long binaryToUncalibratedAnalogTemperature(uint16 bytesTemperature, double *analogTemperature)
{
  *analogTemperature = (double)bytesTemperature*0.013021;
  return 0;
}


long I2C(HANDLE hDevice, uint8 I2COptions, uint8 SpeedAdjust, uint8 SDAPinNum, uint8 SCLPinNum, uint8 Address, uint8 NumI2CBytesToSend, uint8 NumI2CBytesToReceive, uint8 *I2CBytesCommand, uint8 *Errorcode, uint8 *AckArray, uint8 *I2CBytesResponse) {
  uint8 *sendBuff, *recBuff;
  int sendChars, recChars, sendSize, recSize, i, ret;
  uint16 checksumTotal = 0;
  uint32 ackArrayTotal, expectedAckArray;

  *Errorcode = 0;
  ret = 0;
  sendSize = 6 + 8 + ((NumI2CBytesToSend%2 != 0)?(NumI2CBytesToSend + 1):(NumI2CBytesToSend));
  recSize = 6 + 6 + ((NumI2CBytesToReceive%2 != 0)?(NumI2CBytesToReceive + 1):(NumI2CBytesToReceive));

  sendBuff = malloc(sizeof(uint8)*sendSize);
  recBuff = malloc(sizeof(uint8)*recSize);

  sendBuff[sendSize - 1] = 0;

  //I2C command
  sendBuff[1] = (uint8)(0xF8);          //command byte
  sendBuff[2] = (sendSize - 6)/2;       //number of data words = 4 + NumI2CBytesToSend
  sendBuff[3] = (uint8)(0x3B);          //extended command number

  sendBuff[6] = I2COptions;             //I2COptions
  sendBuff[7] = SpeedAdjust;            //SpeedAdjust
  sendBuff[8] = SDAPinNum;              //SDAPinNum
  sendBuff[9] = SCLPinNum;              //SCLPinNum
  sendBuff[10] = Address;               //Address
  sendBuff[11] = 0;                     //Reserved
  sendBuff[12] = NumI2CBytesToSend;     //NumI2CByteToSend
  sendBuff[13] = NumI2CBytesToReceive;  //NumI2CBytesToReceive

  for(i = 0; i < NumI2CBytesToSend; i++)
    sendBuff[14 + i] = I2CBytesCommand[i];  //I2CByte

  extendedChecksum(sendBuff, sendSize);

  //Sending command to U3
  sendChars = LJUSB_BulkWrite(hDevice, U3_PIPE_EP1_OUT, sendBuff, sendSize);
  if(sendChars < sendSize)
  {
    if(sendChars == 0)
      printf("I2C Error : write failed\n");
    else
      printf("I2C Error : did not write all of the buffer\n");
    ret = -1;
    goto cleanmem;
  }

  //Reading response from U3
  recChars = LJUSB_BulkRead(hDevice, U3_PIPE_EP1_IN, recBuff, recSize);
  if(recChars < recSize)
  {
    if(recChars == 0)
      printf("I2C Error : read failed\n");
    else
    {
      printf("I2C Error : did not read all of the buffer\n");
      if(recChars >= 12)
        *Errorcode = recBuff[6];
    }
    ret = -1;
    goto cleanmem;
  }

  *Errorcode = recBuff[6];

  AckArray[0] = recBuff[8];
  AckArray[1] = recBuff[9];
  AckArray[2] = recBuff[10];
  AckArray[3] = recBuff[11];

  for(i = 0; i < NumI2CBytesToReceive; i++)
    I2CBytesResponse[i] = recBuff[12 + i];

  if((uint8)(extendedChecksum8(recBuff)) != recBuff[0])
  {
    printf("I2C Error : read buffer has bad checksum (%d)\n", recBuff[0]);
    ret = -1;
  }

  if(recBuff[1] != (uint8)(0xF8))
  {
    printf("I2C Error : read buffer has incorrect command byte (%d)\n", recBuff[1]);
    ret = -1;
  }

  if(recBuff[2] != (uint8)((recSize - 6)/2))
  {
    printf("I2C Error : read buffer has incorrect number of data words (%d)\n", recBuff[2]);
    ret = -1;
  }

  if(recBuff[3] != (uint8)(0x3B))
  {
    printf("I2C Error : read buffer has incorrect extended command number (%d)\n", recBuff[3]);
    ret = -1;
  }

  checksumTotal = extendedChecksum16(recBuff, recSize);
  if( (uint8)((checksumTotal / 256) & 0xff) != recBuff[5] || (uint8)(checksumTotal & 255) != recBuff[4])
  {
    printf("I2C error : read buffer has bad checksum16 (%d)\n", checksumTotal);
    ret = -1;
  }

  //ackArray should ack the Address byte in the first ack bit, but did not until firmware 1.44
  ackArrayTotal = AckArray[0] + AckArray[1]*256 + AckArray[2]*65536 + AckArray[3]*16777216;
  expectedAckArray = pow(2.0,  NumI2CBytesToSend+1)-1;
  if(ackArrayTotal != expectedAckArray)
    printf("I2C error : expected an ack of %d, but received %d\n", expectedAckArray, ackArrayTotal);

cleanmem:
  free(sendBuff);
  free(recBuff);
  sendBuff = NULL;
  recBuff = NULL;

  return ret;
}


long eAIN(HANDLE Handle, u3CalibrationInfo *CalibrationInfo, long ConfigIO, long *DAC1Enable, long ChannelP, long ChannelN, double *Voltage, long Range, long Resolution, long Settling, long Binary, long Reserved1, long Reserved2)
{
  uint8 sendDataBuff[3], recDataBuff[2];
  uint8 FIOAnalog, EIOAnalog, curFIOAnalog, curEIOAnalog, curTCConfig;
  uint8 settling, quicksample, Errorcode, ErrorFrame;
  uint8 outDAC1Enable;
  uint16 bytesVT;
  long error;
  double hwver;
  int hv;

  if(isCalibrationInfoValid(CalibrationInfo) != 1)
  {
    printf("eAIN error: calibration information is required");
    return -1;
  }

  hwver = CalibrationInfo->hardwareVersion;
  hv = CalibrationInfo->highVoltage;

  if(ChannelP < 0 || (ChannelP > 15 && ChannelP != 30 && ChannelP != 31))
  {
    printf("eAIN error: Invalid positive channel\n");
    return -1;
  }

  if(ChannelN < 0 || (ChannelN > 15 && ChannelN != 30 && ChannelN != 31)
     || (hwver >= 1.30 && hv == 1 && ((ChannelP < 4 && ChannelN != 31)
                                       || ChannelN < 4)))
  {
    printf("eAIN error: Invalid negative channel\n");
    return -1;
  }

  if(hwver >= 1.30 && hv == 1 && ChannelP < 4) {}
  else if(ConfigIO != 0)
  {
    FIOAnalog = 0;
    EIOAnalog = 0;

    //Setting ChannelP and ChannelN channels to analog using
    //FIOAnalog and EIOAnalog
    if(ChannelP <= 7)
      FIOAnalog = pow(2, ChannelP);
    else if(ChannelP <= 15)
      EIOAnalog = pow(2, (ChannelP - 8));

    if(ChannelN <= 7)
      FIOAnalog = FIOAnalog | (int)pow(2, ChannelN);
    else if(ChannelN <= 15)
      EIOAnalog = EIOAnalog | (int)pow(2, (ChannelN - 8));

    //Using ConfigIO to get current FIOAnalog and EIOAnalog settings
    if((error = ehConfigIO(Handle, 0, 0, 0, 0, 0, &curTCConfig, &outDAC1Enable, &curFIOAnalog, &curEIOAnalog)) != 0)
      return error;

    *DAC1Enable = outDAC1Enable;

    if( !(FIOAnalog == curFIOAnalog && EIOAnalog == curEIOAnalog) )
    {
      //Creating new FIOAnalog and EIOAnalog settings
      FIOAnalog = FIOAnalog | curFIOAnalog;
      EIOAnalog = EIOAnalog | curEIOAnalog;

      //Using ConfigIO to set new FIOAnalog and EIOAnalog settings
      if((error = ehConfigIO(Handle, 12, curTCConfig, 0, FIOAnalog, EIOAnalog, NULL, NULL, &curFIOAnalog, &curEIOAnalog)) != 0)
        return error;
    }
  }

  /* Setting up Feedback command to read analog input */
  sendDataBuff[0] = 1;    //IOType is AIN

  settling = (Settling != 0) ? 1 : 0;
  quicksample = (Resolution != 0) ? 1 : 0;
  sendDataBuff[1] = (uint8)ChannelP + settling*64 + quicksample*128;  //Positive channel (bits 0-4), LongSettling (bit 6)
                                                                      //QuickSample (bit 7)
  sendDataBuff[2] = (uint8)ChannelN;   //Negative channel

  if(ehFeedback(Handle, sendDataBuff, 3, &Errorcode, &ErrorFrame, recDataBuff, 2) < 0)
    return -1;
  if(Errorcode)
    return (long)Errorcode;

  bytesVT = recDataBuff[0] + recDataBuff[1]*256;

  if(Binary != 0)
  {
    *Voltage = (double)bytesVT;
  }
  else
  {
    if(ChannelP == 30)
    {
      if(binaryToCalibratedAnalogTemperature(CalibrationInfo, bytesVT, Voltage) < 0)
        return -1;
    }
    else
    {
      if(hwver < 1.30)
        error = binaryToCalibratedAnalogVoltage(CalibrationInfo, (int)(*DAC1Enable), ChannelN, bytesVT, Voltage);
      else
        error = binaryToCalibratedAnalogVoltage_hw130(CalibrationInfo, ChannelP, ChannelN, bytesVT, Voltage);
      if(error < 0)
        return -1;
    }
  }

  return 0;
}


long eDAC(HANDLE Handle, u3CalibrationInfo *CalibrationInfo, long ConfigIO, long Channel, double Voltage, long Binary, long Reserved1, long Reserved2)
{
  uint8 sendDataBuff[3];
  uint8 byteV, DAC1Enabled, Errorcode, ErrorFrame;
  uint16 bytesV;
  long error, sendSize;

  if(isCalibrationInfoValid(CalibrationInfo) != 1)
  {
    printf("eDAC error: calibration information is required");
    return -1;
  }

  if(Channel < 0 || Channel > 1)
  {
    printf("eDAC error: Invalid DAC channel\n");
    return -1;
  }

  if(ConfigIO != 0 && Channel == 1 && CalibrationInfo->hardwareVersion < 1.30)
  {
   //Using ConfigIO to enable DAC1
    error = ehConfigIO(Handle, 2, 0, 1, 0, 0, NULL, &DAC1Enabled, NULL, NULL);
    if(error != 0)
      return error;
  }

  /* Setting up Feedback command to set DAC */
  if(CalibrationInfo->hardwareVersion < 1.30)
  {
    sendSize = 2;

    sendDataBuff[0] = 34 + Channel;  //IOType is DAC0/1 (8 bit)

    if(analogToCalibratedBinary8BitVoltage(CalibrationInfo, (int)Channel, Voltage, &byteV) < 0)
      return -1;

    sendDataBuff[1] = byteV;      //Value
  }
  else
  {
    sendSize = 3;

    sendDataBuff[0] = 38 + Channel;  //IOType is DAC0/1 (16 bit)

    if(analogToCalibratedBinary16BitVoltage(CalibrationInfo, (int)Channel, Voltage, &bytesV) < 0)
      return -1;

    sendDataBuff[1] = (uint8)(bytesV&255);          //Value LSB
    sendDataBuff[2] = (uint8)((bytesV&65280)/256);  //Value MSB
  }

  if(ehFeedback(Handle, sendDataBuff, sendSize, &Errorcode, &ErrorFrame, NULL, 0) < 0)
    return -1;
  if(Errorcode)
    return (long)Errorcode;

  return 0;
}


long eDI(HANDLE Handle, long ConfigIO, long Channel, long *State)
{
  uint8 sendDataBuff[4], recDataBuff[1];
  uint8 Errorcode, ErrorFrame;
  uint8 FIOAnalog, EIOAnalog, curFIOAnalog, curEIOAnalog, curTCConfig;
  long error;


  if(Channel < 0 || Channel > 19)
  {
    printf("eDI error: Invalid DI channel\n");
    return -1;
  }

  if(ConfigIO != 0 && Channel <= 15)
  {
    FIOAnalog = 255;
    EIOAnalog = 255;

    //Setting Channel to digital using FIOAnalog and EIOAnalog
    if(Channel <= 7)
      FIOAnalog = 255 - pow(2, Channel);
    else
      EIOAnalog = 255 - pow(2, (Channel - 8));

    //Using ConfigIO to get current FIOAnalog and EIOAnalog settings
    error = ehConfigIO(Handle, 0, 0, 0, 0, 0, &curTCConfig, NULL, &curFIOAnalog, &curEIOAnalog);
    if(error != 0)
      return error;

    if( !(FIOAnalog == curFIOAnalog && EIOAnalog == curEIOAnalog))
    {
      //Creating new FIOAnalog and EIOAnalog settings
      FIOAnalog = FIOAnalog & curFIOAnalog;
      EIOAnalog = EIOAnalog & curEIOAnalog;

      //Using ConfigIO to set new FIOAnalog and EIOAnalog settings
      error = ehConfigIO(Handle, 12, curTCConfig, 0, FIOAnalog, EIOAnalog, NULL, NULL, &curFIOAnalog, &curEIOAnalog);
      if(error != 0)
        return error;
    }
  }

  /* Setting up Feedback command to set digital Channel to input and to read from it */
  sendDataBuff[0] = 13;       //IOType is BitDirWrite
  sendDataBuff[1] = Channel;  //IONumber(bits 0-4) + Direction (bit 7)

  sendDataBuff[2] = 10;       //IOType is BitStateRead
  sendDataBuff[3] = Channel;  //IONumber

  if(ehFeedback(Handle, sendDataBuff, 4, &Errorcode, &ErrorFrame, recDataBuff, 1) < 0)
    return -1;
  if(Errorcode)
    return (long)Errorcode;

  *State = recDataBuff[0];
  return 0;
}


long eDO(HANDLE Handle, long ConfigIO, long Channel, long State)
{
  uint8 sendDataBuff[4];
  uint8 Errorcode, ErrorFrame, FIOAnalog, EIOAnalog;
  uint8 curFIOAnalog, curEIOAnalog, curTCConfig;
  long error;


  if(Channel < 0 || Channel > 19)
  {
    printf("eD0 error: Invalid DI channel\n");
    return -1;
  }

  if(ConfigIO != 0 && Channel <= 15)
  {
    FIOAnalog = 255;
    EIOAnalog = 255;

    //Setting Channel to digital using FIOAnalog and EIOAnalog
    if(Channel <= 7)
      FIOAnalog = 255 - pow(2, Channel);
    else
      EIOAnalog = 255 - pow(2, (Channel - 8));

    //Using ConfigIO to get current FIOAnalog and EIOAnalog settings
    error = ehConfigIO(Handle, 0, 0, 0, 0, 0, &curTCConfig, NULL, &curFIOAnalog, &curEIOAnalog);
    if(error != 0)
      return error;

    if( !(FIOAnalog == curFIOAnalog && EIOAnalog == curEIOAnalog))
    {
      //Using ConfigIO to get current FIOAnalog and EIOAnalog settings
      FIOAnalog = FIOAnalog & curFIOAnalog;
      EIOAnalog = EIOAnalog & curEIOAnalog;

      //Using ConfigIO to set new FIOAnalog and EIOAnalog settings
      error = ehConfigIO(Handle, 12, curTCConfig, 0, FIOAnalog, EIOAnalog, NULL, NULL, &curFIOAnalog, &curEIOAnalog);
      if(error != 0)
        return error;
    }
  }

  /* Setting up Feedback command to set digital Channel to output and to set the state */
  sendDataBuff[0] = 13;             //IOType is BitDirWrite
  sendDataBuff[1] = Channel + 128;  //IONumber(bits 0-4) + Direction (bit 7)

  sendDataBuff[2] = 11;             //IOType is BitStateWrite
  sendDataBuff[3] = Channel + 128*((State > 0) ? 1 : 0);  //IONumber(bits 0-4) + State (bit 7)

  if(ehFeedback(Handle, sendDataBuff, 4, &Errorcode, &ErrorFrame, NULL, 0) < 0)
    return -1;
  if(Errorcode)
    return (long)Errorcode;

  return 0;
}


long eTCConfig(HANDLE Handle, long *aEnableTimers, long *aEnableCounters, long TCPinOffset, long TimerClockBaseIndex, long TimerClockDivisor, long *aTimerModes, double *aTimerValues, long Reserved1, long Reserved2)
{
  uint8 sendDataBuff[8];
  uint8 FIOAnalog, EIOAnalog, curFIOAnalog, curEIOAnalog;
  uint8 TimerCounterConfig, curTimerCounterConfig, Errorcode, ErrorFrame;
  int sendDataBuffSize, numTimers, numCounters, i;
  long error;

  if(TCPinOffset < 0 && TCPinOffset > 8)
  {
    printf("eTCConfig error: Invalid TimerCounterPinOffset\n");
    return -1;
  }

  /* ConfigTimerClock */
  if(TimerClockBaseIndex == LJ_tc2MHZ || TimerClockBaseIndex ==  LJ_tc6MHZ || TimerClockBaseIndex == LJ_tc24MHZ ||
     TimerClockBaseIndex == LJ_tc500KHZ_DIV || TimerClockBaseIndex == LJ_tc2MHZ_DIV || TimerClockBaseIndex == LJ_tc6MHZ_DIV ||
     TimerClockBaseIndex == LJ_tc24MHZ_DIV)
    TimerClockBaseIndex = TimerClockBaseIndex - 10;
  else if(TimerClockBaseIndex == LJ_tc4MHZ || TimerClockBaseIndex ==  LJ_tc12MHZ || TimerClockBaseIndex == LJ_tc48MHZ ||
     TimerClockBaseIndex == LJ_tc1MHZ_DIV || TimerClockBaseIndex == LJ_tc4MHZ_DIV || TimerClockBaseIndex == LJ_tc12MHZ_DIV ||
     TimerClockBaseIndex == LJ_tc48MHZ_DIV)
    TimerClockBaseIndex = TimerClockBaseIndex - 20;

  error = ehConfigTimerClock(Handle, (uint8)(TimerClockBaseIndex + 128), (uint8)TimerClockDivisor, NULL, NULL);
  if(error != 0)
    return error;

  //Using ConfigIO to get current FIOAnalog and curEIOAnalog settings
  error = ehConfigIO(Handle, 0, 0, 0, 0, 0, NULL, NULL, &curFIOAnalog, &curEIOAnalog);
  if(error != 0)
    return error;

  numTimers = 0;
  numCounters = 0;
  TimerCounterConfig = 0;
  FIOAnalog = 255;
  EIOAnalog = 255;

  for(i = 0; i < 2; i++)
  {
    if(aEnableTimers[i] != 0)
      numTimers++;
    else
      i = 999;
  }

  for(i = 0; i < 2; i++)
  {
    if(aEnableCounters[i] != 0)
    {
      numCounters++;
      TimerCounterConfig += pow(2, (i+2));
    }
  }

  TimerCounterConfig += numTimers + TCPinOffset*16;

  for(i = 0; i < numCounters + numTimers; i++)
  {
    if(i + TCPinOffset < 8)
      FIOAnalog = FIOAnalog - pow(2, i + TCPinOffset);
    else
      EIOAnalog = EIOAnalog - pow(2, (i + TCPinOffset - 8));
  }

  FIOAnalog = FIOAnalog & curFIOAnalog;
  EIOAnalog = EIOAnalog & curEIOAnalog;
  error = ehConfigIO(Handle, 13, TimerCounterConfig, 0, FIOAnalog, EIOAnalog, &curTimerCounterConfig, NULL, &curFIOAnalog, &curEIOAnalog);
  if(error != 0)
    return error;

  if(numTimers > 0)
  {
    /* Feedback */
    for(i = 0; i < 8; i++)
      sendDataBuff[i] = 0;

    for(i = 0; i < numTimers; i++)
    {
      sendDataBuff[i*4] = 43 + i*2;                                         //TimerConfig
      sendDataBuff[1 + i*4] = (uint8)aTimerModes[i];                        //TimerMode
      sendDataBuff[2 + i*4] = (uint8)(((long)aTimerValues[i])&0x00ff);        //Value LSB
      sendDataBuff[3 + i*4] = (uint8)((((long)aTimerValues[i])&0xff00)/256);  //Value MSB
    }

    sendDataBuffSize = 4*numTimers;

    if(ehFeedback(Handle, sendDataBuff, sendDataBuffSize, &Errorcode, &ErrorFrame, NULL, 0) < 0)
      return -1;
    if(Errorcode)
      return (long)Errorcode;
  }

  return 0;
}


long eTCValues(HANDLE Handle, long *aReadTimers, long *aUpdateResetTimers, long *aReadCounters, long *aResetCounters, double *aTimerValues, double *aCounterValues, long Reserved1, long Reserved2)
{
  uint8 sendDataBuff[12], recDataBuff[16];
  uint8 Errorcode, ErrorFrame;
  int sendDataBuffSize, recDataBuffSize, i, j;
  int numTimers, dataCountCounter, dataCountTimer;
 

  /* Feedback */
  numTimers = 0;
  dataCountCounter = 0;
  dataCountTimer = 0;
  sendDataBuffSize = 0;
  recDataBuffSize = 0;

  for(i = 0; i < 2; i++)
  {
    if(aReadTimers[i] != 0 || aUpdateResetTimers[i] != 0)
    {
      sendDataBuff[sendDataBuffSize] = 42 + i*2;                                          //Timer
      sendDataBuff[1 + sendDataBuffSize] = ((aUpdateResetTimers[i] != 0) ? 1 : 0);        //UpdateReset
      sendDataBuff[2 + sendDataBuffSize] = (uint8)(((long)aTimerValues[i])&0x00ff);       //Value LSB
      sendDataBuff[3 + sendDataBuffSize] = (uint8)((((long)aTimerValues[i])&0xff00)/256); //Value MSB
      sendDataBuffSize += 4;
      recDataBuffSize += 4;
      numTimers++;
    }
  }

  for(i = 0; i < 2; i++)
  {
    if(aReadCounters[i] != 0 || aResetCounters[i] != 0)
    {
      sendDataBuff[sendDataBuffSize] = 54 + i;                                 //Counter
      sendDataBuff[1 + sendDataBuffSize] = ((aResetCounters[i] != 0) ? 1 : 0); //Reset
      sendDataBuffSize += 2;
      recDataBuffSize += 4;
    }
  }

  if(ehFeedback(Handle, sendDataBuff, sendDataBuffSize, &Errorcode, &ErrorFrame, recDataBuff, recDataBuffSize) < 0)
    return -1;
  if(Errorcode)
    return (long)Errorcode;

  for(i = 0; i < 2; i++)
  {
    aTimerValues[i] = 0;
    if(aReadTimers[i] != 0)
    {
      for(j = 0; j < 4; j++)
        aTimerValues[i] += (double)((long)recDataBuff[j + dataCountTimer*4]*pow(2, 8*j));
    }
    if(aReadTimers[i] != 0 || aUpdateResetTimers[i] != 0)
      dataCountTimer++;

    aCounterValues[i] = 0;
    if(aReadCounters[i] != 0)
    {
      for(j = 0; j < 4; j++)
        aCounterValues[i] += (double)((long)recDataBuff[j + numTimers*4 + dataCountCounter*4]*pow(2, 8*j));
    }
    if(aReadCounters[i] != 0 || aResetCounters[i] != 0)
      dataCountCounter++;
  }

  return 0;
}


long ehConfigIO(HANDLE hDevice, uint8 inWriteMask, uint8 inTimerCounterConfig, uint8 inDAC1Enable, uint8 inFIOAnalog, uint8 inEIOAnalog, uint8 *outTimerCounterConfig, uint8 *outDAC1Enable, uint8 *outFIOAnalog, uint8 *outEIOAnalog)
{
  uint8 sendBuff[12], recBuff[12];
  uint16 checksumTotal;
  int sendChars, recChars;

  sendBuff[1] = (uint8)(0xF8);  //Command byte
  sendBuff[2] = (uint8)(0x03);  //Number of data words
  sendBuff[3] = (uint8)(0x0B);  //Extended command number

  sendBuff[6] = inWriteMask;  //Writemask

  sendBuff[7] = 0;            //Reserved
  sendBuff[8] = inTimerCounterConfig;  //TimerCounterConfig
  sendBuff[9] = inDAC1Enable; //DAC1 enable : not enabling
  sendBuff[10] = inFIOAnalog; //FIOAnalog
  sendBuff[11] = inEIOAnalog; //EIOAnalog
  extendedChecksum(sendBuff, 12);

  //Sending command to U3
  if( (sendChars = LJUSB_BulkWrite(hDevice, U3_PIPE_EP1_OUT, sendBuff, 12)) < 12)
  {
    if(sendChars == 0)
      printf("ehConfigIO error : write failed\n");
    else
      printf("ehConfigIO error : did not write all of the buffer\n");
    return -1;
  }

  //Reading response from U3
  if( (recChars = LJUSB_BulkRead(hDevice, U3_PIPE_EP1_IN, recBuff, 12)) < 12)
  {
    if(recChars == 0)
      printf("ehConfigIO error : read failed\n");
    else
      printf("ehConfigIO error : did not read all of the buffer\n");
    return -1;
  }

  checksumTotal = extendedChecksum16(recBuff, 12);
  if( (uint8)((checksumTotal / 256 ) & 0xff) != recBuff[5])
  {
    printf("ehConfigIO error : read buffer has bad checksum16(MSB)\n");
    return -1;
  }

  if( (uint8)(checksumTotal & 0xff) != recBuff[4])
  {
    printf("ehConfigIO error : read buffer has bad checksum16(LBS)\n");
    return -1;
  }

  if( extendedChecksum8(recBuff) != recBuff[0])
  {
    printf("ehConfigIO error : read buffer has bad checksum8\n");
    return -1;
  }

  if( recBuff[1] != (uint8)(0xF8) || recBuff[2] != (uint8)(0x03) || recBuff[3] != (uint8)(0x0B) )
  {
    printf("ehConfigIO error : read buffer has wrong command bytes\n");
    return -1;
  }

  if( recBuff[6] != 0)
  {
    printf("ehConfigIO error : read buffer received errorcode %d\n", recBuff[6]);
    return (int)recBuff[6];
  }

  if(outTimerCounterConfig != NULL)
    *outTimerCounterConfig = recBuff[8];
  if(outDAC1Enable != NULL)
    *outDAC1Enable = recBuff[9];
  if(outFIOAnalog != NULL)
    *outFIOAnalog = recBuff[10];
  if(outEIOAnalog != NULL)
    *outEIOAnalog = recBuff[11];

  return 0;
}


long ehConfigTimerClock(HANDLE hDevice, uint8 inTimerClockConfig, uint8 inTimerClockDivisor, uint8 *outTimerClockConfig, uint8 *outTimerClockDivisor)
{
  uint8 sendBuff[10], recBuff[10];
  uint16 checksumTotal;
  int sendChars, recChars;

  sendBuff[1] = (uint8)(0xF8);  //Command byte
  sendBuff[2] = (uint8)(0x02);  //Number of data words
  sendBuff[3] = (uint8)(0x0A);  //Extended command number

  sendBuff[6] = 0;   //Reserved
  sendBuff[7] = 0;   //Reserved

  sendBuff[8] = inTimerClockConfig;  //TimerClockConfig
  sendBuff[9] = inTimerClockDivisor;    //TimerClockDivisor
  extendedChecksum(sendBuff, 10);

  //Sending command to U3
  if( (sendChars = LJUSB_BulkWrite(hDevice, U3_PIPE_EP1_OUT, sendBuff, 10)) < 10)
  {
    if(sendChars == 0)
      printf("ehConfigTimerClock error : write failed\n");
    else
      printf("ehConfigTimerClock error : did not write all of the buffer\n");
    return -1;
  }

  //Reading response from U3
  if( (recChars = LJUSB_BulkRead(hDevice, U3_PIPE_EP1_IN, recBuff, 10)) < 10)
  {
    if(recChars == 0)
      printf("ehConfigTimerClock error : read failed\n");
    else
      printf("ehConfigTimerClock error : did not read all of the buffer\n");
    return -1;
  }

  checksumTotal = extendedChecksum16(recBuff, 10);
  if( (uint8)((checksumTotal / 256 ) & 0xff) != recBuff[5])
  {
    printf("ehConfigTimerClock error : read buffer has bad checksum16(MSB)\n");
    return -1;
  }

  if( (uint8)(checksumTotal & 0xff) != recBuff[4])
  {
    printf("ehConfigTimerClock error : read buffer has bad checksum16(LBS)\n");
    return -1;
  }

  if( extendedChecksum8(recBuff) != recBuff[0])
  {
    printf("ehConfigTimerClock error : read buffer has bad checksum8\n");
    return -1;
  }

  if( recBuff[1] != (uint8)(0xF8) || recBuff[2] != (uint8)(0x02) || recBuff[3] != (uint8)(0x0A) )
  {
    printf("ehConfigTimerClock error : read buffer has wrong command bytes\n");
    return -1;
  }

  if(outTimerClockConfig != NULL)
    *outTimerClockConfig = recBuff[8];

  if(outTimerClockDivisor != NULL)
    *outTimerClockDivisor = recBuff[9];

  if( recBuff[6] != 0)
  {
    printf("ehConfigTimerClock error : read buffer received errorcode %d\n", recBuff[6]);
    return recBuff[6];
  }

  return 0;
}


long ehFeedback(HANDLE hDevice, uint8 *inIOTypesDataBuff, long inIOTypesDataSize, uint8 *outErrorcode, uint8 *outErrorFrame, uint8 *outDataBuff, long outDataSize)
{
  uint8 *sendBuff, *recBuff;
  uint16 checksumTotal;
  int sendChars, recChars, i, sendDWSize, recDWSize, commandBytes, ret;

  ret = 0;
  commandBytes = 6;

  if(((sendDWSize = inIOTypesDataSize + 1)%2) != 0)
    sendDWSize++;
  if(((recDWSize = outDataSize + 3)%2) != 0)
    recDWSize++;

  sendBuff = malloc(sizeof(uint8)*(commandBytes + sendDWSize));
  recBuff = malloc(sizeof(uint8)*(commandBytes + recDWSize));
  if(sendBuff == NULL || recBuff == NULL) {
    ret = -1;
    goto cleanmem;
  }

  sendBuff[sendDWSize + commandBytes - 1] = 0;

  /* Setting up Feedback command */
  sendBuff[1] = (uint8)(0xF8);  //Command byte
  sendBuff[2] = sendDWSize/2;   //Number of data words (.5 word for echo, 1.5
                                //words for IOTypes)
  sendBuff[3] = (uint8)(0x00);  //Extended command number

  sendBuff[6] = 0;    //Echo

  for(i = 0; i < inIOTypesDataSize; i++)
    sendBuff[i+commandBytes+1] = inIOTypesDataBuff[i];

  extendedChecksum(sendBuff, (sendDWSize+commandBytes));

  //Sending command to U3
  if( (sendChars = LJUSB_BulkWrite(hDevice, U3_PIPE_EP1_OUT, sendBuff, (sendDWSize+commandBytes))) < sendDWSize+commandBytes)
  {
    if(sendChars == 0)
      printf("ehFeedback error : write failed\n");
    else
      printf("ehFeedback error : did not write all of the buffer\n");
    ret = -1;
    goto cleanmem;
  }

  //Reading response from U3
  if( (recChars = LJUSB_BulkRead(hDevice, U3_PIPE_EP1_IN, recBuff, (commandBytes+recDWSize))) < commandBytes+recDWSize)
  {
    if(recChars == -1)
    {
      printf("ehFeedback error : read failed\n");
      ret = -1;
      goto cleanmem;
    }
    else if(recChars < 8)
    {
      printf("ehFeedback error : response buffer is too small\n");
      ret = -1;
      goto cleanmem;
    }
    else
      printf("ehFeedback error : did not read all of the expected buffer (received %d, expected %d )\n", recChars, commandBytes+recDWSize);
  }

  checksumTotal = extendedChecksum16(recBuff, recChars);
  if( (uint8)((checksumTotal / 256 ) & 0xff) != recBuff[5])
  {
    printf("ehFeedback error : read buffer has bad checksum16(MSB)\n");
    ret = -1;
    goto cleanmem;
  }

  if( (uint8)(checksumTotal & 0xff) != recBuff[4])
  {
    printf("ehFeedback error : read buffer has bad checksum16(LBS)\n");
    ret = -1;
    goto cleanmem;
  }

  if( extendedChecksum8(recBuff) != recBuff[0])
  {
    printf("ehFeedback error : read buffer has bad checksum8\n");
    ret = -1;
    goto cleanmem;
  }

  if( recBuff[1] != (uint8)(0xF8) || recBuff[3] != (uint8)(0x00) )
  {
    printf("ehFeedback error : read buffer has wrong command bytes \n");
    ret = -1;
    goto cleanmem;
  }

  *outErrorcode = recBuff[6];
  *outErrorFrame = recBuff[7];

  for(i = 0; i+commandBytes+3 < recChars && i < outDataSize; i++)
    outDataBuff[i] = recBuff[i+commandBytes+3];

cleanmem:
  free(sendBuff);
  free(recBuff);
  sendBuff = NULL;
  recBuff = NULL;

  return ret;
}
