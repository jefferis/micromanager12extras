///////////////////////////////////////////////////////////////////////////////
// FILE:          LabJack.h
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

#ifndef _LabJack_H_
#define _LabJack_H_

#include "../../MMDevice/MMDevice.h"
#include "../../MMDevice/DeviceBase.h"
#include <string>
#include <map>

//////////////////////////////////////////////////////////////////////////////
// Error codes
//
//#define ERR_UNKNOWN_LABEL 100
#define ERR_UNKNOWN_POSITION 101
#define ERR_INITIALIZE_FAILED 102
#define ERR_WRITE_FAILED 103
#define ERR_CLOSE_FAILED 104
#define ERR_BOARD_NOT_FOUND 105

class CLJShutter : public CShutterBase<CLJShutter>  
{
public:
   CLJShutter();
   ~CLJShutter();
  
   // MMDevice API
   // ------------
   int Initialize();
   int Shutdown();
  
   void GetName(char* pszName) const;
   bool Busy();
   
   // Shutter API
   int SetOpen(bool open = true);
   int GetOpen(bool& open);
   int Fire(double deltaT);

   // action interface
   // ----------------
   int OnOnOff(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
   int WriteToPort(long lnValue);
   long openTimeUs_;
   std::string name_;

   bool initialized_;
};

class CLJSwitch : public CStateDeviceBase<CLJSwitch>  
{
public:
   CLJSwitch();
   ~CLJSwitch();
  
   // MMDevice API
   // ------------
   int Initialize();
   int Shutdown();
  
   void GetName(char* pszName) const;
   bool Busy() {return busy_;}
   
   unsigned long GetNumberOfPositions()const {return numPos_;}

   // action interface
   // ----------------
   int OnState(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
   int OpenPort(const char* pszName, long lnValue);
   int WriteToPort(long lnValue);
   int ClosePort();

   bool initialized_;
   bool busy_;
   long numPos_;
};

class CLJDA : public CSignalIOBase<CLJDA>  
{
public:
   CLJDA(unsigned channel, const char* name);
   ~CLJDA();
  
   // MMDevice API
   // ------------
   int Initialize();
   int Shutdown();
  
   void GetName(char* pszName) const;
   bool Busy() {return busy_;}

   int SetSignal(double volts);
   int GetSignal(double& /*volts*/) {return DEVICE_UNSUPPORTED_COMMAND;}
   int GetLimits(double& minVolts, double& maxVolts) {minVolts = minV_; maxVolts = maxV_; return DEVICE_OK;}

   // action interface
   // ----------------
   int OnVolts(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
   int WriteToPort(long lnValue);
   int SetVolts(double v);

   bool initialized_;
   bool busy_;
   double minV_;
   double maxV_;
   unsigned int encoding_;
   unsigned int resolution_;
   unsigned channel_;
   std::string name_;
};

#endif //_LabJack_H_
