///////////////////////////////////////////////////////////////////////////////
// FILE:          CoreCallback.cpp
// PROJECT:       Micro-Manager
// SUBSYSTEM:     MMCore
//-----------------------------------------------------------------------------
// DESCRIPTION:   Callback object for MMCore device interface. Encapsulates
//                (bottom) internal API for calls going from devices to the 
//                core.
//
//                This class is essentialy an extension of the CMMCore class
//                and has full access to CMMCore private members.
//              
// AUTHOR:        Nenad Amodaj, nenad@amodaj.com, 01/05/2007

// COPYRIGHT:     University of California, San Francisco, 2007
//
// LICENSE:       This file is distributed under the "Lesser GPL" (LGPL) license.
//                License text is included with the source distribution.
//
//                This file is distributed in the hope that it will be useful,
//                but WITHOUT ANY WARRANTY; without even the implied warranty
//                of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
//                IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//                CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//                INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES.
//
// CVS:           $Id: CoreCallback.h 2 2007-02-27 23:33:17Z nenad $
//

#include "CoreCallback.h"
#include "CircularBuffer.h"

int CoreCallback::InsertImage(const MM::Device* /*caller*/, const unsigned char* buf, unsigned width, unsigned height, unsigned byteDepth, MM::ImageMetadata* pMd)
{
   if (core_->cbuf_->InsertImage(buf, width, height, byteDepth, pMd))
      return DEVICE_OK;
   else
      return DEVICE_BUFFER_OVERFLOW;
}

int CoreCallback::InsertMultiChannel(const MM::Device* /*caller*/,
                              const unsigned char* buf,
                              unsigned numChannels,
                              unsigned width,
                              unsigned height,
                              unsigned byteDepth,
                              MM::ImageMetadata* pMd)
{
   if (core_->cbuf_->InsertMultiChannel(buf, numChannels, width, height, byteDepth, pMd))
      return DEVICE_OK;
   else
      return DEVICE_BUFFER_OVERFLOW;
}

void CoreCallback::SetAcqStatus(const MM::Device* /*caller*/, int /*statusCode*/)
{
   // ???
}

int CoreCallback::OpenFrame(const MM::Device* /*caller*/)
{
   return DEVICE_OK;
}

int CoreCallback::CloseFrame(const MM::Device* /*caller*/)
{
   return DEVICE_OK;
}

int CoreCallback::AcqFinished(const MM::Device* /*caller*/, int /*statusCode*/)
{
   // close the shutter if we are in auto mode
   if (core_->autoShutter_ && core_->shutter_)
   {
      core_->shutter_->SetOpen(false);
      core_->waitForDevice(core_->shutter_);
   }
   return DEVICE_OK;
}

int CoreCallback::PrepareForAcq(const MM::Device* /*caller*/)
{
   // open the shutter if we are in auto mode
   if (core_->autoShutter_ && core_->shutter_)
   {
      core_->shutter_->SetOpen(true);
      core_->waitForDevice(core_->shutter_);
   }
   return DEVICE_OK;
}

/**
 * Handler for the status change event from the device.
 */
int CoreCallback::OnStatusChanged(const MM::Device* /* caller */)
{
   return DEVICE_OK;
}

/**
 * Handler for the property change event from the device.
 */
int CoreCallback::OnPropertiesChanged(const MM::Device* /* caller */)
{
   if (core_->externalCallback_)
      core_->externalCallback_->onPropertiesChanged();

   return DEVICE_OK;
}
   
/**
 * Handler for the operation finished event from the device.
 */
int CoreCallback::OnFinished(const MM::Device* /* caller */)
{
   return DEVICE_OK;
}
