///////////////////////////////////////////////////////////////////////////////
//FILE:          ChannelSpec.java
//PROJECT:       Micro-Manager
//SUBSYSTEM:     mmstudio
//-----------------------------------------------------------------------------
//
// AUTHOR:       Nenad Amodaj, nenad@amodaj.com, November 10, 2005
//
// COPYRIGHT:    University of California, San Francisco, 2006
//
// LICENSE:      This file is distributed under the BSD license.
//               License text is included with the source distribution.
//
//               This file is distributed in the hope that it will be useful,
//               but WITHOUT ANY WARRANTY; without even the implied warranty
//               of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
//               IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//               CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//               INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES.
//
// CVS:          $Id: ChannelSpec.java 320 2007-07-03 00:54:41Z nenad $
//
package org.micromanager.utils;
import java.awt.Color;

/**
 * Channel acquisition protocol. 
 */
public class ChannelSpec {
   public static final String DEFAULT_CHANNEL_GROUP = "Channel";
   
   public String config_ = "";
   public double exposure_ = 10.0; // ms
   public double zOffset_ = 0.0; // um
   public Color color_ = Color.gray;
   public ContrastSettings contrast8_;
   public ContrastSettings contrast16_;
   public String name_ = "";
   public int skipFactorFrame_ = 0;
   
   public ChannelSpec(){
      contrast8_ = new ContrastSettings(0, 255);
      contrast16_ = new ContrastSettings(0, 65535);
      color_ = Color.WHITE;
   }
}

