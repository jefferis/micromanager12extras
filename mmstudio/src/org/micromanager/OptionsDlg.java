///////////////////////////////////////////////////////////////////////////////
//FILE:          OptionsDlg.java
//PROJECT:       Micro-Manager
//SUBSYSTEM:     mmstudio
//-----------------------------------------------------------------------------
//
// AUTHOR:       Nenad Amodaj, nenad@amodaj.com, September 12, 2006
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
// CVS:          $Id: OptionsDlg.java 990 2008-02-26 17:43:43Z nico $

package org.micromanager;

import java.awt.Dimension;
import java.awt.Font;
import java.awt.Rectangle;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.prefs.BackingStoreException;
import java.util.prefs.Preferences;

import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JTextField;
import javax.swing.SpringLayout;

import mmcorej.CMMCore;

import org.micromanager.api.DeviceControlGUI;
import org.micromanager.utils.GUIColors;
import org.micromanager.utils.MMDialog;

/**
 * Options dialog for MMStudio.
 *
 */
public class OptionsDlg extends MMDialog {
   private static final long serialVersionUID = 1L;
   private JTextField bufSizeField_;
   private MMOptions opts_;
   private CMMCore core_;
   private SpringLayout springLayout;
   private Preferences mainPrefs_;
   private JComboBox comboDisplayBackground_;
   private DeviceControlGUI parent_;
   private GUIColors guiColors_;

   /**
    * Create the dialog
    */
   public OptionsDlg(MMOptions opts, CMMCore core, Preferences mainPrefs, DeviceControlGUI parent) {
      super();
      parent_ = parent;
      addWindowListener(new WindowAdapter() {
         public void windowClosing(final WindowEvent e) {
            savePosition();
         }
      });
      setResizable(false);
      setModal(true);
      opts_ = opts;
      core_ = core;
      mainPrefs_ = mainPrefs;
      setTitle("Micro-Manager Options");
      springLayout = new SpringLayout();
      getContentPane().setLayout(springLayout);
      setBounds(100, 100, 362, 246);
      guiColors_ = new GUIColors();
      Dimension buttonSize = new Dimension(100, 20);

      if (opts_.displayBackground.equals("Day"))
         setBackground(java.awt.SystemColor.control);
      else if (opts_.displayBackground.equals("Night"))
         setBackground(java.awt.Color.gray);
      Preferences root = Preferences.userNodeForPackage(this.getClass());
      setPrefsNode(root.node(root.absolutePath() + "/OptionsDlg"));
      
      Rectangle r = getBounds();
      loadPosition(r.x, r.y);

      final JCheckBox debugLogEnabledCheckBox = new JCheckBox();
      debugLogEnabledCheckBox.setToolTipText("Set extra verbose logging for dubugging purposes");
      debugLogEnabledCheckBox.addActionListener(new ActionListener() {
         public void actionPerformed(final ActionEvent e) {
            opts_.debugLogEnabled = debugLogEnabledCheckBox.isSelected();
            core_.enableDebugLog(opts_.debugLogEnabled);
         }
      });
      debugLogEnabledCheckBox.setText("Debug log enabled");
      getContentPane().add(debugLogEnabledCheckBox);
      springLayout.putConstraint(SpringLayout.SOUTH, debugLogEnabledCheckBox, 35, SpringLayout.NORTH, getContentPane());
      springLayout.putConstraint(SpringLayout.NORTH, debugLogEnabledCheckBox, 12, SpringLayout.NORTH, getContentPane());
      springLayout.putConstraint(SpringLayout.EAST, debugLogEnabledCheckBox, 190, SpringLayout.WEST, getContentPane());
      springLayout.putConstraint(SpringLayout.WEST, debugLogEnabledCheckBox, 10, SpringLayout.WEST, getContentPane());

      final JButton clearLogFileButton = new JButton();
      clearLogFileButton.setToolTipText("Erases all entries in the current log file (recommended)");
      clearLogFileButton.addActionListener(new ActionListener() {
         public void actionPerformed(final ActionEvent e) {
            core_.clearLog();
         }
      });
      clearLogFileButton.setFont(new Font("", Font.PLAIN, 10));
      clearLogFileButton.setText("Clear log file");
      clearLogFileButton.setPreferredSize(buttonSize);
      getContentPane().add(clearLogFileButton);
      //springLayout.putConstraint(SpringLayout.SOUTH, clearLogFileButton, 166, SpringLayout.NORTH, getContentPane());
      springLayout.putConstraint(SpringLayout.NORTH, clearLogFileButton, 140, SpringLayout.NORTH, getContentPane());

      final JButton clearRegistryButton = new JButton();
      clearRegistryButton.setToolTipText("Clears all persistent settings and returns to defaults");
      clearRegistryButton.addActionListener(new ActionListener() {
         public void actionPerformed(final ActionEvent e) {
            try {
               boolean previouslyRegistered = mainPrefs_.getBoolean(RegistrationDlg.REGISTRATION, false);
               mainPrefs_.clear();
               Preferences acqPrefs = mainPrefs_.node(mainPrefs_.absolutePath() + "/" + AcqControlDlg.ACQ_SETTINGS_NODE);
               acqPrefs.clear();
               
               // restore registration flag
               mainPrefs_.putBoolean(RegistrationDlg.REGISTRATION, previouslyRegistered);
               
            } catch (BackingStoreException exc) {
               JOptionPane.showMessageDialog(OptionsDlg.this, exc.getMessage());
            }
         }
      });
      clearRegistryButton.setText("Clear registry");
      clearRegistryButton.setFont(new Font("", Font.PLAIN, 10));
      clearRegistryButton.setPreferredSize(buttonSize);
      getContentPane().add(clearRegistryButton);
      //springLayout.putConstraint(SpringLayout.SOUTH, clearRegistryButton, 199, SpringLayout.NORTH, getContentPane());
      springLayout.putConstraint(SpringLayout.NORTH, clearRegistryButton, 173, SpringLayout.NORTH, getContentPane());

      final JButton okButton = new JButton();
      okButton.addActionListener(new ActionListener() {
         public void actionPerformed(final ActionEvent e) {
            opts_.circularBufferSizeMB = Integer.parseInt(bufSizeField_.getText());
            savePosition();
            dispose();
         }
      });
      okButton.setText("Close");
      okButton.setFont(new Font("", Font.PLAIN, 10));
      okButton.setPreferredSize(buttonSize);
      getContentPane().add(okButton);
      springLayout.putConstraint(SpringLayout.EAST, okButton, -5, SpringLayout.EAST, getContentPane());
      springLayout.putConstraint(SpringLayout.WEST, okButton, 250, SpringLayout.WEST, getContentPane());
      springLayout.putConstraint(SpringLayout.NORTH, okButton, 12, SpringLayout.NORTH, getContentPane());
      springLayout.putConstraint(SpringLayout.SOUTH, okButton, 35, SpringLayout.NORTH, getContentPane());
      
      debugLogEnabledCheckBox.setSelected(opts_.debugLogEnabled);

      final JCheckBox multithreadedAcquisitionCheckBox = new JCheckBox();
      multithreadedAcquisitionCheckBox.addActionListener(new ActionListener() {
         public void actionPerformed(ActionEvent arg0) {
            opts_.multiThreadedAcqEnabled = multithreadedAcquisitionCheckBox.isSelected();
         }
      });
      multithreadedAcquisitionCheckBox.setText("Multi-threaded Acquisition");
      getContentPane().add(multithreadedAcquisitionCheckBox);
      springLayout.putConstraint(SpringLayout.EAST, multithreadedAcquisitionCheckBox, 200, SpringLayout.WEST, getContentPane());
      springLayout.putConstraint(SpringLayout.WEST, multithreadedAcquisitionCheckBox, 0, SpringLayout.WEST, debugLogEnabledCheckBox);
      springLayout.putConstraint(SpringLayout.EAST, clearRegistryButton, 110, SpringLayout.WEST, multithreadedAcquisitionCheckBox);
      springLayout.putConstraint(SpringLayout.WEST, clearRegistryButton, 0, SpringLayout.WEST, multithreadedAcquisitionCheckBox);
      springLayout.putConstraint(SpringLayout.EAST, clearLogFileButton, 110, SpringLayout.WEST, multithreadedAcquisitionCheckBox);
      springLayout.putConstraint(SpringLayout.WEST, clearLogFileButton, 0, SpringLayout.WEST, multithreadedAcquisitionCheckBox);
      springLayout.putConstraint(SpringLayout.SOUTH, multithreadedAcquisitionCheckBox, 60, SpringLayout.NORTH, getContentPane());
      multithreadedAcquisitionCheckBox.setSelected(opts_.multiThreadedAcqEnabled);

      final JCheckBox doNotAskForConfigFileCheckBox = new JCheckBox();
      doNotAskForConfigFileCheckBox.addActionListener(new ActionListener() {
         public void actionPerformed(ActionEvent arg0) {
            opts_.doNotAskForConfigFile = doNotAskForConfigFileCheckBox.isSelected();
         }
      });
      doNotAskForConfigFileCheckBox.setText("Do not ask for config file");
      getContentPane().add(doNotAskForConfigFileCheckBox);
      springLayout.putConstraint(SpringLayout.EAST, doNotAskForConfigFileCheckBox, 200, SpringLayout.WEST, getContentPane());
      springLayout.putConstraint(SpringLayout.WEST, doNotAskForConfigFileCheckBox, 0, SpringLayout.WEST, debugLogEnabledCheckBox);
      springLayout.putConstraint(SpringLayout.SOUTH, doNotAskForConfigFileCheckBox, 85, SpringLayout.NORTH, getContentPane());
      doNotAskForConfigFileCheckBox.setSelected(opts_.doNotAskForConfigFile);

      final JLabel sequenceBufferSizeLabel = new JLabel();
      sequenceBufferSizeLabel.setText("Sequence buffer size [MB]");
      getContentPane().add(sequenceBufferSizeLabel);
      springLayout.putConstraint(SpringLayout.EAST, sequenceBufferSizeLabel, 210, SpringLayout.WEST, getContentPane());
      springLayout.putConstraint(SpringLayout.WEST, sequenceBufferSizeLabel, 15, SpringLayout.WEST, getContentPane());
      springLayout.putConstraint(SpringLayout.SOUTH, sequenceBufferSizeLabel, 109, SpringLayout.NORTH, getContentPane());
      springLayout.putConstraint(SpringLayout.NORTH, sequenceBufferSizeLabel, 95, SpringLayout.NORTH, getContentPane());

      bufSizeField_ = new JTextField(Integer.toString(opts_.circularBufferSizeMB));
      getContentPane().add(bufSizeField_);
      springLayout.putConstraint(SpringLayout.SOUTH, bufSizeField_, 110, SpringLayout.NORTH, getContentPane());
      springLayout.putConstraint(SpringLayout.NORTH, bufSizeField_, 90, SpringLayout.NORTH, getContentPane());
      springLayout.putConstraint(SpringLayout.EAST, bufSizeField_, 300, SpringLayout.WEST, getContentPane());
      springLayout.putConstraint(SpringLayout.WEST, bufSizeField_, 220, SpringLayout.WEST, getContentPane());

      final JLabel displayLabel = new JLabel();
      //displayLabel.setFont(new Font("Arial", Font.PLAIN, 10));
      displayLabel.setText("Display-Background");
      getContentPane().add(displayLabel); 
      springLayout.putConstraint(SpringLayout.EAST, displayLabel, 170, SpringLayout.WEST, getContentPane());
      springLayout.putConstraint(SpringLayout.WEST, displayLabel, 15, SpringLayout.WEST, getContentPane());
      springLayout.putConstraint(SpringLayout.SOUTH, displayLabel, 133, SpringLayout.NORTH, getContentPane());
      springLayout.putConstraint(SpringLayout.NORTH, displayLabel, 117, SpringLayout.NORTH, getContentPane());

      comboDisplayBackground_ = new JComboBox(guiColors_.styleOptions);
      comboDisplayBackground_.setFont(new Font("Arial", Font.PLAIN, 10));              
      comboDisplayBackground_.setMaximumRowCount(2);
      comboDisplayBackground_.setSelectedItem(opts_.displayBackground);
      comboDisplayBackground_.addActionListener(new ActionListener() {
         public void actionPerformed(ActionEvent e) {
            changeBackground();
         }
      });
      getContentPane().add(comboDisplayBackground_);
      springLayout.putConstraint(SpringLayout.EAST, comboDisplayBackground_, 331, SpringLayout.WEST, getContentPane());
      springLayout.putConstraint(SpringLayout.WEST, comboDisplayBackground_, 220, SpringLayout.WEST, getContentPane());
      springLayout.putConstraint(SpringLayout.SOUTH, comboDisplayBackground_, 139, SpringLayout.NORTH, getContentPane());
      springLayout.putConstraint(SpringLayout.NORTH, comboDisplayBackground_, 116, SpringLayout.NORTH, getContentPane());
   }

   private void changeBackground() {
       String background = (String)comboDisplayBackground_.getSelectedItem();
       opts_.displayBackground = background;
       setBackground(guiColors_.background.get(background));

       if (parent_ != null) // test for null just to avoid crashes (should never be null)
       {
          // set background and trigger redraw of parent and its descendant windows
          parent_.setBackgroundStyle(background);
       }
   }

}
