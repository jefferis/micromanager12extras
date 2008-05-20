import org.micromanager.metadata.AcquisitionData;
import org.micromanager.metadata.MMAcqDataException;
import mmcorej.CMMCore;

/* Test_AcquisitionDatareate.java
 * Created on March 11, 2006
 *
 * MicroManager sample code
 */

/**
 * Example program to demonstrate accessing acquired data generated by MicroManager.
 * The program uses AcquistionData class to access a collection of files stored
 * on disk, in such way that they appear as a single multi-dimensional image object.
 * 
 * Requires MMJ_.jar and ij.jar on the classpath.
 */
public class Test_AcquisitionDataCreate {

   public static void main(String[] args) {
      
      final String acqDir = "c:/AcquisitionData/test/create";
      int frames = 5;
      int slices = 1;
      
      CMMCore mmc = new CMMCore();
      try {
         mmc.loadSystemConfiguration("c:/projects/micromanager1.2/bin/MMConfig_demo.cfg");
         int w = (int)mmc.getImageWidth();
         int h = (int)mmc.getImageHeight();
         int depth = (int)mmc.getBytesPerPixel();
         
         // instantiate 5d image object
         AcquisitionData ad = new AcquisitionData();
         ad.createNew("newdata", acqDir);
         ad.setImagePhysicalDimensions(w, h, depth);
         
         String channels[] = new String[2];
         channels[0] = new String("DAPI");
         channels[1] = new String("FITC");
         
         ad.setDimensions(frames, channels.length, slices);
         ad.setChannelNames(channels);
         
         for (int i=0; i<frames; i++) {
            for (int j=0; j<channels.length; j++) {
               mmc.setConfig("Channel", channels[j]);
               mmc.waitForSystem();
               mmc.snapImage();
               Object img = mmc.getImage();
               ad.insertImage(img, i, j, 0);
            }
         }
         
      } catch (MMAcqDataException e) {
         // TODO Auto-generated catch block
         e.printStackTrace();
      } catch (Exception e1) {
         // TODO Auto-generated catch block
         e1.printStackTrace();
      }
   }
}
