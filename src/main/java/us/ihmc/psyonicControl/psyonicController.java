package us.ihmc.psyonicControl;

import us.ihmc.abilityhand.AbilityHandBLEManager;
import us.ihmc.abilityhand.AbilityHandIndividualFingerControlCommand;

public class psyonicController
{
   private final String[] handAddresses = new String[] {"DE:76:4F:34:6F:E1", "F9:C8:0F:A7:A4:D5"};
   private final AbilityHandIndividualFingerControlCommand leftHandControlCommand = new AbilityHandIndividualFingerControlCommand();
   private final AbilityHandIndividualFingerControlCommand rightHandControlCommand = new AbilityHandIndividualFingerControlCommand();

   AbilityHandBLEManager bleManager = new AbilityHandBLEManager(handAddresses);

   public int bluetoothConnect()
   {
      try
      {
         return bleManager.connect();
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
   }
   public void bluetoothDisconnect()
   {
      try
      {
         bleManager.disconnect();
      }
      catch(InterruptedException e)
      {
         throw new RuntimeException(e);
      }
   }
   public void setFingerSpeeds()
   {
      leftHandControlCommand.setThumbFlexorPeriod(0.0F);
      leftHandControlCommand.setThumbRotatorPeriod(0.0F);
      leftHandControlCommand.setIndexPeriod(0.0F);
      leftHandControlCommand.setMiddlePeriod(0.0F);
      leftHandControlCommand.setRingPeriod(0.0F);
      leftHandControlCommand.setPinkyPeriod(0.0F);
      rightHandControlCommand.setThumbFlexorPeriod(0.0F);
      rightHandControlCommand.setThumbRotatorPeriod(0.0F);
      rightHandControlCommand.setIndexPeriod(0.0F);
      rightHandControlCommand.setMiddlePeriod(0.0F);
      rightHandControlCommand.setRingPeriod(0.0F);
      rightHandControlCommand.setPinkyPeriod(0.0F);
   }
   public void setFingerAngles(float angle, int fingerNum, int x, int hand)
   {
      if(angle > 150)
      {
         angle = 150;
      }
      else if (angle <= 0)
      {
         angle = 0;
      }
      if(fingerNum == 0)
      {
         if (x == 0)
         {
            if(hand == 0)
            {
               leftHandControlCommand.setThumbFlexorPosition(angle);
            }
            if(hand == 1)
            {
               rightHandControlCommand.setThumbFlexorPosition(angle);
            }

         }
         else
         {
            if(hand == 0)
            {
               angle = -(80 - angle);
               if (angle > 0)
               {
                  angle = 0;
               }
               else if (angle < -80)
               {
                  angle = -80;
               }

               leftHandControlCommand.setThumbRotatorPosition(angle);
            }
            if(hand == 1)
            {
               angle = -(60 - angle);
               if (angle > 0)
               {
                  angle = 0;
               }
               else if (angle < -80)
               {
                  angle = -80;
               }
               rightHandControlCommand.setThumbRotatorPosition(angle);
            }
         }
      }
      if(fingerNum == 1)
      {
         if(hand == 0)
         {
            leftHandControlCommand.setIndexPosition(angle);
         }
         if(hand == 1)
         {
            rightHandControlCommand.setIndexPosition(angle);
         }
      }
      else if(fingerNum == 2)
      {
         if(hand == 0)
         {
            leftHandControlCommand.setMiddlePosition(angle);
         }
         if(hand == 1)
         {
            rightHandControlCommand.setMiddlePosition(angle);
         }
      }
      else if(fingerNum == 3)
      {
         if(hand == 0)
         {
            leftHandControlCommand.setRingPosition(angle);
         }
         if(hand == 1)
         {
            rightHandControlCommand.setRingPosition(angle);
         }
      }
      else if(fingerNum == 4)
      {
         if(hand == 0)
         {
            leftHandControlCommand.setPinkyPosition(angle);
         }
         if(hand == 1)
         {
            rightHandControlCommand.setPinkyPosition(angle);
         }
      }
   }
   public void sendCommand()
   {
      bleManager.sendIndividualCommand(handAddresses[0], leftHandControlCommand);
      bleManager.sendIndividualCommand(handAddresses[1], rightHandControlCommand);
   }
}