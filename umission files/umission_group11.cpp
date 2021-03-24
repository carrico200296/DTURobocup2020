/***************************************************************************
 *   Copyright (C) 2016-2020 by DTU (Christian Andersen)                        *
 *   jca@elektro.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/



#include <sys/time.h>
#include <cstdlib>

#include "umission.h"
#include "utime.h"
#include "ulibpose2pose.h"

using namespace cv;


UMission::UMission(UBridge * regbot, UCamera * camera)
{
  cam = camera;
  bridge = regbot;
  threadActive = 100;
  // initialize line list to empty
  for (int i = 0; i < missionLineMax; i++)
  { // add to line list 
    lines[i] = lineBuffer[i];    
    // terminate c-strings strings - good practice, but not needed
    lines[i][0] = '\0';
  }
  // start mission thread
  th1 = new thread(runObj, this);
}


UMission::~UMission()
{
  printf("Mission class destructor\n");
}


void UMission::run()
{
  while (not active and not th1stop)
    usleep(100000);
    //printf("UMission::run:  active=%d, th1stop=%d\n", active, th1stop);
  if (not th1stop)
    runMission();
  printf("UMission::run: mission thread ended\n");
}
  
void UMission::printStatus()
{
  printf("# ------- Mission ----------\n");
  printf("# active = %d, finished = %d\n", active, finished);
  printf("# mission part=%d, in state=%d\n", mission, missionState);
}
  
/**
 * Initializes the communication with the robobot_bridge and the REGBOT.
 * It further initializes a (maximum) number of mission lines 
 * in the REGBOT microprocessor. */
void UMission::missionInit()
{ // stop any not-finished mission
  bridge->send("robot stop\n");
  // clear old mission
  bridge->send("robot <clear\n");
  //
  // add new mission with 3 threads
  // one (100) starting at event 30 and stopping at event 31
  // one (101) starting at event 31 and stopping at event 30
  // one (  1) used for idle and initialisation of hardware
  // the mission is started, but staying in place (velocity=0, so servo action)
  //
  bridge->send("robot <add thread=1\n");
  // Irsensor should be activated a good time before use 
  // otherwise first samples will produce "false" positive (too short/negative).
  bridge->send("robot <add irsensor=1,vel=0:dist<0.2\n");
  //
  // alternating threads (100 and 101, alternating on event 30 and 31 (last 2 events)
  bridge->send("robot <add thread=100,event=30 : event=31\n");
  for (int i = 0; i < missionLineMax; i++)
    // send placeholder lines, that will never finish
    // are to be replaced with real mission
    // NB - hereafter no lines can be added to these threads, just modified
    bridge->send("robot <add vel=0 : time=0.1\n");
  //
  bridge->send("robot <add thread=101,event=31 : event=30\n");
  for (int i = 0; i < missionLineMax; i++)
    // send placeholder lines, that will never finish
    bridge->send("robot <add vel=0 : time=0.1\n");
  usleep(10000);
  //
  //
  // send subscribe to bridge
  bridge->pose->subscribe();
  bridge->edge->subscribe();
  bridge->motor->subscribe();
  bridge->event->subscribe();
  bridge->joy->subscribe();
  bridge->motor->subscribe();
  bridge->info->subscribe();
  bridge->irdist->subscribe();
  bridge->imu->subscribe();
  usleep(10000);
  // there maybe leftover events from last mission
  bridge->event->clearEvents();
}


void UMission::sendAndActivateSnippet(char ** missionLines, int missionLineCnt)
{
  // Calling sendAndActivateSnippet automatically toggles between thread 100 and 101. 
  // Modifies the currently inactive thread and then makes it active. 
  const int MSL = 100;
  char s[MSL];
  int threadToMod = 101;
  int startEvent = 31;
  // select Regbot thread to modify
  // and event to activate it
  if (threadActive == 101)
  {
    threadToMod = 100;
    startEvent = 30;
  }
  if (missionLineCnt > missionLineMax)
  {
    printf("# ----------- error - too many lines ------------\n");
    printf("# You tried to send %d lines, but there is buffer space for %d only!\n", missionLineCnt, missionLineMax);
    printf("# set 'missionLineMax' to a higher number in 'umission.h' about line 57\n");
    printf("# (not all lines will be send)\n");
    printf("# -----------------------------------------------\n");
    missionLineCnt = missionLineMax;
  }
  // send mission lines using '<mod ...' command
  for (int i = 0; i < missionLineCnt; i++)
  { // send lines one at a time
    if (strlen((char*)missionLines[i]) > 0)
    { // send a modify line command
      snprintf(s, MSL, "<mod %d %d %s\n", threadToMod, i+1, missionLines[i]);
      bridge->send(s); 
    }
    else
      // an empty line will end code snippet too
      break;
  }
  // let it sink in (10ms)
  usleep(10000);
  // Activate new snippet thread and stop the other  
  snprintf(s, MSL, "<event=%d\n", startEvent);
  bridge->send(s);
  // save active thread number
  threadActive = threadToMod;
}


//////////////////////////////////////////////////////////

/**
 * Thread for running the mission(s)
 * All missions segments are called in turn based on mission number
 * Mission number can be set at parameter when starting mission command line.
 * 
 * The loop also handles manual override for the gamepad, and resumes
 * when manual control is released.
 * */
void UMission::runMission()
{ /// current mission number
  mission = fromMission;
  int missionOld = mission;
  bool regbotStarted = false;
  /// end flag for current mission
  bool ended = false;
  /// manuel override - using gamepad
  bool inManual = false;
  /// debug loop counter
  int loop = 0;
  // keeps track of mission state
  missionState = 0;
  int missionStateOld = missionState;
  // fixed string buffer
  const int MSL = 120;
  char s[MSL];
  /// initialize robot mission to do nothing (wait for mission lines)
  missionInit();
  /// start (the empty) mission, ready for mission snippets.
  bridge->send("start\n"); // ask REGBOT to start controlled run (ready to execute)
  bridge->send("oled 3 waiting for REGBOT\n");
  ///
  for (int i = 0; i < 3; i++)
  {
    if (not bridge->info->isHeartbeatOK())
    { // heartbeat should come at least once a second
      sleep(1);
    }
  }
  if (not bridge->info->isHeartbeatOK())
  { // heartbeat should come at least once a second
    system("espeak \"Oops, no usable connection with robot.\" -ven+f4 -s130 -a60 2>/dev/null &"); 
    bridge->send("oled 3 Oops: Lost REGBOT!");
    printf("# ---------- error ------------\n");
    printf("# No heartbeat from robot. Bridge or REGBOT is stuck\n");
    printf("# You could try restart ROBOBOT bridge ('b' from mission console) \n");
    printf("# -----------------------------\n");
    stop();
  }
  /// loop in sequence every mission until they report ended
  while (not finished and not th1stop)
  { // stay in this mission loop until finished
    loop++;
    // test for manuel override (joy is short for joystick or gamepad)
    if (bridge->joy->manual)
    { // just wait, do not continue mission
      usleep(20000);
      if (not inManual)
        system("espeak \"Mission paused.\" -ven+f4 -s130 -a40 2>/dev/null &"); 
      inManual = true;
      bridge->send("oled 3 GAMEPAD control\n");
    }
    else
    { // in auto mode
      if (not regbotStarted)
      { // wait for start event is received from REGBOT
        // - in response to 'bot->send("start\n")' earlier
        if (bridge->event->isEventSet(33))
        { // start mission (button pressed)
          // printf("Mission::runMission: starting mission (part from %d to %d)\n", fromMission, toMission);
          regbotStarted = true;
        }
      }
      else
      { // mission in auto mode
        if (inManual)
        { // just entered auto mode, so tell.
          inManual = false;
          system("espeak \"Mission resuming.\" -ven+f4 -s130 -a40 2>/dev/null &");
          bridge->send("oled 3 running AUTO\n");
        }
        switch(mission)
        {
          // As due to coronavirus we were not able to run the missions one after the other. Each challenge has been added
          // in each mission and here it is set which mission will be run.
          case 1: // running auto mission
            //ended = mission1(missionState);
	          ended = true;
            break;
          case 2:
            //ended = mission2(missionState);
	          ended = true;
            break;
          case 3:
            //ended = mission3(missionState);
            ended = true;
            break;
          case 4:
            ended = mission4(missionState);
            break;
          default:
            // no more missions - end everything
            finished = true;
            break;
        }
        if (ended)
        { // start next mission part in state 0
          mission++;
          ended = false;
          missionState = 0;
        }
        // show current state on robot display
        if (mission != missionOld or missionState != missionStateOld)
        { // update small O-led display on robot - when there is a change
          UTime t;
          t.now();
          snprintf(s, MSL, "oled 4 mission %d state %d\n", mission, missionState);
          bridge->send(s);
          if (logMission != NULL)
          {
            fprintf(logMission, "%ld.%03ld %d %d\n", 
                    t.getSec(), t.getMilisec(),
                    missionOld, missionStateOld
            );
            fprintf(logMission, "%ld.%03ld %d %d\n", 
                    t.getSec(), t.getMilisec(),
                    mission, missionState
            );
          }
          missionOld = mission;
          missionStateOld = missionState;
        }
      }
    }
    //
    // check for general events in all modes
    // gamepad buttons 0=green, 1=red, 2=blue, 3=yellow, 4=LB, 5=RB, 6=back, 7=start, 8=Logitech, 9=A1, 10 = A2
    // gamepad axes    0=left-LR, 1=left-UD, 2=LT, 3=right-LR, 4=right-UD, 5=RT, 6=+LR, 7=+-UD
    // see also "ujoy.h"
    if (bridge->joy->button[BUTTON_RED])
    { // red button -> save image
      if (not cam->saveImage)
      {
        printf("UMission::runMission:: button 1 (red) pressed -> save image\n");
        cam->saveImage = true;
      }
    }
    if (bridge->joy->button[BUTTON_YELLOW])
    { // yellow button -> make ArUco analysis
      if (not cam->doArUcoAnalysis)
      {
        printf("UMission::runMission:: button 3 (yellow) pressed -> do ArUco\n");
        cam->doArUcoAnalysis = true;
      }
    }
    // are we finished - event 0 disables motors (e.g. green button)
    if (bridge->event->isEventSet(0))
    { // robot say stop
      finished = true;
      printf("Mission:: insist we are finished\n");
    }
    else if (mission > toMission)
    { // stop robot
      // make an event 0
      bridge->send("stop\n");
      // stop mission loop
      finished = true;
    }
    // release CPU a bit (10ms)
    usleep(10000);
  }
  bridge->send("stop\n");
  snprintf(s, MSL, "espeak \"%s finished.\"  -ven+f4 -s130 -a12  2>/dev/null &", bridge->info->robotname);
  system(s); 
  printf("Mission:: all finished\n");
  bridge->send("oled 3 finished\n");
}


////////////////////////////////////////////////////////////

/**
 * Run mission
 * \param state is kept by caller, but is changed here
 *              therefore defined as reference with the '&'.
 *              State will be 0 at first call.
 * \returns true, when finished. */
bool UMission::mission1(int & state)
{
  bool finished = false;
  // First commands to send to robobot in given mission
  // (robot sends event 1 after driving 1 meter)):
  switch (state)
  {
    case 0:
      // tell the operatior what to do
      printf("# Mission 1: press green to start.\n");
      //system("espeak \"press green to start\" -ven+f4 -s130 -a5 2>/dev/null &"); 
      bridge->send("oled 5 press green to start");
      state++;
      break;
    case 1:
      if (bridge->joy->button[BUTTON_GREEN])
        state = 10;
      break;
    case 10: // CHALLENGE 1: make a curve and find a 90 degree intersection and follow it
      // Follow line on the left edge until there is crossing with a certainty of 15
      // Left edge because it will follow the right side of the line with 0 cm margin
      snprintf(lines[0], MAX_LEN, "vel=0.3, edgel=0.0, white=1: dist=5.0, xl>15");
      sendAndActivateSnippet(lines, sizeof(lines));

      state = 11;
      break;

    case 11:
      if (bridge->event->isEventSet(1))
      { 
        // Finished!
        state = 999;
        play.stopPlaying();
      }
      break;
    default:
      printf("Mission 1 ended \n");
      bridge->send("oled 5 \"mission 1 ended.\"");
      finished = true;
      break;
  }
  return finished;
}


/**
 * Run mission
 * \param state is kept by caller, but is changed here
 *              therefore defined as reference with the '&'.
 *              State will be 0 at first call.
 * \returns true, when finished. */
bool UMission::mission2(int & state)
{
  bool finished = false;
  // First commands to send to robobot in given mission
  // (robot sends event 1 after driving 1 meter)):
  switch (state)
  {
    case 0:
      // tell the operatior what to do
      printf("# Mission 2: press green to start.\n");
      //system("espeak \"press green to start\" -ven+f4 -s130 -a5 2>/dev/null &"); 
      bridge->send("oled 5 press green to start");
      state++;
      break;
    case 1:
      if (bridge->joy->button[BUTTON_GREEN]) {
        state = 10;
      }
      break;
    case 10: // CHALLENGE 2: avoid an object while following a line
      // Follow line for 5 meters or until an object is within 20 cm
      // Execute the event 1
      snprintf(lines[0], MAX_LEN, "vel=0.2, edgel=0.0, white=1: dist=5.0, ir2<0.2");
      snprintf(lines[1], MAX_LEN, "vel=0.0: time=1.0");
      snprintf(lines[2], MAX_LEN, "event=1");
      sendAndActivateSnippet(lines, sizeof(lines));
      state = 12;
      break;
    case 12:
      if (bridge->event->isEventSet(1))
      {
	      // After the event is on, 
        // Turn left 90 degrees
        snprintf(lines[0], MAX_LEN, "vel=0.2, tr=0.0: time=4.0, turn=90.0"); //Note 90 degrees turns left, -90 turns right
        // Continue straight until the object is not seen by the IR
        snprintf(lines[1], MAX_LEN, "vel=0.2: dist=2.0, ir1>0.5");
        // Continue straight further as the IR sensor is in the middle of the robot
        snprintf(lines[2], MAX_LEN, "vel=0.2: dist=0.1");
        // Turn again 90 degrees to start driving past the object
        snprintf(lines[3], MAX_LEN, "vel=0.2, tr=0.0: time=4.0, turn=-90.0");
        // Wait set as a debug method
        snprintf(lines[4], MAX_LEN, "vel=0.0: time=2.0");
        // Set event 2
        snprintf(lines[5], MAX_LEN, "event=2");
        sendAndActivateSnippet(lines,sizeof(lines));
        
        state = 13;
      }
      break;
    case 13:
      if (bridge->event->isEventSet(2))
      {
        // Continue until the distance is more than half a meter (we drove further than the object length)
        snprintf(lines[0], MAX_LEN, "vel=0.2: dist=2.0, ir1>0.5");
        // Drive forward as the IR is in the beginning of the robot and if turn now, it would crash.
        snprintf(lines[1], MAX_LEN, "vel=0.2: dist=0.2");
        // It is safer to drive less thn 90 degrees to avoid coming closer to the object.
        snprintf(lines[2], MAX_LEN, "vel=0.2, tr=0.0: time=4.0, turn=-70.0");
        // Set event 3
        snprintf(lines[3], MAX_LEN, "event=3");
        sendAndActivateSnippet(lines,sizeof(lines));

        state = 14;
      }
      break;
    case 14:
      if (bridge->event->isEventSet(3))
      {
        // Go straight until a line is found and continue the line.
        snprintf(lines[0], MAX_LEN, "vel=0.2, edgel=0.0, white=1: dist=5.0, xl>15");
        snprintf(lines[1], MAX_LEN, "vel=0.2, edgel=0.0, white=1: dist=5.0");
        snprintf(lines[2], MAX_LEN, "event=4");

        sendAndActivateSnippet(lines,sizeof(lines));

        state = 15;
      }
      break;
    case 15:
      // Finished!
      if (bridge->event->isEventSet(4))
      { 
        state = 999;
      }
      break;
    case 999:
    default:
      printf("Mission 2 ended \n");
      bridge->send("oled 5 \"mission 2 ended.\"");
      finished = true;
      play.stopPlaying();
      break;
  }
  return finished;
}



/**
 * Run mission
 * \param state is kept by caller, but is changed here
 *              therefore defined as reference with the '&'.
 *              State will be 0 at first call.
 * \returns true, when finished. */
bool UMission::mission3(int & state)
{
  bool finished = false;
  switch (state)
  {
    case 0:
      // tell the operatior what to do
      printf("# Mission 3: press green to start.\n");
      //system("espeak \"press green to start\" -ven+f4 -s130 -a5 2>/dev/null &"); 
      bridge->send("oled 5 press green to start");
      state++;
      break;
    case 1:
      if (bridge->joy->button[BUTTON_GREEN]) {
        state = 10;
      }
      break;
    case 10: // CHALLENGE 4: go on a platform, do a circle and go down
      // Follow line until line is not found
      snprintf(lines[0], MAX_LEN, "vel=0.4, edgel=0.0, white=1: dist=5.0, lv=0");
      // Continue a bit further
      snprintf(lines[1], MAX_LEN, "vel=0.3: dist=0.15");
      // Wait and set event 1
      snprintf(lines[2], MAX_LEN, "vel=0.0: time=1.0");
      snprintf(lines[3], MAX_LEN, "event=1");
      sendAndActivateSnippet(lines, sizeof(lines));
      state = 12;
      break;
    case 12:
      if (bridge->event->isEventSet(1))
      {
        // After the event 1 is set we are on the platform.
        printf("We are on the platform! \n");
        // Turn 60 degrees.
        snprintf(lines[0], MAX_LEN, "vel=0.2, tr=0.0: time=4.0, turn=-60");
        // Stop as a debug method
        snprintf(lines[1], MAX_LEN, "vel=0.0: time=1.0");
        // Do the circle of about 270 degrees as the first turn already moves a bit further in the circle.
        snprintf(lines[2], MAX_LEN, "vel=0.2, tr=-0.38: time=20.0, turn=270");
        // Set event 2
        snprintf(lines[3], MAX_LEN, "event=2");
        sendAndActivateSnippet(lines,sizeof(lines));
        
        state = 13;
      }
      break;
    case 13:
      if (bridge->event->isEventSet(2))
      {
        // After the event 2 is on we have done the circular trajectory, 
        printf("We have done the circular trajectory! \n");
        // Turn to the outside of the circle
        snprintf(lines[0], MAX_LEN, "vel=0.2, tr=0.0: time=4.0, turn=-54");
        // Stop as a debug method
        snprintf(lines[1], MAX_LEN, "vel=0.0: time=1.0");
        // Slowly continue straight until the robot finds a line with certainty of 5
      	snprintf(lines[2], MAX_LEN, "vel=0.1, edgel=0.0, white=1: dist=5.0, lv=5");
        // Follow the line faster
      	snprintf(lines[3], MAX_LEN, "vel=0.2, edgel=0.0, white=1: dist=5.0");
        snprintf(lines[4], MAX_LEN, "event=3");
        sendAndActivateSnippet(lines,sizeof(lines));
        
        state = 14;
      }
      break;
    case 14:
      // Finished!
      if (bridge->event->isEventSet(3))
      {
        state = 999;
      }
      break;
    case 999:
    default:
      printf("Mission 3 ended \n");
      bridge->send("oled 5 \"mission 3 ended.\"");
      finished = true;
      play.stopPlaying();
      break;
  }
  return finished;
}


/**
 * Run mission
 * \param state is kept by caller, but is changed here
 *              therefore defined as reference with the '&'.
 *              State will be 0 at first call.
 * \returns true, when finished. */
bool UMission::mission4(int & state)
{
  bool finished = false;
  switch (state)
  {
    case 0:
      printf("# Mission 4: press green to start challenge 3.\n");
      bridge->send("oled 5 press green to start");
      state++;
      break;
    case 1: // CHALLENGE 3: find, grab, move and deliver a ball.
      if (bridge->joy->button[BUTTON_GREEN]) {
        printf("Challenge 3: find, grab, move and deliver a ball!\n");

        bool hardcoded == true;
        if (hardcoded) {
          state = 10; // Methods for harcoded version are from 10
        } else {
          state = 20; // Methods for camera version are from 20
        }
      }
      break;

    /////////////////////////////////////////////////////////////////////
    // Hardcoded version
    case 10: 
      // Go to the ball position
      printf("Puting servo up and going forward.\n");
      // To make sure servo is up. Put servo up.
      snprintf(lines[0], MAX_LEN, "servo=2, pservo=1000, vservo=0: time=2.0");
      // Move forward to the position of the ball.
      snprintf(lines[1], MAX_LEN, "servo=2, pservo=1000, vservo=0, vel=0.3: dist=0.50, time=7.0");
      // Set velocity to 0 to make sure it stays in the same position.
      snprintf(lines[2], MAX_LEN, "servo=2, pservo=1000, vservo=0, vel=0.0: time=2.0");
      snprintf(lines[3], MAX_LEN, "event=1: time=1.0");
      sendAndActivateSnippet(lines, sizeof(lines));
      state = 11;
      break;

    case 11:
      // When event 1 is set, 
      // Put the servo down to grab the ball.
      if (bridge->event->isEventSet(1))
      {
        // Put servo down
        snprintf(lines[0], MAX_LEN, "servo=2, pservo=-65, vservo=1: time=10.0");
        // Set event 2
        snprintf(lines[1], MAX_LEN, "event=2: time=1.0");
        sendAndActivateSnippet(lines, sizeof(lines));
        state = 12;
      }
      break;
    case 12:
      // When event2 is set, ball is grabbed
      // Move to the desired position to leave the ball and release it
      if (bridge->event->isEventSet(2))
      {
        // Turn 90 degrees
        snprintf(lines[0], MAX_LEN, "servo=2, pservo=-65, vservo=0, vel=0.15, tr=0.0: turn=90, time=3.0");
        // Continue forward for 15 centimeters
        snprintf(lines[1], MAX_LEN, "servo=2, pservo=-65, vservo=0, vel=0.15: dist=0.2, time=5.0");
        // Stop
        snprintf(lines[2], MAX_LEN, "servo=2, pservo=-65, vservo=1, vel=0.0: time=5.0");
        // Move the servo up slowly to release the ball 
        snprintf(lines[3], MAX_LEN, "servo=2, pservo=500, vservo=1: time=5.0");
        // Set event 3
        snprintf(lines[4], MAX_LEN, "event=3: time=1.0");
        sendAndActivateSnippet(lines, sizeof(lines));
        state = 13;
      }
      break;
    case 13:
      // When event 3 is set, ball has been released.
      // Go away from ball
      if (bridge->event->isEventSet(3))
      {
        // Turn around and continue forward 
        snprintf(lines[0], MAX_LEN, "servo=2, pservo=500, vservo=0, vel=0.2, tr=0.0: turn=120, time=15.0");
        snprintf(lines[1], MAX_LEN, "servo=2, pservo=500, vservo=0, vel=0.3: dist=0.5, time=2.0");
        // Set event 10 when finished
        snprintf(lines[1], MAX_LEN, "event=10");

        sendAndActivateSnippet(lines, sizeof(lines));

        state = 998;
      }
      break;

    /////////////////////////////////////////////////////////////////////
    // Camera version
    case 20:
      {
        // Get the image 
        printf("Getting image and analizing it.\n");
        timeval time_frame;
        Mat src;
        time_frame = cam -> capture(src);
        
        // Make sure it is working.
        if(src.empty())
        {
          printf("No frame captured\n");
          return finished;
        }
      
        // Define lines to send in case every action is false.
        const char* line0 = "vel=0.0: time=1.0";
        const char* line1 = "vel=0.0: time=1.0";

        // Create the elements that will receive the instructions
        bool straight = false, left = false, right = false, ready_to_grab = false;
        // Call the funciton detect ball. It will retrieve what the robot should do.
        cam -> detectBall(src, straight, left, right, ready_to_grab);

        printf("Going straight %d, left %d, right %d, ready to grab %d.\n", straight, left, right, ready_to_grab);

        if(straight){
          printf("Going straight.\n"); 
          line0 = "vel=0.2: dist=0.05, time=3.0";
        }
        if(left){
          printf("Going left.\n"); 
          line1 = "vel=0.2, tr=0.0: turn=5, time=3.0";
        }
        if(right){
          printf("Going rigth.\n");
          line1 = "vel=0.2, tr=0.0: turn=-5, time=3.0";
        }

        // Go to next state to grab the ball if it is ready to grab
        if(ready_to_grab){
          printf("Ready to grab the ball.\n");
          state = 21;
        }else{
          // If it is not ready to grab, move as it has been set
          state = 30;
          snprintf(lines[0], MAX_LEN, line0);
          snprintf(lines[1], MAX_LEN, line1);
          // Set event 1
          snprintf(lines[2], MAX_LEN, "event=1: time=1.0");
          sendAndActivateSnippet(lines, sizeof(lines));
        }
        break;
      }

    case 21:
      {
        // In case 21 the ball is in position to grab
        printf("Grabbing ball and moving somewhere else.\n");
        // Grab the ball putting the arm down
        snprintf(lines[0], MAX_LEN, "servo=2, pservo=-65, vservo=1, time=5.0");
        // Move to robot to another pose
        snprintf(lines[1], MAX_LEN, "servo=2, pservo=-65, vservo=0, vel=0.15, tr=0.0: turn=90, time=3.0");
        snprintf(lines[2], MAX_LEN, "servo=2, pservo=-65, vservo=0, vel=0.15: dist=0.2, time=5.0");
        // Stop
        snprintf(lines[3], MAX_LEN, "servo=2, pservo=-65, vservo=1, vel=0.0: time=5.0");
        // Move the servo up slowly to release the ball 
        snprintf(lines[4], MAX_LEN, "servo=2, pservo=500, vservo=1: time=5.0");
        // Move to robot to another pose
        snprintf(lines[5], MAX_LEN, "servo=2, pservo=500, vservo=0, vel=0.2, tr=0.0: turn=120, time=15.0");
        snprintf(lines[6], MAX_LEN, "servo=2, pservo=500, vservo=0, vel=0.3: dist=0.5, time=2.0");
        // Stop and set event 10
        snprintf(lines[7], MAX_LEN, "vel=0.0: time=1.0");
        snprintf(lines[8], MAX_LEN, "event=10: time=1.0");
        sendAndActivateSnippet(lines,sizeof(lines));
	
	      state = 998;
      }
      break;

    case 30:
      // This case with event 1 is made to wait for the robot to finish the lines sent to get another image.
      if (bridge->event->isEventSet(1))
      {
        printf("Event 1 is set.\n");
        state = 20;
      }
      break;

    case 998:
      // Finished!
      if (bridge->event->isEventSet(10))
      {
        state = 999;
      }
      break;
    case 999:
    default:
      printf("Mission 4 ended\n");
      bridge->send("oled 5 mission 4 ended.");
      finished = true;
      break;
  }
  return finished;
}


void UMission::openLog()
{
  // make logfile
  const int MNL = 100;
  char date[MNL];
  char name[MNL];
  UTime appTime;
  appTime.now();
  appTime.getForFilename(date);
  // construct filename ArUco
  snprintf(name, MNL, "log_mission_%s.txt", date);
  logMission = fopen(name, "w");
  if (logMission != NULL)
  {
    const int MSL = 50;
    char s[MSL];
    fprintf(logMission, "%% Mission log started at %s\n", appTime.getDateTimeAsString(s));
    fprintf(logMission, "%% Start mission %d end mission %d\n", fromMission, toMission);
    fprintf(logMission, "%% 1  Time [sec]\n");
    fprintf(logMission, "%% 2  mission number.\n");
    fprintf(logMission, "%% 3  mission state.\n");
  }
  else
    printf("#UCamera:: Failed to open image logfile\n");
}

void UMission::closeLog()
{
  if (logMission != NULL)
  {
    fclose(logMission);
    logMission = NULL;
  }
}
