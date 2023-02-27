#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iterator>
#include <string>
using namespace std;

#pragma once
class D1
{
 private:
  // Define Telegrams for frequently used objects
  // State machine
  const unsigned char sendShutdown[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 6, 0};
  const unsigned char sendSwitchOn[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 7, 0};
  const unsigned char sendOperationEnable[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 15, 0};
  const unsigned char resetDryveStatus[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 0, 1};

  // Telegrams for resetting the dryve status
  const unsigned char sendResetError[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 0, 1};
  const unsigned char sendResetArray[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 143, 0};

  // Telegrams to read status and values of objects
  const unsigned char readStatusWord[19] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2};
  const unsigned char readModesDisplay[19] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 97, 0, 0, 0, 0, 1};
  const unsigned char readPositionActualValue[19] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 100, 0, 0, 0, 0, 4};
  const unsigned char readVelocityActualValue[19] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 108, 0, 0, 0, 0, 4};
  const unsigned char readCurrentActualValue[19] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 120, 0, 0, 0, 0, 4};
  const unsigned char readFollowingErrorActualValue[19] = {
      0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 244, 0, 0, 0, 0, 4};
  const unsigned char readErrorCode[19] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 63, 0, 0, 0, 0, 2};
  const unsigned char readSIUnitFactor[19] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 168, 0, 0, 0, 0, 4};
  const unsigned char readControllerTemp[19] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 32, 19, 0, 0, 0, 0, 4};
  unsigned char readBuffer[19] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4};

  // Telegrams for initial parameters
  unsigned char sendFeedRate[23] = {0, 0, 0, 0, 0, 17, 0, 43, 13, 1, 0, 0, 96, 146, 1, 0, 0, 0, 4, 0, 0, 0, 0};
  unsigned char sendShaftRevolutions[23] = {0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 96, 146, 2, 0, 0, 0, 1, 1};
  unsigned char sendSIUnitFactor[23] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 168, 0, 0, 0, 0, 4, 0, 0, 0, 0};

  // Telegrams to set the mode of operation
  unsigned char sendModeOfOperation[20] = {0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 96, 96, 0, 0, 0, 0, 1, 0};

  // Telegrams for homing/referencing
  const unsigned char sendModeHoming[20] = {0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 96, 96, 0, 0, 0, 0, 1, 6};
  unsigned char sendSwitchVelocity[23] = {0, 0, 0, 0, 0, 17, 0, 43, 13, 1, 0, 0, 96, 153, 1, 0, 0, 0, 4, 0, 0, 0, 0};
  unsigned char sendZeroVelocity[23] = {0, 0, 0, 0, 0, 17, 0, 43, 13, 1, 0, 0, 96, 153, 2, 0, 0, 0, 4, 0, 0, 0, 0};
  unsigned char sendHomingAccelration[23] = {0, 0, 0, 0, 0, 17, 0, 43, 13, 1, 0, 0, 96, 154, 0, 0, 0, 0, 4, 0, 0, 0, 0};

  // Telegrams for Profile Position Mode and Profile Velocity Mode
  const unsigned char sendModeProfilePosition[20] = {0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 96, 96, 0, 0, 0, 0, 1, 1};
  const unsigned char sendModeProfileVelocity[20] = {0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 96, 96, 0, 0, 0, 0, 1, 3};
  unsigned char sendProfileVelocity[23] = {0, 0, 0, 0, 0, 17, 0, 43, 13, 1, 0, 0, 96, 129, 0, 0, 0, 0, 4, 0, 0, 0, 0};
  unsigned char sendProfileAcceleration[23] = {0,  0,   0, 0, 0, 17, 0, 43, 13, 1, 0, 0,
                                               96, 131, 0, 0, 0, 0,  4, 0,  0,  0, 0};
  unsigned char sendProfileDecceleration[23] = {0,  0,   0, 0, 0, 17, 0, 43, 13, 1, 0, 0,
                                                96, 132, 0, 0, 0, 0,  4, 0,  0,  0, 0};
  unsigned char sendTargetVelocity[23] = {0, 0, 0, 0, 0, 17, 0, 43, 13, 1, 0, 0, 96, 255, 0, 0, 0, 0, 4, 0, 0, 0, 0};
  unsigned char sendTargetPosition[23] = {0, 0, 0, 0, 0, 17, 0, 43, 13, 1, 0, 0, 96, 122, 0, 0, 0, 0, 4, 0, 0, 0, 0};

  // Telegrams to start a movement
  const unsigned char sendStartMovement[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 31, 0};
  const unsigned char sendStartMovementRel[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 95, 0};
  const unsigned char sendResetStart[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 15, 0};
  const unsigned char sendResetStartAbs[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 15, 0};
  const unsigned char sendResetStartRel[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 79, 0};

  // Status Word to check if the homing or the movement was completet correctly
  const unsigned char statusReady[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22};
  const unsigned char statusReady2[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 8, 6};
  const unsigned char statusReady3[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 8, 2};
  const unsigned char statusReady4[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 8, 34};
  const unsigned char statusReady5[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 64, 22};
  const unsigned char statusReady6[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 64, 18};
  const unsigned char statusReady7[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 64, 2};

  // Status Word to that indicates that an error occured
  const unsigned char statusError[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 8, 6};
  const unsigned char statusError2[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 8, 38};
  const unsigned char statusError3[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 8, 16};
  const unsigned char statusError4[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 8, 22};

  // Status Word to check if the shutdown was done correctly
  const unsigned char statusShutdown[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 33, 6};
  const unsigned char statusShutdown2[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 33, 22};
  const unsigned char statusShutdown3[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 33, 2};

  // Status Word to check if the switch on was done correctly
  const unsigned char statusSwitchOn[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 35, 6};
  const unsigned char statusSwitchOn2[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 35, 22};
  const unsigned char statusSwitchOn3[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 35, 2};

  // Status Word to check if the operation enable was done correctly
  const unsigned char statusOperationEnable[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 6};
  const unsigned char statusOperationEnable2[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 0, 0,
                                                    0, 96, 65, 0, 0, 0,  0, 2,  39, 22};
  const unsigned char statusOperationEnable3[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 0, 0,
                                                    0, 96, 65, 0, 0, 0,  0, 2,  39, 2};

  // Variable to activate and deactivate the debug mode
  bool debug;

  // Buffer variable to store the received telegram from the D1
  unsigned char recvbuf[23];

  void startConnection(std::string ipAddress, int port);   // Function to establish the connection with the D1
  int fourBytesToInt(unsigned char data[]);                // Function to convert 4 bytes to an integer value

 public:
  int sock = 0;

  D1(std::string ipAddress, int port)
  {
    startConnection(ipAddress, port);
  }

  // Functions are defined in the D1.cpp file
  void setDebugModeON();    // Function to activate the debug mode
  void setDebugModeOFF();   // Function to deactivate the debug mode
  void sendCommand(unsigned char data[],
                   unsigned int arraySize,
                   long value);   // Function to send an integer Value into a given object
  void sendSetCommand(const unsigned char data[],
                      unsigned int arraySize);   // Function to send constant telegrams (e.g. "Shutdown","Switch On" and
                                                 // "Operation Enable")
  void readCommand(
      const unsigned char data[],
      unsigned int arraySize);   // Function to read an object and store the telegram it in the variable recvbuf[]
  void checkForDryveError();     // Function that checks if the D1 is in an error state
  void waitForReady();           // Function that waits as long as the dryve is not back to the "Ready" State
  void waitForHoming();          // Functions that waits for the referencing/homing to be done
  void resetStatus();            // Function that resets the dryve status
  void setShutdown();            // Functions that send the telegram to start the Shutdown
  void setSwitchOn();            // Functions that send the telegram to start the SwitchOn
  void setOperationEnable();     // Functions that send the telegram to start the Operation Enable
  void runStateMachine();        // Function that runs through the whole State machine
  void setModeOfOperation(unsigned char mode);   // Function that set the Mode of Operation
  void homing(
      float switchVelo,
      float zeroVelo,
      float homingAcc);   // Function "Homing"; executing a homing with given velocity for switch, velocity for zero and
                          // homing acceleration; mode of homing and offset has to be set beforehand in the GUI
  void profilePositionAbs(float position,
                          float velo,
                          float accel,
                          float decel = 0);   // Function "Profile Position Mode"; Move to an absolute position with
                                              // given velocity, acceleration and deceleration
  void profilePositionRel(float position,
                          float velo,
                          float accel,
                          float decel = 0);   // Function "Profile Position Mode"; Move to an relative position with
                                              // given velocity, acceleration and deceleration
  void profileVelocity(
      float velo,
      float accel,
      float decel = 0);   // Function "Profile Position Mode"; Move with a set target velocity, acceleration and
                          // deceleration; Movement starts directly when the target velocity is not 0
  int readObjectValue(char objectindex1,
                      char objectindex2,
                      int subindex = 0);   // Function that reads the value of a given object and returns its value
  float getSIUnitFactor();   // Function to get the SI Unit Factor, which is needed to convert object values to mm|� or
                             // mm/s|�/s or mm/s�|�/s�
};
