#include "dryve_d1_gate/D1.hpp"

void D1::setDebugModeON()
{
  debug = true;
}

void D1::setDebugModeOFF()
{
  debug = false;
}

void D1::startConnection(std::string ipAddress, int port)
{
  // create socket
  if ((this->sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
    perror("socket failed");
    exit(EXIT_FAILURE);
  }
  else
  {
    std::cout << "Socket created!" << std::endl;
  }

  // Fill in a hint structure
  sockaddr_in hint{};
  hint.sin_family = AF_INET;
  hint.sin_port = htons(port);

  if (inet_pton(AF_INET, ipAddress.c_str(), &hint.sin_addr) <= 0)
  {
    perror("inet_pton failed");
    exit(EXIT_FAILURE);
  }
  else
  {
    std::cout << "inet_pton succeeded!" << std::endl;
  }

  // connect to server
  if (connect(sock, (sockaddr*) &hint, sizeof(hint)) < 0)
  {
    perror("connect failed");
    exit(EXIT_FAILURE);
  }
  else
  {
    std::cout << "Connected to the D1!" << std::endl;
  }
}

void D1::sendCommand(unsigned char data[], unsigned int arraySize, long value)
{
  unsigned char arrayOfByte[4];
  unsigned char recvbuffer[19];
  unsigned char handShake[] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  // Swap the object and subindex bytes of the handshake to the bytes of the send telegram
  handShake[12] = data[12];
  handShake[13] = data[13];
  handShake[14] = data[14];
  // Conversion of the entered integer value to 4 bytes
  memcpy(arrayOfByte, &value, sizeof value);
  data[19] = arrayOfByte[0];
  data[20] = arrayOfByte[1];
  data[21] = arrayOfByte[2];
  data[22] = arrayOfByte[3];

  int sendResult = send(sock, (char*) data, arraySize / sizeof(data[0]), 0);

  if (sendResult == arraySize)
  {
    // Wait for response
    std::memset(recvbuffer, 0, 19);
    int bytesReceived = recv(sock, (char*) recvbuffer, 19, 0);
    if (bytesReceived > 0)
    {
      if (debug == true)
      {
        std::cout << "Bytes received: " << bytesReceived << std::endl;
        for (int i = 0; i < bytesReceived; i++)
        {
          // Echo response to console
          printf("%d ", recvbuffer[i]);
        }
        std::cout << std::endl;
      }
      while (std::equal(std::begin(recvbuffer), std::end(recvbuffer), std::begin(handShake)) != true)
      {
        std::cout << "Wait for Handshake" << std::endl;
      }
      if (debug == true)
      {
        std::cout << "Telegram send correctly!" << std::endl;
      }
    }
  }
  else
  {
    perror("Can't send telegram to D1, Err: ");
    exit(EXIT_FAILURE);
  }
}

void D1::sendSetCommand(const unsigned char data[], unsigned int arraySize)
{
  // Send of const telegrams
  unsigned char recvbuffer[19];
  unsigned char handShake[] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  // Swap the object and subindex bytes of the handshake to the bytes of the send telegram
  handShake[12] = data[12];
  handShake[13] = data[13];
  handShake[14] = data[14];

  int sendResult = send(sock, (char*) data, arraySize / sizeof(data[0]), 0);

  if (sendResult == arraySize)
  {
    // Wait for response
    std::memset(recvbuffer, 0, 19);
    int bytesReceived = recv(sock, (char*) recvbuffer, 19, 0);
    if (bytesReceived > 0)
    {
      if (debug == true)
      {
        std::cout << "Bytes received: " << bytesReceived << std::endl;
        for (int i = 0; i < bytesReceived; i++)
        {
          // Print response to console for debugging
          printf("%d ", recvbuffer[i]);
        }
        std::cout << std::endl;
      }
      while (std::equal(std::begin(recvbuffer), std::end(recvbuffer), std::begin(handShake)) != true)
      {
        std::cout << "Wait for Handshake" << std::endl;
      }

      if (debug == true)
      {
        std::cout << "Telegram send correctly!" << std::endl;
      }
    }
  }
  else
  {
    perror("Can't send telegram to D1, Err: ");
    exit(EXIT_FAILURE);
  }
}

void D1::readCommand(const unsigned char data[], unsigned int arraySize)
{
  int sendResult = send(sock, (char*) data, arraySize / sizeof(data[0]), 0);

  if (sendResult == arraySize)
  {
    // Wait for response
    std::memset(recvbuf, 0, sizeof(recvbuf));
    int bytesReceived = recv(sock, (char*) recvbuf, sizeof(recvbuf), 0);
    if (bytesReceived > 0)
    {
      if (debug == true)
      {
        std::cout << "Bytes received: " << bytesReceived << std::endl;
        for (int i = 0; i < bytesReceived; i++)
        {
          // Echo response to console
          printf("%d ", recvbuf[i]);
        }
        std::cout << std::endl;
      }
    }
  }
  else
  {
    perror("Can't send telegram to D1, Err: ");
    exit(EXIT_FAILURE);
  }
}

int D1::readObjectValue(char objectindex1, char objectindex2, int subindex)
{
  readBuffer[12] = int(objectindex1);
  readBuffer[13] = int(objectindex2);
  readBuffer[14] = subindex;

  int sendResult = send(sock, (char*) readBuffer, 19 / sizeof(readBuffer[0]), 0);

  std::cout << "sendResult: " << sendResult << std::endl;
  std::cout << "sizeof(readBuffer): " << sizeof(readBuffer) << std::endl;

  if (sendResult == sizeof(readBuffer))
  {
    // Wait for response
    std::memset(recvbuf, 0, sizeof(recvbuf));
    int bytesReceived = recv(sock, (char*) recvbuf, sizeof(recvbuf), 0);
    if (bytesReceived > 0)
    {
      if (debug == true)
      {
        std::cout << "Bytes received: " << bytesReceived << std::endl;
        for (int i = 0; i < bytesReceived; i++)
        {
          // Echo response to console
          printf("%d ", recvbuf[i]);
        }
        std::cout << std::endl;
      }

      int x = fourBytesToInt(recvbuf);
      return x;
    }
  }
  else
  {
    perror("Can't send telegram to D1, Err: ");
    exit(EXIT_FAILURE);
  }
}

float D1::getSIUnitFactor()
{
  readCommand(readSIUnitFactor, sizeof(readSIUnitFactor));
  // Read the SI Unit Position calculation of the multiplication factor when linear movement(byte 2 == 01h) is set;

  // for further informations please see manual chapter "Detailed description Motion Control Object" Object 60A8h and
  // Object 6092h
  if (recvbuf[21] == 1)
  {
    // Equation to calculate the multiplication factor from the recieved byte 3 of object 60A8h
    if (recvbuf[22] < 5)
    {
      float siUnitFactor = (std::pow(10, -3) / std::pow(10, recvbuf[22]));
      return siUnitFactor;
    }
    else
    {
      float siUnitFactor = (std::pow(10, -3) / std::pow(10, recvbuf[22] - 256));
      return siUnitFactor;
    }
  }
  // Read the SI Unit Positionand calculation of the multiplication factor when rotary movement(byte 2 == 41h) is set;
  // for further informations please see manual chapter "Detailed description Motion Control Object" Object 60A8h and
  // Object 6092h
  else if (recvbuf[21] == 65)
  {
    // Equation to calculate the multiplication factor from the recieved byte 3 of object 60A8h
    if (recvbuf[22] < 5)
    {
      float siUnitFactor = (1 / std::pow(10, recvbuf[22]));
      return siUnitFactor;
    }
    else
    {
      float siUnitFactor = (1 / std::pow(10, recvbuf[22] - 256));
      return siUnitFactor;
    }
  }
}

int D1::fourBytesToInt(unsigned char data[])
{
  // Conversion from 4 received bytes to integer
  int intValue;
  unsigned char buffer[4]{};
  buffer[0] = data[19];
  buffer[1] = data[20];
  buffer[2] = data[21];
  buffer[3] = data[22];
  memcpy(&intValue, buffer, sizeof(int));
  return intValue;
}

void D1::checkForDryveError()
{
  readCommand(readStatusWord, sizeof(readStatusWord));
  if (std::equal(std::begin(statusError), std::end(statusError), std::begin(recvbuf)) ||
      std::equal(std::begin(statusError2), std::end(statusError2), std::begin(recvbuf)) ||
      std::equal(std::begin(statusError3), std::end(statusError3), std::begin(recvbuf)) ||
      std::equal(std::begin(statusError4), std::end(statusError4), std::begin(recvbuf)))
  {
    // Read the Error Code and print it in the console
    int errorCode = readObjectValue(0x60, 0x3f);
    std::string error;
    if (errorCode == 25376)
    {
      error = "E01 Error Configuration";
    }
    if (errorCode == 8992)
    {
      error = "E02 Motor Over-Current";
    }
    if (errorCode == 8977)
    {
      error = "E03 Encoder Over-Current";
    }
    if (errorCode == 8978)
    {
      error = "E04 10 V Output Over Current";
    }
    if (errorCode == 20756)
    {
      error = "E05 I/O Supply Low";
    }
    if (errorCode == 12834)
    {
      error = "E06 Logic Supply Low";
    }
    if (errorCode == 12562)
    {
      error = "E07 Logic Supply High";
    }
    if (errorCode == 12833)
    {
      error = "E08 Load Supply Low";
    }
    if (errorCode == 12817)
    {
      error = "E09 Load Supply High";
    }
    if (errorCode == 17168)
    {
      error = "E10 Temperature High";
    }
    if (errorCode == 34321)
    {
      error = "E11 Following Error";
    }
    if (errorCode == 65280)
    {
      error = "E12 Limit Switch";
    }
    if (errorCode == 29446)
    {
      error = "E13 Hall Sensor";
    }
    if (errorCode == 29445)
    {
      error = "E14 Encoder";
    }
    if (errorCode == 65281)
    {
      error = "E15 Encoder Channel A";
    }
    if (errorCode == 65282)
    {
      error = "E16 Encoder Channel B";
    }
    if (errorCode == 65283)
    {
      error = "E17 Encoder Channel I";
    }
    if (errorCode == 28944)
    {
      error = "E21 Braking Resistor Overload";
    }
    std::cerr << "ERROR: " << error << std::endl;
    exit(1);
  }
}

void D1::waitForReady()
{
  do
  {
    readCommand(readStatusWord, sizeof(readStatusWord));
    checkForDryveError();
    if (debug == true)
    {
      std::cout << "Waiting for the Movement to be finished!" << std::endl;
    }

  } while (std::equal(std::begin(statusReady), std::end(statusReady), std::begin(recvbuf)) != true &&
           std::equal(std::begin(statusReady2), std::end(statusReady2), std::begin(recvbuf)) != true &&
           std::equal(std::begin(statusReady5), std::end(statusReady5), std::begin(recvbuf)) != true &&
           std::equal(std::begin(statusReady6), std::end(statusReady6), std::begin(recvbuf)) != true);
}

void D1::waitForHoming()
{
  do
  {
    readCommand(readStatusWord, sizeof(readStatusWord));
    checkForDryveError();
    if (debug == true)
    {
      std::cout << "Waiting for the Homing to be finished!" << std::endl;
    }

  } while (std::equal(std::begin(statusReady), std::end(statusReady), std::begin(recvbuf)) != true &&
           std::equal(std::begin(statusReady2), std::end(statusReady2), std::begin(recvbuf)) != true &&
           std::equal(std::begin(statusReady3), std::end(statusReady3), std::begin(recvbuf)) != true &&
           std::equal(std::begin(statusReady4), std::end(statusReady4), std::begin(recvbuf)) != true &&
           std::equal(std::begin(statusReady5), std::end(statusReady5), std::begin(recvbuf)) != true &&
           std::equal(std::begin(statusReady6), std::end(statusReady6), std::begin(recvbuf)) != true &&
           std::equal(std::begin(statusReady7), std::end(statusReady7), std::begin(recvbuf)) != true);
}

void D1::setShutdown()
{
  sendSetCommand(sendShutdown, sizeof(sendShutdown));
  do
  {
    readCommand(readStatusWord, sizeof(readStatusWord));
    if (debug == true)
    {
      std::cout << "Waiting for the Shutdown!" << std::endl;
    }

  } while (std::equal(std::begin(statusShutdown), std::end(statusShutdown), std::begin(recvbuf)) != true &&
           std::equal(std::begin(statusShutdown2), std::end(statusShutdown2), std::begin(recvbuf)) != true &&
           std::equal(std::begin(statusShutdown3), std::end(statusShutdown3), std::begin(recvbuf)) != true);
}

void D1::setSwitchOn()
{
  sendSetCommand(sendSwitchOn, sizeof(sendSwitchOn));
  do
  {
    readCommand(readStatusWord, sizeof(readStatusWord));
    if (debug == true)
    {
      std::cout << "Waiting for Switch On!" << std::endl;
    }

  } while (std::equal(std::begin(statusSwitchOn), std::end(statusSwitchOn), std::begin(recvbuf)) != true &&
           std::equal(std::begin(statusSwitchOn2), std::end(statusSwitchOn2), std::begin(recvbuf)) != true &&
           std::equal(std::begin(statusSwitchOn3), std::end(statusSwitchOn3), std::begin(recvbuf)) != true);
}

void D1::setOperationEnable()
{
  sendSetCommand(sendOperationEnable, sizeof(sendOperationEnable));
  do
  {
    readCommand(readStatusWord, sizeof(readStatusWord));
    if (debug == true)
    {
      std::cout << "Waiting for enable Operation!" << std::endl;
    }

  } while (
      std::equal(std::begin(statusOperationEnable), std::end(statusOperationEnable), std::begin(recvbuf)) != true &&
      std::equal(std::begin(statusOperationEnable2), std::end(statusOperationEnable2), std::begin(recvbuf)) != true &&
      std::equal(std::begin(statusOperationEnable3), std::end(statusOperationEnable3), std::begin(recvbuf)) != true);
}

void D1::resetStatus()
{
  sendSetCommand(resetDryveStatus, sizeof(resetDryveStatus));
}

void D1::runStateMachine()

{
  sendSetCommand(sendResetError, sizeof(sendResetError));
  sendSetCommand(sendResetArray, sizeof(sendResetArray));
  checkForDryveError();
  resetStatus();
  setShutdown();
  setSwitchOn();
  setOperationEnable();
}

void D1::setModeOfOperation(unsigned char mode)
{
  unsigned char statusModeDisplay[] = {0, 0, 0, 0, 0, 14, 0, 43, 13, 0, 0, 0, 96, 97, 0, 0, 0, 0, 1, mode};
  sendModeOfOperation[19] = mode;
  sendSetCommand(sendModeOfOperation, sizeof(sendModeOfOperation));
  do
  {
    readCommand(readModesDisplay, sizeof(readModesDisplay));
    if (debug == true)
    {
      std::cout << "Waiting for the Mode of Operation to be set!" << std::endl;
    }

  } while (std::equal(std::begin(statusModeDisplay), std::end(statusModeDisplay), std::begin(recvbuf)) != true);
}

void D1::homing(float switchVelo, float zeroVelo, float homingAcc)
{
  setModeOfOperation(6);
  float siFactor = getSIUnitFactor();
  long switchVelocity = switchVelo * siFactor;
  long zeroVelocity = zeroVelo * siFactor;
  long homingAccel = homingAcc * siFactor;
  sendCommand(sendSwitchVelocity, sizeof(sendSwitchVelocity), switchVelocity);
  sendCommand(sendZeroVelocity, sizeof(sendZeroVelocity), zeroVelocity);
  sendCommand(sendHomingAccelration, sizeof(sendHomingAccelration), homingAccel);

  // Checks if the D1 is in an error state
  checkForDryveError();

  // Start Movement and toggle back bit 4
  sendSetCommand(sendResetStart, sizeof(sendResetStart));
  sendSetCommand(sendStartMovement, sizeof(sendStartMovement));

  // Wait for homing to end
  waitForHoming();
}

void D1::profilePositionAbs(float position, float velo, float accel, float decel)
{
  setModeOfOperation(1);
  float siFactor = getSIUnitFactor();
  long pos = position * siFactor;
  long velocity = velo * siFactor;
  long acc = accel * siFactor;
  long dec = decel * siFactor;
  sendCommand(sendTargetPosition, sizeof(sendTargetPosition), pos);
  sendCommand(sendProfileVelocity, sizeof(sendProfileVelocity), velocity);
  sendCommand(sendProfileAcceleration, sizeof(sendProfileAcceleration), acc);
  sendCommand(sendProfileDecceleration, sizeof(sendProfileDecceleration), dec);

  // Checks if the D1 is in an error state
  checkForDryveError();

  // Start Movement and toggle back bit 4
  sendSetCommand(sendResetStartAbs, sizeof(sendResetStartAbs));
  sendSetCommand(sendStartMovement, sizeof(sendStartMovement));

  // Wait for Movement to end
  waitForReady();
}

void D1::profilePositionRel(float position, float velo, float accel, float decel)
{
  setModeOfOperation(1);
  float siFactor = getSIUnitFactor();
  long pos = position * siFactor;
  long velocity = velo * siFactor;
  long acc = accel * siFactor;
  long dec = decel * siFactor;
  sendCommand(sendTargetPosition, sizeof(sendTargetPosition), pos);
  sendCommand(sendProfileVelocity, sizeof(sendProfileVelocity), velocity);
  sendCommand(sendProfileAcceleration, sizeof(sendProfileAcceleration), acc);
  sendCommand(sendProfileDecceleration, sizeof(sendProfileDecceleration), dec);

  // Checks if the D1 is in an error state
  checkForDryveError();

  // Start Movement and toggle back bit 4
  sendSetCommand(sendResetStartRel, sizeof(sendResetStartRel));
  sendSetCommand(sendStartMovementRel, sizeof(sendStartMovementRel));

  // Wait for Movement to end
  waitForReady();
}

void D1::profileVelocity(float velo, float accel, float decel)
{
  setModeOfOperation(3);
  float siFactor = getSIUnitFactor();
  long velocity = velo * siFactor;
  long acc = accel * siFactor;
  long dec = decel * siFactor;
  sendCommand(sendProfileAcceleration, sizeof(sendProfileAcceleration), acc);
  sendCommand(sendProfileDecceleration, sizeof(sendProfileDecceleration), dec);

  // Checks if the D1 is in an error state
  checkForDryveError();

  // Start movement by sending a target velocity value != 0 (0 for stop)
  sendCommand(sendTargetVelocity, sizeof(sendTargetVelocity), velocity);
}
