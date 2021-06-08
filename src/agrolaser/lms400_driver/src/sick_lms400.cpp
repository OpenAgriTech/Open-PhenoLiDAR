/*
 * Copyright (c) 2009 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
 * Code based on the LGPL Player SICK LMS400 driver by Nico Blodow and Radu Bogdan Rusu
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#include <lms400_driver/sick_lms400.h>
#include <angles/angles.h>

const int CMD_BUFFER_SIZE = 255;


////////////////////////////////////////////////////////////////////////////////
// Constructor.
lms400_driver::sick_lms_400::sick_lms_400 (const char* host, int port, int debug_mode)
{
  portno_   = port;
  hostname_ = host;
  verbose_  = debug_mode;
  memset (command_, 0, BUF_SIZE);
  MeasurementQueue_ = new std::vector<MeasurementQueueElement_t >;
}

////////////////////////////////////////////////////////////////////////////////
// Connect to the LMS400 unit using hostname:portno
// Returns 0 if connection was successful, -1 otherwise
int
  lms400_driver::sick_lms_400::Connect ()
{
  // Create a socket
  sockfd_ = socket (AF_INET, SOCK_STREAM, 0);
  if (sockfd_ < 0)
    return (-1);

  // Get the network host entry
  memset (&serv_addr_, 0, sizeof (serv_addr_));
  serv_addr_.sin_port = htons (portno_);
  serv_addr_.sin_family = AF_INET;
#if defined (HAVE_GETADDRINFO)
  addr_ptr = NULL;
  if (getaddrinfo (hostname_, NULL, NULL, &(addr_ptr)))
  {
    ROS_ERROR ("getaddrinfo() failed with error");
    return (-1);
  }
  assert (addr_ptr);
  assert (addr_ptr->ai_addr);
  if ((addr_ptr->ai_addr->sa_family) != AF_INET)
  {
    ROS_ERROR ("unsupported internet address family");
    return (-1);
  }
  serv_addr.sin_addr.s_addr = (reinterpret_cast<struct sockaddr_in*> (addr_ptr->ai_addr))->sin_addr.s_addr;
  freeaddrinfo (addr_ptr);
  addr_ptr = NULL;
#else
  server_ = gethostbyname (hostname_);
  if ((server_) == NULL)
    return (-1);
  memcpy (&(serv_addr_.sin_addr.s_addr), server_->h_addr, server_->h_length);
#endif

  // Attempt to connect
  if (connect (sockfd_, reinterpret_cast<struct sockaddr*> (&serv_addr_), sizeof (serv_addr_)) < 0)
    return (-1);

  return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Disconnect from the LMS400 unit
// Returns 0 if connection was successful, -1 otherwise
int
  lms400_driver::sick_lms_400::Disconnect ()
{
  return (close (sockfd_));
}

////////////////////////////////////////////////////////////////////////////////
// Enable/Disable extended RIS (Remission Information System) detectivity
int
  lms400_driver::sick_lms_400::EnableRIS (int onoff)
{
  char cmd[CMD_BUFFER_SIZE];
  snprintf (cmd, CMD_BUFFER_SIZE, "sWN MDblex %i", onoff);
  SendCommand (cmd);

  if (ReadAnswer () != 0)
    return (-1);
  ExtendedRIS_ = onoff;
  return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Set the encoder type (0-4)
int
  lms400_driver::sick_lms_400::SetEncoderType (int encoder_type)
{
  char cmd[CMD_BUFFER_SIZE];
  snprintf (cmd, CMD_BUFFER_SIZE, "sWN IOencm %i", encoder_type);
  SendCommand (cmd);

  if (ReadAnswer () != 0)
    return (-1);
  EncoderType_ = encoder_type;
  return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Set the mean filter parameters
int
  lms400_driver::sick_lms_400::SetMeanFilterParameters (int num_scans)
{
  char cmd[CMD_BUFFER_SIZE];
  snprintf (cmd, CMD_BUFFER_SIZE, "sWN FLmean 0 %i", num_scans);
  SendCommand (cmd);

  if (ReadAnswer () != 0)
    return (-1);
  MeanFilterNumScans_ = num_scans;
  return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Set the range filter parameters
int
  lms400_driver::sick_lms_400::SetRangeFilterParameters (float range_min, float range_max)
{
  char cmd[CMD_BUFFER_SIZE];
  snprintf (cmd, CMD_BUFFER_SIZE, "sWN FLrang %+f %+f", (float)range_min, (float)range_max);
  SendCommand (cmd);

  if (ReadAnswer () != 0)
    return (-1);
  RangeFilterBottomLimit_ = range_min;
  RangeFilterTopLimit_    = range_max;
  return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Enable filters using a filter mask
int
  lms400_driver::sick_lms_400::EnableFilters (int filter_mask)
{
  char cmd[CMD_BUFFER_SIZE];
  snprintf (cmd, CMD_BUFFER_SIZE, "sWN FLsel %+i", filter_mask);
  SendCommand (cmd);

  if (ReadAnswer () != 0)
    return (-1);
  FilterMask_ = filter_mask;
  return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Takes a string containing an ip adress and returns an array of 4 u_chars
unsigned char*
  lms400_driver::sick_lms_400::ParseIP (char* ip)
{
  char* tmp = (char*) malloc (strlen (ip) + 1);
  unsigned char* _ip = (unsigned char*) malloc (4);

  strcpy (tmp, ip);
  _ip[0] = atoi (strtok (tmp, "."));
  for (int i = 1; i < 4; i++)
    _ip[i] = atoi (strtok (NULL, "."));

  free (tmp);
  return (_ip);
}

////////////////////////////////////////////////////////////////////////////////
// Set the desired userlevel by logging in with the appropriate password
int
  lms400_driver::sick_lms_400::SetUserLevel (int8_t userlevel, const char* password)
{
  char cmd[CMD_BUFFER_SIZE];
  snprintf (cmd, CMD_BUFFER_SIZE, "sMN SetAccessMode %d %s", userlevel, password);
  SendCommand (cmd);
  return (ReadConfirmationAndAnswer ());
}

////////////////////////////////////////////////////////////////////////////////
// Fills string pointed to by macadress with the MAC adress read from the sensor
int
  lms400_driver::sick_lms_400::GetMACAddress (char** macaddress)
{
  char *mac = (char*) malloc (20);
  int index = 0;
  char* tmp;

  SendCommand ("sRN EImac ");
  if (ReadAnswer () != 0)
    return (-1);

  strtok ((char*) buffer_, " ");
  strtok (NULL, " ");

  for (int i = 0; i < 6; i++)
  {
    tmp = strtok (NULL, "-");
    strncpy (&mac[index], tmp, 2);
    index += 2;
    mac[index++] = ':';
  }

  mac[--index] = 0;
  *macaddress = mac;
  return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Set the IP address of the LMS400
int
  lms400_driver::sick_lms_400::SetIP (char* ip)
{
  unsigned char* ip_str;
  ip_str = ParseIP (ip);
  char cmd[80];

  snprintf (cmd, 80, "sWN EIip %X %X %X %X", ip_str[0], ip_str[1], ip_str[2], ip_str[3]);
  free (ip_str);
  SendCommand (cmd);

  return (ReadAnswer ());
}

////////////////////////////////////////////////////////////////////////////////
// Set the gateway address for the Ethernet interface
int
  lms400_driver::sick_lms_400::SetGateway (char* gw)
{
  unsigned char* gw_str;
  gw_str = ParseIP (gw);
  char cmd[CMD_BUFFER_SIZE];

  snprintf (cmd, CMD_BUFFER_SIZE, "sWN EIgate %X %X %X %X", gw_str[0], gw_str[1], gw_str[2], gw_str[3]);
  free (gw_str);
  SendCommand (cmd);

  return (ReadAnswer ());
}

////////////////////////////////////////////////////////////////////////////////
// Set the subnet mask for the Ethernet interface
int
  lms400_driver::sick_lms_400::SetNetmask (char* mask)
{
  unsigned char* mask_str;
  mask_str = ParseIP (mask);
  char cmd[CMD_BUFFER_SIZE];

  snprintf (cmd, CMD_BUFFER_SIZE, "sWN EImask %X %X %X %X", mask_str[0], mask_str[1], mask_str[2], mask_str[3]);
  free (mask_str);
  SendCommand (cmd);

  return (ReadAnswer ());
}

////////////////////////////////////////////////////////////////////////////////
// Set port for TCP/IP communication
int
  lms400_driver::sick_lms_400::SetPort (uint16_t port)
{
  char cmd[CMD_BUFFER_SIZE];

  snprintf (cmd,CMD_BUFFER_SIZE, "sWN EIport %04X", port);
  SendCommand (cmd);

  return (ReadAnswer ());
}

////////////////////////////////////////////////////////////////////////////////
// Reset the LMS400 unit
int
  lms400_driver::sick_lms_400::ResetDevice ()
{
  const char* cmd = "sMN mDCreset ";
  SendCommand (cmd);

  return (ReadAnswer ());
}

////////////////////////////////////////////////////////////////////////////////
// Terminate configuration and change back to userlevel 0
int
  lms400_driver::sick_lms_400::TerminateConfiguration ()
{
  const char* cmd = "sMN Run";
  SendCommand (cmd);

  return (ReadConfirmationAndAnswer ());
}

////////////////////////////////////////////////////////////////////////////////
// Set the laser angular resolution. Requires userlevel 2. Unused for now.
int
  lms400_driver::sick_lms_400::SetAngularResolution (const char* password, float ang_res,
                                                 float angle_start, float angle_range)
{
  char cmd[CMD_BUFFER_SIZE];
  snprintf (cmd, CMD_BUFFER_SIZE, "sMN mSCconfigbyang 04 %s %+f 01 %+f %+f",
           password, ang_res, angle_start, angle_range);
  SendCommand (cmd);

  return (ReadConfirmationAndAnswer ());
}

////////////////////////////////////////////////////////////////////////////////
// Set the laser scanning frequency. Requires userlevel 2. Unused for now.
int
  lms400_driver::sick_lms_400::SetScanningFrequency (const char* password, float freq,
                                    float angle_start, float angle_range)
{
  char cmd[CMD_BUFFER_SIZE];
  snprintf (cmd, CMD_BUFFER_SIZE, "sMN mSCconfigbyfreq 04 %s %+f 01 %+f %+f",
           password, freq, angle_start, angle_range);
  SendCommand (cmd);

  return (ReadConfirmationAndAnswer ());
}

////////////////////////////////////////////////////////////////////////////////
// Set both resolution and frequency without going to a higher user level (?)
int
  lms400_driver::sick_lms_400::SetResolutionAndFrequency (float freq, float ang_res,
                                         float angle_start, float angle_range)
{
  char cmd[CMD_BUFFER_SIZE];
  snprintf (cmd, CMD_BUFFER_SIZE, "sMN mSCsetscanconfig %+.2f %+.2f %+.2f %+.2f",
    freq, ang_res, angle_start, angle_range);
  SendCommand (cmd);

  int error = ReadConfirmationAndAnswer ();

  // If no error, parse the results
  if (error == 0)
  {
    strtok ((char*)buffer_, " "); strtok (NULL, " ");
    int ErrorCode = strtol (strtok (NULL, " "), NULL, 16);
    long int sf = strtol (strtok (NULL, " "), NULL, 16);
    long int re = strtol (strtok (NULL, " "), NULL, 16);

    if ((ErrorCode != 0) && (verbose_ > 0))
      printf (">> Warning: got an error code %d\n", ErrorCode);

    scanning_frequency_ = sf;
    resolution_         = re;

    if (verbose_ > 0)
      printf (">> Measured value quality is: %ld [5-10]\n",
        strtol (strtok (NULL, " "), NULL, 16));
  }

  return (error);
}

////////////////////////////////////////////////////////////////////////////////
// Start a measurement for both distance and intensity or just distance.
int
  lms400_driver::sick_lms_400::StartMeasurement (bool intensity)
{
  char cmd[CMD_BUFFER_SIZE];
  if (intensity)
    snprintf (cmd, CMD_BUFFER_SIZE, "sMN mLRreqdata %x", 0x20);
  else
    snprintf (cmd, CMD_BUFFER_SIZE, "sMN mLRreqdata %x", 0x21);

  SendCommand (cmd);

  return (ReadConfirmationAndAnswer ());
}

////////////////////////////////////////////////////////////////////////////////
// Read a measurement
sensor_msgs::LaserScan
  lms400_driver::sick_lms_400::ReadMeasurement (std_msgs::UInt16 &encoder_position_)
{
  sensor_msgs::LaserScan scan;

  char cs_read = 0, cs_calc = 0;
  int length  = 0;
  int current = 0;

  memset (buffer_, 0, 256);
  if (!MeasurementQueue_->empty ())
  {
    if (verbose_ > 0)
      ROS_DEBUG (">>> Reading from queue...\n");
    memcpy (buffer_, (char*) MeasurementQueue_->front ().string, MeasurementQueue_->front ().length + 1);
    free (MeasurementQueue_->front ().string);
    MeasurementQueue_->erase (MeasurementQueue_->begin ());
  }
  else
  {
    if (verbose_ == 2)
      ROS_DEBUG (">>> Queue empty. Reading from socket...\n");
    n_ = read (sockfd_, buffer_, 8);
    if (n_ < 0)
    {
      if (verbose_ > 0)
        ROS_DEBUG (">>> E: error reading from socket!\n");
      return (scan);
    }
    if (buffer_[0] != 0x02 || buffer_[1] != 0x02 || buffer_[2] != 0x02 || buffer_[3] != 0x02)
    {
      if (verbose_ > 0)
        ROS_DEBUG (">>> E: error expected 4 bytes STX's!\n");
      n_ = read (sockfd_, buffer_, 255);
      return (scan);
    }

    // find message length
    length = ( (buffer_[4] << 24) | (buffer_[5] << 16) | (buffer_[6] <<  8) | (buffer_[7]) );
    do
    {
      n_ = read (sockfd_, &buffer_[current], length-current);
      current += n_;
    } while (current < length);

    // read checksum:
    int ret = read (sockfd_, &cs_read, 1);
    if (ret < 1)
    {
      ROS_ERROR ("LMS400 didnt get any data in read %d",ret);
      return (scan);
    }

    for (int i = 0; i < length; i++)
      cs_calc ^= buffer_[i];

    if (cs_calc != cs_read)
    {
      if (verbose_ > 0)
        ROS_WARN (">>> E: checksums do not match!\n");
      return (scan);
    }
  }

  // parse measurements header and fill in the configuration parameters
  MeasurementHeader_t meas_header;
  memcpy (&meas_header, (void *)buffer_, sizeof (MeasurementHeader_t));

  float min_angle  = meas_header.StartingAngle / 10000.0;
  float resolution = meas_header.AngularStepWidth / 10000.0;
  float max_angle  = ((float) meas_header.NumberMeasuredValues) * resolution + min_angle;
  //float scanning_frequency = meas_header.ScanningFrequency;

  if (verbose_ == 2)
    ROS_DEBUG (">>> Reading %d values from %f to %f", meas_header.NumberMeasuredValues, meas_header.StartingAngle / 10000.0,
               ((float) meas_header.NumberMeasuredValues) * resolution + min_angle);

  uint16_t distance = 0;
  uint8_t remission = 0;
  int index = sizeof (MeasurementHeader_t);
  uint16_t encoder_position = 0;

  // Fill in the appropriate values
  scan.angle_min       = angles::from_degrees (min_angle);
  scan.angle_max       = angles::from_degrees (max_angle);
  scan.angle_increment = angles::from_degrees (resolution);
  scan.range_min       = 0.7;
  scan.range_max       = 3.6;
  scan.ranges.resize (meas_header.NumberMeasuredValues);
  scan.intensities.resize (meas_header.NumberMeasuredValues);

  memcpy (&scan.scan_time, &buffer_[sizeof(MeasurementHeader_t) + meas_header.NumberMeasuredValues * 3 + 14], 2);

  // Parse the read buffer and copy values into our distance/intensity buffer
  for (int i = 0; i < meas_header.NumberMeasuredValues; i++)
  {
    if (meas_header.Format == 0x20 || meas_header.Format == 0x21)
    {
      memcpy (&distance, (void *)&buffer_[index], sizeof (uint16_t) );
      index += sizeof (uint16_t);
    }
    if (meas_header.Format == 0x20 || meas_header.Format == 0x22)
    {
      memcpy (&remission, (void *)&buffer_[index], sizeof (uint8_t) );
      index += sizeof (uint8_t);
    }
    scan.ranges[i]      = distance * meas_header.DistanceScaling / 1000.0;
    scan.intensities[i] = remission * meas_header.RemissionScaling;

    if (verbose_ == 2)
      ROS_DEBUG (" >>> [%i] dist: %i\t remission: %i", i, distance * meas_header.DistanceScaling, remission * meas_header.RemissionScaling);
  }

  index += 3 * sizeof (uint16_t);
  memcpy (&encoder_position, (void *)&buffer_[index], sizeof (uint16_t) );
  encoder_position_.data = encoder_position;
  index += sizeof (uint16_t);

  if (verbose_ == 2)
    ROS_DEBUG (">>> Encoder position: %d", encoder_position);

  scan.header.frame_id = "lms400_base";
  scan.header.stamp = ros::Time::now ();

  return (scan);
}

////////////////////////////////////////////////////////////////////////////////
// Stop a measurement
int
  lms400_driver::sick_lms_400::StopMeasurement ()
{
  char cmd[CMD_BUFFER_SIZE];
  snprintf (cmd, CMD_BUFFER_SIZE, "sMN mLRstopdata");
  SendCommand (cmd);

  return (ReadConfirmationAndAnswer ());
}

////////////////////////////////////////////////////////////////////////////////
// Send a command to the laser unit. Returns -1 on error.
int
  lms400_driver::sick_lms_400::SendCommand (const char* cmd)
{
  if (verbose_ > 0)
    ROS_DEBUG (">> Sent: \"%s\"\n", cmd);
  AssembleCommand ((unsigned char *) cmd, strlen (cmd));

  n_ = write (sockfd_, command_, commandlength_);
  if (n_ < 0)
    return (-1);

  return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Read a result from the laser unit.
int
  lms400_driver::sick_lms_400::ReadResult ()
{
  memset (buffer_, 0, 256);
  n_ = read (sockfd_, buffer_, 8);
  if (n_ < 0)
    return (-1);

  if (buffer_[0] != 0x02 || buffer_[1] != 0x02 || buffer_[2] != 0x02 || buffer_[3] != 0x02)
  {
    if (verbose_ > 0)
      ROS_WARN ("> E: expected 4 bytes STX's!");
    n_ = read (sockfd_, buffer_, 255);
    return (-1);
  }

  // Find message length
  int length = ( (buffer_[4] << 24) | (buffer_[5] << 16) | (buffer_[6] <<  8) | (buffer_[7]) );
  int current = 0;
  do
  {
    n_ = read (sockfd_, &buffer_[current], length-current);
    current += n_;
  } while (current < length);

  bufferlength_ = length;
  if ((verbose_ > 0) && (buffer_[0] != 0x20))
    ROS_DEBUG (">> Received: \"%s\"\n", buffer_);

  // Check for error
  if (strncmp ((const char*)buffer_, "sFA", 3) == 0)
  {
    strtok ((char*)buffer_, " ");
    ROS_DEBUG (">> E: Got an error message with code 0x%s\n", strtok (NULL, " "));
  }

  // Read checksum:
  char cs_read = 0;
  int ret = read (sockfd_, &cs_read, 1);
  if (ret < 1)
  {
    ROS_ERROR ("LMS400 didnt get any data in read %d",ret);
    return (-1);
  }

  if (buffer_[0] == 's')
    return (0);
  else if (buffer_[0] == 0x20)
    return (ReadResult ());
  else if (bufferlength_ > sizeof (MeasurementHeader_t))
  {
    if (verbose_ > 0)
      ROS_DEBUG (">>>> ReadResult: probably found a data packet!\n>>>>             %s\n", buffer_);
    // Don't throw away our precious measurement, queue it for later use :)
    unsigned char* tmp = (unsigned char*) malloc (bufferlength_ + 1);
    memcpy (tmp, buffer_, bufferlength_ + 1);
    MeasurementQueueElement_t q;
    q.string = tmp;
    q.length = bufferlength_;
    MeasurementQueue_->push_back (q);
    // and then, try to read what we actually wanted to read...
    return (ReadResult ());
  }

  return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Read an answer from the laser unit
int
  lms400_driver::sick_lms_400::ReadAnswer ()
{
  return (ReadResult ());
}

////////////////////////////////////////////////////////////////////////////////
// Read a confirmation and an answer from the laser unit
int
  lms400_driver::sick_lms_400::ReadConfirmationAndAnswer ()
{
  ReadResult ();
  if (buffer_[0] == 's' && buffer_[1] == 'F' && buffer_[2] == 'A')
    return (-1);
  else
    return (ReadResult ());
}

////////////////////////////////////////////////////////////////////////////////
// adds a header and the checksum to the command to be sent
int
  lms400_driver::sick_lms_400::AssembleCommand (unsigned char* cmd, int len)
{
  unsigned char checksum = 0;
  int index = 0;

  command_[0]  = 0x02;  // Messages start with 4 STX's
  command_[1]  = 0x02;
  command_[2]  = 0x02;
  command_[3]  = 0x02;
  command_[4]  = (len >> 24) & 0xff; // then message length
  command_[5]  = (len >> 16) & 0xff;
  command_[6]  = (len >>  8) & 0xff;
  command_[7]  = (len      ) & 0xff;

  for (index = 0; index < len; index++)
  {
    command_[index + 8]  = cmd[index];
    checksum ^= cmd[index];
  }
  command_[8 + len] = checksum;
  command_[9 + len] = 0x00;

  commandlength_ = 9 + len;
  return (0);
}
