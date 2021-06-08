/*
 * Copyright (c) 2009 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
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

/**
@mainpage

@htmlinclude manifest.html

\author Radu Bogdan Rusu

@b lms400_node is a node providing access to a SICK LMS400 laser sensor via the asr_sick_lms_400 library.
The code is based on the LGPL-ed LMS400 Player driver by Nico Blodow and Radu Bogdan Rusu.
**/

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt16.h>

#include <lms400_driver/sick_lms400.h>

using namespace std;
using namespace ros;
using namespace lms400_driver;
using namespace geometry_msgs;
using namespace sensor_msgs;

class LMS400Node
{
  protected:
    ros::NodeHandle nh_;
  public:

    sick_lms_400 lms_;
    LaserScan scan_;
    std_msgs::UInt16 encoder_position_;

    Publisher scan_pub_;
    Publisher encoder_pub_;

    // TCP/IP connection parameters
    string hostname_, password_;
    int    port_;

    // Filter settings
    int filter_;
    int mean_filter_params_;
    double range_filter_params_min_, range_filter_params_max_;

    // Turn intensity data on/off
    bool intensity_;

    // Turn laser on/off
    bool laser_enabled_;

    // Basic measurement parameters
    double angular_resolution_, scanning_frequency_;
    double min_angle_, max_angle_;
    int eRIS_;
    int encoder_type_;

    // Encoder settings
    int encoder_type;

    // Password for changing to userlevel 3 (service)
    bool loggedin_;
    int debug_;

    ////////////////////////////////////////////////////////////////////////////////
    LMS400Node (ros::NodeHandle &n) : nh_(n), debug_ (0)
    {
      nh_.param ("hostname", hostname_, string ("192.168.0.1"));
      // Userlevel 3 password (hashed). Default: servicelevel/81BE23AA (Service (userlevel 3) password. Used for enabling/disabling and/or setting the filter parameters.

      nh_.param ("password", password_, string ("81BE23AA"));
      nh_.param ("port", port_, 2111);

      // Filter settings. Valid values are:
      // 0 (disabled)
      // 1 (enable median filter)
      // 2 (enable edge filter)
      // 4 (enable range filter)
      // 8 (enable mean filter)
      // Notes : 1) You can combine the filters as required. If several filters are active, then the filters act one after the other on the result of the
      //            previous filter. The processing in this case follows the following sequence: edge filter, median filter, range filter, mean filter.
      //         2) You can use PLAYER_LASER_REQ_SET_FILTER to enable/disable the filters as well as set their parameters from the client. The parameters of the
      //            filters are stored in a float array in the sequence mentioned above. Since the current LMS400 firmware version (1.20) supports setting the
      //            parameters for the range and mean filters, the order of the parameters in the (player_laser_set_filter_config_t - float parameters[]) array,
      //            provided that both range and mean are enabled is: [BottomLimit TopLimit Mean]
      nh_.param ("filter", filter_, 11);     // Enable median, edge and mean filter

      // Enable extended RIS detectivity. If you want to measure objects with remission values < 10%,
      // you can extend the so-called Remission Information System (RIS) on the LMS4000.
      nh_.param ("enable_eRIS", eRIS_, 1);

      // Define the number of means for the mean filter. Possible values: 2..200.
      nh_.param ("mean_filter_parameter", mean_filter_params_, 3);

      // Define a specific range within which measured values are valid and are output. Possible values: [+700.0...+3000.0 <bottom limit>...+3000.0]
      nh_.param ("range_filter_parameter_min", range_filter_params_min_, 700.0);
      nh_.param ("range_filter_parameter_max", range_filter_params_max_, 3000.0);

      {
        // Angular resolution. Valid values are: 0.1 ..1 (with 0.1 degree increments)
        nh_.param ("angular_resolution", angular_resolution_, 0.1);

        // Scanning frequency. Valid values are: - 200..500Hz (on the LMS400-0000) / 360..500Hz (on the LMS400-1000)
        nh_.param ("scanning_frequency", scanning_frequency_, 360.0);

        nh_.param ("enable_laser", laser_enabled_, true);
        nh_.param ("enable_intensity", intensity_, true);

        // Defines the minimum / maximum angle of the laser unit (where the scan should start / end). Valid values: 55-125 degrees.
        nh_.param ("min_angle", min_angle_, 55.0);
        nh_.param ("max_angle", max_angle_, 125.0);

        // Encoder type. Valid values are: 0 - No encoder, 1 - pulse encoder, 2 - phase encoder,
        // 3 - level encoder, 4 - constant speed
        nh_.param ("encoder_type", encoder_type_, 2);
      }

      scan_pub_ = nh_.advertise<LaserScan>("laser_scan", 10);
      encoder_pub_ = nh_.advertise<std_msgs::UInt16>("encoder_position", 10);

      loggedin_  = false;
    }

    ////////////////////////////////////////////////////////////////////////////////
    ~LMS400Node ()
    {
      stop ();
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Start
    int
      start ()
    {
      // Open the LMS400 device
      lms_ = sick_lms_400 (hostname_.c_str (), port_, debug_);

      // Attempt to connect to the laser unit
      if (lms_.Connect () != 0)
      {
        ROS_ERROR ("> [SickLMS400] Connecting to SICK LMS400 on [%s:%d]...[failed!]", hostname_.c_str (), port_);
        return (-1);
      }
      ROS_INFO ("> [SickLMS400] Connecting to SICK LMS400 on [%s:%d]... [done]", hostname_.c_str (), port_);

      // Stop the measurement process (in case it's running from another instance)
      lms_.StopMeasurement ();

      if (strncmp (password_.c_str (), "NULL", 4) != 0)
      {
        // Login to userlevel 3
        if (lms_.SetUserLevel (4, password_.c_str ()) != 0)
          ROS_WARN ("> [SickLMS400] Unable to change userlevel to 'Service' using %s", password_.c_str ());
        else
        {
          loggedin_ = true;
          // Enable or disable filters
          if ((mean_filter_params_ >= 2) && (mean_filter_params_ <= 200))
            lms_.SetMeanFilterParameters (mean_filter_params_);

          if ((range_filter_params_min_ >= 700.0) && (range_filter_params_max_> range_filter_params_min_) && (range_filter_params_max_ <= 3600.0))
            lms_.SetRangeFilterParameters ((float)range_filter_params_min_, (float)range_filter_params_max_);

          lms_.EnableFilters (filter_);

          ROS_INFO ("> [SickLMS400] Enabling selected filters (%d, %f, %f, %d)... [done]", mean_filter_params_, (float)range_filter_params_min_, (float)range_filter_params_max_, filter_);
        }
      }
      else
        ROS_WARN ("> [SickLMS400] Userlevel 3 password not given. Filter(s) disabled!");

      // Enable extended RIS detectivity
      if (eRIS_)
      {
        lms_.EnableRIS (1);
        ROS_INFO ("> [SickLMS400] Enabling extended RIS detectivity... [done]");
      }

      // Enable extended RIS detectivity
      if (encoder_type_)
      {
        lms_.SetEncoderType (encoder_type_);
        ROS_INFO ("> [SickLMS400] Enabling encoder settings... [done]");
      }

      // Set scanning parameters
      if (lms_.SetResolutionAndFrequency (scanning_frequency_, angular_resolution_, min_angle_, max_angle_ - min_angle_) != 0)
        ROS_ERROR ("> [SickLMS400] Couldn't set values for resolution, frequency, and min/max angle. Using previously set values.");
      else
        ROS_INFO ("> [SickLMS400] Enabling user values for resolution (%f), frequency (%f) and min/max angle (%f/%f)... [done]",
                  angular_resolution_, scanning_frequency_, min_angle_, max_angle_);

      return (0);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Stop
    int
      stop ()
    {
      // Stop the measurement process
      lms_.StopMeasurement ();
      // Set back to userlevel 0
      lms_.TerminateConfiguration ();
      // Disconnect from the laser unit
      lms_.Disconnect ();

      ROS_INFO ("> [SickLMS400] SICK LMS400 driver shutting down... [done]");
      return (0);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Set new measurement values and restart scanning
    void
      restartMeasurementWithNewValues (float scanning_frequency, float angular_resolution,
                                       float min_angle, float diff_angle, int intensity,
                                       bool laser_enabled)
    {
      // Stop the measurement process
      lms_.StopMeasurement ();

      // Change userlevel to 3
      if (lms_.SetUserLevel (4, password_.c_str ()) != 0)
      {
        ROS_WARN ("> Unable to change userlevel to 'Service' using %s", password_.c_str ());
        if (laser_enabled)
          lms_.StartMeasurement (intensity);
      }
      else
      {
        // Set the angular resolution and frequency
        if (lms_.SetResolutionAndFrequency (scanning_frequency, angular_resolution, min_angle, diff_angle) == 0)
          // Re-start the measurement process
          if (laser_enabled)
            lms_.StartMeasurement (intensity);
      }
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Obtain a set of interesting parameters from the parameter server
    void
      getParametersFromServer ()
    {
      // LMS400 related parameters
      bool laser_enabled;
      nh_.getParam ("enable_laser", laser_enabled);
      // New value specified
      if (laser_enabled != laser_enabled_)
      {
        ROS_INFO ("New enable_laser parameter received (%d)", laser_enabled);
        laser_enabled_ = laser_enabled;

        if (!laser_enabled_)
          lms_.StopMeasurement ();
        else
          lms_.StartMeasurement (intensity_);
      }

      bool intensity;
      nh_.getParam ("enable_intensity", intensity);
      // New value specified
      if (intensity != intensity_)
      {
        ROS_INFO ("New enable_intensity parameter received (%d)", intensity);
        intensity_ = intensity;
      }

      double angular_resolution;
      nh_.getParam ("angular_resolution", angular_resolution);
      // New value specified
      if (angular_resolution != angular_resolution_)
      {
        ROS_INFO ("New angular_resolution parameter received (%f)", angular_resolution);
        angular_resolution_ = angular_resolution;
        restartMeasurementWithNewValues (scanning_frequency_, angular_resolution_, min_angle_, max_angle_ - min_angle_, intensity_, laser_enabled_);
      }

      double min_angle;
      nh_.getParam ("min_angle", min_angle);
      // New value specified
      if (min_angle != min_angle_)
      {
        ROS_INFO ("New min_angle parameter received (%f)", min_angle);
        min_angle_ = min_angle;
        restartMeasurementWithNewValues (scanning_frequency_, angular_resolution_, min_angle_, max_angle_ - min_angle_, intensity_, laser_enabled_);
      }

      double max_angle;
      nh_.getParam ("max_angle", max_angle);
      // New value specified
      if (max_angle != max_angle_)
      {
        ROS_INFO ("New max_angle parameter received (%f)", max_angle);
        max_angle_ = max_angle;
        restartMeasurementWithNewValues (scanning_frequency_, angular_resolution_, min_angle_, max_angle_ - min_angle_, intensity_, laser_enabled_);
      }

      double scanning_frequency;
      nh_.getParam ("scanning_frequency", scanning_frequency);
      // New value specified
      if (scanning_frequency != scanning_frequency_)
      {
        ROS_INFO ("New scanning_frequency parameter received (%f)", scanning_frequency);
        scanning_frequency_ = scanning_frequency;
        restartMeasurementWithNewValues (scanning_frequency_, angular_resolution_, min_angle_, max_angle_ - min_angle_, intensity_, laser_enabled_);
      }
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Spin (!)
    bool
      spin ()
    {
      // Start Continous measurements
      lms_.StartMeasurement (intensity_);

      while (nh_.ok ())
      {
        // Change certain parameters in the cameras, based on values from the parameter server
        //getParametersFromServer ();

        // Refresh data only if laser power is on
        if (laser_enabled_)
        {
          scan_ = lms_.ReadMeasurement (encoder_position_);

          if (scan_.ranges.size () != 0) {
            scan_pub_.publish (scan_);
            encoder_pub_.publish(encoder_position_);
          }
          //ROS_INFO ("Publishing %d measurements.", (int)scan_.ranges.size ());
        }
        ros::spinOnce ();
      }

      return (true);
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "lms400_node");
  ros::NodeHandle n("~");
  LMS400Node c(n);

  if (c.start () == 0)
    c.spin ();

  return (0);
}
/* ]--- */
