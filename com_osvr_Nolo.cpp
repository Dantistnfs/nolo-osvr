/** @file
    @brief OSVR plugin for LYRobotix Nolo trackers

    @date 2017

    @author
    Yann Vernier
    <http://yann.vernier.se>
*/

// Copyright 2014 Sensics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Internal Includes
#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/AnalogInterfaceC.h>
#include <osvr/PluginKit/ButtonInterfaceC.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>
#include <osvr/Util/Logger.h>
#include <osvr/Util/LogNames.h>

// Generated JSON header file
#include "com_osvr_Nolo_json.h"

// Library/third-party includes
#include <hidapi.h>

// Standard includes
#include <iostream>
#include <bitset>
#include <wchar.h>
#include <string.h>

// Anonymous namespace to avoid symbol collision
namespace {
#define TICK_LEN (1.0f / 120000.0f)
#define CLICK_DELAY 40 //TICKS
#define DOUBLE_CLICK_TIMEOUT  40 //TICKS
#define deltat (1.0f/ 120000.0f)    // sampling period in seconds (shown as 8.333 ms)
#define gyroMeasError 3.14159265358979f * (5.0f / 180.0f)     // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError                // compute bet


typedef enum
{
	//LEGACY firmware < 2.0
	NOLO_LEGACY_CONTROLLER_TRACKER = 165,
	NOLO_LEGACY_HMD_TRACKER = 166,
	//firmware > 2.0
	NOLO_CONTROLLER_0_HMD_SMP1 = 16,
	NOLO_CONTROLLER_1_HMD_SMP2 = 17,
} nolo_irq_cmd;

typedef struct {
    int clicked = 0;
    bool pressed = 0;
    int clicktime = 0;
    bool double_click = 0;
    int double_click_timeout = 0;
} controller_state;





  // btea_decrypt function
  #include "btea.c"

  #if 1
  void hexdump(unsigned char *data, int len) {
    char fill = std::cout.fill();
    std::streamsize w = std::cout.width();
    std::ios_base::fmtflags f = std::cout.flags();
    for (int i=0; i<len; i++) {
      std::cout << std::setw(2) << std::setfill('0')
		<< std::hex << (int)data[i];
      if (i%8==7)
	std::cout << " ";
    }
    std::cout << std::endl;
    std::cout.fill(fill);
    std::cout.width(w);
    std::cout.flags(f);
  };
  #endif
  

static int NUM_AXIS    = 4;
static int NUM_BUTTONS = 6;
class NoloDevice {
  public:
    NoloDevice(OSVR_PluginRegContext ctx,
	       char *path) {
        /// Create the initialization options
        OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

	m_hid = hid_open_path(path);

        /// Indicate that we'll want 7 analog channels.
        //osvrDeviceAnalogConfigure(opts, &m_analog, 2*3+1);
	// update this to 9 analog channels to include the trigger
        osvrDeviceAnalogConfigure(opts, &m_analog, NUM_AXIS*2+1);
	/// And 6 buttons per controller
        osvrDeviceButtonConfigure(opts, &m_button, 2*NUM_BUTTONS);
	/// And tracker capability
        osvrDeviceTrackerConfigure(opts, &m_tracker);

        /// Create the device token with the options
        m_dev.initAsync(ctx, "LYRobotix Nolo", opts);

        /// Send JSON descriptor
        m_dev.sendJsonDescriptor(com_osvr_Nolo_json);

        /// Register update callback
        m_dev.registerUpdateCallback(this);

	memset(&oldreports[0][0], 0, sizeof oldreports);

	for (int i = 0; i < 2; i++){
		for (int j = 0; j < 4; j++){
			cont_quats[i][j] = 0.0;
			if (j == 0)
			    cont_quats[i][j] = 1.0;
		}

	}
	   //beta = sqrt(3.0 / 4.0)/10;// * GyroMeasError 
    }
    ~NoloDevice() {
      std::cout << "Nolo: deleting " << this << std::endl;
      hid_close(m_hid);
    }
  
    OSVR_ReturnCode update() {
	unsigned char buf[64];
	const int cryptwords = (64-4)/4, cryptoffset=1;
	static const uint32_t key[4] =
	{0x875bcc51, 0xa7637a66, 0x50960967, 0xf8536c51};
	uint32_t cryptpart[cryptwords];
	int i;

	// TODO: timestamp frame received?
	if (!m_hid)
	    OSVR_RETURN_FAILURE;
	//std::cout << "Reading HID data from " << m_hid << std::endl;
	if(hid_read(m_hid, buf, sizeof buf) != sizeof buf)
	return OSVR_RETURN_FAILURE;
	osvrTimeValueGetNow(&m_lastreport_time);

	// Check for duplicate reports before decrypting
	/*
	switch (buf[0]) {
	// OLD FIRMWARE
	    case NOLO_LEGACY_CONTROLLER_TRACKER:
	    case NOLO_CONTROLLER_0_HMD_SMP1:
	    case NOLO_CONTROLLER_1_HMD_SMP2:
	    case NOLO_LEGACY_HMD_TRACKER:
		if (memcmp(oldreports[buf[0]-0xa5], buf, sizeof buf))
		    memcpy(oldreports[buf[0]-0xa5], buf, sizeof buf);
		else
		    return OSVR_RETURN_SUCCESS;	// Duplicate report
		break;
	    default:
		return OSVR_RETURN_SUCCESS; // Unknown report
	    }
	   */
	//std::cout << "R ";
	//hexdump(buf, sizeof buf);
	// Decrypt encrypted portion
	for (i=0; i<cryptwords; i++) {
	cryptpart[i] =
	    ((uint32_t)buf[cryptoffset+4*i  ])<<0  |
	    ((uint32_t)buf[cryptoffset+4*i+1])<<8  |
	    ((uint32_t)buf[cryptoffset+4*i+2])<<16 |
	    ((uint32_t)buf[cryptoffset+4*i+3])<<24;
	}
	btea_decrypt(cryptpart, cryptwords, 1, key);
	for (i=0; i<cryptwords; i++) {
	    buf[cryptoffset+4*i  ] = cryptpart[i]>>0;
	    buf[cryptoffset+4*i+1] = cryptpart[i]>>8;
	    buf[cryptoffset+4*i+2] = cryptpart[i]>>16;
	    buf[cryptoffset+4*i+3] = cryptpart[i]>>24;
	}
	switch (buf[0]) {
	    case NOLO_LEGACY_CONTROLLER_TRACKER:  // controllers frame
		decodeControllerCV1(0, buf+1);
		decodeControllerCV1(1, buf+64-controllerLength);
		break;
	    case NOLO_LEGACY_HMD_TRACKER:
		decodeHeadsetMarkerCV1(buf+0x15);
		decodeBaseStationCV1(buf+0x36);
		break;
	    case NOLO_CONTROLLER_0_HMD_SMP1:
		decodeFirm2(0, buf);
		break;
	    case NOLO_CONTROLLER_1_HMD_SMP2:
		decodeFirm2(1, buf);
		break;

	}
	return OSVR_RETURN_SUCCESS;
	}

    // Sets vibration strength in percent
    int setVibration(unsigned char left,
		     unsigned char right) {
      unsigned char data[4] = {0xaa, 0x66, left, right};
      return hid_write(m_hid, data, sizeof data);
    }
  
  private:
    void decodeFirm2(int controller_num, unsigned char *data){
	//std::cout << "Decoding Tracker :" << std::endl;
	nolo_decode_hmd_marker(data);
	//std::cout << "Decoding Controller :" << controller_num << std::endl; 
        nolo_decode_controller(controller_num, data);

	//std::cout << "Found double click on both controllers" << std::endl;
      	if ((cont_states[0].double_click + cont_states[1].double_click) == 2){
		//Homing routine
	}
    }

    void nolo_decode_controller_orientation(unsigned char* data, int idx, OSVR_OrientationState *quat)
    {	
	double ax,ay,az;
	double gx,gy,gz;
	ax = (float)read16(data + 0);
	az = (float)read16(data + 2);
	ay = (float)read16(data + 4);
	gx = (float)read16(data + 6);
	gz = (float)read16(data + 8);
	gy = (float)read16(data + 10);


	double SEq_1 = cont_quats[idx][0];
	double SEq_2 = cont_quats[idx][1];
	double SEq_3 = cont_quats[idx][2];
	double SEq_4 = cont_quats[idx][3];

	double a_x = ax;
	double a_y = ay;
	double a_z = az;


	double w_x = gx;
	double w_y = gy;
	double w_z = gz;


	// An efficient orientation filter for inertial andinertial/magnetic sensor arrays Sebastian O.H. Madgwick
	double norm;                  // vector norm
	double SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4;  // quaternion derrivative from gyroscopes elements
	double f_1, f_2, f_3;                                                   // objective function elements
	double J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33;              // objective function Jacobian elements
	double SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4;              // estimated direction of the gyroscope error
	// Axulirary variables to avoid reapeated calcualtions
	double halfSEq_1 = 0.5f * SEq_1;
	double halfSEq_2 = 0.5f * SEq_2;
	double halfSEq_3 = 0.5f * SEq_3;
	double halfSEq_4 = 0.5f * SEq_4;
	double twoSEq_1 = 2.0f * SEq_1;
	double twoSEq_2 = 2.0f * SEq_2;
	double twoSEq_3 = 2.0f * SEq_3;
	// Normalise the accelerometer measurement
	norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
	a_x /= norm;
	a_y /= norm;
	a_z /= norm;
	// Compute the objective function and Jacobian
	f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
	f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
	f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
	J_11or24 = twoSEq_3;             // J_11 negated in matrix multiplication
	J_12or23 = 2.0f * SEq_4;
	J_13or22 = twoSEq_1;           // J_12 negated in matrix multiplication
	J_14or21 = twoSEq_2;J_32 = 2.0f * J_14or21;               // negated in matrix multiplication
	J_33 = 2.0f * J_11or24;                                                 // negated in matrix multiplication
	// Compute the gradient (matrix multiplication)
	SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
	SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
	SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
	SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;	
	// Normalise the gradient
	norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
	SEqHatDot_1 /= norm;
	SEqHatDot_2 /= norm;
	SEqHatDot_3 /= norm;
	SEqHatDot_4 /= norm;
	// Compute the quaternion derrivative measured by gyroscopes
	SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
	SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
	SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
	SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
	// Compute then integrate the estimated quaternion derrivative
	SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
	SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
	SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
	SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
	// Normalise quaternion
	norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
	SEq_1 /= norm;
	SEq_2 /= norm;
	SEq_3 /= norm;
	SEq_4 /= norm;
	
	double q1 = SEq_1;
	double q2 = SEq_2;
	double q3 = SEq_3;
	double q4 = SEq_4;

	cont_quats[idx][0] = q1;
	cont_quats[idx][1] = q2;
	cont_quats[idx][2] = q3;
	cont_quats[idx][3] = q4;

	osvrQuatSetW(quat, q1);
	osvrQuatSetX(quat, q2);
	osvrQuatSetY(quat, q3);
	osvrQuatSetZ(quat, q4);


    }


    void nolo_decode_controller(int idx, unsigned char *data){
      enum ControllerOffsets {
	    hwversion   = 0,	// guessed!
	    fwversion   = 1,
	    position    = 1,
	    orientation = 1+6,
	    ofsbuttons  = 1+6+12,
	    touchid     = 1+6+12+1,
	    touchx      = 1+6+12+1,
	    touchy      = 1+6+12+2,
	    battery     = 1+6+12+4,
      };
      OSVR_PoseState pose;
      uint8_t buttons, bit;
      int trigger_pressed = 0;

      decodePositionV2(data + position, &pose.translation);
	nolo_decode_controller_orientation(data + orientation, idx, &pose.rotation);
      osvrDeviceTrackerSendPoseTimestamped(m_dev, m_tracker, &pose, 2+idx, &m_lastreport_time);
      
      buttons = read8(data + ofsbuttons);
      for (bit=0; bit<NUM_BUTTONS; bit++){
	osvrDeviceButtonSetValueTimestamped(m_dev, m_button,
				 (buttons & 1<<bit ? OSVR_BUTTON_PRESSED
				  : OSVR_BUTTON_NOT_PRESSED), idx*6+bit,
				 &m_lastreport_time);
	if(bit == 1){
	    if(buttons & 1<<bit){
		trigger_pressed = 1;
	    }else{
		trigger_pressed = 0;
	    }
	}
      }
      // next byte is touch ID bitmask (identical to buttons bit 5)

      // Touch X and Y coordinates
      // //assumes 0 to 255
      // normalize from -1 to 1
      // z = 2*[x - min / (max - min) - 1]
      // z = 2*(x - 0 / (255 - 0) - 1]
      // z = 2*(x/255) -1
      // normalize 0 to 1
      // x/255
      double axis_value;
      if (data[touchid]) {  // Only report touch if there is one
        axis_value = 2*(int)data[touchx]/255.0 - 1;
        // invert axis
        axis_value *= -1;
        osvrDeviceAnalogSetValueTimestamped(m_dev, m_analog, axis_value,   idx*NUM_AXIS+0, &m_lastreport_time);
        axis_value = 2*(int)data[touchy]/255.0 -1;
        axis_value *= -1;
        osvrDeviceAnalogSetValueTimestamped(m_dev, m_analog, axis_value, idx*NUM_AXIS+1, &m_lastreport_time);
      }
      // trigger (emulated analog axis)
      osvrDeviceAnalogSetValueTimestamped(m_dev, m_analog, trigger_pressed, idx*NUM_AXIS+2, &m_lastreport_time);
      // battery level
      axis_value = data[battery]/255.0; 
      osvrDeviceAnalogSetValueTimestamped(m_dev, m_analog, axis_value, idx*NUM_AXIS+3, &m_lastreport_time);


      //std::cout << idx <<std::endl;
      if ((buttons & 1<<3) == 8){ //clicked on power button
	cont_states[idx].pressed = 1;
	    //int clicked = 0;
	    //int pressed = 0;
	    //int clicktime = 0;
      }
      else { //it's either key up or no press
	      if ((cont_states[idx].clicked == 0) & (cont_states[idx].pressed == 1)){
		      cont_states[idx].pressed = 0;
		      cont_states[idx].clicktime = 1;
		      cont_states[idx].clicked += 1;
	      }
	      else if ((cont_states[idx].clicked > 0) & (cont_states[idx].pressed == 1) & (cont_states[idx].clicktime < CLICK_DELAY )){
		    std::cout << cont_states[idx].clicktime << std::endl;
		    std::cout << "DOUBLE_CLICK" << std::endl;
		    cont_states[idx].clicked = 0;
		    cont_states[idx].double_click = 1;
		    cont_states[idx].double_click_timeout = DOUBLE_CLICK_TIMEOUT;
		    cont_states[idx].clicktime = 0;
		    cont_states[idx].pressed = 0;
	      }


		      //if (
			//      cont_states[idx].clicked = 0;
			//}

      }
     
      // add time between clicks 
      if (cont_states[idx].clicktime > 0){
	cont_states[idx].clicktime += 1; 
	    //std::cout << cont_states[idx].clicked << "," << cont_states[idx].clicktime << std::endl;
	      if (cont_states[idx].clicktime > CLICK_DELAY){
		cont_states[idx].clicktime = 0;
		cont_states[idx].clicked = 0;
	      }

      }

      if (cont_states[idx].double_click){
		cont_states[idx].double_click_timeout -= 1;

		if (cont_states[idx].double_click_timeout == 0){
		    cont_states[idx].double_click = 0;
		    std::cout << "DOUBLE CLICK TIMEOUTED" << std::endl;
		    reset_controller_rotation(idx);
		}


      }
      //check for homing buttons press
    }
    
    void reset_controller_rotation(int idx){
	for (int j = 0; j < 4; j++){
		cont_quats[idx][j] = 0.0;
		if (j == 0)
		    cont_quats[idx][j] = 1.0;
	}

    }


    void nolo_decode_hmd_marker(const unsigned char* data){
	enum MarkerOffsets {
	    hwversion    = 0,	// guessed!
	    fwversion    = 1,
	    position     = 25, // skip controller data
	    orientation  = 25+12, // controller data + position
      };
	OSVR_PoseState hmd;
	decodePositionV2(data + position, &hmd.translation);
	decodeOrientationV2(data + orientation, &hmd.rotation);

      osvrDeviceTrackerSendPoseTimestamped(m_dev, m_tracker, &hmd, 1,
		      &m_lastreport_time);
    }





    int16_t read16(const unsigned char* buffer)  //from https://github.com/OpenHMD/OpenHMD/blob/master/src/drv_nolo/packet.c
    {
	int16_t ret = *buffer | (*(buffer + 1) << 8);
	return ret;
    }

    uint8_t read8(const unsigned char* buffer)
    {
	uint8_t ret = *buffer;
	return ret;
    }




    void decodePositionV2(const unsigned char *data,
			    OSVR_PositionState *pos) {
	const double scale = 0.0001;
	float x = (float)read16(data);
	float y = (float)read16(data + 2);
	float z = (float)read16(data + 4);
	osvrVec3SetX(pos, x*scale);
	osvrVec3SetY(pos, y*scale);
	osvrVec3SetZ(pos, z*scale);
    }


    void decodeOrientationV2(const unsigned char *data, OSVR_OrientationState *quat){
	data += 12;
	double w,i,j,k, scale;
	  w = read16(data);
	  i = read16(data+2);
	  j = read16(data+4);
	  k = read16(data+6);

	  scale = 1.0 / 16384;
	  //std::cout << "Scale: " << scale << std::endl;
	  w *= scale;
	  i *= scale;
	  j *= scale;
	  k *= scale;
	  // Reorder
	  osvrQuatSetW(quat, w);
	  osvrQuatSetX(quat, i);
	  osvrQuatSetY(quat, k);
	  osvrQuatSetZ(quat, -j);
    }

    unsigned char oldreports[2][64];
    void decodePosition(const unsigned char *data,
			OSVR_PositionState *pos) {
      const double scale = 0.0001;
      osvrVec3SetX(pos, scale * (int16_t)(data[0]<<8 | data[1]));
      osvrVec3SetY(pos, scale * (int16_t)(data[2]<<8 | data[3]));
      osvrVec3SetZ(pos, scale *          (data[4]<<8 | data[5]));
    }
    void decodeOrientation(const unsigned char *data,
			   OSVR_OrientationState *quat) {
      double w,i,j,k, scale;
      // CV1 order
      w = (int16_t)(data[0]<<8 | data[1]);
      i = (int16_t)(data[2]<<8 | data[3]);
      j = (int16_t)(data[4]<<8 | data[5]);
      k = (int16_t)(data[6]<<8 | data[7]);
      // Normalize (unknown if scale is constant)
      //scale = 1.0/sqrt(i*i+j*j+k*k+w*w);
      // Turns out it is fixed point. But the android driver author
      // either didn't know, or didn't trust it.
      // Unknown if normalizing it helps OSVR!
      scale = 1.0 / 16384;
      //std::cout << "Scale: " << scale << std::endl;
      w *= scale;
      i *= scale;
      j *= scale;
      k *= scale;
      // Reorder
      osvrQuatSetW(quat, w);
      osvrQuatSetX(quat, i);
      osvrQuatSetY(quat, k);
      osvrQuatSetZ(quat, -j);
    }
    void decodeControllerCV1(int idx, unsigned char *data) {
      enum ControllerOffsets {
	    hwversion   = 0,	// guessed!
	    fwversion   = 1,
	    position    = 3,
	    orientation = 3+3*2,
	    ofsbuttons  = 3+3*2+4*2,
	    touchid     = 3+3*2+4*2+1,
	    touchx      = 3+3*2+4*2+2,
	    touchy      = 3+3*2+4*2+3,
	    battery     = 3+3*2+4*2+4,
      };
      OSVR_PoseState pose;
      uint8_t buttons, bit;
      int trigger_pressed = 0;

      if (data[hwversion] != 2 || data[fwversion] != 1) {
	// Unknown version
	/* Happens when controllers aren't on. 
	std::cout << "Nolo: Unknown controller " << idx
		  << " version " << (int)data[0] << " " << (int)data[1]
		  << std::endl;
	*/
	return;
      }

      decodePosition(data+position, &pose.translation);
      decodeOrientation(data+orientation, &pose.rotation);

      osvrDeviceTrackerSendPoseTimestamped(m_dev, m_tracker, &pose, 2+idx, &m_lastreport_time);
      
      buttons = data[ofsbuttons];
      // TODO: report buttons for both controllers in one call?
      for (bit=0; bit<NUM_BUTTONS; bit++){
	osvrDeviceButtonSetValueTimestamped(m_dev, m_button,
				 (buttons & 1<<bit ? OSVR_BUTTON_PRESSED
				  : OSVR_BUTTON_NOT_PRESSED), idx*6+bit,
				 &m_lastreport_time);
	if(bit == 1){
	    if(buttons & 1<<bit){
		trigger_pressed = 1;
	    }else{
		trigger_pressed = 0;
	    }
	}
      }
      // next byte is touch ID bitmask (identical to buttons bit 5)

      // Touch X and Y coordinates
      // //assumes 0 to 255
      // normalize from -1 to 1
      // z = 2*[x - min / (max - min) - 1]
      // z = 2*(x - 0 / (255 - 0) - 1]
      // z = 2*(x/255) -1
      // normalize 0 to 1
      // x/255
      double axis_value;
      if (data[touchid]) {  // Only report touch if there is one
        axis_value = 2*data[touchx]/255.0 - 1;
        // invert axis
        axis_value *= -1;
        osvrDeviceAnalogSetValueTimestamped(m_dev, m_analog, axis_value,   idx*NUM_AXIS+0, &m_lastreport_time);
        axis_value = 2*((int)data[touchy])/255.0 -1;
        axis_value *= -1;
        osvrDeviceAnalogSetValueTimestamped(m_dev, m_analog, axis_value, idx*NUM_AXIS+1, &m_lastreport_time);
      }
      // trigger (emulated analog axis)
      osvrDeviceAnalogSetValueTimestamped(m_dev, m_analog, trigger_pressed, idx*NUM_AXIS+2, &m_lastreport_time);
      // battery level
      axis_value = data[battery]/255.0; 
      osvrDeviceAnalogSetValueTimestamped(m_dev, m_analog, axis_value, idx*NUM_AXIS+3, &m_lastreport_time);
    }
    void decodeHeadsetMarkerCV1(unsigned char *data) {
      enum MarkerOffsets {
	    hwversion    = 0,	// guessed!
	    fwversion    = 1,
	    position     = 3,
	    homeposition = 3+3*2,
	    orientation  = 3+2*3*2+1,
      };
      if (data[hwversion] != 2 || data[fwversion] != 1) {
	/* Happens with corrupt packages (mixed with controller data)
	std::cout << "Nolo: Unknown headset marker"
		  << " version " << (int)data[0] << " " << (int)data[1]
		  << std::endl;
	*/
	// Unknown version
	return;
      }

      OSVR_PositionState home;
      OSVR_PoseState hmd;
      
      decodePosition(data+position, &hmd.translation);
      decodePosition(data+homeposition, &home);
      decodeOrientation(data+orientation, &hmd.rotation);

      // Tracker viewer kept using the home for head.
      // Something wrong with how they handle the descriptors. 
      osvrDeviceTrackerSendPositionTimestamped(m_dev, m_tracker, &home, 0,
		      &m_lastreport_time);

      osvrDeviceTrackerSendPoseTimestamped(m_dev, m_tracker, &hmd, 1,
		      &m_lastreport_time);
    }

    void decodeBaseStationCV1(unsigned char *data) {
      enum MarkerOffsets {
	    hwversion = 0,	// guessed!
	    fwversion = 1,
	    battery   = 2
      };
      if (data[hwversion] != 2 || data[fwversion] != 1)
	// Unknown version
	return;

      osvrDeviceAnalogSetValueTimestamped(m_dev, m_analog, data[battery], 2*NUM_AXIS,
		      &m_lastreport_time);
    }
  
    static const int controllerLength = 3 + (3+4)*2 + 2 + 2 + 1;
    double cont_quats[2][4];
    controller_state cont_states[2];
    osvr::pluginkit::DeviceToken m_dev;
    hid_device* m_hid;
    OSVR_AnalogDeviceInterface m_analog;
    OSVR_ButtonDeviceInterface m_button;
    OSVR_TrackerDeviceInterface m_tracker;
    OSVR_TimeValue m_lastreport_time;
};

class HardwareDetection {
  public:
    HardwareDetection() : m_found(false) {}
    OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx) {
    auto log = osvr::util::log::make_logger(osvr::util::log::OSVR_SERVER_LOG);
    log->info() << "Was previously found?: " << m_found;

      // TODO: probe for new devices only
      if (m_found)
	return OSVR_RETURN_SUCCESS;

      struct hid_device_info *hid_devices, *cur_dev;
      hid_devices = hid_enumerate(0x0483, 0x5750);
      log->info() << "Is there hid devices?";
      if (!hid_devices)
	{
	log->info() << "Not found hid devies";
	return OSVR_RETURN_FAILURE;
	}
      log->info() << "Found hid devies";

      for (cur_dev = hid_devices; cur_dev; cur_dev = cur_dev->next) {
	log->info() << "Manufacturer : " << (char*)cur_dev->manufacturer_string << " Device: " << (char*)cur_dev->product_string;
	if (wcscmp(cur_dev->manufacturer_string, L"LYRobotix")==0){
	    log->info() << "Found LYRobotix device";
	    if (wcscmp(cur_dev->product_string, L"NOLO")==0) {
                log-> info() << "Detected firmware <2.0, for the best result please upgrade your NOLO firmware above 2.0";
		osvr::pluginkit::registerObjectForDeletion
		(ctx, new NoloDevice(ctx, cur_dev->path));
		m_found = true;
	    }
	    if (wcscmp(cur_dev->product_string, L"NOLO HMD")==0) {
                log-> info() << "Detected NOLO firmware >2.0";
		osvr::pluginkit::registerObjectForDeletion
		(ctx, new NoloDevice(ctx, cur_dev->path));
		
		m_found = true;
	    }

	
	}
      }
      
      hid_free_enumeration(hid_devices);
      return OSVR_RETURN_SUCCESS;
    }

  private:
    /// @brief Have we found our device yet? (this limits the plugin to one
    /// instance)
    bool m_found;
};
} // namespace

OSVR_PLUGIN(com_osvr_Nolo) {
    hid_init();
    osvr::pluginkit::PluginContext context(ctx);

    /// Register a detection callback function object.
    context.registerHardwareDetectCallback(new HardwareDetection());

    return OSVR_RETURN_SUCCESS;
}
