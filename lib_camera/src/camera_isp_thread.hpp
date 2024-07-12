
#pragma once

#include "api.h"
#include "camera_mipi.h"
#include "camera_isp.h"
#include "camera_utils.h"
#include <xcore/channel_streaming.h>
#include <xcore/select.h>

C_API_START
char camera_isp_packet_handler(
  const mipi_packet_t* pkt,
  image_cfg_t* image_cfg,
  chanend_t c_isp_to_user);
C_API_END

template<typename T_SENSOR>
inline void camera_isp_thread(
  T_SENSOR &sensor,
  streaming_chanend_t c_pkt,
  streaming_chanend_t c_ctrl,
  chanend_t c_cam[N_CH_USER_ISP])
{
  
  mipi_packet_t __attribute__((aligned(8))) packet_buffer[MIPI_PKT_BUFFER_COUNT];
  mipi_packet_t* pkt;
  unsigned pkt_idx = 0;

  // channel unpack
  chanend_t c_user_isp = c_cam[CH_USER_ISP];
  chanend_t c_isp_user = c_cam[CH_ISP_USER];

  // Image configuration
  image_cfg_t image;
  image.ptr = NULL;

  // Wait for the sensor to start
  delay_milliseconds_cpp(1200); 

  // Give the MIPI packet receiver a first buffer
  s_chan_out_word(c_pkt, (unsigned)&packet_buffer[pkt_idx]);


  SELECT_RES(
    CASE_THEN(c_pkt, on_c_pkt_change),
    CASE_THEN(c_user_isp, on_c_user_isp_change)) {

  on_c_pkt_change: { // attending mipi_packet_rx
    pkt = (mipi_packet_t*)s_chan_in_word(c_pkt);
    pkt_idx = (pkt_idx + 1) & (MIPI_PKT_BUFFER_COUNT - 1);
    s_chan_out_word(c_pkt, (unsigned)&packet_buffer[pkt_idx]);
    if(camera_isp_packet_handler(pkt, &image, c_isp_user)) {
      sensor.stream_stop();
    }
    continue;
    }
  on_c_user_isp_change: { // attending user_app
    // user petition
    chan_in_buf_byte(c_user_isp, (uint8_t*)&image, sizeof(image_cfg_t)); // recieve info from user
    // Start camera
    sensor.stream_start();
    continue;
    }
  }

}
