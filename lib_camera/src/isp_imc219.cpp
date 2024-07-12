
#include "camera_isp_thread.hpp"


#include <xcore/assert.h>
#include "print.h"

#include "camera.h"
#include "camera_mipi.h"
#include "sensor_wrapper.h"
#include "sensor_base.hpp"
#include "camera_utils.h"


#include "imx219.hpp"

using namespace sensor;



void isp_imx219_thread(
  streaming_chanend_t c_pkt,
  streaming_chanend_t c_ctrl,
  chanend_t c_cam[N_CH_USER_ISP]) 
{
  i2c_master_t i2c_ctx;
  i2c_config_t i2c_conf;
  
  // I2C settings
  i2c_conf.device_addr = I2C_DEV_ADDR;
  i2c_conf.speed = I2C_DEV_SPEED;
  i2c_conf.p_scl = XS1_PORT_4E;
  i2c_conf.p_sda = XS1_PORT_4E;
  i2c_conf.i2c_ctx_ptr = &i2c_ctx;
  // Sensor settings
  resolution_t res = {
    .sensor_width = SENSOR_WIDHT,
    .sensor_height = SENSOR_HEIGHT
  };

  auto sensor = IMX219 {
    (i2c_config_t)i2c_conf, 
    (resolution_t)res, 
    (pixel_format_t)MIPI_DT_RAW8, 
    (binning_t)CONFIG_BINNING, 
    (centralise_t)CONFIG_CENTRALISE
  };

  printstr("Camera init\n");
  int ret = 0;
  ret |= sensor.initialize();
  delay_milliseconds_cpp(100);
  ret |= sensor.configure();
  delay_milliseconds_cpp(500);
  ret |= sensor.stream_start();
  delay_milliseconds_cpp(500);
  xassert((ret == 0) && "Could not initialise camera");
  printstr("Camera_started and configured\n");

  camera_isp_thread(sensor, c_pkt, c_ctrl, c_cam);
}

