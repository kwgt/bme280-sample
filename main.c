#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <getopt.h>

#include <pthread.h>
#include <signal.h>
#include <setjmp.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <sys/timerfd.h>

#include "i2c.h"

#define N(x)                    (sizeof(x)/sizeof(*x))
#define ERR_DEFAULT             __LINE__

#undef USE_FLOAT
                               
#define REG_RESET               0xE0
#define REG_CTLR_HUM            0xF2
#define REG_STATUS              0xF3
#define REG_CTLR_MEAS           0xF4
#define REG_ID                  0xD0
                               
#define REG_PRESS_MSB           0xF7
#define REG_PRESS_LSB           0xF8
#define REG_PRESS_XSB           0xF9
                               
#define REG_TEMP_MSB            0xFA
#define REG_TEMP_LSB            0xFB
#define REG_TEMP_XSB            0xFC
                               
#define REG_HUM_MSB             0xFD
#define REG_HUM_LSB             0xFE
                               
#define RESET_VAL               0xB6

#define VAL_CTR_HUM(x)          (((x) << 0) & 0x07)
#define VAL_CTR_MEAS(x,y,z) \
        ((((x) << 5) & 0xe0) | (((y) << 2) & 0x8c) | ((z) & 0x03))

#define U16(l,m)                (uint16_t)(((((uint16_t)(m)) << 8) & 0xff00)| \
                                           ((((uint16_t)(l)) << 0) & 0x00ff))

#define S16(l,m)                (int16_t)(((((int16_t)(m)) << 8) & 0xff00)| \
                                          ((((int16_t)(l)) << 0) & 0x00ff))

typedef struct {
  uint16_t t1;
  int16_t  t2;
  int16_t  t3;

  uint16_t p1;
  int16_t  p2;
  int16_t  p3;
  int16_t  p4;
  int16_t  p5;
  int16_t  p6;
  int16_t  p7;
  int16_t  p8;
  int16_t  p9;

  uint8_t  h1;
  int16_t  h2;
  uint8_t  h3;
  int16_t  h4;
  int16_t  h5;
  int8_t   h6;

  int32_t  t_fine;
} bme280_calib_t;


static double
calc_temperature(uint8_t msb, uint8_t lsb, uint8_t xsb, bme280_calib_t* clb)
{
  double ret;
  int32_t raw;

  raw = (((int32_t)msb << 12) & 0x000ff000)|
        (((int32_t)lsb <<  4) & 0x00000ff0)|
        (((int32_t)xsb >>  4) & 0x0000000f);

#ifdef USE_FLOAT
  double v1;
  double v2;

  v1  = (((double)raw / 16384.0) -
            ((double)clb->t1 / 1024.0)) * (double)clb->t2;

  v2  = ((double)raw / 131072.0) - ((double)clb->t1 / 8192.0);
  v2  = (v2 * v2) * (double)clb->t3;

  clb->t_fine = (int32_t)(v1 + v2);

  ret = (double)(clb->t_fine) / 5120.0; 

#else /* defined(USE_FLOAT) */
  int32_t v1;
  int32_t v2;

  v1  = (((raw >> 3) - ((int32_t)clb->t1 << 1)) * (int32_t)clb->t2) >> 11;

  v2  = (raw >> 4) - ((int32_t)clb->t1);
  v2  = (((v2 * v2) >> 12) * (int32_t)clb->t3) >> 14;

  clb->t_fine = v1 + v2;

  ret = (double)(((clb->t_fine * 5) + 128) >> 8) / 100.0; 
#endif /* defined(USE_FLOAT) */

  return ret;
}

static double
calc_humidity(uint8_t msb, uint8_t lsb, bme280_calib_t* clb)
{
  double ret;
  int32_t raw;

  raw = (((int32_t)msb << 8) & 0x0000ff00)|
        (((int32_t)lsb << 0) & 0x000000ff);
 
#ifdef USE_FLOAT
      
  ret = (double)clb->t_fine - 76800.0;

  ret = ((double)raw - (((double)clb->h4 * 64.0) +
          ((double)clb->h5 / 16384.0) * ret)) *
            (((double)clb->h2 / 65536.0) *
               (1.0 + ((double)clb->h6 / 67108864.0) * ret *
                 (1.0 + ((double)clb->h3 / 67108864.0) * ret)));

  ret *= (1.0 - (((double)clb->h1 * ret) / 524288.0));
  
  if (ret > 100.0) {
    ret = 100.0;
  } if (ret < 0.0) {
    ret = 100.0;
  }

#else /* defined(USE_FLOAT) */
  int32_t v;

  v  = clb->t_fine - 76800;

  v  = (((((raw << 14) - ((int32_t)clb->h4 << 20) -
         ((int32_t)clb->h5 * v)) + 16384) >> 15) *
           (((((((v * (int32_t)clb->h6) >> 10) *
             (((v * (int32_t)clb->h3) >> 11) + 32768)) >> 10) + 2097152) *
               ((int32_t)clb->h2) + 8192) >> 14));

  v -= ((((v >> 15) * (v >> 15)) >> 7) * (int32_t)clb->h1) >> 4;
  v  = (v < 0)? 0: v;
  v  = (v > 419430400)? 419430400: v;

  ret   = (double)(v >> 12) / 1024.0;
#endif /* defined(USE_FLOAT) */

  return ret;
}

static double
calc_air_pressure(uint8_t msb, uint8_t lsb, uint8_t xsb, bme280_calib_t* clb)
{
  double ret;
  int32_t raw;

  raw = (((int32_t)msb << 12) & 0x000ff000)|
        (((int32_t)lsb <<  4) & 0x00000ff0)|
        (((int32_t)xsb >>  4) & 0x0000000f);

#ifdef USE_FLOAT
  double v1;
  double v2;

  v1  = ((double)clb->t_fine / 2.0) - 64000.0;

  v2  = v1 * v1 * (double)clb->p6 / 32768.0;
  v2 += v1 * (double)clb->p5 * 2.0;
  v2  = (v2 / 4.0) + ((double)clb->p4 * 65536.0);

  v1  = (((double)clb->p3 * v1 * v1 / 524288.0) + ((double)clb->p2 * v1));
  v1 /= 524288.0;
  v1  = (1.0 + (v1 / 32768.0)) * (double)clb->p1;

  if (v1 == 0.0) {
    ret = 0.0;
  } else {
    ret  = 1048576.0 - (double)raw; 
    ret  = (ret - (v2 / 4096.0)) * 6250.0 / v1;
    v1   = ((double)clb->p9 * ret * ret) / 2147483648.0;
    v2   = ret * (double)clb->p8 / 32768.0;
    ret += (v1 + v2 + (double)clb->p7) / 16.0;
    ret /= 100.0;
  }

#else /* defined(USE_FLOAT) */
  int32_t v1;
  int32_t v2;
  uint32_t pa;

  v1  = (clb->t_fine >> 1) - 64000;

  v2  = (((v1 >> 2) * (v1 >> 2)) >> 11) * clb->p6;
  v2 += (v1 * clb->p5) << 1; 
  v2  = (v2 >> 2) + (clb->p4 << 16);

  v1  = (((clb->p3 * (((v1 >> 2) * (v1 >> 2)) >> 13)) >> 3) +
            ((clb->p2 * v1) >> 1)) >> 18;
  v1  = ((v1 + 32768) * clb->p1) >> 15;

  if (v1 == 0) {
    ret = 0.0;
  } else {
    pa  = ((uint32_t)((1048576 - raw) - (v2 >> 12))) * 3125;

    if (pa < 0x80000000) {
      pa = (pa << 1) / (uint32_t)v1;
    } else {
      pa = (pa / (uint32_t)v1) * 2;
    }

    v1  = (clb->p9 * (((pa >> 3) * (pa >> 3)) >> 13)) >> 12;
    v2  = (((int32_t)pa >> 2) * clb->p8) >> 13;
    pa  = (uint32_t)((int32_t)pa + ((v1 + v2 + clb->p7) >> 4));

    ret = pa / 100.0;
  }
#endif /* defined(USE_FLOAT) */

  return ret;
}

int
reset(i2c_t* sensor)
{
  int err;
  uint8_t val;

  do {
    err = i2c_rd_u8(sensor, REG_ID, &val);
    if (err) {
      fprintf(stderr, "ID regsiter read failed.\n");
      break;
    }

    if (val != 0x60) {
      fprintf(stderr, "ID value not match.\n");
      break;
    }

    err = i2c_wr_u8(sensor, REG_RESET, RESET_VAL);
    if (err) {
      fprintf(stderr, "RESET regsiter write failed.\n");
      break;
    }

    do {
      usleep(1000000);
      i2c_rd_u8(sensor, REG_RESET, &val);
    } while(val != 0);
  } while(0);

  return err;
}

int
setup(i2c_t* sensor)
{
  int err;
  const uint8_t osrs_h = 1;
  const uint8_t osrs_t = 1;
  const uint8_t osrs_p = 1;
  const uint8_t mode   = 3;

  do {
    err = i2c_wr_u8(sensor, REG_CTLR_HUM, osrs_h & 0x07);
    if (err) {
      fprintf(stderr, "CTRL_HUM regsiter write failed.\n");
      break;
    }

    err = i2c_wr_u8(sensor, REG_CTLR_MEAS, VAL_CTR_MEAS(osrs_t, osrs_p, mode));
    if (err) {
      fprintf(stderr, "CTRL_MEAS regsiter write failed.\n");
      break;
    }
  } while(0);

  return err;
}


int
read_calib(i2c_t* sensor, bme280_calib_t* clb)
{
  uint8_t msb;
  uint8_t lsb;
  uint8_t xsb;

  /*
   * for temperature
   */
  i2c_rd_u8(sensor, 0x88, &lsb);
  i2c_rd_u8(sensor, 0x89, &msb);

  clb->t1 = U16(lsb, msb);

  i2c_rd_u8(sensor, 0x8A, &lsb);
  i2c_rd_u8(sensor, 0x8B, &msb);

  clb->t2 = S16(lsb, msb);

  i2c_rd_u8(sensor, 0x8C, &lsb);
  i2c_rd_u8(sensor, 0x8D, &msb);

  clb->t3 = S16(lsb, msb);

  /*
   * for air pressure
   */
  i2c_rd_u8(sensor, 0x8E, &lsb);
  i2c_rd_u8(sensor, 0x8F, &msb);

  clb->p1 = U16(lsb, msb);

  i2c_rd_u8(sensor, 0x90, &lsb);
  i2c_rd_u8(sensor, 0x91, &msb);

  clb->p2 = S16(lsb, msb);

  i2c_rd_u8(sensor, 0x92, &lsb);
  i2c_rd_u8(sensor, 0x93, &msb);

  clb->p3 = S16(lsb, msb);

  i2c_rd_u8(sensor, 0x94, &lsb);
  i2c_rd_u8(sensor, 0x95, &msb);

  clb->p4 = S16(lsb, msb);

  i2c_rd_u8(sensor, 0x96, &lsb);
  i2c_rd_u8(sensor, 0x97, &msb);

  clb->p5 = S16(lsb, msb);

  i2c_rd_u8(sensor, 0x98, &lsb);
  i2c_rd_u8(sensor, 0x99, &msb);

  clb->p6 = S16(lsb, msb);

  i2c_rd_u8(sensor, 0x9A, &lsb);
  i2c_rd_u8(sensor, 0x9B, &msb);

  clb->p7 = S16(lsb, msb);

  i2c_rd_u8(sensor, 0x9C, &lsb);
  i2c_rd_u8(sensor, 0x9D, &msb);

  clb->p8 = S16(lsb, msb);

  i2c_rd_u8(sensor, 0x9E, &lsb);
  i2c_rd_u8(sensor, 0x9F, &msb);

  clb->p9 = S16(lsb, msb);


  /*
   * for humidity
   */
  i2c_rd_u8(sensor, 0xA1, &lsb);

  clb->h1 = lsb;

  i2c_rd_u8(sensor, 0xE1, &lsb);
  i2c_rd_u8(sensor, 0xE2, &msb);

  clb->h2 = S16(lsb, msb);

  i2c_rd_u8(sensor, 0xE3, &lsb);

  clb->h3 = lsb;

  i2c_rd_u8(sensor, 0xE4, &lsb);
  i2c_rd_u8(sensor, 0xE5, &msb);
  i2c_rd_u8(sensor, 0xE6, &xsb);

  clb->h4 = (((int16_t)lsb << 4) & 0x0ff0)|
            (((int16_t)msb << 0) & 0x000f);

  clb->h5 = (((int16_t)msb >> 4) & 0x000f)|
            (((int16_t)xsb << 4) & 0x00ff);

  i2c_rd_u8(sensor, 0xE7, &lsb);

  clb->h6 = (int8_t)lsb;

  /*
   * humidity cariblator
   */
  clb->t_fine = -1;

  return 0;
}

int
main(int argc, char* argv[])
{
  i2c_t* sensor;
  uint8_t val;
  uint8_t msb;
  uint8_t lsb;
  uint8_t xsb;

  double temp;
  double hum;
  double press;

  bme280_calib_t clb;

  i2c_new(1, 0x76, &sensor);

  reset(sensor);
  setup(sensor);
  read_calib(sensor, &clb);


  while(1) {
    do {
      i2c_rd_u8(sensor, REG_STATUS, &val);
      usleep(1000);
    } while(val & 0x08);

    i2c_rd_u8(sensor, REG_TEMP_MSB, &msb);
    i2c_rd_u8(sensor, REG_TEMP_LSB, &lsb);
    i2c_rd_u8(sensor, REG_TEMP_XSB, &xsb);
    temp = calc_temperature(msb, lsb, xsb, &clb);

    i2c_rd_u8(sensor, REG_HUM_MSB, &msb);
    i2c_rd_u8(sensor, REG_HUM_LSB, &lsb);
    hum = calc_humidity(msb, lsb, &clb);

    i2c_rd_u8(sensor, REG_PRESS_MSB, &msb);
    i2c_rd_u8(sensor, REG_PRESS_LSB, &lsb);
    i2c_rd_u8(sensor, REG_PRESS_XSB, &xsb);
    press = calc_air_pressure(msb, lsb, xsb, &clb);

    printf("%f %f %f\n", temp, hum, press);

    sleep(1);
  }


  return 0;
}
