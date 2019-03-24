/*
 * I2C interface library
 *
 *  Copyright (C) 2016 Hiroshi Kuwagata.  All rights reserved.
 */

/*
 * $Id: i2c.h 112 2016-09-25 23:24:27Z pi $
 */

#ifndef __I2C_H__
#define __I2C_H__

typedef struct {
  int bus;
  int slave;
  int fd;
  uint8_t buf[8];
} i2c_t;

extern int i2c_new(int bus, int slave, i2c_t** _obj);
extern int i2c_destroy(i2c_t* ptr);
extern int i2c_rd_u8(i2c_t* ptr, uint8_t addr, uint8_t* val);
extern int i2c_wr_u8(i2c_t* ptr, uint8_t addr, uint8_t val);
extern int i2c_rd_u16(i2c_t* ptr, uint8_t addr, uint16_t* val);
extern int i2c_wr_u16(i2c_t* ptr, uint8_t addr, uint16_t val);
extern int i2c_rd_s16(i2c_t* ptr, uint8_t addr, int16_t* val);

extern int i2c_wr_c8(i2c_t* ptr, uint8_t cmd);
extern int i2c_wr(i2c_t* ptr, const uint8_t* data, int byte);

#endif /* !defined(__I2C_H__) */
