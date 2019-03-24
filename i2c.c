/*
 * I2C interface library
 *
 *  Copyright (C) 2016 Hiroshi Kuwagata.  All rights reserved.
 */

/*
 * $Id: i2c.c 113 2016-09-25 23:26:28Z kgt $
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <signal.h>
#include <setjmp.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "i2c.h"

#define ERR_DEFAULT       __LINE__

int
i2c_new(int bus, int slave, i2c_t** _obj)
{
  int ret;
  int fd;
  int err;
  i2c_t* obj;
  char path[32];

  /*
   * initialize
   */
  ret = 0;
  fd  = -1;
  obj = NULL;

  /*
   * error check
   */
  if (_obj == NULL) {
    ret = ERR_DEFAULT;
  }

  if (!ret) do {
    /*
     * alloc new object
     */
    obj = (i2c_t*)malloc(sizeof(i2c_t));
    if (obj == NULL) {
      perror(NULL);
      ret = ERR_DEFAULT;
      break;
    }

    /*
     * open ic2 bus device
     */
    sprintf(path, "/dev/i2c-%d", bus);
    fd = open(path, O_RDWR);
    if (fd < 0) {
      perror(NULL);
      ret = ERR_DEFAULT;
      break;
    }

    /*
     * select slave device
     */
    err = ioctl(fd, I2C_SLAVE, slave);
    if (err < 0) {
      perror(NULL);
      ret = ERR_DEFAULT;
      break;
    }

    /*
     * put return parameter
     */
    obj->bus   = bus;
    obj->slave = slave;
    obj->fd    = fd;

    *_obj  = obj;
  } while(0);

  /*
   * post process
   */
  if (ret) {
    if (obj != NULL) free(obj);
    if (fd >= 0) close(fd);
  }

  return ret;
}

int
i2c_destroy(i2c_t* ptr)
{
  int ret;

  /*
   * initialize
   */
  ret = 0;

  /*
   * error check
   */
  if (ptr == NULL) {
    ret = ERR_DEFAULT;
  }

  if (!ret) do {
    /*
     * close device
     */
    if (ptr->fd >= 0) {
      close(ptr->fd);
    }

    /*
     * release memory
     */
    free(ptr);
  } while(0);

  return ret;
}

int
i2c_rd_u8(i2c_t* ptr, uint8_t addr, uint8_t* val)
{
  int ret;
  int err;

  /*
   * initialize
   */
  ret = 0;

  /*
   * error check
   */
  if (ptr == NULL) {
    ret = ERR_DEFAULT;

  } else if (val == NULL) {
    ret = ERR_DEFAULT;
  }

  /*
   * read data
   */
  if (!ret) do {
    err = write(ptr->fd, &addr, 1);
    if (err < 0) {
      perror(NULL);
      ret = ERR_DEFAULT;
      break;
    }

    err = read(ptr->fd, ptr->buf, 1);
    if (err < 0) {
      perror(NULL);
      ret = ERR_DEFAULT;
      break;
    }
  } while(0);

  /*
   * put return parameter
   */
  if (!ret) {
    *val = ptr->buf[0];
  }

  return ret;
}

int
i2c_wr_u8(i2c_t* ptr, uint8_t addr, uint8_t val)
{
  int ret;
  int err;

  /*
   * initialize
   */
  ret = 0;

  /*
   * error check
   */
  if (ptr == NULL) {
    ret = ERR_DEFAULT;
  }

  /*
   * read data
   */
  if (!ret) {
    ptr->buf[0] = addr;
    ptr->buf[1] = val;

    err = write(ptr->fd, ptr->buf, 2);
    if (err < 0) {
      perror(NULL);
      ret = ERR_DEFAULT;
    }
  }

  return ret;
}

int
i2c_rd_u16(i2c_t* ptr, uint8_t addr, uint16_t* val)
{
  int ret;
  int err;

  /*
   * initialize
   */
  ret = 0;

  /*
   * error check
   */
  if (ptr == NULL) {
    ret = ERR_DEFAULT;

  } else if (val == NULL) {
    ret = ERR_DEFAULT;
  }

  /*
   * read data
   */
  if (!ret) {
    err = write(ptr->fd, &addr, 1);
    if (err < 0) {
      perror(NULL);
      ret = ERR_DEFAULT;
    }
  }

  if (!ret) {
    err = read(ptr->fd, ptr->buf, 2);
    if (err < 0) {
      perror(NULL);
      ret = ERR_DEFAULT;
    }
  }

  /*
   * put return parameter
   */
  if (!ret) {
    *val = ((ptr->buf[0] << 8) & 0xff00) | ((ptr->buf[1] << 0) & 0x00ff);
  }

  return ret;
}

int
i2c_wr_u16(i2c_t* ptr, uint8_t addr, uint16_t val)
{
  int ret;
  int err;

  /*
   * initialize
   */
  ret = 0;

  /*
   * error check
   */
  if (ptr == NULL) {
    ret = ERR_DEFAULT;
  }

  /*
   * read data
   */
  if (!ret) {
    ptr->buf[0] = addr;
    ptr->buf[1] = (val >> 8) & 0xff;
    ptr->buf[2] = (val >> 0) & 0xff;

    err = write(ptr->fd, ptr->buf, 3);
    if (err < 0) {
      perror(NULL);
      ret = ERR_DEFAULT;
    }
  }

  return ret;
}

int
i2c_rd_s16(i2c_t* ptr, uint8_t addr, int16_t* val)
{
  int ret;
  int err;

  /*
   * initialize
   */
  ret = 0;

  /*
   * error check
   */
  if (ptr == NULL) {
    ret = ERR_DEFAULT;

  } else if (val == NULL) {
    ret = ERR_DEFAULT;
  }

  /*
   * read data
   */
  if (!ret) {
    err = write(ptr->fd, &addr, 1);
    if (err < 0) {
      perror(NULL);
      ret = ERR_DEFAULT;
    }
  }

  if (!ret) {
    err = read(ptr->fd, ptr->buf, 2);
    if (err < 0) {
      perror(NULL);
      ret = ERR_DEFAULT;
    }
  }

  /*
   * put return parameter
   */
  if (!ret) {
    *val = ((ptr->buf[0] << 8) & 0xff00) | ((ptr->buf[1] << 0) & 0x00ff);
  }

  return ret;
}

/*
 * write 8bit command
 */
int
i2c_wr_c8(i2c_t* ptr, uint8_t cmd)
{
  int ret;
  int err;

  /*
   * initialize
   */
  ret = 0;

  /*
   * error check
   */
  if (ptr == NULL) {
    ret = ERR_DEFAULT;
  }

  /*
   * write data
   */
  if (!ret) {
    err = write(ptr->fd, &cmd, 1);
    if (err < 0) {
      perror(NULL);
      ret = ERR_DEFAULT;
    }
  }

  return ret;
}

int
i2c_wr(i2c_t* ptr, const uint8_t* data, int bytes)
{
  int ret;
  int err;

  /*
   * initialize
   */
  ret = 0;

  /*
   * error check
   */
  if (ptr == NULL) {
    ret = ERR_DEFAULT;
  } else if (data == NULL) {
    ret = ERR_DEFAULT;
  } else if (bytes < 0) {
    ret = ERR_DEFAULT;
  }

  /*
   * write data
   */
  if (!ret) {
    err = write(ptr->fd, data, bytes);
    if (err < 0) {
      perror(NULL);
      ret = ERR_DEFAULT;
    }
  }

  return ret;
}
