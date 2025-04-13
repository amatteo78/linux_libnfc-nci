/******************************************************************************
 *  Copyright 2021 NXP
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

 #include <errno.h>
#include <fcntl.h>
#ifdef ANDROID
#include <hardware/nfc.h>
#endif
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <NfccI2cTransport.h>
#include <NfccAltTransport.h>
#include <phNfcStatus.h>
#include <phNxpLog.h>
#include <string.h>
#include "phNxpNciHal_utils.h"
//section add to compatibility with libgpiod 2.x
#include <gpiod.h>
//section to add dynamic read gpiochip and pin
#include <phNxpConfig.h>
//end section

#define CRC_LEN 2
#define NORMAL_MODE_HEADER_LEN 3
#define FW_DNLD_HEADER_LEN 2
#define FW_DNLD_LEN_OFFSET 1
#define NORMAL_MODE_LEN_OFFSET 2
#define FRAGMENTSIZE_MAX PHNFC_I2C_FRAGMENT_SIZE
extern phTmlNfc_i2cfragmentation_t fragmentation_enabled;
extern phTmlNfc_Context_t* gpphTmlNfc_Context;

//section add to compatibility with libgpiod 2.x
struct gpiod_chip *ven_chip = NULL;
struct gpiod_chip *irq_chip = NULL;
struct gpiod_chip *fwdnld_chip = NULL;
struct gpiod_line_request *VEN_line = NULL;
struct gpiod_line_request *IRQ_line = NULL;
struct gpiod_line_request *FWDNLD_line = NULL;
//section to add dynamic read gpiochip and pin
char chip_ven_path[64] = "/dev/gpiochip3";
char chip_irq_path[64] = "/dev/gpiochip6";
char chip_fwd_path[64] = "/dev/gpiochip4";
unsigned long pin_ven = 1;
unsigned long pin_irq = 3;
unsigned long pin_fwd = 9;
//end section

NfccAltTransport::NfccAltTransport() {
  iEnableFd = 0;
  iInterruptFd = 0;
}

/*******************************************************************************
**
** Function         Flushdata
**
** Description      Reads payload of FW rsp from NFCC device into given buffer
**
** Parameters       pDevHandle - valid device handle
**                  pBuffer    - buffer for read data
**                  numRead    - number of bytes read by calling function
**
** Returns          always returns -1
**
*******************************************************************************/
int NfccAltTransport::Flushdata(void* pDevHandle, uint8_t* pBuffer,
                                int numRead) {
  int retRead = 0;
  uint16_t totalBtyesToRead =
      pBuffer[FW_DNLD_LEN_OFFSET] + FW_DNLD_HEADER_LEN + CRC_LEN;
  /* we shall read totalBtyesToRead-1 as one byte is already read by calling
   * function*/
  retRead = read((intptr_t)pDevHandle, pBuffer + numRead, totalBtyesToRead - 1);
  if (retRead > 0) {
    numRead += retRead;
    phNxpNciHal_print_packet("RECV", pBuffer, numRead);
  } else if (retRead == 0) {
    NXPLOG_TML_E("%s _i2c_read() [pyld] EOF", __func__);
  } else {
    if (bFwDnldFlag == false) {
      NXPLOG_TML_D("%s _i2c_read() [hdr] received", __func__);
      phNxpNciHal_print_packet("RECV", pBuffer - numRead,
                               NORMAL_MODE_HEADER_LEN);
    }
    NXPLOG_TML_E("%s _i2c_read() [pyld] errno : %x", __func__, errno);
  }
  SemPost();
  return -1;
}

/*******************************************************************************
**
** Function         Reset
**
** Description      Reset NFCC device, using VEN pin
**
** Parameters       pDevHandle     - valid device handle
**                  eType          - reset level
**
** Returns           0   - reset operation success
**                  -1   - reset operation failure
**
*******************************************************************************/
int NfccAltTransport::NfccReset(void* pDevHandle, NfccResetType eType) {
  int ret = -1;
  NXPLOG_TML_D("%s, VEN eType %ld", __func__, eType);

  if (NULL == pDevHandle) {
    return -1;
  }
  switch (eType) {
    case MODE_POWER_OFF:
      gpio_set_fwdl(0);
      gpio_set_ven(0);
      break;
    case MODE_POWER_ON:
      gpio_set_fwdl(0);
      gpio_set_ven(1);
      break;
    case MODE_FW_DWNLD_WITH_VEN:
      gpio_set_fwdl(1);
      gpio_set_ven(0);
      gpio_set_ven(1);
      break;
    case MODE_FW_DWND_HIGH:
      gpio_set_fwdl(1);
      break;
    case MODE_POWER_RESET:
      gpio_set_ven(0);
      gpio_set_ven(1);
      break;
    case MODE_FW_GPIO_LOW:
      gpio_set_fwdl(0);
      break;
    default:
      NXPLOG_TML_E("%s, VEN eType %ld", __func__, eType);
      return -1;
  }
  if ((eType != MODE_FW_DWNLD_WITH_VEN) && (eType != MODE_FW_DWND_HIGH)) {
    EnableFwDnldMode(false);
  }
  if ((eType == MODE_FW_DWNLD_WITH_VEN) || (eType == MODE_FW_DWND_HIGH)) {
    EnableFwDnldMode(true);
  }

  return ret;
}

/*******************************************************************************
**
** Function         GetNfcState
**
** Description      Get NFC state
**
** Parameters       pDevHandle     - valid device handle
** Returns           0   - unknown
**                   1   - FW DWL
**                   2 	 - NCI
**
*******************************************************************************/
int NfccAltTransport::GetNfcState(void* pDevHandle) {
  int ret = NFC_STATE_UNKNOWN;
  NXPLOG_TML_D("%s ", __func__);
  if (NULL == pDevHandle) {
    return ret;
  }
  ret = ioctl((intptr_t)pDevHandle, NFC_GET_NFC_STATE);
  NXPLOG_TML_D("%s :nfc state = %d", __func__, ret);
  return ret;
}
/*******************************************************************************
**
** Function         EnableFwDnldMode
**
** Description      updates the state to Download mode
**
** Parameters       True/False
**
** Returns          None
*******************************************************************************/
void NfccAltTransport::EnableFwDnldMode(bool mode) { bFwDnldFlag = mode; }

/*******************************************************************************
**
** Function         IsFwDnldModeEnabled
**
** Description      Returns the current mode
**
** Parameters       none
**
** Returns           Current mode download/NCI
*******************************************************************************/
bool_t NfccAltTransport::IsFwDnldModeEnabled(void) { return bFwDnldFlag; }

/*******************************************************************************
**
** Function         SemPost
**
** Description      sem_post 2c_read / write
**
** Parameters       none
**
** Returns          none
*******************************************************************************/
void NfccAltTransport::SemPost() {
  int sem_val = 0;
  sem_getvalue(&mTxRxSemaphore, &sem_val);
  if (sem_val == 0) {
    sem_post(&mTxRxSemaphore);
  }
}

/*******************************************************************************
**
** Function         SemTimedWait
**
** Description      Timed sem_wait for avoiding i2c_read & write overlap
**
** Parameters       none
**
** Returns          Sem_wait return status
*******************************************************************************/
int NfccAltTransport::SemTimedWait() {
  NFCSTATUS status = NFCSTATUS_FAILED;
  long sem_timedout = 500 * 1000 * 1000;
  int s = 0;
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec += 0;
  ts.tv_nsec += sem_timedout;
  while ((s = sem_timedwait(&mTxRxSemaphore, &ts)) == -1 && errno == EINTR) {
    continue; /* Restart if interrupted by handler */
  }
  if (s != -1) {
    status = NFCSTATUS_SUCCESS;
  } else if (errno == ETIMEDOUT && s == -1) {
    NXPLOG_TML_E("%s :timed out errno = 0x%x", __func__, errno);
  }
  return status;
}

/*******************************************************************************
**
** Function         GetIrqState
**
** Description      Get state of IRQ GPIO
**
** Parameters       pDevHandle - valid device handle
**
** Returns          The state of IRQ line i.e. +ve if read is pending else Zer0.
**                  In the case of IOCTL error, it returns -ve value.
**
*******************************************************************************/
int NfccAltTransport::GetIrqState(void* pDevHandle) {
  int ret = -1;

  NXPLOG_TML_D("%s Enter", __func__);
  int len;
  char buf[2];

  if (iInterruptFd <= 0) {
    NXPLOG_TML_E("Error with interrupt-detect pin (%d)", iInterruptFd);
    return (-1);
  }

  // Seek to the start of the file
  lseek(iInterruptFd, SEEK_SET, 0);

  // Read the field_detect line
  len = read(iInterruptFd, buf, 2);

  if (len != 2) {
    NXPLOG_TML_E("Error with interrupt-detect pin (%s)", strerror(errno));
    return (0);
  }

  NXPLOG_TML_D("%s exit: state = %d", __func__, (buf[0] != '0'));
  return (buf[0] != '0');
}

int NfccAltTransport::verifyPin(int pin, int isoutput, int edge) {
  char buf[40];
  // Check if gpio pin has already been created
  int hasGpio = 0;
  NXPLOG_TML_D("%s Enter", __func__);
  sprintf(buf, "/sys/class/gpio/gpio%d", pin);
  NXPLOG_TML_D("Pin %s\n", buf);
  int fd = open(buf, O_RDONLY);
  if (fd <= 0) {
    // Pin not exported yet
    NXPLOG_TML_D("Create pin %s\n", buf);
    if ((fd = open("/sys/class/gpio/export", O_WRONLY)) > 0) {
      sprintf(buf, "%d", pin);
      if (write(fd, buf, strlen(buf)) == strlen(buf)) {
        hasGpio = 1;
        usleep(100 * 1000);
      }
    } else {
      NXPLOG_TML_E("open failed for /sys/class/gpio/export\n");
      return -1;
    }
  } else {
    NXPLOG_TML_E("System already has pin %s\n", buf);
    hasGpio = 1;
  }
  close(fd);

  if (hasGpio) {
    // Make sure it is an output
    sprintf(buf, "/sys/class/gpio/gpio%d/direction", pin);
    NXPLOG_TML_D("Direction %s\n", buf);
    fd = open(buf, O_WRONLY);
    if (fd <= 0) {
      NXPLOG_TML_E("Could not open direction port '%s' (%s)", buf,
                   strerror(errno));
      return -1;
    } else {
      if (isoutput) {
        if (write(fd, "out", 3) == 3) {
          NXPLOG_TML_D("Pin %d now an output\n", pin);
        }
        close(fd);

        // Open pin and make sure it is off
        sprintf(buf, "/sys/class/gpio/gpio%d/value", pin);
        fd = open(buf, O_RDWR);
        if (fd <= 0) {
        }
        close(fd);

        // Open pin and make sure it is off
        sprintf(buf, "/sys/class/gpio/gpio%d/value", pin);
        fd = open(buf, O_RDWR);
        if (fd <= 0) {
          NXPLOG_TML_E("Could not open value port '%s' (%s)", buf,
                       strerror(errno));
          return -1;
        } else {
          if (write(fd, "0", 1) == 1) {
            NXPLOG_TML_D("Pin %d now off\n", pin);
          }
          return (fd);  // Success
        }
      } else {
        if (write(fd, "in", 2) == 2) {
          NXPLOG_TML_D("Pin %d now an input\n", pin);
        }
        close(fd);

        if (edge != EDGE_NONE) {
          // Open pin edge control
          sprintf(buf, "/sys/class/gpio/gpio%d/edge", pin);
          NXPLOG_TML_D("Edge %s\n", buf);
          fd = open(buf, O_RDWR);
          if (fd <= 0) {
            NXPLOG_TML_E("Could not open edge port '%s' (%s)", buf,
                         strerror(errno));
            return -1;
          } else {
            char* edge_str = "none";
            switch (edge) {
              case EDGE_RISING:
                edge_str = "rising";
                break;
              case EDGE_FALLING:
                edge_str = "falling";
                break;
              case EDGE_BOTH:
                edge_str = "both";
                break;
            }
            int l = strlen(edge_str);
            NXPLOG_TML_D("Edge-string %s - %d\n", edge_str, l);
            if (write(fd, edge_str, l) == l) {
              NXPLOG_TML_D("Pin %d trigger on %s\n", pin, edge_str);
            }
            close(fd);
          }
        }

        // Open pin
        sprintf(buf, "/sys/class/gpio/gpio%d/value", pin);
        NXPLOG_TML_D("Value %s\n", buf);
        fd = open(buf, O_RDONLY);
        if (fd <= 0) {
          NXPLOG_TML_E("Could not open value port '%s' (%s)", buf,
                       strerror(errno));
          return -1;
        } else {
          return (fd);  // Success
        }
      }
    }
  }
  return (0);
}

/*************************************************************************************************
   **
   ** Function         gpio_set_ven, gpio_set_fwdl (not Official)
   **
   ** Description      function to set pin VEN and FWDNLD to use with libgpiod 2.x
   **
   ** Parameters       value
   **
   ** Returns          
   **
   **                  not Officiale from NXP add from matteo.abrile@gmail.com 
   **
   ***********************************************************************************************/

void NfccAltTransport::gpio_set_ven(int value) {
  if (VEN_line) {
    gpiod_line_request_set_value(VEN_line, PIN_ENABLE, value ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
  }
  usleep(10*1000); // wait for 10ms
}

void NfccAltTransport::gpio_set_fwdl(int value) {
  if (FWDNLD_line) {
    gpiod_line_request_set_value(FWDNLD_line, PIN_FWDNLD, value ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
  }
  usleep(10*1000); // wait for 10ms
}

/*************************************************************************************************
   **
   ** Function         wait4interrupt (not Official)
   **
   ** Description      function to update when IRQ comes to use with libgpiod 2.x
   **
   ** Parameters       void
   **
   ** Returns          
   **
   **                  not Officiale from NXP add from matteo.abrile@gmail.com 
   **
   ***********************************************************************************************/

void NfccAltTransport::wait4interrupt(void) {
  if (!IRQ_line) return;

  while (true) {
    int value = gpiod_line_request_get_value(IRQ_line, PIN_INT);
    if (value < 0) {
      NXPLOG_TML_E("Errore lettura IRQ: %s", strerror(errno));
      break; // or retry
    }
    if (value == 1) break;
    usleep(100); // avoid busy loop
  }

  NXPLOG_TML_D("IRQ high comes");
}

/*************************************************************************************************
   **
   ** Function         ConfigurePin (not Official)
   **
   ** Description      Configure Pins such as IRQ, VEN, Firmware Download using for libgpiod 2.X
   **
   ** Parameters       none
   **
   ** Returns          NFCSTATUS_SUCCESS - on Success/ -1 on Failure
   **
   **                  not Officiale from NXP add from matteo.abrile@gmail.com 
   **
   ***********************************************************************************************/

int NfccAltTransport::ConfigurePin() {
  // section to add dynamic read gpiochip and pin
  GetNxpStrValue("NXP_GPIO_VEN_CHIP", chip_ven_path, sizeof(chip_ven_path));
  GetNxpNumValue("NXP_GPIO_VEN_PIN", &pin_ven, sizeof(pin_ven));
  GetNxpStrValue("NXP_GPIO_IRQ_CHIP", chip_irq_path, sizeof(chip_irq_path));
  GetNxpNumValue("NXP_GPIO_IRQ_PIN", &pin_irq, sizeof(pin_irq));
  GetNxpStrValue("NXP_GPIO_FWD_CHIP", chip_fwd_path, sizeof(chip_fwd_path));
  GetNxpNumValue("NXP_GPIO_FWD_PIN", &pin_fwd, sizeof(pin_fwd));

  // select chip
  ven_chip = gpiod_chip_open(chip_ven_path);
  fwdnld_chip = gpiod_chip_open(chip_fwd_path);
  irq_chip = gpiod_chip_open(chip_irq_path);

  if (!ven_chip || !fwdnld_chip || !irq_chip) {
    NXPLOG_TML_E("Error during chips open");
    return -1;
  }

  // same Config
  struct gpiod_request_config *req_cfg = gpiod_request_config_new();
  gpiod_request_config_set_consumer(req_cfg, "nxp-nfc");

  // IRQ INPUT
  struct gpiod_line_settings *settings_irq = gpiod_line_settings_new();
  gpiod_line_settings_set_direction(settings_irq, GPIOD_LINE_DIRECTION_INPUT);
  gpiod_line_settings_set_edge_detection(settings_irq, GPIOD_LINE_EDGE_RISING);
  gpiod_line_settings_set_bias(settings_irq, GPIOD_LINE_BIAS_DISABLED);

  struct gpiod_line_config *config_irq = gpiod_line_config_new();
  unsigned int irq_offsets[] = { (unsigned int)pin_irq };
  gpiod_line_config_add_line_settings(config_irq, irq_offsets, 1, settings_irq);
  IRQ_line = gpiod_chip_request_lines(irq_chip, req_cfg, config_irq);

  // delay before configure VEN and FWDNLD
  usleep(10 * 1000);

  // VEN e FWDNLD
  struct gpiod_line_settings *settings_out = gpiod_line_settings_new();
  gpiod_line_settings_set_direction(settings_out, GPIOD_LINE_DIRECTION_OUTPUT);
  gpiod_line_settings_set_output_value(settings_out, GPIOD_LINE_VALUE_ACTIVE);
  gpiod_line_settings_set_active_low(settings_out, false);

  struct gpiod_line_config *config_out = gpiod_line_config_new();

  unsigned int ven_offsets[] = { (unsigned int)pin_ven };
  gpiod_line_config_add_line_settings(config_out, ven_offsets, 1, settings_out);
  VEN_line = gpiod_chip_request_lines(ven_chip, req_cfg, config_out);

  unsigned int fwd_offsets[] = { (unsigned int)pin_fwd };
  gpiod_line_config_add_line_settings(config_out, fwd_offsets, 1, settings_out);
  FWDNLD_line = gpiod_chip_request_lines(fwdnld_chip, req_cfg, config_out);

  // final check
  if (!IRQ_line || !VEN_line || !FWDNLD_line) {
    NXPLOG_TML_E("Errors during lines configurations");
    return -1;
  }

  // Cleanup structures
  gpiod_line_settings_free(settings_irq);
  gpiod_line_settings_free(settings_out);
  gpiod_line_config_free(config_irq);
  gpiod_line_config_free(config_out);
  gpiod_request_config_free(req_cfg);

  return NFCSTATUS_SUCCESS;
}