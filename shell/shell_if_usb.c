#include <stdio.h>
#include <stdlib.h>
#include "fsl_device_registers.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_class.h"
#include "usb_device_cdc_acm.h"
#include "usb_device_ch9.h"

#include "usb_device_descriptor.h"
#include "virtual_com.h"
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
#include "fsl_sysmpu.h"
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

#if ((defined FSL_FEATURE_SOC_USBPHY_COUNT) && (FSL_FEATURE_SOC_USBPHY_COUNT > 0U))
#include "usb_phy.h"
#endif

#include "shell.h"
#include "shell_if_usb.h"
#include "circ_buffer.h"

#if defined(FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED) && (FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED > 0U) && \
    defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE) && (USB_DEVICE_CONFIG_KEEP_ALIVE_MODE > 0U) &&             \
    defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U)
extern uint8_t USB_EnterLowpowerMode(void);
#endif
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void USB_DeviceClockInit(void);
static void USB_DeviceIsrEnable(void);

static usb_status_t USB_DeviceCdcVcomCallback(class_handle_t handle, uint32_t event, void *param);
static usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param);

/*******************************************************************************
 * Variables
 ******************************************************************************/
extern usb_device_endpoint_struct_t g_UsbDeviceCdcVcomDicEndpoints[];
extern usb_device_class_struct_t g_UsbDeviceCdcVcomConfig;
/* Data structure of virtual com device */
usb_cdc_vcom_struct_t s_cdcVcom;

/* Line coding of cdc device */
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
static uint8_t s_lineCoding[LINE_CODING_SIZE] =
{
  /* E.g. 0x00,0xC2,0x01,0x00 : 0x0001C200 is 115200 bits per second */
  (LINE_CODING_DTERATE >> 0U) & 0x000000FFU,
  (LINE_CODING_DTERATE >> 8U) & 0x000000FFU,
  (LINE_CODING_DTERATE >> 16U) & 0x000000FFU,
  (LINE_CODING_DTERATE >> 24U) & 0x000000FFU,
  LINE_CODING_CHARFORMAT,
  LINE_CODING_PARITYTYPE,
  LINE_CODING_DATABITS
};

/* Abstract state of cdc device */
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
static uint8_t s_abstractState[COMM_FEATURE_DATA_SIZE] = {(STATUS_ABSTRACT_STATE >> 0U) & 0x00FFU,
                                                          (STATUS_ABSTRACT_STATE >> 8U) & 0x00FFU};

/* Country code of cdc device */
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
static uint8_t s_countryCode[COMM_FEATURE_DATA_SIZE] = {(COUNTRY_SETTING >> 0U) & 0x00FFU,
                                                        (COUNTRY_SETTING >> 8U) & 0x00FFU};

/* CDC ACM information */
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static usb_cdc_acm_info_t s_usbCdcAcmInfo;
/* Data buffer for receiving and sending*/
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_currRecvBuf[DATA_BUFF_SIZE];
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_currSendBuf[DATA_BUFF_SIZE];
volatile static uint32_t s_recvSize = 0;

#define SHELL_TX_BUFFER_SIZE        1024

static ShellIntf              _shell_usb_if;
static uint8_t                _rx_buffer[CLI_RX_BUFFER_LENGTH];
static uint8_t                _tx_buffer[SHELL_TX_BUFFER_SIZE];
static CircBuffer             _rx_cb;
static CircBuffer             _tx_cb;
static volatile bool          _tx_busy = false;

/* USB device class information */
static usb_device_class_config_struct_t s_cdcAcmConfig[1] =
{
  {
    USB_DeviceCdcVcomCallback,
    0,
    &g_UsbDeviceCdcVcomConfig,
  }
};

/* USB device class configuration information */
static usb_device_class_config_list_struct_t s_cdcAcmConfigList = 
{
  s_cdcAcmConfig,
  USB_DeviceCallback,
  1,
};

#if defined(FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED) && (FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED > 0U) && \
    defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE) && (USB_DEVICE_CONFIG_KEEP_ALIVE_MODE > 0U) &&             \
    defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U)
volatile static uint8_t s_waitForDataReceive = 0;
volatile static uint8_t s_comOpen            = 0;
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/

void
USB0_IRQHandler(void)
{
  USB_DeviceKhciIsrFunction(s_cdcVcom.deviceHandle);
  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
     exception return operation might vector to incorrect interrupt */
  __DSB();
}

static void
USB_DeviceClockInit(void)
{
  SystemCoreClockUpdate();
  CLOCK_EnableUsbfs0Clock(kCLOCK_UsbSrcIrc48M, 48000000U);
}

static void
USB_DeviceIsrEnable(void)
{
  uint8_t irqNumber;

  uint8_t usbDeviceKhciIrq[] = USB_IRQS;
  irqNumber                  = usbDeviceKhciIrq[CONTROLLER_ID - kUSB_ControllerKhci0];

  /* Install isr, set priority, and enable IRQ. */
  NVIC_SetPriority((IRQn_Type)irqNumber, USB_DEVICE_INTERRUPT_PRIORITY);
  EnableIRQ((IRQn_Type)irqNumber);
}

/*!
 * @brief CDC class specific callback function.
 *
 * This function handles the CDC class specific requests.
 *
 * @param handle          The CDC ACM class handle.
 * @param event           The CDC ACM class event type.
 * @param param           The parameter of the class specific request.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t
USB_DeviceCdcVcomCallback(class_handle_t handle, uint32_t event, void *param)
{
  uint32_t len;
  uint8_t *uartBitmap;
  usb_device_cdc_acm_request_param_struct_t *acmReqParam;
  usb_device_endpoint_callback_message_struct_t *epCbParam;
  usb_status_t error          = kStatus_USB_Error;
  usb_cdc_acm_info_t *acmInfo = &s_usbCdcAcmInfo;
  acmReqParam                 = (usb_device_cdc_acm_request_param_struct_t *)param;
  epCbParam                   = (usb_device_endpoint_callback_message_struct_t *)param;

  switch (event)
  {
  case kUSB_DeviceCdcEventSendResponse:
    {
      if ((epCbParam->length != 0) && (!(epCbParam->length % g_UsbDeviceCdcVcomDicEndpoints[0].maxPacketSize)))
      {
        /* If the last packet is the size of endpoint, then send also zero-ended packet,
         ** meaning that we want to inform the host that we do not have any additional
         ** data, so it can flush the output.
         */
        error = USB_DeviceCdcAcmSend(handle, USB_CDC_VCOM_BULK_IN_ENDPOINT, NULL, 0);
      }
      else if ((1 == s_cdcVcom.attach) && (1 == s_cdcVcom.startTransactions))
      {
        if ((epCbParam->buffer != NULL) || ((epCbParam->buffer == NULL) && (epCbParam->length == 0)))
        {
          /* User: add your own code for send complete event */
          /* Schedule buffer for next receive event */
          error = USB_DeviceCdcAcmRecv(handle, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_currRecvBuf,
              g_UsbDeviceCdcVcomDicEndpoints[0].maxPacketSize);
          _tx_busy = false;
#if defined(FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED) && (FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED > 0U) && \
          defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE) && (USB_DEVICE_CONFIG_KEEP_ALIVE_MODE > 0U) &&             \
          defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U)
          s_waitForDataReceive = 1;
          USB0->INTEN &= ~USB_INTEN_SOFTOKEN_MASK;
#endif
        }
      }
      else
      {
      }
    }
    break;

  case kUSB_DeviceCdcEventRecvResponse:
    {
      if ((1 == s_cdcVcom.attach) && (1 == s_cdcVcom.startTransactions))
      {
        s_recvSize = epCbParam->length;

#if defined(FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED) && (FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED > 0U) && \
        defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE) && (USB_DEVICE_CONFIG_KEEP_ALIVE_MODE > 0U) &&             \
        defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U)
        s_waitForDataReceive = 0;
        USB0->INTEN |= USB_INTEN_SOFTOKEN_MASK;
#endif
        if (!s_recvSize)
        {
          /* Schedule buffer for next receive event */
          error = USB_DeviceCdcAcmRecv(handle, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_currRecvBuf,
              g_UsbDeviceCdcVcomDicEndpoints[0].maxPacketSize);
#if defined(FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED) && (FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED > 0U) && \
          defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE) && (USB_DEVICE_CONFIG_KEEP_ALIVE_MODE > 0U) &&             \
          defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U)
          s_waitForDataReceive = 1;
          USB0->INTEN &= ~USB_INTEN_SOFTOKEN_MASK;
#endif
        }
      }
    }
    break;

  case kUSB_DeviceCdcEventSerialStateNotif:
    ((usb_device_cdc_acm_struct_t *)handle)->hasSentState = 0;
    error                                                 = kStatus_USB_Success;
    break;

  case kUSB_DeviceCdcEventSendEncapsulatedCommand:
    break;

  case kUSB_DeviceCdcEventGetEncapsulatedResponse:
    break;

  case kUSB_DeviceCdcEventSetCommFeature:
    if (USB_DEVICE_CDC_FEATURE_ABSTRACT_STATE == acmReqParam->setupValue)
    {
      if (1 == acmReqParam->isSetup)
      {
        *(acmReqParam->buffer) = s_abstractState;
      }
      else
      {
        *(acmReqParam->length) = 0;
      }
    }
    else if (USB_DEVICE_CDC_FEATURE_COUNTRY_SETTING == acmReqParam->setupValue)
    {
      if (1 == acmReqParam->isSetup)
      {
        *(acmReqParam->buffer) = s_countryCode;
      }
      else
      {
        *(acmReqParam->length) = 0;
      }
    }
    else
    {
    }
    error = kStatus_USB_Success;
    break;

  case kUSB_DeviceCdcEventGetCommFeature:
    if (USB_DEVICE_CDC_FEATURE_ABSTRACT_STATE == acmReqParam->setupValue)
    {
      *(acmReqParam->buffer) = s_abstractState;
      *(acmReqParam->length) = COMM_FEATURE_DATA_SIZE;
    }
    else if (USB_DEVICE_CDC_FEATURE_COUNTRY_SETTING == acmReqParam->setupValue)
    {
      *(acmReqParam->buffer) = s_countryCode;
      *(acmReqParam->length) = COMM_FEATURE_DATA_SIZE;
    }
    else
    {
    }
    error = kStatus_USB_Success;
    break;

  case kUSB_DeviceCdcEventClearCommFeature:
    break;

  case kUSB_DeviceCdcEventGetLineCoding:
    *(acmReqParam->buffer) = s_lineCoding;
    *(acmReqParam->length) = LINE_CODING_SIZE;
    error                  = kStatus_USB_Success;
    break;

  case kUSB_DeviceCdcEventSetLineCoding:
    {
      if (1 == acmReqParam->isSetup)
      {
        *(acmReqParam->buffer) = s_lineCoding;
      }
      else
      {
        *(acmReqParam->length) = 0;
      }
    }
    error = kStatus_USB_Success;
    break;

  case kUSB_DeviceCdcEventSetControlLineState:
    {
      s_usbCdcAcmInfo.dteStatus = acmReqParam->setupValue;
      /* activate/deactivate Tx carrier */
      if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_CARRIER_ACTIVATION)
      {
        acmInfo->uartState |= USB_DEVICE_CDC_UART_STATE_TX_CARRIER;
      }
      else
      {
        acmInfo->uartState &= (uint16_t)~USB_DEVICE_CDC_UART_STATE_TX_CARRIER;
      }

      /* activate carrier and DTE. Com port of terminal tool running on PC is open now */
      if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE)
      {
        acmInfo->uartState |= USB_DEVICE_CDC_UART_STATE_RX_CARRIER;
      }
      /* Com port of terminal tool running on PC is closed now */
      else
      {
        acmInfo->uartState &= (uint16_t)~USB_DEVICE_CDC_UART_STATE_RX_CARRIER;
      }

      /* Indicates to DCE if DTE is present or not */
      acmInfo->dtePresent = (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE) ? 1 : 0;

      /* Initialize the serial state buffer */
      acmInfo->serialStateBuf[0] = NOTIF_REQUEST_TYPE;                /* bmRequestType */
      acmInfo->serialStateBuf[1] = USB_DEVICE_CDC_NOTIF_SERIAL_STATE; /* bNotification */
      acmInfo->serialStateBuf[2] = 0x00;                              /* wValue */
      acmInfo->serialStateBuf[3] = 0x00;
      acmInfo->serialStateBuf[4] = 0x00; /* wIndex */
      acmInfo->serialStateBuf[5] = 0x00;
      acmInfo->serialStateBuf[6] = UART_BITMAP_SIZE; /* wLength */
      acmInfo->serialStateBuf[7] = 0x00;
      /* Notify to host the line state */
      acmInfo->serialStateBuf[4] = acmReqParam->interfaceIndex;
      /* Lower byte of UART BITMAP */
      uartBitmap    = (uint8_t *)&acmInfo->serialStateBuf[NOTIF_PACKET_SIZE + UART_BITMAP_SIZE - 2];
      uartBitmap[0] = acmInfo->uartState & 0xFFu;
      uartBitmap[1] = (acmInfo->uartState >> 8) & 0xFFu;
      len           = (uint32_t)(NOTIF_PACKET_SIZE + UART_BITMAP_SIZE);
      if (0 == ((usb_device_cdc_acm_struct_t *)handle)->hasSentState)
      {
        error = USB_DeviceCdcAcmSend(handle, USB_CDC_VCOM_INTERRUPT_IN_ENDPOINT, acmInfo->serialStateBuf, len);
        if (kStatus_USB_Success != error)
        {
          //usb_echo("kUSB_DeviceCdcEventSetControlLineState error!");
        }
        ((usb_device_cdc_acm_struct_t *)handle)->hasSentState = 1;
      }

      /* Update status */
      if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_CARRIER_ACTIVATION)
      {
        /*  To do: CARRIER_ACTIVATED */
      }
      else
      {
        /* To do: CARRIER_DEACTIVATED */
      }
      if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE)
      {
        /* DTE_ACTIVATED */
        if (1 == s_cdcVcom.attach)
        {
          s_cdcVcom.startTransactions = 1;
#if defined(FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED) && (FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED > 0U) && \
          defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE) && (USB_DEVICE_CONFIG_KEEP_ALIVE_MODE > 0U) &&             \
          defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U)
          s_waitForDataReceive = 1;
          USB0->INTEN &= ~USB_INTEN_SOFTOKEN_MASK;
          s_comOpen = 1;
          //usb_echo("USB_APP_CDC_DTE_ACTIVATED\r\n");
#endif
        }
      }
      else
      {
        /* DTE_DEACTIVATED */
        if (1 == s_cdcVcom.attach)
        {
          s_cdcVcom.startTransactions = 0;
        }
      }
    }
    break;
  case kUSB_DeviceCdcEventSendBreak:
    break;
  default:
    break;
  }

  return error;
}

/*!
 * @brief USB device callback function.
 *
 * This function handles the usb device specific requests.
 *
 * @param handle          The USB device handle.
 * @param event           The USB device event type.
 * @param param           The parameter of the device specific request.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t
USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param)
{
  usb_status_t error = kStatus_USB_Error;
  uint16_t *temp16   = (uint16_t *)param;
  uint8_t *temp8     = (uint8_t *)param;

  switch (event)
  {
    case kUSB_DeviceEventBusReset:
      {
        s_cdcVcom.attach               = 0;
        s_cdcVcom.currentConfiguration = 0U;
#if (defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)) || \
        (defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))
        /* Get USB speed to configure the device, including max packet size and interval of the endpoints. */
        if (kStatus_USB_Success == USB_DeviceClassGetSpeed(CONTROLLER_ID, &s_cdcVcom.speed))
        {
          USB_DeviceSetSpeed(handle, s_cdcVcom.speed);
        }
#endif
      }
      break;
    case kUSB_DeviceEventSetConfiguration:
      if (0U == (*temp8))
      {
        s_cdcVcom.attach               = 0;
        s_cdcVcom.currentConfiguration = 0U;
      }
      else if (USB_CDC_VCOM_CONFIGURE_INDEX == (*temp8))
      {
        s_cdcVcom.attach               = 1;
        s_cdcVcom.currentConfiguration = *temp8;
        /* Schedule buffer for receive */
        USB_DeviceCdcAcmRecv(s_cdcVcom.cdcAcmHandle, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_currRecvBuf,
            g_UsbDeviceCdcVcomDicEndpoints[0].maxPacketSize);
      }
      else
      {
        error = kStatus_USB_InvalidRequest;
      }
      break;
    case kUSB_DeviceEventSetInterface:
      if (s_cdcVcom.attach)
      {
        uint8_t interface        = (uint8_t)((*temp16 & 0xFF00U) >> 0x08U);
        uint8_t alternateSetting = (uint8_t)(*temp16 & 0x00FFU);
        if (interface < USB_CDC_VCOM_INTERFACE_COUNT)
        {
          s_cdcVcom.currentInterfaceAlternateSetting[interface] = alternateSetting;
        }
      }
      break;
    case kUSB_DeviceEventGetConfiguration:
      break;
    case kUSB_DeviceEventGetInterface:
      break;
    case kUSB_DeviceEventGetDeviceDescriptor:
      if (param)
      {
        error = USB_DeviceGetDeviceDescriptor(handle, (usb_device_get_device_descriptor_struct_t *)param);
      }
      break;
    case kUSB_DeviceEventGetConfigurationDescriptor:
      if (param)
      {
        error = USB_DeviceGetConfigurationDescriptor(handle,
            (usb_device_get_configuration_descriptor_struct_t *)param);
      }
      break;
    case kUSB_DeviceEventGetStringDescriptor:
      if (param)
      {
        /* Get device string descriptor request */
        error = USB_DeviceGetStringDescriptor(handle, (usb_device_get_string_descriptor_struct_t *)param);
      }
      break;
    default:
      break;
  }

  return error;
}

static uint8_t
shell_if_usb_get_rx_data(ShellIntf* intf, uint8_t* data)
{
  if(circ_buffer_dequeue(&_rx_cb, data, 1, false) == false)
  {
    return false;
  }
  return true;
}

static void
shell_if_usb_put_tx_data(ShellIntf* intf, uint8_t* data, uint16_t len)
{
  // we don't care buffer overflow
  circ_buffer_enqueue(&_tx_cb, data, len, false);
}

void
shell_if_usb_init(void)
{
  USB_DeviceClockInit();
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
  SYSMPU_Enable(SYSMPU, 0);
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

  s_cdcVcom.speed        = USB_SPEED_FULL;
  s_cdcVcom.attach       = 0;
  s_cdcVcom.cdcAcmHandle = (class_handle_t)NULL;
  s_cdcVcom.deviceHandle = NULL;

  if (kStatus_USB_Success != USB_DeviceClassInit(CONTROLLER_ID, &s_cdcAcmConfigList, &s_cdcVcom.deviceHandle))
  {
    //usb_echo("USB device init failed\r\n");
  }
  else
  {
    //usb_echo("USB device CDC virtual com demo\r\n");
    s_cdcVcom.cdcAcmHandle = s_cdcAcmConfigList.config->classHandle;
  }

  _shell_usb_if.cmd_buffer_ndx    = 0;
  _shell_usb_if.get_rx_data       = shell_if_usb_get_rx_data;
  _shell_usb_if.put_tx_data       = shell_if_usb_put_tx_data;

  circ_buffer_init(&_rx_cb, _rx_buffer, CLI_RX_BUFFER_LENGTH, NULL, NULL);
  circ_buffer_init(&_tx_cb, _tx_buffer, SHELL_TX_BUFFER_SIZE, NULL, NULL);

  INIT_LIST_HEAD(&_shell_usb_if.lh);
  shell_if_register(&_shell_usb_if);

  USB_DeviceIsrEnable();

  /*Add one delay here to make the DP pull down long enough to allow host to detect the previous disconnection.*/
  SDK_DelayAtLeastUs(5000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
  USB_DeviceRun(s_cdcVcom.deviceHandle);
}

void
shell_if_usb_task(void)
{
  usb_status_t error = kStatus_USB_Error;

  if ((1 == s_cdcVcom.attach) && (1 == s_cdcVcom.startTransactions))
  {
    if ((0 != s_recvSize) && (USB_CANCELLED_TRANSFER_LENGTH != s_recvSize))
    {
      circ_buffer_enqueue(&_rx_cb, s_currRecvBuf, s_recvSize, false);
      s_recvSize = 0;
      shell_handle_rx(&_shell_usb_if);
    }

    if(circ_buffer_is_empty(&_tx_cb, false) == false && _tx_busy == false)
    {
      int to_send = _tx_cb.num_bytes > DATA_BUFF_SIZE ? DATA_BUFF_SIZE : _tx_cb.num_bytes;

      circ_buffer_dequeue(&_tx_cb, s_currSendBuf, to_send, false);

      _tx_busy = true;
      error = USB_DeviceCdcAcmSend(s_cdcVcom.cdcAcmHandle, USB_CDC_VCOM_BULK_IN_ENDPOINT, s_currSendBuf, to_send);

      if (error != kStatus_USB_Success)
      {
        /* Failure to send Data Handling code here */
      }
    }
  }
}
