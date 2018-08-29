#include "usbd_descriptor.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"
#include "usbd_msc.h"
#include "usbd_cdc.h"

static uint16_t usbd_builder_CfgDesc_len = 0;
static uint8_t*  usbd_builder_CfgDesc = NULL;

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4   
#endif
/* USB Base device Configuration Descriptor */
static uint8_t usbd_base_CfgDesc[USB_BASE_CONFIG_DESC_SIZ] =
{
  0x09, /* bLength: Configuation Descriptor size */
  USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION, /* bDescriptorType: Configuration */
  USB_BASE_CONFIG_DESC_SIZ + USB_MSC_CONFIG_DESC_SIZ + USB_CDC_CONFIG_DESC_SIZ,
  /* wTotalLength: Bytes returned */
  0x00,
  0x03,         /*bNumInterfaces: 3 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x02,         /*iConfiguration: Index of string descriptor describing the configuration*/
  0xC0,         /*bmAttributes: bus powered and Supports Remote Wakeup */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/
  /* 09 */
  
	/* each composit device */
	
};

/**
  * @brief  usbd_composite_GetCfgDesc 
  *         return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
uint8_t  *usbd_builder_GetCfgDesc (uint16_t *length)
{	
	// printf("%s\r\n", __func__);
	if( usbd_builder_CfgDesc == NULL ){
		uint8_t* ptr = NULL;
		uint8_t* pCfg = NULL;
		usbd_builder_CfgDesc_len = USB_BASE_CONFIG_DESC_SIZ + USB_MSC_CONFIG_DESC_SIZ + USB_CDC_CONFIG_DESC_SIZ;
		usbd_builder_CfgDesc = (uint8_t*) malloc(usbd_builder_CfgDesc_len);
		ptr = usbd_builder_CfgDesc;
		/* add base descriptor */
		memcpy(ptr, usbd_base_CfgDesc, USB_BASE_CONFIG_DESC_SIZ);
		
		ptr += USB_BASE_CONFIG_DESC_SIZ;
		uint16_t size = 0;
		
		pCfg = MSC_GetCfgDesc(&size);
		memcpy(ptr, pCfg, size);
		ptr += size;
		
		size = 0;
		pCfg = NULL;
		pCfg = CDC_GetCfgDesc(&size);
		memcpy(ptr, pCfg, size);
		ptr+=size;
		
		// printf("Length: %d Actual: %d\r\n", usbd_composite_CfgDesc_len, (ptr - usbd_composite_CfgDesc));
		
	}
	
  *length = usbd_builder_CfgDesc_len;
  return usbd_builder_CfgDesc;
}