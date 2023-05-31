/* SPI Master Half Duplex EEPROM example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_private/esp_clk.h"

#include "sdkconfig.h"
#include "esp_log.h"
//#include "spi_eeprom.h"
//#include "w5100s.h"
#include "wizchip_conf.h"
#include "socket.h"
#include "Application/loopback/loopback.h"

spi_device_handle_t spi;

#define SOCKET_LOOPBACK 0
#define PORT_LOOPBACK 5000
#define ETHERNET_BUF_MAX_SIZE (1024 * 2)
#define _LOOPBACK_DEBUG1_
wiz_NetInfo gWIZNETINFO = { .mac = {0x00,0x08,0xdc,0xff,0xff,0xff},
							.ip = {192,168,15,111},
							.sn = {255, 255, 255, 0},
							.gw = {192, 168, 15, 1},
							.dns = {168, 126, 63, 1},
							.lla = {0xfe,0x80, 0x00,0x00,
								 0x00,0x00, 0x00,0x00,
								 0x02,0x08, 0xdc,0xff,
								 0xfe,0xff, 0xff,0xff},
				            .gua={0x20,0x01,0x02,0xb8,
								 0x00,0x10,0x00,0x01,
								 0x02,0x08,0xdc,0xff,
								 0xfe,0xff,0xff,0xff},
				            .sn6={0xff,0xff,0xff,0xff,
								 0xff,0xff,0xff,0xff,
								 0x00,0x00,0x00, 0x00,
								 0x00,0x00,0x00,0x00},
				            .gw6={0xfe, 0x80, 0x00,0x00,
								  0x00,0x00,0x00,0x00,
								  0x02,0x00, 0x87,0xff,
								  0xfe,0x08, 0x4c,0x81}
							};
static wiz_NetInfo temp_g_net_info;
static uint8_t g_loopback_buf[ETHERNET_BUF_MAX_SIZE] = {
    0,
};

void wizchip_initialize(void);
//void network_initialize(wiz_NetInfo net_info1);
//void network_initialize(void);
//void print_network_information(wiz_NetInfo net_info);
void print_network_information(void);
//int32_t loopback_tcps1(uint8_t sn, uint8_t* buf, uint16_t port);
/*
 This code demonstrates how to use the SPI master half duplex mode to read/write a AT932C46D EEPROM (8-bit mode).
*/
#define WIZNET_HOST    VSPI_HOST//HSPI_HOST//VSPI_HOST
#    define PIN_NUM_MISO 19
#    define PIN_NUM_MOSI 23
#    define PIN_NUM_CLK  18
#    define PIN_NUM_CS   5
#    define PIN_NUM_INT   4
#    define PIN_NUM_RESET  16
#if 0
#ifdef CONFIG_IDF_TARGET_ESP32
#  ifdef CONFIG_EXAMPLE_USE_SPI1_PINS
#    define WIZNET_HOST    SPI1_HOST
// Use default pins, same as the flash chip.
#    define PIN_NUM_MISO 7
#    define PIN_NUM_MOSI 8
#    define PIN_NUM_CLK  6
#  else
#    define WIZNET_HOST    HSPI_HOST
#    define PIN_NUM_MISO 18
#    define PIN_NUM_MOSI 23
#    define PIN_NUM_CLK  19
#  endif

#  define PIN_NUM_CS   13
#elif defined CONFIG_IDF_TARGET_ESP32S2
#  define WIZNET_HOST    SPI2_HOST

#  define PIN_NUM_MISO 37
#  define PIN_NUM_MOSI 35
#  define PIN_NUM_CLK  36
#  define PIN_NUM_CS   34
#elif defined CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C2
#  define WIZNET_HOST    SPI2_HOST

#  define PIN_NUM_MISO 2
#  define PIN_NUM_MOSI 7
#  define PIN_NUM_CLK  6
#  define PIN_NUM_CS   10

#elif CONFIG_IDF_TARGET_ESP32S3
#  define WIZNET_HOST    SPI2_HOST

#  define PIN_NUM_MISO 13
#  define PIN_NUM_MOSI 11
#  define PIN_NUM_CLK  12
#  define PIN_NUM_CS   10
#endif
#endif
//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
void wiznet_spi_pre_transfer_callback(spi_transaction_t *t)
{
    //int dc=(int)t->user;
    //gpio_set_level(PIN_NUM_DC, dc);
}

static const char TAG[] = "main";

#if 0
static esp_err_t w5500_write_buf( uint32_t address, const void *value, uint32_t len)
{
    esp_err_t ret = ESP_OK;

    spi_transaction_t trans = {
        .cmd = 0xf0,//(address >> W5500_ADDR_OFFSET),
        .addr = (address & 0xFFFF),
        .length = 8 * len,
        .tx_buffer = value
    };
    if (spi_device_polling_transmit(spi, &trans) != ESP_OK) {
        ESP_LOGE(TAG, "%s(%d): spi transmit failed", __FUNCTION__, __LINE__);
        ret = ESP_FAIL;
    }
    return ret;
}

static esp_err_t w5500_read_buf(uint32_t address, void *value, uint32_t len)
{
    esp_err_t ret = ESP_OK;
//spi_device_acquire_bus(spi, portMAX_DELAY);
    spi_transaction_t trans = {
        .flags = len <= 4 ? SPI_TRANS_USE_RXDATA : 0, // use direct reads for registers to prevent overwrites by 4-byte boundary writes
        .cmd = 0x0f,//(address >> W5500_ADDR_OFFSET),
        .addr = (address & 0xFFFF),
        .length = 8 * len,
        .rx_buffer = value
    };
    if (spi_device_polling_transmit(spi, &trans) != ESP_OK) {
        ESP_LOGE(TAG, "%s(%d): spi transmit failed", __FUNCTION__, __LINE__);
        ret = ESP_FAIL;
    }
    //spi_device_release_bus(spi);

    if ((trans.flags&SPI_TRANS_USE_RXDATA) && len <= 4) {
        memcpy(value, trans.rx_data, len);  // copy register values to output
    }
    return ret;
}
#else
void Wizchip_write_buf(uint8_t *AddrSel, uint8_t *value, uint16_t len)
{
    esp_err_t ret = ESP_OK;

#if (_WIZCHIP_ == W6100)
    spi_transaction_t trans = {
        //.flags = len <= 4 ? SPI_TRANS_USE_TXDATA : 0,
        .cmd = (AddrSel[0]<<8)|(AddrSel[1]&0x00FF),//0xf0,//(address >> W5500_ADDR_OFFSET),
        .addr = AddrSel[2],//(address & 0xFFFF),
        .length = 8 * len,
        .tx_buffer = value
    };
#endif
    if (spi_device_polling_transmit(spi, &trans) != ESP_OK) {
        ESP_LOGE(TAG, "%s(%d): spi transmit failed", __FUNCTION__, __LINE__);
        ret = ESP_FAIL;
    }
    //spi_device_release_bus(spi);
    //return ret;
}

void Wizchip_read_buf(uint8_t *AddrSel, uint8_t *value, uint16_t len)
{
    esp_err_t ret = ESP_OK;
//spi_device_acquire_bus(spi, portMAX_DELAY);

#if(_WIZCHIP_ == W6100)
    spi_transaction_t trans = {
        .flags = len <= 4 ? SPI_TRANS_USE_RXDATA : 0, // use direct reads for registers to prevent overwrites by 4-byte boundary writes
        .cmd = (AddrSel[0]<<8)|(AddrSel[1]&0x00FF),//0xf0,//(address >> W5500_ADDR_OFFSET),
        .addr = AddrSel[2],//(address & 0xFFFF),
        .length = 8 * len,
        .rx_buffer = value
    };
#endif
    if (spi_device_polling_transmit(spi, &trans) != ESP_OK) {
        ESP_LOGE(TAG, "%s(%d): spi transmit failed", __FUNCTION__, __LINE__);
        ret = ESP_FAIL;
    }

    if ((trans.flags&SPI_TRANS_USE_RXDATA) && len <= 4) {
        memcpy(value, trans.rx_data, len);  // copy register values to output
    }
    //spi_device_release_bus(spi);
    //return ret;
}
#endif
void reset_pin_set(void)
{
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = 1<<PIN_NUM_RESET;//GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

void app_main(void)
{
    esp_err_t ret;
    //spi_device_handle_t spi;
    int i = 0;
    //int32_t loopback_ret = 0;
    uint8_t temp_recv[2];
    uint8_t temp_Addr[3];
    uint16_t temp_spi_clk = 30;
    uint8_t syslock = SYS_NET_LOCK;
    #if   (_WIZCHIP_ == W5100S)
    ESP_LOGI(TAG, "Wiznet W5100 TEST \r\n");
    #elif (_WIZCHIP_ == W5500)
    ESP_LOGI(TAG, "Wiznet W5500 TEST \r\n");
    #elif (_WIZCHIP_ == W6100)
    ESP_LOGI(TAG, "Wiznet W6100 TEST \r\n");
    #endif
#ifndef CONFIG_EXAMPLE_USE_SPI1_PINS
    ESP_LOGI(TAG, "Initializing bus SPI%d...", WIZNET_HOST+1);
    spi_bus_config_t buscfg={
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    
    spi_device_interface_config_t devcfg={
#ifdef CONFIG_LCD_OVERCLOCK
        .clock_speed_hz=26*1000*1000,           //Clock out at 26 MHz
#else
        .clock_speed_hz=temp_spi_clk*1000*1000,           //Clock out at 10 MHz
#endif
#if   (_WIZCHIP_ == W5100S)
        .address_bits = 16,
        .command_bits = 8,
#elif (_WIZCHIP_ == W5500)
        .address_bits = 8,
        .command_bits = 16,
#elif (_WIZCHIP_ == W6100)
        .address_bits = 8,
        .command_bits = 16,
#endif
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=wiznet_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
        .flags = SPI_DEVICE_NO_DUMMY,
    };
    reset_pin_set();
    gpio_set_level(PIN_NUM_RESET, 0);
    //Initialize the SPI bus
    ret = spi_bus_initialize(WIZNET_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(WIZNET_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "app clk = %d hz",esp_clk_apb_freq());
    ESP_LOGI(TAG, "SPI CLK = %d Mhz\r\n",temp_spi_clk);
    vTaskDelay(50);
    gpio_set_level(PIN_NUM_RESET, 1);
    vTaskDelay(20);
    #if   (_WIZCHIP_ == W5100S)
    temp_Addr[0] = 0x0F;
    temp_Addr[1] = 0x00;
    temp_Addr[2] = 0x80;
    Wizchip_read_buf(temp_Addr, &temp_recv, sizeof(temp_recv));
    //ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "W5100 VER =%02X \r\n", temp_recv);
    #elif (_WIZCHIP_ == W5500)
    temp_Addr[0] = 0x00;
    temp_Addr[1] = 0x39;
    temp_Addr[2] = 0x00;
    Wizchip_read_buf(temp_Addr, &temp_recv, sizeof(temp_recv));
    ESP_LOGI(TAG, "W5500 VER =%02X \r\n", temp_recv);
    #elif (_WIZCHIP_ == W6100)
    temp_Addr[0] = 0x00;
    temp_Addr[1] = 0x02;
    temp_Addr[2] = 0x00;
    Wizchip_read_buf(temp_Addr, temp_recv, 2);
    ESP_LOGI(TAG, "W6100 VER = %02X %02X\r\n", temp_recv[0], temp_recv[1]);
    #endif
    
    //printf("< W6100 clock TEST!! >\r\n");
	wizchip_initialize();
    
	ctlwizchip(CW_SYS_UNLOCK,& syslock);
    temp_recv[0] = getNETLCKR();
    ESP_LOGI(TAG, "NETLCKR = %02X", temp_recv[0]);
	ctlnetwork(CN_SET_NETINFO,&gWIZNETINFO);

	printf("VERSION(%x) = %.2x \r\n", _VER_,getVER());
	print_network_information();
    //print_network_information(temp_g_net_info);

    //print_network_information();
    ESP_LOGI(TAG,"socket mem size");
    for(i=0; i<8; i++)
    {
        ESP_LOGI(TAG,"%d - Rx:%02X Tx:%02X ", i, getSn_TXBUF_SIZE(i), getSn_RXBUF_SIZE(i));
    }
    ESP_LOGI(TAG, "socket 0 status = %02X",getSn_SR(0));
#else
    ESP_LOGI(TAG, "Attach to main flash bus...");
#endif
#if 0
    eeprom_config_t eeprom_config = {
        .cs_io = PIN_NUM_CS,
        .host = WIZNET_HOST,
        .miso_io = PIN_NUM_MISO,
    };
#ifdef CONFIG_EXAMPLE_INTR_USED
    eeprom_config.intr_used = true;
    gpio_install_isr_service(0);
#endif

    eeprom_handle_t eeprom_handle;

    ESP_LOGI(TAG, "Initializing device...");
    ret = spi_eeprom_init(&eeprom_config, &eeprom_handle);
    ESP_ERROR_CHECK(ret);

    ret = spi_eeprom_write_enable(eeprom_handle);
    ESP_ERROR_CHECK(ret);

    const char test_str[] = "Hello World!";
    ESP_LOGI(TAG, "Write: %s", test_str);
    for (int i = 0; i < sizeof(test_str); i++) {
        // No need for this EEPROM to erase before write.
        ret = spi_eeprom_write(eeprom_handle, i, test_str[i]);
        ESP_ERROR_CHECK(ret);
    }

    uint8_t test_buf[32] = "";
    for (int i = 0; i < sizeo8305268f(test_str); i++) {
        ret = spi_eeprom_read(eeprom_handle, i, &test_buf[i]);
        ESP_ERROR_CHECK(ret);
    }
    ESP_LOGI(TAG, "Read: %s", test_buf);

    ESP_LOGI(TAG, "Example finished.");
#endif
    ESP_LOGI(TAG, "loopback start");
    while (1) {
        // Add your main loop handling code here.
        //loopback_ret = loopback_tcps1(0,g_loopback_buf,5000);
        loopback_tcps(0,g_loopback_buf,5000, AS_IPV4);
        #if 0
        if(loopback_ret != 1)
            ESP_LOGI(TAG, "loopback ret = %d", (int)loopback_ret);
            #endif
        vTaskDelay(1);
    }
}

void wizchip_initialize(void)
{
    /* Deselect the FLASH : chip select high */
    //wizchip_deselect();

    /* CS function register */
    //reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);

    /* SPI function register */
    //reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);

    reg_wizchip_spi_cbfunc(0, 0, Wizchip_read_buf, Wizchip_write_buf);

#ifdef USE_SPI_DMA
    reg_wizchip_spiburst_cbfunc(wizchip_read_burst, wizchip_write_burst);
#endif
    /* W5x00 initialize */
    //uint8_t temp;
  uint8_t temp;
	unsigned char W6100_AdrSet[2][8] = {{2, 2, 2, 2, 2, 2, 2, 2}, {2, 2, 2, 2, 2, 2, 2, 2}};
	do
	{
		if (ctlwizchip(CW_GET_PHYLINK, (void *)&temp) == -1)
		{
			printf("Unknown PHY link status.\r\n");
		}
	} while (temp == PHY_LINK_OFF);
	printf("PHY OK.\r\n");

	temp = IK_DEST_UNREACH;

	if (ctlwizchip(CW_INIT_WIZCHIP, (void *)W6100_AdrSet) == -1)
	{
		printf("W6100 initialized fail.\r\n");
	}

	if (ctlwizchip(CW_SET_INTRMASK, &temp) == -1)
	{
		printf("W6100 interrupt\r\n");
	}
}
#if 0
void network_initialize(wiz_NetInfo net_info1)
{
    ctlnetwork(CN_SET_NETINFO, (void *)&net_info1);
}
#endif
void print_network_information(void)
{
    wiz_NetInfo temp_gWIZNETINFO ={0,};
	wizchip_getnetinfo(&temp_gWIZNETINFO);
	printf("Mac address: %02x:%02x:%02x:%02x:%02x:%02x\n\r",temp_gWIZNETINFO.mac[0],temp_gWIZNETINFO.mac[1],temp_gWIZNETINFO.mac[2],temp_gWIZNETINFO.mac[3],temp_gWIZNETINFO.mac[4],temp_gWIZNETINFO.mac[5]);
	printf("IP address : %d.%d.%d.%d\n\r",temp_gWIZNETINFO.ip[0],temp_gWIZNETINFO.ip[1],temp_gWIZNETINFO.ip[2],temp_gWIZNETINFO.ip[3]);
	printf("SN Mask	   : %d.%d.%d.%d\n\r",temp_gWIZNETINFO.sn[0],temp_gWIZNETINFO.sn[1],temp_gWIZNETINFO.sn[2],temp_gWIZNETINFO.sn[3]);
	printf("Gate way   : %d.%d.%d.%d\n\r",temp_gWIZNETINFO.gw[0],temp_gWIZNETINFO.gw[1],temp_gWIZNETINFO.gw[2],temp_gWIZNETINFO.gw[3]);
	printf("DNS Server : %d.%d.%d.%d\n\r",temp_gWIZNETINFO.dns[0],temp_gWIZNETINFO.dns[1],temp_gWIZNETINFO.dns[2],temp_gWIZNETINFO.dns[3]);
	printf("LLA  : %.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X\r\n",temp_gWIZNETINFO.lla[0],temp_gWIZNETINFO.lla[1],temp_gWIZNETINFO.lla[2],temp_gWIZNETINFO.lla[3],\
									temp_gWIZNETINFO.lla[4],temp_gWIZNETINFO.lla[5],temp_gWIZNETINFO.lla[6],temp_gWIZNETINFO.lla[7],\
									temp_gWIZNETINFO.lla[8],temp_gWIZNETINFO.lla[9],temp_gWIZNETINFO.lla[10],temp_gWIZNETINFO.lla[11],\
									temp_gWIZNETINFO.lla[12],temp_gWIZNETINFO.lla[13],temp_gWIZNETINFO.lla[14],temp_gWIZNETINFO.lla[15]);
	printf("GUA  : %.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X\n\r",temp_gWIZNETINFO.gua[0],temp_gWIZNETINFO.gua[1],temp_gWIZNETINFO.gua[2],temp_gWIZNETINFO.gua[3],\
									temp_gWIZNETINFO.gua[4],temp_gWIZNETINFO.gua[5],temp_gWIZNETINFO.gua[6],temp_gWIZNETINFO.gua[7],\
									temp_gWIZNETINFO.gua[8],temp_gWIZNETINFO.gua[9],temp_gWIZNETINFO.gua[10],temp_gWIZNETINFO.gua[11],\
									temp_gWIZNETINFO.gua[12],temp_gWIZNETINFO.gua[13],temp_gWIZNETINFO.gua[14],temp_gWIZNETINFO.gua[15]);
	printf("SN6  : %.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X\n\r",temp_gWIZNETINFO.sn6[0],temp_gWIZNETINFO.sn6[1],temp_gWIZNETINFO.sn6[2],temp_gWIZNETINFO.sn6[3],\
									temp_gWIZNETINFO.sn6[4],temp_gWIZNETINFO.sn6[5],temp_gWIZNETINFO.sn6[6],temp_gWIZNETINFO.sn6[7],\
									temp_gWIZNETINFO.sn6[8],temp_gWIZNETINFO.sn6[9],temp_gWIZNETINFO.sn6[10],temp_gWIZNETINFO.sn6[11],\
									temp_gWIZNETINFO.sn6[12],temp_gWIZNETINFO.sn6[13],temp_gWIZNETINFO.sn6[14],temp_gWIZNETINFO.sn6[15]);
	printf("GW6  : %.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X\r\n",temp_gWIZNETINFO.gw6[0],temp_gWIZNETINFO.gw6[1],temp_gWIZNETINFO.gw6[2],temp_gWIZNETINFO.gw6[3],\
									temp_gWIZNETINFO.gw6[4],temp_gWIZNETINFO.gw6[5],temp_gWIZNETINFO.gw6[6],temp_gWIZNETINFO.gw6[7],\
									temp_gWIZNETINFO.gw6[8],temp_gWIZNETINFO.gw6[9],temp_gWIZNETINFO.gw6[10],temp_gWIZNETINFO.gw6[11],\
									temp_gWIZNETINFO.gw6[12],temp_gWIZNETINFO.gw6[13],temp_gWIZNETINFO.gw6[14],temp_gWIZNETINFO.gw6[15]);
}
#if 0
int32_t loopback_tcps1(uint8_t sn, uint8_t* buf, uint16_t port)
{
   int32_t ret;
   uint16_t size = 0, sentsize=0;
    uint8_t status = 0;
#ifdef _LOOPBACK_DEBUG1_
   uint8_t destip[4];
   uint16_t destport;
#endif
    status = getSn_SR(sn);
    //ESP_LOGI(TAG,"status = %02X",status);
   switch(status)
   {
      case SOCK_ESTABLISHED :
         if(getSn_IR(sn) & Sn_IR_CON)
         {
#ifdef _LOOPBACK_DEBUG1_
			getSn_DIPR(sn, destip);
			destport = getSn_DPORT(sn);

			ESP_LOGI(TAG,"%d:Connected - %d.%d.%d.%d : %d\r\n",sn, destip[0], destip[1], destip[2], destip[3], destport);
#endif
			setSn_IR(sn,Sn_IR_CON);
         }
		 if((size = getSn_RX_RSR(sn)) > 0) // Don't need to check SOCKERR_BUSY because it doesn't not occur.
         {
			if(size > DATA_BUF_SIZE) size = DATA_BUF_SIZE;
			ret = recv(sn, buf, size);

			if(ret <= 0) return ret;      // check SOCKERR_BUSY & SOCKERR_XXX. For showing the occurrence of SOCKERR_BUSY.
			size = (uint16_t) ret;
			sentsize = 0;

			while(size != sentsize)
			{
				ret = send(sn, buf+sentsize, size-sentsize);
				if(ret < 0)
				{
					close(sn);
					return ret;
				}
				sentsize += ret; // Don't care SOCKERR_BUSY, because it is zero.
			}
         }
         break;
      case SOCK_CLOSE_WAIT :
#ifdef _LOOPBACK_DEBUG1_
         ESP_LOGI(TAG,"%d:CloseWait\r\n",sn);
#endif
         if((ret = disconnect(sn)) != SOCK_OK) return ret;
#ifdef _LOOPBACK_DEBUG1_
         ESP_LOGI(TAG,"%d:Socket Closed\r\n", sn);
#endif
         break;
      case SOCK_INIT :
#ifdef _LOOPBACK_DEBUG1_
    	 ESP_LOGI(TAG,"%d:Listen, TCP server loopback, port [%d]\r\n", sn, port);
#endif
         if( (ret = listen(sn)) != SOCK_OK) return ret;
         break;
      case SOCK_CLOSED:
#ifdef _LOOPBACK_DEBUG1_
         ESP_LOGI(TAG,"%d:TCP server loopback start\r\n",sn);
#endif
         if((ret = socket(sn, Sn_MR_TCP, port, 0x00)) != sn) return ret;
#ifdef _LOOPBACK_DEBUG1_
         ESP_LOGI(TAG,"%d:Socket opened\r\n",sn);
#endif
         break;
      default:
        return status;
         break;
   }
   return 1;
}
#endif