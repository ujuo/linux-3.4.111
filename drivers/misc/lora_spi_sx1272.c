/*
 *  lora_spi_sx1272.c - Linux kernel module for
 * 	SX1272 LoRa/rf_modulater driver.
 *
 *  Copyright (c) 2016 STcube Inc.,
 *  All right reserved by Seungwoo Kim <ksw.stcube.com> 
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>

#include <linux/major.h>
#include <linux/errno.h>
#include <linux/tty.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/vt_kern.h>
#include <linux/selection.h>
#include <linux/console.h>

#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/leds.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <asm/mach-types.h>

#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinconf-sunxi.h>

#include <mach/gpio.h>
#include <mach/platform.h>
#include <mach/sys_config.h>

#include "sx1272_device.h"

#if (0)
#define DBGOUT(msg...)	{ printk(KERN_INFO "[sx1272] " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

#define DEBUG_DUMP	0

#define SX1272_3WIRE_DRV_NAME	"sx1272_spi"
#define SX1272_DEV_MAJOR			220

#define DRIVER_VERSION		"0.65"
#define SX1272_DRIVER_VERSION	0x0065

//#define SX1272_RESET_GPIO  (PAD_GPIO_C + 20)

typedef struct {
	int init;
	char  pin_name[32];
	
	int gpio;
	int mul_sel;
	int pull;
	int drv_level;
	int data;
	
	unsigned int config_mul;
	unsigned int config_pull;
	unsigned int config_drv;
	unsigned int config_data;
} rf_gpio_cfg;

typedef struct {
	struct spi_device *spi;
	unsigned int status;
	unsigned int flag;
	unsigned int base_freq;
	unsigned int chan;
	unsigned int version;
	int virq;

	unsigned char reg_1d;
	unsigned char reg_1e;
	unsigned char reg_31;
	unsigned char reg_37;
	unsigned char frq_msb;
	unsigned char frq_mib;
	unsigned char frq_lsb;
	
	rf_gpio_cfg rx_switch;
	rf_gpio_cfg tx_switch;
	rf_gpio_cfg reset;
	rf_gpio_cfg irq;
	
	struct mutex                rcv_lock;
	struct mutex                send_lock;

	wait_queue_head_t           waitq;
	
	struct workqueue_struct 	*send_workq;
	struct work_struct  		work;
	
	unsigned char buffer[128];
} sx1272_device_t;

sx1272_device_t *sx1272;
static struct class *sx1272_class;

//static struct	NX_GPIO_RegisterSet *gpioc = (struct NX_GPIO_RegisterSet *)(IO_ADDRESS(PHY_BASEADDR_GPIO + 0x80));

static ssize_t sx1272_store_val(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev);
	unsigned char tmp[2];
	unsigned long val;

	if (strict_strtoul(buf, 10, &val) < 0)
		return -EINVAL;

	tmp[0] = val >> 8;
	tmp[1] = val & 0xff;
	spi_write(spi, tmp, sizeof(tmp));
	return count;
}

static DEVICE_ATTR(value, S_IWUSR, NULL, sx1272_store_val);

static struct attribute *sx1272_attributes[] = {
	&dev_attr_value.attr,
	NULL
};

static const struct attribute_group sx1272_attr_group = {
	.attrs = sx1272_attributes,
};

static inline void spi_message_init_with_transfers(struct spi_message *m, struct spi_transfer *xfers, unsigned int num_xfers)
{
	unsigned int i;

	spi_message_init(m);
	for (i = 0; i < num_xfers; ++i)
		spi_message_add_tail(&xfers[i], m);
}

static inline int spi_sync_transfer(struct spi_device *spi, struct spi_transfer *xfers, unsigned int num_xfers)
{
	struct spi_message msg;

	spi_message_init_with_transfers(&msg, xfers, num_xfers);
	spi->bits_per_word = 8;

	return spi_sync(spi, &msg);
}
/*
 * gpio function
*/
static int rf_gpio_request(rf_gpio_cfg *list, int count_max)
{
	int ret;
	
	if (list == NULL || list->gpio == GPIO_INDEX_INVALID) {
		return -1;
	}

	sunxi_gpio_to_name(list->gpio, list->pin_name);
	ret = gpio_request(list->gpio, NULL);
	if(0 != ret) {
		printk("GPIO request failed gpio_name=%s, gpio=%d ret=0x%x\n", list->pin_name, list->gpio, ret);
		return -2;
	}
	return 0;
}

static int rf_gpio_set(rf_gpio_cfg *list, int count_max)
{
	if (list == NULL || list->gpio == GPIO_INDEX_INVALID) {
		return -1;
	}
	if (list->init == 0) {
		sunxi_gpio_to_name(list->gpio, list->pin_name);
		list->config_mul = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_FUNC, list->mul_sel);
		pin_config_set(SUNXI_PINCTRL, list->pin_name, list->config_mul);

		list->config_pull = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_PUD, list->pull);
		if (list->pull != GPIO_PULL_DEFAULT) {
			pin_config_set(SUNXI_PINCTRL, list->pin_name, list->config_pull);
		}
		if (list->drv_level != GPIO_DRVLVL_DEFAULT) {
			list->config_drv = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_DRV, list->drv_level);
			pin_config_set(SUNXI_PINCTRL, list->pin_name, list->config_drv);
		}
		if (list->data != GPIO_DATA_DEFAULT) {
			list->config_data = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_DAT, list->data);
			pin_config_set(SUNXI_PINCTRL, list->pin_name, list->config_data);
		}
		list->init = 1;
	} else {
		list->config_data = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_DAT, list->data);
		pin_config_set(SUNXI_PINCTRL, list->pin_name, list->config_data);
	}

	return 0;
}
/*
 * irq routine
 */
static irqreturn_t sx1272_irq(int irq, void *dev_id)
{
	sx1272_device_t *sx = (sx1272_device_t *)dev_id;
	
	wake_up_interruptible(&sx->waitq);
	queue_work(sx->send_workq, &sx->work);
	
	return IRQ_HANDLED;
}
/*
 * work queue function
 */
static void send_work_func(struct work_struct *work)
{
}

/*--------------------------------------------------------------------------------
 * file_operations
 ---------------------------------------------------------------------------------*/
static int sx1272_open(struct inode *inode, struct file *flip)
{
	DBGOUT("%s (minor:%d)\n", __func__, minor);
	
	flip->private_data = sx1272;

	return 0;
}

static int sx1272_release(struct inode *inode, struct file *flip)
{
	DBGOUT("%s (minor:%d, table:%d)\n", __func__, minor, NUM_OF_PARTS);

	return 0;
}


static void sx1272_reset(sx1272_device_t *sx)
{
	// Something to reset device ?
	sx->reset.data = 0;
	rf_gpio_set(&sx->reset, 1);
	/* sleep some time */
	msleep(1);
	sx->reset.data = 1;
	rf_gpio_set(&sx->reset, 1);
}

void sx1272_write_reg(sx1272_device_t *sx, unsigned char reg, unsigned char byte)
{
	struct spi_transfer xfer;
	unsigned char rxbuf[4];
	unsigned char txbuf[4];
	
	txbuf[0] = reg | 0x80;
	txbuf[1] = byte;
	xfer.rx_buf = rxbuf;
	xfer.tx_buf = txbuf;
	xfer.len = 2;
	spi_sync_transfer(sx->spi, &xfer, 1);
}

void sx1272_read_reg_delta(sx1272_device_t *sx, unsigned char reg, unsigned char *data)
{
	struct spi_transfer xfer;
	unsigned char rxbuf[4];
	unsigned char txbuf[4];

	txbuf[0] = reg & 0x7F;
	xfer.rx_buf = rxbuf;
	xfer.tx_buf = txbuf;
	xfer.len = 2;
	spi_sync_transfer(sx->spi, &xfer, 1);
}

#define FSTEP (32000000.0 / 524288.0)

void set_freq(sx1272_device_t *sx, int freq, int chan)
{
	/* Calc frequency by freq into MSB, MIB, LSB */
	/* Freq = Fstep * Frf(23:0) */
	/* Frf = Freq / Fstep, Fstep = 61.0Hz */
	int frq_val;
	long long ffr; 
	
	/* 
		We can't using floating point in kernel,
		so I decide to go 64bit muliply and 32bit divide.
		32000000/ 524288 = 15625/256.
		so multiply 256 and divide by 15625 yield same resolution result of integer
		calculation.
	*/
	ffr = freq * 256;
	ffr = do_div(ffr , 15625);

	frq_val = ffr;
	
	sx->frq_msb = (frq_val >> 16) & 0xFF;
	sx->frq_mib = (frq_val >> 8) & 0xFF;
	sx->frq_lsb = frq_val & 0xFF;
	sx1272_write_reg(sx, 0x06, sx->frq_msb);
	sx1272_write_reg(sx, 0x07, sx->frq_mib);
	sx1272_write_reg(sx, 0x08, sx->frq_lsb);
}

void rf_switch(sx1272_device_t *sx, int rx_on, int tx_on)
{
	/* Do as commanded */
	sx->rx_switch.data = rx_on;
	rf_gpio_set(&sx->rx_switch, 1);
	sx->tx_switch.data = tx_on;
	rf_gpio_set(&sx->tx_switch, 1);
}

void sx1272_rf_init(sx1272_device_t *sx)
{
	msleep(10);
	
	/* Set def values */
	sx->reg_1d = 0x23;
	sx->reg_1e = 0xb4;
	sx->reg_31 = 0x03;
	sx->reg_37 = 0x0A;
	
	sx1272_write_reg(sx, 0x01, 0x00); /* RF MODE to sleep */
	sx1272_write_reg(sx, 0x01, 0x80); /* RF MODE to LoRa */
	sx1272_write_reg(sx, 0x01, 0x81); /* RF MODE Wake from sleep */
	set_freq(sx, sx->base_freq, sx->chan);
	sx1272_write_reg(sx, 0x09, 0x8F); /* Enable 20dbm O/P limit */
	sx1272_write_reg(sx, 0x0A, 0x19);
	sx1272_write_reg(sx, 0x0B, 0x2B);
	sx1272_write_reg(sx, 0x0C, 0x23);
	sx1272_write_reg(sx, 0x1D, sx->reg_1d);
	sx1272_write_reg(sx, 0x1E, sx->reg_1e);
	sx1272_write_reg(sx, 0x1F, 0xFF); /* Receiver timeout */
	sx1272_write_reg(sx, 0x20, 0x00); /* Preamble MSB */
	sx1272_write_reg(sx, 0x21, 0x08); /* Preamble LSB */
	sx1272_write_reg(sx, 0x22, 10);   /* Payload length */
	sx1272_write_reg(sx, 0x23, 0xFF); /* Payload max length */
	sx1272_write_reg(sx, 0x24, 0x01); /* HOP period */
	sx1272_write_reg(sx, 0x31, sx->reg_31);
	sx1272_write_reg(sx, 0x33, 0x27);
	sx1272_write_reg(sx, 0x37, sx->reg_37);
	sx1272_write_reg(sx, 0x5A, 0x87);
	/* Do anything mode ? */
}

void load_fifo_tx_pointer(sx1272_device_t *sx)
{
	unsigned char ptr;
	sx1272_read_reg_delta(sx, 0x0E, &ptr); /* tx fifo ptr */
	sx1272_write_reg(sx, 0x0D, ptr);
}

void load_fifo_rx_pointer(sx1272_device_t *sx)
{
	unsigned char ptr1, ptr2;

	sx1272_read_reg_delta(sx, 0x25, &ptr1); /* rx fifo ptr */
	sx1272_read_reg_delta(sx, 0x13, &ptr2); /* rx fifo ptr */
	ptr1 = ptr2 - ptr1;
	
	sx1272_write_reg(sx, 0x0D, ptr1);
}

unsigned char read_fifo(sx1272_device_t *sx)
{
	unsigned char data;

	sx1272_read_reg_delta(sx, 0, &data);

	return data;
}

void read_rssi(sx1272_device_t *sx, char *rssi, char *snr)
{
	sx1272_read_reg_delta(sx, 0x1A, rssi); /* rx rssi */
	sx1272_read_reg_delta(sx, 0x19, snr);  /* rx snr */
}

void wait_tx_done(sx1272_device_t *sx)
{
	unsigned char status;
	int cnt;

	cnt = 5000;
	while (1) {
		sx1272_read_reg_delta(sx, 0x12, &status); /* irq status */
		if (status & 0x08)
			break;
		cnt--;
		if (cnt == 0) break;
		msleep(0);
	}
}

/* Now make basic function to service ioctl */
void open_rf_tx_lora(sx1272_device_t *sx, int freq, int chan)
{
	set_freq(sx, freq, chan);
	sx1272_write_reg(sx, 0x01, 0x81); /* RF MODE STANDBY */
	load_fifo_tx_pointer(sx);
	rf_switch(sx, false, true); /* rx clear, tx set */
	
	sx1272_write_reg(sx, 0x11, 0x87); /* IRQ MASKS */
	sx1272_write_reg(sx, 0x12, 0xFF); /* IRQ FLAGS */
}

void close_rf_tx_lora(sx1272_device_t *sx)
{
	sx1272_write_reg(sx, 0x01, 0x82); /* RF MODE TX */         
	sx1272_write_reg(sx, 0x01, 0x83); /* RF MODE TX */
	wait_tx_done(sx);
	rf_switch(sx, true, false); /* rx set, tx clear */
}

void open_rf_rx_lora(sx1272_device_t *sx, int freq, int chan)
{
	set_freq(sx, freq, chan);
	sx1272_write_reg(sx, 0x01, 0x85); /* RF MODE STANDBY */
	
	load_fifo_tx_pointer(sx); /* Is it RX pointer? */
	rf_switch(sx, true, false); /* rx clear, tx set */
	
	sx1272_write_reg(sx, 0x11, 0x87); /* IRQ MASKS */
	sx1272_write_reg(sx, 0x12, 0xFF); /* IRQ FLAGS */
}

void close_rf_rx_lora(sx1272_device_t *sx)
{
	sx1272_write_reg(sx, 0x12, 0xFF); /* IRQ FLAGS */
}

void send_rf_data_lora(sx1272_device_t *sx, int size, char *data)
{
	char *ptr = data;
	int i;

	sx1272_write_reg(sx, 0x22, size); /* This is payload length */
	for (i=0; i<size; i++) {
		sx1272_write_reg(sx, 0x80, *ptr++); /* load fifo */
	}
}


/*------------------------------------------------------------------------------- 
	IOCTL service routine.
--------------------------------------------------------------------------------*/
static long sx1272_ioctl(struct file * filp, unsigned int cmd, unsigned long arg)
{
	int ret   = 0;
	sx1272_device_t *sx = filp->private_data;

	DBGOUT("%s (minor:%d, table:%d, cmd:0x%x, nr:%d)\n",
		__func__, minor, NUM_OF_PARTS, cmd, _IOC_NR(cmd));

	switch(cmd)	{
		case IOCTL_SX1272_INIT :
			{
				/* Do init and return status */
				sx1272_rf_init(sx); 
				if (copy_to_user((void*)arg, (const void *)&sx->version, sizeof(int)))
					break;
			}
			break;
		case IOCTL_SX1272_GET_STATUS :
			{
				if (copy_to_user((void*)arg, (const void *)&sx->status, sizeof(int)))
					break;
			}
			break;
			/* Basic Output */
		case IOCTL_SX1272_REG_WRITE :
			{
				sx1272_rw_struc_t rws; 
				if (copy_from_user(&rws, (const void *)arg,  sizeof(sx1272_rw_struc_t)))
					break;
				sx1272_write_reg(sx, rws.reg, rws.data);
			}
			break;
		case IOCTL_SX1272_REG_READ_DELTA :
			{
				sx1272_rw_struc_t rws; 
				if (copy_from_user(&rws, (const void *)arg,  sizeof(sx1272_rw_struc_t)))
					break;
				sx1272_read_reg_delta(sx, rws.reg, &rws.data);
				if (copy_to_user((void*)arg, (const void *)&rws, sizeof(sx1272_rw_struc_t)))
					break;
			}
			break;
		case IOCTL_SX1272_RESET :
			{
				sx1272_reset(sx);
			}
			break;
		case IOCTL_SX1272_RF_SWITCH :
			{
				int sws;
				if (copy_from_user(&sws, (const void *)arg,  sizeof(int)))
					break;
				rf_switch(sx, (sws & 0x01) != 0, (sws & 0x02) != 0);
			}
			break;
		
			/* Advanced functions */
		case IOCTL_SX1272_TX_DATA :
			{
				sx1272_data_struc_t *psd;
				unsigned char *ptr;

				psd = (sx1272_data_struc_t *)&sx->buffer[0];
				ptr = (unsigned char *)arg;
				
				if (copy_from_user(psd, (const void *)ptr,  sizeof(int))) /* Get data length information */
					break;
				
				if (psd->size >= 1) {
					if (copy_from_user(psd->data, (const void *)&ptr[4], psd->size))
						break;
				}
				open_rf_tx_lora(sx, sx->base_freq, sx->chan);
				// send data
				send_rf_data_lora(sx, psd->size, psd->data);
				// stop buffering and sensing data.
				close_rf_tx_lora(sx);
			}
			break;
		case IOCTL_SX1272_WAIT_RX :
			{
				// Do nothing but work thread done?

				//
			}
			break;
		case IOCTL_SX1272_SET_FREQ_N_CHAN :
			{
				sx1272_freq_chan_t fcs;

				if (copy_from_user(&fcs, (const void *)arg,  sizeof(sx1272_freq_chan_t)))
					break;
				sx->base_freq = fcs.base_freq;
				sx->chan = fcs.chan;
			}
			break;
		default:
			DBGOUT("Fail, unknown ioctl ...\n");
			return -1;
	}
	DBGOUT("IoCtl (cmd:0x%x, nr:%d, ret:%d) \n\n", cmd, _IOC_NR(cmd), ret);

	return ret;
}

struct file_operations sx1272_fops = {
	.owner 			= THIS_MODULE,
	.open 			= sx1272_open,
	.release		= sx1272_release,
	//.read			= sx1272_read,
	//.write			= sx1272_write,
	.unlocked_ioctl	= sx1272_ioctl,
};

static void get_config(sx1272_device_t *sx)
{
	script_item_u	  val;
	script_item_value_type_e	type;
	
	type = script_get_item("lora_spi", "rx_switch", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_PIO != type) {
		sx->rx_switch.gpio = GPIO_INDEX_INVALID;
		DBGOUT("fetch rx_switch from sys_config failed\n", i);
	} else {
		sx->rx_switch.gpio = val.gpio.gpio;
		sx->rx_switch.mul_sel=val.gpio.mul_sel;
		sx->rx_switch.pull = val.gpio.pull;
		sx->rx_switch.drv_level = val.gpio.drv_level;
		sx->rx_switch.data = val.gpio.data;
	}

	type = script_get_item("lora_spi", "tx_switch", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_PIO != type) {
		sx->tx_switch.gpio = GPIO_INDEX_INVALID;
		DBGOUT("fetch tx_switch from sys_config failed\n", i);
	} else {
		sx->tx_switch.gpio = val.gpio.gpio;
		sx->tx_switch.mul_sel=val.gpio.mul_sel;
		sx->tx_switch.pull = val.gpio.pull;
		sx->tx_switch.drv_level = val.gpio.drv_level;
		sx->tx_switch.data = val.gpio.data;
	}
	
	type = script_get_item("lora_spi", "reset", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_PIO != type) {
		sx->tx_switch.gpio = GPIO_INDEX_INVALID;
		DBGOUT("fetch reset from sys_config failed\n", i);
	} else {
		sx->reset.gpio = val.gpio.gpio;
		sx->reset.mul_sel=val.gpio.mul_sel;
		sx->reset.pull = val.gpio.pull;
		sx->reset.drv_level = val.gpio.drv_level;
		sx->reset.data = val.gpio.data;
	}
	
	type = script_get_item("lora_spi", "irq", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_PIO != type) {
		sx->tx_switch.gpio = GPIO_INDEX_INVALID;
		DBGOUT("fetch reset from sys_config failed\n", i);
	} else {
		sx->irq.gpio = val.gpio.gpio;
		sx->irq.mul_sel=val.gpio.mul_sel;
		sx->irq.pull = val.gpio.pull;
		sx->irq.drv_level = val.gpio.drv_level;
		sx->irq.data = val.gpio.data;
	}
}

static int __devinit sx1272_probe(struct spi_device *spi)
{
	int ret;
	int virq;

	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	sx1272 = kzalloc(sizeof(sx1272_device_t), GFP_KERNEL);
	if (sx1272 == NULL)
		return -ENOMEM;
	
	sx1272->spi = spi;
	dev_set_drvdata(&spi->dev, sx1272);
	
	/* GPIO settings for reset,rf_rx, rf_tx pin */
	get_config(sx1272);
	
	rf_gpio_request(&sx1272->rx_switch, 1);
	rf_gpio_request(&sx1272->tx_switch, 1);
	rf_gpio_request(&sx1272->reset, 1);
	rf_gpio_request(&sx1272->irq, 1);
	virq = gpio_to_irq(sx1272->irq.gpio);
	if (IS_ERR_VALUE(virq)) {
		printk("map gpio [%d] to virq failed, errno = %d\n", sx1272->irq.gpio, virq);
	}

	DBGOUT("gpio [%d] map to virq [%d] ok\n",sx1272->irq.gpio, virq);
	/* request virq */
	ret = devm_request_irq(&spi->dev, virq, sx1272_irq,
			IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, SX1272_3WIRE_DRV_NAME, sx1272);

	if (IS_ERR_VALUE(ret)) {
		printk("Failed to get gpio irq for sx1272 LoRa interface\n");
	}
	sx1272->virq = virq;

	sx1272->version = (SX1272_DRIVER_VERSION );
	
	sx1272_reset(sx1272);

	/* register character driver */
	ret = register_chrdev(SX1272_DEV_MAJOR, "LoRa driver", &sx1272_fops);
	if (0 > ret)	{
		DBGOUT("Fail, register device (%s, major:%d)\n",
			SX1272_3WIRE_DRV_NAME, SX1272_DEV_MAJOR);
		goto err_alloc;
	}

	ret = sysfs_create_group(&spi->dev.kobj, &sx1272_attr_group);
	if (ret < 0)
		goto err_register;
	
	sx1272_class = class_create(THIS_MODULE, SX1272_3WIRE_DRV_NAME);
	if (ERR_PTR == (void *)device_create(sx1272_class, NULL, MKDEV(SX1272_DEV_MAJOR, 0), NULL, SX1272_3WIRE_DRV_NAME))
		goto err_sysfs;

	mutex_init(&sx1272->rcv_lock);
	init_waitqueue_head(&sx1272->waitq);
	mutex_init(&sx1272->send_lock);

	/* create workq */
	sx1272->send_workq = create_singlethread_workqueue("send_wq");
	if (!sx1272->send_workq) {
		ret = -ENOMEM;
		goto err_sysfs;
	}
	INIT_WORK(&sx1272->work, send_work_func);

	printk("sx1272_spi device is ready.\n");

	return 0;
err_sysfs:
	sysfs_remove_group(&spi->dev.kobj, &sx1272_attr_group);

err_register:
	unregister_chrdev(SX1272_DEV_MAJOR, "LoRa driver");

err_alloc:
	kfree(sx1272);

	return ret;
}

static int __devexit sx1272_remove(struct spi_device *spi)
{
	sx1272_device_t *ssd = dev_get_drvdata(&spi->dev);
	
	unregister_chrdev(SX1272_DEV_MAJOR, "LoRa driver");
	sysfs_remove_group(&spi->dev.kobj, &sx1272_attr_group);
	
	kfree(ssd);

	return 0;
}

static struct spi_driver sx1272_driver = {
	.driver = {
		.name	= SX1272_3WIRE_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe	= sx1272_probe,
	.remove	= __devexit_p(sx1272_remove),
};

static int __init sx1272_spi_init(void)
{
	return spi_register_driver(&sx1272_driver);
}

static void __exit sx1272_spi_exit(void)
{
	spi_unregister_driver(&sx1272_driver);
}

MODULE_AUTHOR("Seungwoo Kim <ksw@stcube.com>");
MODULE_DESCRIPTION("SX1272 Lora SPI driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(sx1272_spi_init);
module_exit(sx1272_spi_exit);

