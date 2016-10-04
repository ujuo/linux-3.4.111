/*
 * sunxi-nand mtd driver
 * Copyright(c) 2016 STcube Inc.
 * All right reserved by Seungwoo Kim <ksw@stcube.com> and following authurs
 *
 * Copyright (C) 2013 Boris BREZILLON <b.brezillon.dev@gmail.com>
 *
 * Derived from:
 *	https://github.com/yuq/sunxi-nfc-mtd
 *	Copyright (C) 2013 Qiang Yu <yuq825@gmail.com>
 *
 *	https://github.com/hno/Allwinner-Info
 *	Copyright (C) 2013 Henrik Nordström <Henrik Nordström>
 *
 *	Copyright (C) 2013 Dmitriy B. <rzk333@gmail.com>
 *	Copyright (C) 2013 Sergey Lapin <slapin@ossfans.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_mtd.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/version.h>
#include <mach/sys_config.h>
#include <mach/nand_platform.h>

#include "sunxi_nand.h"

#define USING_PLATFORM_RESOURCES	1
#if defined(CONFIG_OF)
#define USING_DEVICE_TREE	1
#define USING_SUNXI_SCRIPT	0
#else
#define USING_DEVICE_TREE	0
#define USING_SUNXI_SCRIPT	1
#endif

/**  ksw : 2016-04-07 

   MTD only can handle ONE OOBSIZE, ONE WRITESIZE so on.
   so multiple nand chips are ridiculus idea for
   current mtd structure
   So only one nand for one mtd.
   Sunxi nfc can handle multiple chips, but one mtd?, not a good idea.
   I would withdraw mtd from sunxi_nfc, and insert it to sunxi_nand_chip.

**/

extern int sunxi_nand_chips_init(struct device *dev, struct sunxi_nfc *nfc);
extern void sunxi_nand_chips_cleanup(struct sunxi_nfc *nfc);

static int sunxi_nfc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *r;
	struct sunxi_nfc *nfc;
	struct sunxi_pin_struct nand_pins[4]; /* No more than 4 devices can be used. */
#if USING_DEVICE_TREE
	struct resource io_res;
	struct device_node *np = dev->of_node;
	struct device_node *nand_np;
	int tmp;
#endif
#if USING_SUNXI_SCRIPT
	//user_gpio_set_t gpio_set;
	//struct gpio_config gpio_set;
	script_item_u				gpio_set;
	script_item_value_type_e 	type;
	script_item_u				gp_list[24]; /* Data, ALE,CLE,CS,RB */
	char para_name[20];
	int num;
#endif
	int ret=0, i, j;

	nfc = devm_kzalloc(dev, sizeof(*nfc), GFP_KERNEL);
	if (!nfc)
		return -ENOMEM;

	nfc->dev = dev;
	
	spin_lock_init(&nfc->controller.lock);
	init_waitqueue_head(&nfc->controller.wq);
	INIT_LIST_HEAD(&nfc->chips);

#if USING_PLATFORM_RESOURCES
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!request_mem_region(r->start, resource_size(r), pdev->name)) {
		ret = -EBUSY;
		return ret;
	}

	nfc->regs = ioremap(r->start, resource_size(r));
	if (!nfc->regs) {
		ret = -ENOMEM;
		return ret;
	}
	NAND_DBG("regs = %X(PA = %X)\n", (uint32_t) nfc->regs, r->start);
	nfc->irq = platform_get_irq(pdev, 0);
	if (nfc->irq < 0) {
		dev_err(dev, "failed to retrieve irq\n");
		release_mem_region(r->start, resource_size(r));
		return -ENODEV;
	}
	NAND_DBG("irq = %d\n", (uint32_t) nfc->irq);
#if 1	
	nfc->ahb_clk = clk_get(NULL,"ahb_nfc");
	nfc->mod_clk =  clk_get(NULL,"nfc");
	
	ret = clk_enable(nfc->ahb_clk);
	if (ret) {
		release_mem_region(r->start, resource_size(r));
		return ret;
	}
	ret = clk_enable(nfc->mod_clk);

	if (ret) {
		clk_put(nfc->ahb_clk);
		release_mem_region(r->start, resource_size(r));
		return ret;
	}
	clk_set_rate(nfc->mod_clk, 20000000 * 2);
#endif
#endif
#if USING_SUNXI_SCRIPT

#define SUNXI_NFC_BASE_ADDRESS	0x01C03000

#if defined(CONFIG_ARCH_SUN8I)
#define NAND_IRQ	102	//A33
#elif defined(CONFIG_ARCH_SUN5I)
#define NAND_IRQ	37	// A13
#else
#error	"Currently SUN5I(A13/R8) and SUN8I(A33) support only."
#endif
	/* ksw :  does any one has nand%X_para anymore? */
	memset(para_name, 0, 20);
	sprintf(para_name, "nand%d_para", 0);
	type = script_get_item(para_name, "nand_used", &gpio_set);
	if ((type !=  SCIRPT_ITEM_VALUE_TYPE_INT) || (gpio_set.val == 0)) {
		printk("SUNXI NAND is not enabled on script.\n");
		return -1;
	}

#if !USING_PLATFORM_RESOURCES
	nfc->regs = ioremap(SUNXI_NFC_BASE_ADDRESS, 0xFFF); // 4K
	 // IRQ
	nfc->irq = NAND_IRQ;
#endif
	nfc->nres = 1; /* No other chips on this platform */
	/* Now temporary allocate memories for pins */
	nfc->pins = &nand_pins[0];
	for (j=0; j<nfc->nres; j++) {
		int done = 0;
		nfc->pins[j].nsels = NFC_MAX_CS;
		for (i = 0; i < nfc->pins[j].nsels; i++) {
			char pin_name[16];
			// Chip select pins
			memset(pin_name, 0, 16);
			sprintf(pin_name, "nand0_ce%d", i);
			gpio_set.gpio.gpio = -1;
			type = script_get_item(para_name, pin_name, &gpio_set);
			if ((type != SCIRPT_ITEM_VALUE_TYPE_PIO) || (gpio_set.gpio.gpio == 0xFFFFFFFF))
				break;
			//NAND_DBG("gpio_name            = %s\n", gpio_set.gpio_name);
			NAND_DBG("gpio                 = %x\n", gpio_set.gpio.gpio);
			//NAND_DBG("port_num             = %x\n", gpio_set.port_num);
			NAND_DBG("mul_sel              = %x\n", gpio_set.gpio.mul_sel);
			NAND_DBG("pull                 = %x\n", gpio_set.gpio.pull);
			NAND_DBG("drv_level            = %x\n", gpio_set.gpio.drv_level);
			NAND_DBG("data                 = %x\n", gpio_set.gpio.data);

			set_bit(i, &nfc->assigned_cs);
			done = 1;
			
			if (gpio_set.gpio.mul_sel == 1) { /* then GPIO OUT */
				nfc->pins[j].sels[i].cs.type = CSRB_GPIO;
				
				nfc->pins[j].sels[i].cs.info.gpio = 0; /* How can PORT/PORTNUM into gpio? */
			} else {
				nfc->pins[j].sels[i].cs.type = CSRB_NATIVE;
				nfc->pins[j].sels[i].cs.info.nativeid = i;
			}
			// RB pins
			memset(pin_name, 0, 16);
			sprintf(pin_name, "nand0_rb%d", i);
			gpio_set.gpio.gpio = -1;
			type = script_get_item(para_name, pin_name, &gpio_set);
			if ((type != SCIRPT_ITEM_VALUE_TYPE_PIO) || (gpio_set.gpio.gpio == 0xFFFFFFFF))
				break;
			//NAND_DBG("gpio_name            = %s\n", gpio_set.gpio_name);
			NAND_DBG("gpio                 = %x\n", gpio_set.gpio.gpio);
			//NAND_DBG("port_num             = %x\n", gpio_set.port_num);
			NAND_DBG("mul_sel              = %x\n", gpio_set.gpio.mul_sel);
			NAND_DBG("pull                 = %x\n", gpio_set.gpio.pull);
			NAND_DBG("drv_level            = %x\n", gpio_set.gpio.drv_level);
			NAND_DBG("data                 = %x\n", gpio_set.gpio.data);
			if (gpio_set.gpio.mul_sel == 0) { /* then GPIO IN */
				nfc->pins[j].sels[i].rb.type = CSRB_GPIO;
				nfc->pins[j].sels[i].rb.info.gpio = 0; /* How can PORT/PORTNUM into gpio? */
			} else {
				nfc->pins[j].sels[i].rb.type = CSRB_NATIVE;
				nfc->pins[j].sels[i].rb.info.nativeid = i;
			}
			done = 2;
		}
		if (done != 2) {
			/* Then it means break out */
			nfc->pins[j].nsels = j + 1;
			continue;
		}
		j++;
	}
	// GPIO setting check
	//ret = gpio_request_ex("nand_para",NULL);
	num = script_get_pio_list(para_name, (script_item_u **)&gp_list);
	for (i=0; i< num; i++) {
		gp_list[i].gpio.gpio;
		
	}
	
#if 0	/* ksw : Currently, we can do nothing about WP pins. */
	// WP pins
	gpio_set.port = -1;
	ret = script_parser_fetch("nand_para", "nand_wp", (int *)&gpio_set, 64);
	printk("ret=%d\n", ret);
	NAND_DBG("gpio_name            = %s\n", gpio_set.gpio_name);
	NAND_DBG("port                 = %x\n", gpio_set.port);
	NAND_DBG("port_num             = %x\n", gpio_set.port_num);
	NAND_DBG("mul_sel              = %x\n", gpio_set.mul_sel);
	NAND_DBG("pull                 = %x\n", gpio_set.pull);
	NAND_DBG("drv_level            = %x\n", gpio_set.drv_level);
	NAND_DBG("data                 = %x\n", gpio_set.data);
#endif
#endif
#if USING_DEVICE_TREE
	nfc->ahb_clk = devm_clk_get(dev, "ahb1");
	if (IS_ERR(nfc->ahb_clk)) {
		dev_err(dev, "failed to retrieve ahb clk\n");
		return PTR_ERR(nfc->ahb_clk);
	}

	ret = clk_prepare_enable(nfc->ahb_clk);
	if (ret)
		return ret;

	nfc->mod_clk = devm_clk_get(dev, "mod");
	if (IS_ERR(nfc->mod_clk)) {
		dev_err(dev, "failed to retrieve mod clk\n");
		ret = PTR_ERR(nfc->mod_clk);
		goto out_ahb_clk_unprepare;
	}

	ret = clk_prepare_enable(nfc->mod_clk);
	if (ret)
		goto out_ahb_clk_unprepare;
	
	ret = of_address_to_resource(pdev->dev.of_node, 0, &io_res);
	if (ret) {
		dev_err(&pdev->dev, "can't get IO base\n");
		goto out_ahb_clk_unprepare;
	}
	r = &io_res;
	if (!devm_request_mem_region(&pdev->dev, r->start, resource_size(r), DRV_NAME)) {
		dev_err(dev, "Error requesting memory region!\n");
		return -EBUSY;
	}
	nfc->regs = devm_ioremap_nocache(&pdev->dev, r->start,
					    resource_size(r));
	nfc->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	
	nfc->nres = of_get_child_count(pdev->dev.of_node);

	/* Now temporary allocate memories for pins */
	nfc->pins = &nand_pins[0];

	if (nfc->nres > 8) {
		dev_err(dev, "too many NAND chips: %d (max = 8)\n", nfc->nres);
		return -EINVAL;
	}

	j = 0;
	for_each_child_of_node(np, nand_np) {
	//for (j=0; j<nfc->nres; j++) {
		if (!of_get_property(nand_np, "nsels", &nfc->pins[j].nsels)) {
			ret = -EINVAL;
			of_put(nand_np);
			goto fail4;
		}
		nfc->pins[j].nsels /= sizeof(u32);
		if (!nfc->pins[j].nsels) {
			dev_err(dev, "invalid reg property size\n");
			of_put(nand_np);
			ret = -EINVAL;
			goto fail4;
		}
		
		for (i = 0; i < nfc->pins[j].nsels; i++) {
			ret = of_property_read_u32_index(nand_np, "cs", i, &tmp);
			if (ret) {
				dev_err(dev, "could not retrieve reg property: %d\n",
					ret);
				of_put(nand_np);
				goto fail4;
			}
	
			if (tmp > NFC_MAX_CS) {
				dev_err(dev,
					"invalid reg value: %u (max CS = 7)\n",
					tmp);
				ret = -EINVAL;
				of_put(nand_np);
				goto fail4;
			}
	
			if (test_and_set_bit(tmp, &nfc->assigned_cs)) {
				dev_err(dev, "CS %d already assigned\n", tmp);
				ret = -EINVAL;
				of_put(nand_np);
				goto fail4;
			}
	
			nfc->pins[j].sels[i].cs = tmp;
	
			if (!of_property_read_u32_index(nand_np, "allwinner,rb", i, &tmp) &&
				tmp < 2) {
				nfc->pins[j].sels[i].rb.type = CSRB_NATIVE;
				nfc->pins[j].sels[i].rb.info.nativeid = tmp;
			} else {
				ret = of_get_named_gpio(nand_np, "rb-gpios", i);
				if (ret >= 0) {
					tmp = ret;
					nfc->pins[j].sels[i].rb.type = CSRB_GPIO;
					nfc->pins[j].sels[i].rb.info.gpio = tmp;
				} else {
					nfc->pins[j].sels[i].rb.type = CSRB_NONE;
				}
			}
		}
		j++;
	}
#endif

	ret = sunxi_nfc_rst(nfc);
	if (ret) {
		release_mem_region(r->start, resource_size(r));
		return ret;
	}

	writel(0, nfc->regs + NFC_REG_INT);
	
	ret = request_irq(nfc->irq, sunxi_nfc_interrupt, 0, DRV_NAME, nfc);
	
	if (ret) {
		release_mem_region(r->start, resource_size(r));
		return ret;
	}

	platform_set_drvdata(pdev, nfc);

	ret = sunxi_nand_chips_init(dev, nfc);
	if (ret) {
		dev_err(dev, "failed to init nand chips\n");
		return ret;
	}

	return 0;
}

static int sunxi_nfc_remove(struct platform_device *pdev)
{
	struct sunxi_nfc *nfc = platform_get_drvdata(pdev);

	sunxi_nand_chips_cleanup(nfc);
	
	kfree(nfc);

	return 0;
}

#if USING_DEVICE_TREE
static const struct of_device_id sunxi_nfc_ids[] = {
	{ .compatible = "allwinner,sun4i-a10-nand" },
	{ .compatible = "allwinner,sun5i-a13-nand" },
	{ .compatible = "allwinner,sun8i-a33-nand" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sunxi_nfc_ids);
#endif

static struct platform_driver sunxi_nfc_driver = {
	.driver = {
		.name = DRV_NAME,
#if USING_DEVICE_TREE
		.of_match_table = sunxi_nfc_ids,
#endif
	},
	.probe = sunxi_nfc_probe,
	.remove = sunxi_nfc_remove,
};
module_platform_driver(sunxi_nfc_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Seungwoo Kim <ksw@stcube.com>");
MODULE_DESCRIPTION("Allwinner NAND Flash Controller driver");
MODULE_ALIAS("platform:sunxi_nand");
