/*
 * drivers/gpu/ion/sunxi/sunxi_ion.c
 *
 * Copyright(c) 2013-2015 Allwinnertech Co., Ltd.
 *      http://www.allwinnertech.com
 *
 * Author: liugang <liugang@allwinnertech.com>
 *
 * sunxi ion heap realization
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/plist.h>
#include <linux/mutex.h>
#include <linux/rwsem.h>
#include <linux/device.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/anon_inodes.h>
#include <linux/ion.h>
#include <linux/list.h>
#include <linux/memblock.h>
#include <linux/miscdevice.h>
#include <linux/export.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/rbtree.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/dma-buf.h>
#include <linux/vmalloc.h>
#include <linux/genalloc.h>
//#include <linux/secure/te_protocol.h>

#include <mach/sunxi-smc.h>
#include <mach/ion_sunxi.h>
#include <linux/dmaengine.h>
#include <mach/sunxi-dma.h>
#include <asm/setup.h>
#include <linux/module.h>
#include <linux/debugfs.h>

#include "../ion_priv.h"

#define DEV_NAME	"ion-sunxi"

struct ion_device;
struct ion_heap **pheap;
struct ion_heap *carveout_heap = NULL;
int num_heaps;
extern struct tag_mem32 ion_mem;
struct ion_device *idev;
EXPORT_SYMBOL(idev);

bool ion_handle_validate(struct ion_client *client, struct ion_handle *handle);

extern struct ion_heap *carveout_heap;

static unsigned int dump_unit = SZ_64K;

void __dump_carveout_area(struct seq_file *s)
{
	struct ion_carveout_heap *heap = NULL;
	struct gen_pool *pool = NULL;
	struct gen_pool_chunk *chunk;
	int size, total_bits, bits_per_unit;
	int i, index, offset, tmp, busy;
	int busy_cnt = 0, free_cnt = 0;

	if (!carveout_heap) {
		seq_printf(s, "%s(%d) err: carveout_heap is NULL\n", __func__, __LINE__);
		return;
	}

	heap = container_of(carveout_heap, struct ion_carveout_heap, heap);
	pool = heap->pool;

	rcu_read_lock();
	list_for_each_entry_rcu(chunk, &pool->chunks, next_chunk) {
		size = chunk->end_addr - chunk->start_addr;
		total_bits = size >> pool->min_alloc_order;
		bits_per_unit = dump_unit >> pool->min_alloc_order;
		seq_printf(s, "%s(%d): memory 0x%08x~0x%08x, layout(+: free, -: busy, unit: 0x%08xbytes):\n",
			__func__, __LINE__, (u32)chunk->start_addr, (u32)chunk->end_addr, dump_unit);
		busy_cnt = 0;
		free_cnt = 0;
		for (i = 0, tmp = 0, busy = 0; i < total_bits; i++) {
			index = i >> 5;
			offset = i & 31;
			if (!busy && (chunk->bits[index] & (1<<offset)))
				busy = 1;
			if (++tmp == bits_per_unit) {
				busy ? (seq_printf(s, "-"), busy_cnt++) : (seq_printf(s, "+"), free_cnt);
				busy = 0;
				tmp = 0;
			}
		}
		seq_printf(s, "\n");
		seq_printf(s, "free: 0x%08x bytes, busy: 0x%08x bytes\n", free_cnt*dump_unit, busy_cnt*dump_unit);
	}
	rcu_read_unlock();

	return;
}

module_param(dump_unit, uint, 0644);
MODULE_PARM_DESC(dump_unit, "Sunxi ion dump unit(in bytes)");

int sunxi_ion_show(struct seq_file *m, void *unused)
{
	struct ion_carveout_heap *heap = NULL;
	struct gen_pool *pool = NULL;
	struct gen_pool_chunk *chunk;
	int size, total_bits, bits_per_unit;
	int i, index, offset, tmp, busy;
	int busy_cnt = 0, free_cnt = 0;

	if (!carveout_heap) {
		seq_printf(m, "%s(%d) err: carveout_heap is NULL\n", __func__, __LINE__);
		return -EPERM;
	}

	heap = container_of(carveout_heap, struct ion_carveout_heap, heap);
	pool = heap->pool;

	rcu_read_lock();
	list_for_each_entry_rcu(chunk, &pool->chunks, next_chunk) {
		size = chunk->end_addr - chunk->start_addr;
		total_bits = size >> pool->min_alloc_order;
		bits_per_unit = dump_unit >> pool->min_alloc_order;
		seq_printf(m, "%s(%d): memory 0x%08x~0x%08x, layout(+: free, -: busy, unit: 0x%08xbytes):\n",
			__func__, __LINE__, (u32)chunk->start_addr, (u32)chunk->end_addr, dump_unit);
		busy_cnt = 0;
		free_cnt = 0;
		for (i = 0, tmp = 0, busy = 0; i < total_bits; i++) {
			index = i >> 5;
			offset = i & 31;
			if (!busy && (chunk->bits[index] & (1<<offset)))
				busy = 1;
			if (++tmp == bits_per_unit) {
				busy ? (seq_printf(m, "-"), busy_cnt++) : (seq_printf(m, "+"), free_cnt++);
				busy = 0;
				tmp = 0;
			}
		}
		seq_printf(m, "\n");
		seq_printf(m, "free: 0x%08x bytes, busy: 0x%08x bytes\n", free_cnt*dump_unit, busy_cnt*dump_unit);
	}
	rcu_read_unlock();

	return 0;
}

int sunxi_ion_dump_mem(void)
{
	struct ion_carveout_heap *heap = NULL;
	struct gen_pool *pool = NULL;
	struct gen_pool_chunk *chunk;
	int size, total_bits, bits_per_unit;
	int i, index, offset, tmp, busy;
	int busy_cnt = 0, free_cnt = 0;

	if (!carveout_heap) {
		printk("%s(%d) err: carveout_heap is NULL\n", __func__, __LINE__);
		return -EPERM;
	}

	heap = container_of(carveout_heap, struct ion_carveout_heap, heap);
	pool = heap->pool;

	rcu_read_lock();
	list_for_each_entry_rcu(chunk, &pool->chunks, next_chunk) {
		size = chunk->end_addr - chunk->start_addr;
		total_bits = size >> pool->min_alloc_order;
		bits_per_unit = dump_unit >> pool->min_alloc_order;
		printk("%s(%d): memory 0x%08x~0x%08x, layout(+: free, -: busy, unit: 0x%08xbytes):\n",
			__func__, __LINE__, (u32)chunk->start_addr, (u32)chunk->end_addr, dump_unit);
		busy_cnt = 0;
		free_cnt = 0;
		for (i = 0, tmp = 0, busy = 0; i < total_bits; i++) {
			index = i >> 5;
			offset = i & 31;
			if (!busy && (chunk->bits[index] & (1<<offset)))
				busy = 1;
			if (++tmp == bits_per_unit) {
				busy ? (printk("-"), busy_cnt++) : (printk("+"), free_cnt);
				busy = 0;
				tmp = 0;
			}
		}
		printk("\n");
		printk("free: 0x%08x bytes, busy: 0x%08x bytes\n", free_cnt*dump_unit, busy_cnt*dump_unit);
	}
	rcu_read_unlock();

	return 0;
}

void __flush_cache_before_dma(dma_buf_group *pbuf_group)
{
	long start, end;
	int i;

	for(i = 0; i < pbuf_group->cnt; i++) {
		start = pbuf_group->item[i].dst_va;
		end = start + pbuf_group->item[i].size;
		flush_user_range(start, end);
		//flush_clean_user_range(start, end);
	}
}

#define MAX_CHANNEL 6

typedef struct {
	struct dma_chan *chan;      /* dma channel handle */
	wait_queue_head_t dma_wq;   /* wait dma transfer done */
	atomic_t	dma_done;   /* dma done flag, used with dma_wq */
}chan_info;

static void __dma_callback(void *dma_async_param)
{
	chan_info *pinfo = (chan_info *)dma_async_param;

	wake_up_interruptible(&pinfo->dma_wq);
	atomic_set(&pinfo->dma_done, 1);
}

int __multi_dma_copy(dma_buf_group *pbuf_group)
{
	struct dma_async_tx_descriptor *tx = NULL;
	struct dma_slave_config config;
	chan_info dma_chanl[MAX_CHANNEL], *pchan_info;
	int buf_left, cur_trans, start_index;
	int i, ret = -EINVAL, chan_cnt = 0;
	long timeout = 5 * HZ;
	dma_buf_item *pitem;
	dma_cap_mask_t mask;
	dma_cookie_t cookie;

	/* split buf if buf cnt <=3 */
	if(pbuf_group->cnt <= 3) {
		for(i = 0; i < pbuf_group->cnt; i++) {
			pitem = &pbuf_group->item[i];
			if(pitem->size >= SZ_1M) {
				memmove(&pitem[1], pitem, (pbuf_group->cnt - i)*sizeof(*pitem));

				pitem->size >>= 1;
				pitem[1].src_va = pitem->src_va + pitem->size;
				pitem[1].src_pa = pitem->src_pa + pitem->size;
				pitem[1].dst_va = pitem->dst_va + pitem->size;
				pitem[1].dst_pa = pitem->dst_pa + pitem->size;
				pitem[1].size = pitem->size;
				pbuf_group->cnt++;
				break;
			}
		}
	}

	/* request channel */
	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);
	for(i = 0; i < ARRAY_SIZE(dma_chanl); i++, chan_cnt++) {
		if(chan_cnt == pbuf_group->cnt) /* channel enough */
			break;
		pchan_info = &dma_chanl[i];
		pchan_info->chan = dma_request_channel(mask , NULL , NULL);
		if(!pchan_info->chan)
			break;
		init_waitqueue_head(&pchan_info->dma_wq);
		atomic_set(&pchan_info->dma_done, 0);
	}

	buf_left = pbuf_group->cnt;
again:
	start_index = pbuf_group->cnt - buf_left;
	for(i = 0; i < chan_cnt; ) {
		pchan_info = &dma_chanl[i];
		config.direction = DMA_MEM_TO_MEM;
		config.src_addr = 0; /* not used for memcpy */
		config.dst_addr = 0;
		config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		config.src_maxburst = 8;
		config.dst_maxburst = 8;
		config.slave_id = sunxi_slave_id(DRQDST_SDRAM, DRQSRC_SDRAM);
		dmaengine_slave_config(pchan_info->chan, &config);

		tx = pchan_info->chan->device->device_prep_dma_memcpy(pchan_info->chan,
			pbuf_group->item[start_index + i].dst_pa,
			pbuf_group->item[start_index + i].src_pa,
			pbuf_group->item[start_index + i].size,
			DMA_PREP_INTERRUPT | DMA_CTRL_ACK);

		tx->callback = __dma_callback;
		tx->callback_param = pchan_info;

		cookie = dmaengine_submit(tx);

		if(++i == buf_left)
			break;
	}
	cur_trans = i;

	/* start dma */
	for(i = 0; i < cur_trans; i++)
		dma_async_issue_pending(dma_chanl[i].chan);

	for(i = 0; i < cur_trans; i++) {
		ret = wait_event_interruptible_timeout(dma_chanl[i].dma_wq, atomic_read(&dma_chanl[i].dma_done)==1, timeout);
		if(unlikely(-ERESTARTSYS == ret || 0 == ret)) {
			pr_err("%s(%d) err: wait dma done failed!\n", __func__, __LINE__);
			ret = -EIO;
			goto end;
		}
	}

	buf_left -= cur_trans;
	if(buf_left)
		goto again;

	ret = 0;
end:
	for(i = 0; i < chan_cnt; i++)
		dma_release_channel(dma_chanl[i].chan);
	return ret;
}

int __signle_dma_copy(dma_buf_group *pbuf_group)
{
	struct sg_table src_sg_table, dst_sg_table;
	struct dma_async_tx_descriptor *tx = NULL;
	struct dma_slave_config config;
	struct dma_chan *chan;
	struct scatterlist *sg;
	long timeout = 5 * HZ;
	chan_info dma_info;
	dma_cap_mask_t mask;
	dma_cookie_t cookie;
	int i, ret = -EINVAL;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SG, mask); /* to check... */
	dma_cap_set(DMA_MEMCPY, mask);
	chan = dma_request_channel(mask , NULL , NULL);
	if(!chan) {
		pr_err("%s(%d) err: dma_request_channel failed!\n", __func__, __LINE__);
		return -EBUSY;
	}

	if(sg_alloc_table(&src_sg_table, pbuf_group->cnt, GFP_KERNEL)) {
		pr_err("%s(%d) err: alloc src sg_table failed!\n", __func__, __LINE__);
		return -ENOMEM;
	}
	if(sg_alloc_table(&dst_sg_table, pbuf_group->cnt, GFP_KERNEL)) {
		sg_free_table(&src_sg_table);
		pr_err("%s(%d) err: alloc dst sg_table failed!\n", __func__, __LINE__);
		return -ENOMEM;
	}

	/* assign sg buf */
	sg = src_sg_table.sgl;
	for(i = 0; i < pbuf_group->cnt; i++, sg = sg_next(sg)) {
		sg_set_buf(sg, phys_to_virt(pbuf_group->item[i].src_pa), pbuf_group->item[i].size);
		sg_dma_address(sg) = pbuf_group->item[i].src_pa;
	}
	sg = dst_sg_table.sgl;
	for(i = 0; i < pbuf_group->cnt; i++, sg = sg_next(sg)) {
		sg_set_buf(sg, phys_to_virt(pbuf_group->item[i].dst_pa), pbuf_group->item[i].size);
		sg_dma_address(sg) = pbuf_group->item[i].dst_pa;
	}

	config.direction = DMA_MEM_TO_MEM;
	config.src_addr = 0; /* not used for memcpy */
	config.dst_addr = 0;
	config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	config.src_maxburst = 8;
	config.dst_maxburst = 8;
	config.slave_id = sunxi_slave_id(DRQDST_SDRAM, DRQSRC_SDRAM);
	dmaengine_slave_config(chan , &config);

	tx = chan->device->device_prep_dma_sg(chan, dst_sg_table.sgl, pbuf_group->cnt,
		src_sg_table.sgl, pbuf_group->cnt, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);

	/* set callback */
	dma_info.chan = chan;
	init_waitqueue_head(&dma_info.dma_wq);
	atomic_set(&dma_info.dma_done, 0);
	tx->callback = __dma_callback;
	tx->callback_param = &dma_info;

	/* enqueue */
	cookie = dmaengine_submit(tx);
	/* start dma */
	dma_async_issue_pending(chan);

	/* wait transfer over */
	ret = wait_event_interruptible_timeout(dma_info.dma_wq, atomic_read(&dma_info.dma_done)==1, timeout);
	if(unlikely(-ERESTARTSYS == ret || 0 == ret)) {
		pr_err("%s(%d) err: wait dma done failed!\n", __func__, __LINE__);
		goto end;
	}

	ret = 0;
end:
	sg_free_table(&src_sg_table);
	sg_free_table(&dst_sg_table);
	dma_release_channel(chan);
	return ret;
}

int dma_copy_buf(dma_buf_group *pbuf_group)
{
	if(unlikely(!pbuf_group || !pbuf_group->cnt))
		return -EINVAL;

	if(pbuf_group->multi_dma)
		return __multi_dma_copy(pbuf_group);
	else
		return __signle_dma_copy(pbuf_group);
}
extern int sunxi_ion_dump_mem(void);
long sunxi_ion_ioctl(struct ion_client *client, unsigned int cmd, unsigned long arg)
{
	long ret = 0;

	switch(cmd) {
	case ION_IOC_SUNXI_FLUSH_RANGE:
	{
		sunxi_cache_range range;
		if(copy_from_user(&range, (u32 *)arg, sizeof(range))) {
			ret = -EINVAL;
			goto end;
		}
		if(flush_clean_user_range(range.start, range.end)) {
			ret = -EINVAL;
			goto end;
		}
		break;
	}
	case ION_IOC_SUNXI_FLUSH_ALL:
		flush_dcache_all();
		break;
	case ION_IOC_SUNXI_PHYS_ADDR:
	{
		sunxi_phys_data data;
		bool valid;

		if(copy_from_user(&data, (void __user *)arg, sizeof(sunxi_phys_data)))
			return -EFAULT;
		mutex_lock(&client->lock);
		valid = ion_handle_validate(client, data.handle);
		mutex_unlock(&client->lock);
		if(!valid)
			return -EINVAL;
		ret = ion_phys(client, data.handle, (ion_phys_addr_t *)&data.phys_addr, (size_t *)&data.size);
		if(ret)
			return -EINVAL;
		if(copy_to_user((void __user *)arg, &data, sizeof(data)))
			return -EFAULT;
		break;
	}
	case ION_IOC_SUNXI_DMA_COPY:
	{
		dma_buf_group buf_group;

		ret = -EINVAL;
		if(copy_from_user(&buf_group, (u32 *)arg, sizeof(buf_group))) {
			pr_err("%s(%d) err: copy_from_user err\n", __func__, __LINE__);
			goto end;
		}
		if(buf_group.cnt > DMA_BUF_MAXCNT) {
			pr_err("%s(%d) err: buf cnt %d exceed %d\n", __func__, __LINE__,
				buf_group.cnt, DMA_BUF_MAXCNT);
			goto end;
		}

		__flush_cache_before_dma(&buf_group);
		ret = (long)dma_copy_buf(&buf_group);
		break;
	}
	case ION_IOC_SUNXI_DUMP:
		sunxi_ion_dump_mem();
		break;
	default:
		return -ENOTTY;
	}
end:
	return ret;
}

int sunxi_ion_probe(struct platform_device *pdev)
{
	struct ion_platform_data *pdata = pdev->dev.platform_data;
	struct ion_platform_heap *heaps_desc;
	int i, ret = 0;

	pheap = kzalloc(sizeof(struct ion_heap *) * pdata->nr, GFP_KERNEL);
	idev = ion_device_create(sunxi_ion_ioctl);
	if(IS_ERR_OR_NULL(idev)) {
		kfree(pheap);
		return PTR_ERR(idev);
	}

	for(i = 0; i < pdata->nr; i++) {
		heaps_desc = &pdata->heaps[i];
		if(heaps_desc->type == ION_HEAP_TYPE_CARVEOUT) {
			heaps_desc->base = ion_mem.start;
			heaps_desc->size = ion_mem.size;
		}
		if(heaps_desc->type == ION_HEAP_TYPE_SECURE) {
#ifdef			CONFIG_SUNXI_TRUSTZONE
			struct smc_param param;
			param.a0 = TEE_SMC_PLAFORM_OPERATION;
			param.a1 = TE_SMC_GET_DRM_MEM_INFO;
			sunxi_smc_call(&param);
			heaps_desc->base = param.a2;
			heaps_desc->size = param.a3;
			pr_debug("%s: secure-heap base=%x size= %x\n", 
			         __func__, heaps_desc->base, heaps_desc->size);
#endif
		}
		pheap[i] = ion_heap_create(heaps_desc);
		if(IS_ERR_OR_NULL(pheap[i])) {
			ret = PTR_ERR(pheap[i]);
			goto err;
		}
		ion_device_add_heap(idev, pheap[i]);

		if(heaps_desc->type == ION_HEAP_TYPE_CARVEOUT)
			carveout_heap = pheap[i];
	}

	num_heaps = i;
	platform_set_drvdata(pdev, idev);
	return 0;
err:
	while(i--)
		ion_heap_destroy(pheap[i]);
	ion_device_destroy(idev);
	kfree(pheap);
	return ret;
}

int sunxi_ion_remove(struct platform_device *pdev)
{
	struct ion_device *dev = platform_get_drvdata(pdev);

	while(num_heaps--)
		ion_heap_destroy(pheap[num_heaps]);
	kfree(pheap);
	ion_device_destroy(dev);
	return 0;
}

static struct ion_platform_data ion_data = {
#ifndef  CONFIG_SUNXI_TRUSTZONE
	.nr = 3,
#else
	.nr = 4,
#endif
	.heaps = {
		[0] = {
			.type = ION_HEAP_TYPE_SYSTEM,
			.id = (u32)ION_HEAP_TYPE_SYSTEM,
			.name = "sytem",
		},
		[1] = {
			.type = ION_HEAP_TYPE_SYSTEM_CONTIG,
			.id = (u32)ION_HEAP_TYPE_SYSTEM_CONTIG,
			.name = "system_contig",
		},
#ifdef CONFIG_CMA
		[2] = {
			.type = ION_HEAP_TYPE_DMA,
			.id = (u32)ION_HEAP_TYPE_DMA,
			.name = "cma",
			.base = 0, .size = 0,
			.align = 0, .priv = NULL,
		},
#else
		[2] = {
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = (u32)ION_HEAP_TYPE_CARVEOUT,
			.name = "carveout",
			.base = 0, .size = 0,
			.align = 0, .priv = NULL,
		},
#endif
#ifdef          CONFIG_SUNXI_TRUSTZONE
		[3] = {
			.type = ION_HEAP_TYPE_SECURE,
			.id = (u32)ION_HEAP_TYPE_SECURE,
			.name = "secure",
			.base = 0, .size = 0,
			.align = 0, .priv = NULL,
		},
#endif
	}
};
static struct platform_device ion_dev = {
	.name = DEV_NAME,
	.dev = {
		.platform_data = &ion_data,
	}
};
static struct platform_driver ion_driver = {
	.probe = sunxi_ion_probe,
	.remove = sunxi_ion_remove,
	.driver = {.name = DEV_NAME}
};

#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
static int ion_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, sunxi_ion_show, NULL);
}

static const struct file_operations ion_dbg_fops = {
	.owner = THIS_MODULE,
	.open = ion_stats_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif

static int __init sunxi_ion_init(void)
{
	int ret;

#ifdef CONFIG_PROC_FS
	proc_create("sunxi_ion", S_IRUGO, NULL, &ion_dbg_fops);
#endif

	ret = platform_device_register(&ion_dev);
	if(ret)
		return ret;
	return platform_driver_register(&ion_driver);
}

static void __exit sunxi_ion_exit(void)
{
#ifdef CONFIG_PROC_FS
	remove_proc_entry("sunxi_ion", NULL);
#endif
	platform_driver_unregister(&ion_driver);
	platform_device_unregister(&ion_dev);
}

subsys_initcall(sunxi_ion_init);
module_exit(sunxi_ion_exit);
