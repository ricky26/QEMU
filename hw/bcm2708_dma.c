#include "bcm2708.h"
#include "irq.h"

#define DMA_BASE		0x7000
#define DMAx_BASE(x)	(DMA_BASE + 0x100*(x))
#define DMA15_BASE		0xe05000
#define DMAC_BASE		(DMA_BASE + 0xf00)

#define DMA_CHANNEL_SIZE 0xe0
#define DMA_SIZE		0x1000

#define DMA_INT_STATUS	0xe0
#define DMA_ENABLE		0xf0

#define DMA_CS			0x00
#define DMA_ADDR		0x04
#define DMA_TI			0x08
#define DMA_SOURCE_AD	0x0C
#define DMA_DEST_AD		0x10
#define DMA_TXFR_LEN	0x14
#define DMA_STRIDE		0x18
#define DMA_NEXTCB		0x1C
#define DMA_DEBUG		0x20

#define CS_ACTIVE		(1 << 0)
#define CS_END			(1 << 1)
#define CS_INT			(1 << 2)
#define CS_ERROR		(1 << 8)
#define CS_ABORT		(1 << 30)
#define CS_RESET		(1 << 31)

#define TI_SRC_IGN		(1 << 11)
#define TI_SRC_INC		(1 << 8)
#define TI_DEST_IGN		(1 << 7)
#define TI_DEST_INC		(1 << 4)
#define TI_2D			(1 << 1)
#define TI_INTEN		(1 << 0)

//
// Debug Flags
#define DEBUG_DMA
//#define DEBUG_DMA_DMA
//#define DEBUG_IO
//

#define IRQ_DMA0		48
#define IRQ_DMAx(x)		(IRQ_DMA0 + (x))

static target_phys_addr_t hw_map(target_phys_addr_t _addr)
{
	if(_addr >= 0x7e000000)
		return 0x20000000 + (_addr - 0x7e000000);

	if(_addr >= 0x40000000)
		return _addr - 0x40000000;

	return _addr;
}

static void dma_xfer(struct bcm2708_dma_channel *_ch)
{
	while(_ch->addr)
	{
		cpu_physical_memory_read(hw_map(_ch->addr), &_ch->cb, sizeof(_ch->cb));

#ifdef DEBUG_DMA
		fprintf(stderr, "%s: "
				"0x%08x 0x%08x 0x%08x "
				"0x%08x 0x%08x 0x%08x\n",
				__func__, _ch->cb.ti,
				_ch->cb.source_ad, _ch->cb.dest_ad,
				_ch->cb.txfr_len, _ch->cb.stride,
				_ch->cb.nextconbk);
#endif

		// Process this conbk
		uint32_t ti = _ch->cb.ti;
		target_phys_addr_t src=hw_map(_ch->cb.source_ad),
			dest=hw_map(_ch->cb.dest_ad);
		uint32_t xferlen = _ch->cb.txfr_len;
		unsigned row, col, ds, ss;

		if(ti & TI_2D)
		{
			uint32_t stride = _ch->cb.stride;
			row = xferlen & 0xFFFF/4;
			col = xferlen >> 16;
			ds = stride >> 16;
			ss = stride & 0xFFFF;
		}
		else
		{
			row = xferlen/4;
			col = 1;
			ds = ss = 0;
		}
		
		for(;col;col--)
		{
			for(;row;row--)
			{
#ifdef DEBUG_DMA_DMA
				printf("dma 0x%08x->0x%08x.\n", src, dest);
#endif

				uint32_t v = (ti & TI_SRC_IGN) ? 0 : ldl_phys(src);
				if(!(ti & TI_DEST_IGN))
					stl_phys(dest, v);

				if(ti & TI_DEST_INC)
					dest += 4;
				if(ti & TI_SRC_INC)
					src += 4;
			}
			
			if(ti & TI_DEST_INC)
				dest += ds;
			if(ti & TI_SRC_INC)
				src += ss;
		}

		_ch->cs |= CS_END;

		// Check for IRQ
		if(ti & TI_INTEN)
		{	
#ifdef DEBUG_DMA
			fprintf(stderr, "dma triggering IRQ.\n");
#endif
			_ch->cs |= CS_INT;
			qemu_irq_raise(_ch->irq);
		}

		_ch->addr = _ch->cb.nextconbk;
	}
}

static uint64_t dma_read(void *_opaque,
		target_phys_addr_t _addr, unsigned _sz)
{
	struct bcm2708_dma_channel *ch = _opaque;

#ifdef DEBUG_IO
	fprintf(stderr, "%s: 0x%08x.\n",
			__func__, _addr);
#endif

	switch(_addr)
	{
	case DMA_CS:
		return ch->cs;

	case DMA_ADDR:
		return ch->addr;

	case DMA_TI:
		return ch->cb.ti;

	case DMA_SOURCE_AD:
		return ch->cb.source_ad;

	case DMA_DEST_AD:
		return ch->cb.dest_ad;

	case DMA_TXFR_LEN:
		return ch->cb.txfr_len;

	case DMA_STRIDE:
		return ch->cb.stride;

	case DMA_NEXTCB:
		return ch->cb.nextconbk;
		
	case DMA_DEBUG:
		return ch->dbg;

	default:
		fprintf(stderr, "Invalid DMA read at 0x%08x.\n", _addr);
		exit(1);
		return -1;
	};
}

static void dma_write(void *_opaque,
		target_phys_addr_t _addr, uint64_t _data, unsigned _sz)
{
	struct bcm2708_dma_channel *ch = _opaque;

#ifdef DEBUG_IO
		fprintf(stderr, "%s: 0x%08x, 0x%08x.\n",
				__func__, _addr, (uint32_t)_data);
#endif

	switch(_addr)
	{
	case DMA_CS:
        {
		uint32_t d = (uint32_t)_data;
		int active = d & CS_ACTIVE;

		// clear status bits
		ch->cs &=~ (d & (CS_INT | CS_END));

		if(ch->cs & CS_INT)
			qemu_irq_raise(ch->irq);
		else
			qemu_irq_lower(ch->irq);

		if(active)
			dma_xfer(ch);
        }
		break;

	case DMA_ADDR:
		ch->addr = (target_phys_addr_t)_data;
		break;

	default:
		fprintf(stderr, "Invalid DMA write at 0x%08x.\n", _addr);
		exit(1);
	}
}

static const MemoryRegionOps dma_ops = {
	.read = dma_read,
	.write = dma_write,
};

static uint64_t dma_global_read(void *_opaque,
		target_phys_addr_t _addr, unsigned _sz)
{
	struct bcm2708_dma *dma = _opaque;

	switch(_addr)
	{
	case DMA_INT_STATUS:
		return dma->sts;

	case DMA_ENABLE:
		return dma->enable;

	default:
		fprintf(stderr, "Invalid read on DMA at 0x%08x.\n",
				_addr);
		exit(1);
		return -1;
	}
}

static void dma_global_write(void *_opaque,
		target_phys_addr_t _addr, uint64_t _data, unsigned _sz)
{
	struct bcm2708_dma *dma = _opaque;

	switch(_addr)
	{
	case DMA_INT_STATUS:
		dma->sts &=~ _data;
		break;

	case DMA_ENABLE:
		dma->enable = (uint32_t)_data;
		break;

	default:
		fprintf(stderr, "Invalid write on DMA at 0x%08x.\n",
				_addr);
		exit(1);
	}
}

static const MemoryRegionOps dma_global_ops = {
	.read = dma_global_read,
	.write = dma_global_write,
};

void bcm2708_dma_init(struct bcm2708_dma *_dma, struct bcm2708_state *_st)
{

	memset(_dma, 0, sizeof(*_dma));

	_dma->iomem = g_new(MemoryRegion, 1);
	memory_region_init_io(_dma->iomem, &dma_global_ops,
			_dma, "bcm2708.dma", 0xf0);
	memory_region_add_subregion(_st->iomem,
			DMAC_BASE, _dma->iomem);
	
	_dma->parent = _st;
	_dma->enable = -1;

	unsigned i;
	for(i = 0; i < 16; i++)
	{
		struct bcm2708_dma_channel *ch = &_dma->channel[i];
		ch->dma = _dma;
		ch->index = i;
		ch->irq = _st->irqs[IRQ_DMAx(i)];
		
		MemoryRegion *iomem = g_new(MemoryRegion, 1);
		memory_region_init_io(iomem, &dma_ops,
				ch, "bcm2708.dmac", DMA_CHANNEL_SIZE);

		if(i != 15)
			memory_region_add_subregion(_st->iomem,
				    DMAx_BASE(i), iomem);
		else	
			memory_region_add_subregion(_st->iomem,
					DMA15_BASE, iomem);
	}
}

void bcm2708_dma_shutdown(struct bcm2708_dma *_dma)
{
}
