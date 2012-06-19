#include "bcm2708.h"
#include "irq.h"
#include "qemu-timer.h"
#include "arm-misc.h"
#include "sysbus.h"

// Debug flags
//#define DEBUG_IRQ
//#define DEBUG_ST
//#define DEBUG_MBOX
//#define DEBUG_MBOX2
//

#define BCM2708_IO_BASE 0x20000000
#define BCM2708_IO_SIZE 0x10000000

#define ARMCTL_BASE		0xB000
#define ARMCTL_SIZE		0x1000

#define ST_BASE			0x3000
#define ST_SIZE			0x1000

#define GPIO_BASE		0x200000
#define GPIO_SIZE		0x1000

#define UART0_BASE	  	0x201000
#define UART0_SIZE	  	0x1000

#define SDHCI_BASE		0x300000
#define SDHCI_SIZE		0x1000

#define USB_BASE		0x980000
#define USB_SIZE		0x100000

// Weird mapping.
#define IRQ_ARM0	(0 << 5)
#define IRQ_ARM1	(1 << 5)
#define IRQ_ARM2	(2 << 5)

#define IRQ_TIMER0	(IRQ_ARM1 + 0)
#define IRQ_TIMER1	(IRQ_ARM1 + 1)
#define IRQ_TIMER2	(IRQ_ARM1 + 2)
#define IRQ_ST		(IRQ_ARM1 + 3)

#define IRQ_MBOX	(IRQ_ARM0 + 1)
#define IRQ_PND1	(IRQ_ARM0 + 8)
#define IRQ_PND2	(IRQ_ARM0 + 9)
#define IRQ_USB		(IRQ_ARM0 + 11)
#define IRQ_UART0	(IRQ_ARM0 + 19)
#define IRQ_SDHCI	(IRQ_ARM0 + 20)

#define NR_IRQS		(32*3)

//
// ARMCTL
//

#define ARMCTL_CONTROL0		0x000
#define ARMCTL_CONTROL1 	0x440
#define ARMCTL_STATUS		0x444
#define ARMCTL_ERRHLT		0x448
#define ARMCTL_IDSECURE		0x00C
#define ARMCTL_ID			0x44C
#define ARMCTL_TRANSLATE	0x100
#define ARMCTL_IRQPEND0		0x200
#define ARMCTL_IRQPEND1		0x204
#define ARMCTL_IRQPEND2		0x208
#define ARMCTL_IRQFAST		0x20C
#define ARMCTL_IRQENBL0		0x210
#define ARMCTL_IRQENBL1		0x214
#define ARMCTL_IRQENBL2		0x218
#define ARMCTL_IRQDIBL0		0x21C
#define ARMCTL_IRQDIBL1		0x220
#define ARMCTL_IRQDIBL2		0x224
#define ARMCTL_TLOAD		0x400
#define ARMCTL_TVALUE		0x404
#define ARMCTL_TCONTROL		0x408
#define ARMCTL_TIRQCTL		0x40C
#define ARMCTL_TRAWIRQ		0x410
#define ARMCTL_TMSKIRQ		0x414
#define ARMCTL_TRELOAD		0x418
#define ARMCTL_TPREDIV		0x41C
#define ARMCTL_TFREECNT		0x420

#define MBOX_SIZE			0x100
#define MBOX_COUNT			4
#define MBOX_BASE(x)		(0x800 + ((x)*0x100))

#define MBOX_SEM_COUNT		8
#define MBOX_SEM(x, y)		(((y)*4))

#define MBOX_BELL_COUNT		4
#define MBOX_BELL(x, y)		(((y)*4) + 0x40)

#define MBOX_BOX_COUNT		2
#define MBOX_BOX_BASE(x)	(((x)*0x20) + 0x80)
#define MBOX_BOX_WRT(x)		(MBOX_BOX_BASE(x) + 0x00)
#define MBOX_BOX_RD(x)		(MBOX_BOX_BASE(x) + 0x00)
#define MBOX_BOX_POL(x)		(MBOX_BOX_BASE(x) + 0x10)
#define MBOX_BOX_SND(x)		(MBOX_BOX_BASE(x) + 0x14)
#define MBOX_BOX_STA(x)		(MBOX_BOX_BASE(x) + 0x18)
#define MBOX_BOX_CNF(x)		(MBOX_BOX_BASE(x) + 0x1C)
#define MBOX_SEMCLRDBG		(0xE0)
#define MBOX_BELLCLRDBG		(0xE4)
#define MBOX_ALL_IRQS		(0xF8)
#define MBOX_MY_IRQS		(0xFC)

#define MBOX_ST_FULL		(1 << 31)
#define MBOX_ST_EMPTY		(1 << 30)

static void armctl_update(struct bcm2708_state *_st, int _vic)
{
	if(_vic)
	{
		unsigned irq = IRQ_PND1 + _vic - 1;

		if(_st->irqsts[_vic])
			qemu_irq_raise(_st->irqs[irq]);
		else
			qemu_irq_lower(_st->irqs[irq]);
	}
	else
	{
/*		if(_st->fiq & 0x80
				&& (_st->fiq & 0x7f) == 40+bit)
		{
			// FIQ
			if(_st->irqsts[_vic])
				qemu_irq_raise(_st->cpu_irqs[1]);
			else
				qemu_irq_lower(_st->cpu_irqs[1]);
		}
		else*/
		{
			if(_st->irqsts[_vic])
			{
#ifdef DEBUG_IRQ
				printf("raised CPU IRQ.\n");
#endif
				qemu_irq_raise(_st->cpu_irqs[0]);
			}
			else
			{
#ifdef DEBUG_IRQ
				printf("lowered CPU IRQ.\n");
#endif
				qemu_irq_lower(_st->cpu_irqs[0]);
			}
		}
	}
}

static uint64_t armctl_read(void *_opaque,
		target_phys_addr_t _addr, unsigned _sz)
{
	struct bcm2708_state *state = _opaque;

	switch(_addr)
	{
	case ARMCTL_IRQPEND0:
	case ARMCTL_IRQPEND1:
	case ARMCTL_IRQPEND2:
	    {
			int ctl = (_addr - ARMCTL_IRQPEND0)/sizeof(uint32_t);
#ifdef DEBUG_IRQ			
			printf("irqsts-%d = 0x%08x.\n", ctl, state->irqsts[ctl]);
#endif
			return state->irqsts[ctl];
		}
		break;
	}

	fprintf(stderr, "Invalid ARMCTL read. 0x%08x.\n",
			_addr);
	exit(1);
}

static void armctl_write(void *_opaque,
		target_phys_addr_t _addr, uint64_t _data, unsigned _sz)
{
	struct bcm2708_state *state = _opaque;

	switch(_addr)
	{
	case ARMCTL_TCONTROL:
		fprintf(stderr, "warning: arm-timers not implemented.\n");
		break;

	case ARMCTL_IRQPEND0:
	case ARMCTL_IRQPEND1:
	case ARMCTL_IRQPEND2:
	    {
			int ctl = (_addr - ARMCTL_IRQPEND0)/sizeof(uint32_t);
			unsigned i;
			
			for(i = 0; i < 32; i++)
			{
				if(_data & (1 << i))
					qemu_irq_lower(state->irqs[32*ctl + i]);
			}
		}
		break;

	case ARMCTL_IRQENBL0:
	case ARMCTL_IRQENBL1:
	case ARMCTL_IRQENBL2:
	    {
			int ctl = (_addr - ARMCTL_IRQENBL0)/sizeof(uint32_t);
			state->irqmask[ctl] |= _data;
			armctl_update(state, ctl);
#ifdef DEBUG_IRQ
			printf("unmask-%d: 0x%08x.\n", ctl, (uint32_t)_data);
#endif
		}	
		break;

	case ARMCTL_IRQDIBL0:
	case ARMCTL_IRQDIBL1:
	case ARMCTL_IRQDIBL2:
	    {
			int ctl = (_addr - ARMCTL_IRQDIBL0)/sizeof(uint32_t);
			state->irqmask[ctl] &=~ _data;
			armctl_update(state, ctl);
#ifdef DEBUG_IRQ
			printf("mask-%d: 0x%08x.\n", ctl, (uint32_t)_data);
#endif
		}	
		break;

	default:
		fprintf(stderr, "Invalid write to ARMCTL at 0x%08x.\n", _addr);
		exit(1);
		return;
	}
}

static const MemoryRegionOps armctl_ops = {
	.read = armctl_read,
	.write = armctl_write,
};

static void armctl_irq(void *_opaque,
		int _n, int _level)
{
	struct bcm2708_state *state = _opaque;
	unsigned ctl = _n >> 5;
	unsigned bit = _n & 0x1f;
	
	if(_level)
		state->irqsts[ctl] |= (1 << bit) & state->irqmask[ctl];
	else
		state->irqsts[ctl] &=~ (1 << bit);

#ifdef DEBUG_IRQ
	printf("%s(%d, %d): %d = 0x%08x.\n",
			__func__, _n, _level,
			ctl, state->irqsts[ctl]);
#endif

	armctl_update(state, ctl);
}

void bcm2708_mbox_push(struct bcm2708_mbox *_mbox,
		int _idx, uint32_t _msg)
{
	_mbox->msg[_idx] = _msg;
	_mbox->sts[_idx] = (_mbox->sts[_idx] &~ MBOX_ST_EMPTY)
		| MBOX_ST_FULL; // Clear empty, set full

#ifdef DEBUG_MBOX2
	fprintf(stderr, "mbox-%d,%d pushed 0x%08x.\n",
			_mbox->index, _idx, _msg);
#endif

	if(_mbox->handler[_idx].fn)
		_mbox->handler[_idx].fn(
			_mbox->handler[_idx].opaque,
			_mbox, _idx, _msg);
}

uint32_t bcm2708_mbox_pop(struct bcm2708_mbox *_mbox,
		int _idx)
{
	uint32_t ret = _mbox->msg[_idx];
	_mbox->sts[_idx] = (_mbox->sts[_idx] &~ MBOX_ST_FULL)
		| MBOX_ST_EMPTY; // Clear full, set empty

#ifdef DEBUG_MBOX2
	fprintf(stderr, "mbox-%d,%d popped 0x%08x.\n",
			_mbox->index, _idx, ret);
#endif

	if(_mbox->ready[_idx].fn)
		_mbox->ready[_idx].fn(
			_mbox->ready[_idx].opaque,
			_mbox, _idx);

	return ret;
}

uint32_t bcm2708_mbox_peek(struct bcm2708_mbox *_mbox,
		int _idx)
{
#ifdef DEBUG_MBOX2
	fprintf(stderr, "mbox-%d,%d peeked 0x%08x.\n",
			_mbox->index, _idx, _mbox->msg[_idx]);
#endif

	return _mbox->msg[_idx];
}

void bcm2708_mbox_flush(struct bcm2708_mbox *_mbox,
		int _idx)
{
	if(_mbox->sts[_idx] & MBOX_ST_EMPTY)
		return;

	_mbox->sts[_idx] = (_mbox->sts[_idx] &~ MBOX_ST_FULL)
		| MBOX_ST_EMPTY;

	if(_mbox->ready[_idx].fn)
		_mbox->ready[_idx].fn(
			_mbox->ready[_idx].opaque,
			_mbox, _idx);
}

static uint64_t armctl_mbox_read(void *_opaque,
		target_phys_addr_t _addr, unsigned _sz)
{
	struct bcm2708_mbox *state = _opaque;
	int idx;

	switch(_addr)
	{
	case MBOX_BOX_STA(0):
	case MBOX_BOX_STA(1):
		idx = (_addr == MBOX_BOX_STA(0))? 0: 1;

#ifdef DEBUG_MBOX
		fprintf(stderr, "mbox-%d,%d status.\n",
				state->index, idx);
#endif
		return state->sts[idx];

	case MBOX_BOX_RD(0):
	case MBOX_BOX_RD(1):
		idx = (_addr == MBOX_BOX_RD(0))? 0: 1;

#ifdef DEBUG_MBOX
		fprintf(stderr, "mbox-%d,%d read.\n",
				state->index, idx);
#endif
		return bcm2708_mbox_pop(state, idx);

	case MBOX_BOX_POL(0):
	case MBOX_BOX_POL(1):
		idx = (_addr == MBOX_BOX_POL(0))? 0: 1;

#ifdef DEBUG_MBOX
		fprintf(stderr, "mbox-%d,%d poll.\n",
				state->index, idx);
#endif
		return bcm2708_mbox_peek(state, idx);

	default:
		fprintf(stderr, "Bad read on mailbox, 0x%08x.\n",
				_addr);
		exit(1);
	}

	return 0;
}

static void armctl_mbox_write(void *_opaque,
		target_phys_addr_t _addr, uint64_t _data, unsigned _sz)
{
	struct bcm2708_mbox *state = _opaque;
	int idx;

	switch(_addr)
	{
	case MBOX_BOX_CNF(0):
	case MBOX_BOX_CNF(1):
		idx = (_addr == MBOX_BOX_CNF(0))? 0: 1;

#ifdef DEBUG_MBOX
		fprintf(stderr, "mbox-%d,%d config = 0x%08x.\n",
				state->index, idx, (uint32_t)_data);
#endif
		break;

	case MBOX_BOX_WRT(0):
	case MBOX_BOX_WRT(1):
		idx = (_addr == MBOX_BOX_WRT(0))? 0: 1;

#ifdef DEBUG_MBOX
		fprintf(stderr, "mbox-%d,%d write = 0x%08x.\n",
				state->index, idx, (uint32_t)_data);
#endif
		bcm2708_mbox_push(state, idx, (uint32_t)_data);
		break;

	default:
		fprintf(stderr, "Bad write on mailbox, 0x%08x.\n",
				_addr);
		exit(1);
	}
}

static const MemoryRegionOps armctl_mbox_ops = {
	.read = armctl_mbox_read,
	.write = armctl_mbox_write,
};

static void armctl_mbox0_msg0(void *_opaque,
		struct bcm2708_mbox *_mbox,
		int _idx, uint32_t _msg)
{
	qemu_irq_raise(_mbox->parent->irqs[IRQ_MBOX]);
}

static void armctl_mbox0_ready0(void *_opaque,
		struct bcm2708_mbox *_mbox,
		int _idx)
{
	if(_mbox->sts[_idx] & MBOX_ST_EMPTY)
		qemu_irq_lower(_mbox->parent->irqs[IRQ_MBOX]);
}

static void armctl_init(struct bcm2708_state *_st,
		CPUState *_env)
{
	unsigned i;

	MemoryRegion *armmem = g_new(MemoryRegion, 1);
	memory_region_init_io(armmem, &armctl_ops,
			_st, "bcm2708.armctl",
			ARMCTL_SIZE);
	memory_region_add_subregion(_st->iomem,
			ARMCTL_BASE, armmem);

	_st->cpu_irqs = arm_pic_init_cpu(_env);
	_st->irqs = qemu_allocate_irqs(armctl_irq, _st, NR_IRQS);

	memset(&_st->irqsts, 0, sizeof(_st->irqsts));
	memset(&_st->irqmask, -1, sizeof(_st->irqmask));
	_st->irqmask[0] |= (1<<8)|(1<<9); // unmask VIC1-2
	_st->fiq = 0;

	for(i = 0; i < MBOX_COUNT; i++)
	{
		MemoryRegion *mboxmem = g_new(MemoryRegion, 1);
		memory_region_init_io(mboxmem, &armctl_mbox_ops,
				&_st->mbox[i], "bcm2708.mbox",
				MBOX_SIZE);
		memory_region_add_subregion(armmem,
				MBOX_BASE(i), mboxmem);

		memset(&_st->mbox[i], 0, sizeof(_st->mbox[i]));
		
		_st->mbox[i].index = i;
		_st->mbox[i].parent = _st;
	}

	bcm2708_mbox_set_handler(_st, 0, 0,
			armctl_mbox0_msg0, NULL);
	bcm2708_mbox_set_ready(_st, 0, 0,
			armctl_mbox0_ready0, NULL);
}

//
// System Timer
// 

#define ST_INT		0x00
#define ST_CYCLES	0x04
#define ST_NEXT		0x18

static uint64_t st_read(void *_opaque,
		target_phys_addr_t _addr, unsigned _sz)
{
	struct bcm2708_state *state = _opaque;

	switch(_addr)
	{
	case ST_CYCLES:
		return state->st_count;

	default:
		fprintf(stderr, "Outside read to System Timer at 0x%08x.\n",
				_addr);
		exit(1);
		return -1;
	}
}

static void st_write(void *_opaque,
		target_phys_addr_t _addr, uint64_t _data, unsigned _sz)
{
	struct bcm2708_state *state = _opaque;

	switch(_addr)
	{
	case ST_INT:
		qemu_irq_lower(state->irqs[IRQ_ST]);
		break;

	case ST_NEXT:
#ifdef DEBUG_ST
		printf("modded %u\n", (uint32_t)_data);
#endif
		qemu_mod_timer_ns(state->st_timer, qemu_get_clock_ns(vm_clock) + (SCALE_MS/10)*_data);
		break;

	default:
		fprintf(stderr, "Tried to write outside system timer registers "
				"at 0x%08x.\n", _addr);
		exit(1);
	}
}

static const MemoryRegionOps st_ops = {
	.read = st_read,
	.write = st_write,
};

static void st_tick(void *_opaque)
{
	struct bcm2708_state *state = _opaque;

#ifdef DEBUG_ST
	printf("%s: %d, %d.\n", __func__,
			state->st_count, state->st_next);
#endif

	qemu_irq_raise(state->irqs[IRQ_ST]);
}

static void st_init(struct bcm2708_state *_state)
{
	MemoryRegion *stmem = g_new(MemoryRegion, 1);
	memory_region_init_io(stmem, &st_ops,
			_state, "bcm2708.st",
			ST_SIZE);
	memory_region_add_subregion(_state->iomem,
			ST_BASE, stmem);

	_state->st_count = 0;
	_state->st_next = 0;
	_state->st_timer = qemu_new_timer(vm_clock, 10*SCALE_MS, st_tick, _state);
}

//
// GPIO
//

#define GPIO_FSEL(x)		(0x00 + (4*(x)))
#define GPIO_SET(x)			(0x1c + (4*(x)))
#define GPIO_CLR(x)			(0x28 + (4*(x)))
#define GPIO_LEV(x)			(0x34 + (4*(x)))
#define GPIO_EDS(x)			(0x40 + (4*(x)))
#define GPIO_REN(x)			(0x4c + (4*(x)))
#define GPIO_FEN(x)			(0x58 + (4*(x)))
#define GPIO_HEN(x)			(0x64 + (4*(x)))
#define GPIO_LEN(x)			(0x70 + (4*(x)))
#define GPIO_AREN(x)		(0x7c + (4*(x)))
#define GPIO_AFEN(x)		(0x88 + (4*(x)))
#define GPIO_UD(x)			(0x94 + (4*(x)))
#define GPIO_UDCLK(x)		(0x98 + (4*(x)))

static uint64_t gpio_read(void *_opaque,
		target_phys_addr_t _addr, unsigned _sz)
{
    switch(_addr)
	{
	case GPIO_FSEL(0)...GPIO_FSEL(6):
		break;

	case GPIO_SET(0)...GPIO_SET(2):
		break;

	case GPIO_CLR(0)...GPIO_CLR(2):
		break;

	case GPIO_LEV(0)...GPIO_LEV(2):
		break;

	case GPIO_EDS(0)...GPIO_EDS(2):
		break;

	case GPIO_REN(0)...GPIO_REN(2):
		break;

	case GPIO_FEN(0)...GPIO_FEN(2):
		break;

	case GPIO_HEN(0)...GPIO_HEN(2):
		break;

	case GPIO_LEN(0)...GPIO_LEN(2):
		break;

	case GPIO_AREN(0)...GPIO_AREN(2):
		break;

	case GPIO_AFEN(0)...GPIO_AFEN(2):
		break;
	   
	default:
		hw_error("Bad read on GPIO at 0x%08x.\n", _addr);
	}

	return 0;
}

static void gpio_write(void *_opaque,
		target_phys_addr_t _addr, uint64_t _data, unsigned _sz)
{
    switch(_addr)
	{
	case GPIO_FSEL(0)...GPIO_FSEL(6):
		break;

	case GPIO_SET(0)...GPIO_SET(2):
		break;

	case GPIO_CLR(0)...GPIO_CLR(2):
		break;

	case GPIO_LEV(0)...GPIO_LEV(2):
		break;

	case GPIO_EDS(0)...GPIO_EDS(2):
		break;

	case GPIO_REN(0)...GPIO_REN(2):
		break;

	case GPIO_FEN(0)...GPIO_FEN(2):
		break;

	case GPIO_HEN(0)...GPIO_HEN(2):
		break;

	case GPIO_LEN(0)...GPIO_LEN(2):
		break;

	case GPIO_AREN(0)...GPIO_AREN(2):
		break;

	case GPIO_AFEN(0)...GPIO_AFEN(2):
		break;
	   
	default:
		hw_error("Bad write on GPIO at 0x%08x.\n", _addr);
	}
}

static const MemoryRegionOps gpio_ops = {
	.read = gpio_read,
	.write = gpio_write,
};

static void gpio_init(struct bcm2708_state *_st)
{
	MemoryRegion *iomem = g_new(MemoryRegion, 1);
	memory_region_init_io(iomem, &gpio_ops,
			_st, "bcm2708.gpio", GPIO_SIZE);
	memory_region_add_subregion(_st->iomem,
			GPIO_BASE, iomem);
}

//
// BCM2708
//

static uint64_t bcm2708_io_read(void *_opaque,
		target_phys_addr_t _addr, unsigned _sz)
{
	fprintf(stderr, "Invalid IO read at 0x%08x.\n", _addr);
	exit(1);
	return -1;
}

static void bcm2708_io_write(void *_opaque,
		target_phys_addr_t _addr, uint64_t _val, unsigned _sz)
{
	fprintf(stderr, "Invalid IO write at 0x%08x := %llx.\n",
			_addr, (unsigned long long int)_val);
	exit(1);
}

static const MemoryRegionOps bcm2708_io_ops = {
	.read = bcm2708_io_read,
	.write = bcm2708_io_write,
};

void bcm2708_init(struct bcm2708_state *_st, CPUState *_env, MemoryRegion *_bus)
{
	// IO memory
	_st->iomem = g_new(MemoryRegion, 1);
	memory_region_init_io(_st->iomem, &bcm2708_io_ops,
			_st, "bcm2708.io", 
			BCM2708_IO_SIZE);
	memory_region_add_subregion(_bus, BCM2708_IO_BASE, _st->iomem);

	// Peripherals
	armctl_init(_st, _env);
	st_init(_st);
	gpio_init(_st);
	bcm2708_vc_init(&_st->vc, _st);
	
	sysbus_create_simple("pl011", BCM2708_IO_BASE + UART0_BASE, _st->irqs[IRQ_UART0]);
	sysbus_create_simple("sdhci", BCM2708_IO_BASE + SDHCI_BASE, _st->irqs[IRQ_SDHCI]);
	sysbus_create_simple("usb_synopsys",
			BCM2708_IO_BASE + USB_BASE,
			_st->irqs[IRQ_USB]);
}

void bcm2708_shutdown(struct bcm2708_state *_st)
{
	qemu_free_irqs(_st->irqs);
	qemu_free_irqs(_st->cpu_irqs);
}
