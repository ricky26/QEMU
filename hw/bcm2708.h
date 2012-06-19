#ifndef  QEMU_BCM2708_H
#define  QEMU_BCM2708_H

#include "memory.h"
#include "exec-memory.h"
#include "console.h"

struct bcm2708_mbox;
typedef void (*bcm2708_mbox_ready_fn)(void *_opaque,
		struct bcm2708_mbox *_box,
		int _idx);
typedef void (*bcm2708_mbox_msg_fn)(void *_opaque,
		struct bcm2708_mbox *_box,
		int _idx, uint32_t _msg);

typedef struct bcm2708_mbox_ready
{
	void *opaque;
	bcm2708_mbox_ready_fn fn;
} bcm2708_mbox_ready;

typedef struct bcm2708_mbox_msg
{
	void *opaque;
	bcm2708_mbox_msg_fn fn;
} bcm2708_mbox_msg;

struct bcm2708_mbox
{
	int index;
	struct bcm2708_state *parent;

	// Actual messages

	bcm2708_mbox_ready ready[2];
	bcm2708_mbox_msg handler[2];
	uint32_t sts[2];
	uint32_t msg[2];
};

// VC structs

struct bcm2708_vc;
typedef void (*bcm2708_vc_mbox_handler)(struct bcm2708_vc *_vc,
		int _chan, uint32_t _msg);

struct bcm2708_fb
{
	uint32_t xres, yres, xres_virtual, yres_virtual;
	uint32_t pitch, bpp;
	uint32_t xoff, yoff;
	uint32_t base;
	uint32_t screen_size;
};

struct bcm2708_vc
{
	struct bcm2708_state *parent;

	// FB
	DisplayState *disp;
	struct bcm2708_fb fb;
	int fb_invalidate, fb_bpp;
	
	bcm2708_vc_mbox_handler mbox_handler[8];
};

// bcm2708 structs
struct bcm2708_state
{
	MemoryRegion *iomem;
	struct bcm2708_vc vc;
	
	// ST
	int st_count;
	int st_next;
	QEMUTimer *st_timer;

	// ARMCTL
	uint32_t irqsts[3];
	uint32_t irqmask[3];
	uint32_t fiq;
	qemu_irq *cpu_irqs, *irqs;
	struct bcm2708_mbox mbox[4];
};

void bcm2708_init(struct bcm2708_state *_st,
		CPUState *_env, MemoryRegion *_bus);

void bcm2708_shutdown(struct bcm2708_state *_st);

// Used by bcm2708 drivers:

static inline void bcm2708_mbox_set_ready(struct bcm2708_state *_st, int _mbox, int _idx,
		bcm2708_mbox_ready_fn _fn, void *_opaque)
{
	bcm2708_mbox_ready *p = &_st->mbox[_mbox].ready[_idx];
	p->fn = _fn;
	p->opaque = _opaque;
}

static inline void bcm2708_mbox_set_handler(struct bcm2708_state *_st, int _mbox, int _idx,
		bcm2708_mbox_msg_fn _fn, void *_opaque)
{
	bcm2708_mbox_msg *p = &_st->mbox[_mbox].handler[_idx];
	p->fn = _fn;
	p->opaque = _opaque;
}

void bcm2708_mbox_push(struct bcm2708_mbox *_mbox, int _idx, uint32_t _msg);
uint32_t bcm2708_mbox_pop(struct bcm2708_mbox *_mbox, int _idx);
uint32_t bcm2708_mbox_peek(struct bcm2708_mbox *_mbox, int _idx);
void bcm2708_mbox_flush(struct bcm2708_mbox *_mbox, int _idx);

void bcm2708_vc_init(struct bcm2708_vc *_vc, struct bcm2708_state *_st);
void bcm2708_vc_shutdown(struct bcm2708_vc *_vc);

#endif //QEMU_BCM2708_H
