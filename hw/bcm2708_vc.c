#include "bcm2708.h"
#include "console.h"
#include "framebuffer.h"

#define VC_MBOX_POWER	0
#define VC_MBOX_FB		1
#define VC_MBOX_VUART	2
#define VC_MBOX_VCHIQ	3
#define VC_MBOX_LEDS	4
#define VC_MBOX_BUTTONS	5
#define VC_MBOX_TOUCH	6

//
// Debug flags
//#define DEBUG_MBOX
#define DEBUG_POWER
#define DEBUG_FB
//

// =============================
// Unashamedly stolen from pl110
#include "pixel_ops.h"

#define BITS 8
#include "pl110_template.h"
#define BITS 15
#include "pl110_template.h"
#define BITS 16
#include "pl110_template.h"
#define BITS 24
#include "pl110_template.h"
#define BITS 32
#include "pl110_template.h"

enum bppmode
{
    BPP_1,
    BPP_2,
    BPP_4,
    BPP_8,
    BPP_16,
    BPP_32,
    BPP_16_565, /* PL111 only */
    BPP_12      /* PL111 only */
};
// =============================

static void bcm2708_fb_update(void *_opaque)
{
	struct bcm2708_vc *vc = _opaque;
    drawfn* fntable;
    drawfn fn;
    int dest_width;
    int src_width;
    int bpp_offset;
    int first;
    int last;

    switch (ds_get_bits_per_pixel(vc->disp))
	{
    case 0:
        return;
    case 8:
        fntable = pl110_draw_fn_8;
        dest_width = 1;
        break;
    case 15:
        fntable = pl110_draw_fn_15;
        dest_width = 2;
        break;
    case 16:
        fntable = pl110_draw_fn_16;
        dest_width = 2;
        break;
    case 24:
        fntable = pl110_draw_fn_24;
        dest_width = 3;
        break;
    case 32:
        fntable = pl110_draw_fn_32;
        dest_width = 4;
        break;
    default:
        fprintf(stderr, "bcm2708: Bad color depth\n");
        exit(1);
    }

    bpp_offset = 0;//24;
    fn = fntable[vc->fb_bpp + bpp_offset];

    src_width = vc->fb.xres;
    switch (vc->fb_bpp) {
    case BPP_1:
        src_width >>= 3;
        break;
    case BPP_2:
        src_width >>= 2;
        break;
    case BPP_4:
        src_width >>= 1;
        break;
    case BPP_8:
        break;
    case BPP_16:
    case BPP_16_565:
    case BPP_12:
        src_width <<= 1;
        break;
    case BPP_32:
        src_width <<= 2;
        break;
    }

    dest_width *= vc->fb.xres;
    first = 0;
    framebuffer_update_display(vc->disp,
								   vc->fb.base, vc->fb.xres, vc->fb.yres,
								   src_width, dest_width, 0,
								   vc->fb_invalidate,
								   fn, NULL,
								   &first, &last);
    if (first >= 0) {
        dpy_update(vc->disp, 0, first, vc->fb.xres, last - first + 1);
    }
    vc->fb_invalidate = 0;
}

static void bcm2708_fb_invalidate(void *_opaque)
{
	struct bcm2708_vc *vc = _opaque;
	vc->fb_invalidate = 1;
	qemu_console_resize(vc->disp, vc->fb.xres, vc->fb.yres);
}

//
// VC mailbox handlers
//

static void bcm2708_vc_send(struct bcm2708_vc *_vc,
		int _chan, uint32_t _msg)
{
	uint32_t rmsg = (_msg &~ 0xf) | (_chan & 0xf);
	bcm2708_mbox_flush(&_vc->parent->mbox[0], 0);
	bcm2708_mbox_push(&_vc->parent->mbox[0], 0, rmsg);
}

static void bcm2708_vc_fb(struct bcm2708_vc *_vc,
		int _chan, uint32_t _msg)
{
	target_phys_addr_t dma = _msg &~ (0xc << 28);
	cpu_physical_memory_read(dma, &_vc->fb, sizeof(_vc->fb));

	// TODO: much better calculations.

	int pitch;
	switch(_vc->fb.bpp)
	{
	case 8:
		pitch = _vc->fb.xres;
		_vc->fb_bpp = BPP_8;
		break;
	
	case 16:
		pitch = _vc->fb.xres << 1;
		_vc->fb_bpp = BPP_16_565;
		break;

	case 32:
		pitch = _vc->fb.xres << 2;
		_vc->fb_bpp = BPP_32;
		break;

	default:
		pitch = _vc->fb.xres << 1;
		_vc->fb_bpp = BPP_16_565;
		break;
	}

	target_phys_addr_t fbsz = pitch*_vc->fb.yres;
	target_phys_addr_t addr = 128*1024*1024; // Currently hard-coded in kernel?

#ifdef DEBUG_FB
	printf("fb mapped to 0x%08x.\n", addr);
#endif

	_vc->fb_invalidate = 1;
	_vc->fb.pitch = pitch;
	_vc->fb.base = addr;
	_vc->fb.screen_size = fbsz;
	cpu_physical_memory_write(dma, &_vc->fb, sizeof(_vc->fb));


	bcm2708_vc_send(_vc, _chan, 0);

	qemu_console_resize(_vc->disp, _vc->fb.xres, _vc->fb.yres);
}

static void bcm2708_vc_pm_req(struct bcm2708_vc *_vc,
		int _chan, uint32_t _msg)
{
#ifdef DEBUG_POWER
	printf("vc pm req: 0x%08x.\n", _msg);
#endif
	bcm2708_vc_send(_vc, _chan, _msg);
}

static void bcm2708_vc_msg(void *_opaque,
		struct bcm2708_mbox *_mbox,
		int _idx, uint32_t _msg)
{
	struct bcm2708_vc *vc = _opaque;
	
	int chan = _msg & 0xf;
	uint32_t msg = _msg &~ 0xf;

#ifdef DEBUG_MBOX
	printf("vc mbox msg: (%d, 0x%08x).\n",
			chan, msg);
#endif

	bcm2708_mbox_pop(_mbox, _idx);
	
	if(vc->mbox_handler[chan])
		vc->mbox_handler[chan](vc, chan, msg);
}

void bcm2708_vc_init(struct bcm2708_vc *_vc, struct bcm2708_state *_st)
{
	memset(_vc, 0, sizeof(*_vc));
	_vc->parent = _st;

	bcm2708_mbox_set_handler(_st, 0, 1, bcm2708_vc_msg, _vc);

	// Setup handlers
	_vc->mbox_handler[VC_MBOX_POWER] = bcm2708_vc_pm_req;
	_vc->mbox_handler[VC_MBOX_FB] = bcm2708_vc_fb;

	// Initialize display
	_vc->disp = graphic_console_init(bcm2708_fb_update,
			bcm2708_fb_invalidate, NULL, NULL, _vc);
}
