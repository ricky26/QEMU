/*
 * Synopsys DesignWareCore for USB OTG.
 *
 * Copyright (c) 2011 Richard Ian Taylor.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "sysbus.h"
#include "qemu-common.h"
#include "qemu-timer.h"
#include "usb.h"
#include "net.h"
#include "irq.h"
#include "hw.h"

// A memorial to a project lost.
//#include "tcp_usb.h"

//
// Debug Flags
//#define DEBUG_USB
//

#define DEVICE_NAME		"usb_synopsys"

// Maximums supported by OIB
#define USB_NUM_ENDPOINTS	8
#define USB_NUM_FIFOS		16
#define USB_NUM_CHANNELS	16

#define RX_FIFO_DEPTH				0x1C0
#define TX_FIFO_DEPTH				0x1C0
#define TX_FIFO_STARTADDR			0x200
#define PERIODIC_TX_FIFO_STARTADDR	0x21B
#define PERIODIC_TX_FIFO_DEPTH		0x100

// Registers
#define GOTGCTL		0x0
#define GOTGINT		0x4
#define GAHBCFG		0x8
#define GUSBCFG		0xC
#define GRSTCTL		0x10
#define GINTSTS		0x14
#define GINTMSK		0x18
#define GRXSTSR		0x1C
#define GRXSTSP		0x20
#define GRXFSIZ		0x24
#define GNPTXFSIZ	0x28
#define GNPTXFSTS	0x2C
#define GSNPSID		0x40
#define GHWCFG1		0x44
#define GHWCFG2		0x48
#define GHWCFG3		0x4C
#define GHWCFG4		0x50
#define GLPMCTL		0x54
#define HPTXFSIZ	0x100
#define DIEPTXF(x)	(0x100 + (4*(x)))

// Host Registers
#define HCFG		0x400
#define HFIR		0x404
#define HFNUM		0x408
#define HPTXSTS		0x410
#define HAINTSTS	0x414
#define HAINTMSK	0x418

#define HPRT		0x440

#define USB_HREGS	0x500
#define USB_HREGS_SIZE	(0x20*16)

// Devce Registers
#define DCFG		0x800
#define DCTL		0x804
#define DSTS		0x808
#define DIEPMSK		0x810
#define DOEPMSK		0x814
#define DAINTSTS	0x818
#define DAINTMSK	0x81C
#define DTKNQR1		0x820
#define DTKNQR2		0x824
#define DTKNQR3		0x830
#define DTKNQR4		0x834
#define USB_INREGS	0x900
#define USB_OUTREGS	0xB00
#define USB_EPREGS_SIZE 0x200

#define FIFO_SIZE		(0x1000/4)
#define USB_FIFO_START	0x1000
#define USB_FIFO_SIZE	(FIFO_SIZE*4*(USB_NUM_FIFOS+1))
#define USB_FIFO_END	(USB_FIFO_START+USB_FIFO_SIZE)

#define PCGCCTL     0xE00

#define PCGCCTL_ONOFF_MASK  3   // bits 0, 1
#define PCGCCTL_ON          0
#define PCGCCTL_OFF         1

#define GOTGCTL_BSESSIONVALID 	(1 << 19)
#define GOTGCTL_ASESSIONVALID 	(1 << 18)
#define GOTGCTL_CONIDSTS	  	(1 << 16)
#define GOTGCTL_SESSIONREQUEST	(1 << 1)

#define GAHBCFG_DMAEN (1 << 5)
#define GAHBCFG_BSTLEN_SINGLE (0 << 1)
#define GAHBCFG_BSTLEN_INCR (1 << 1)
#define GAHBCFG_BSTLEN_INCR4 (3 << 1)
#define GAHBCFG_BSTLEN_INCR8 (5 << 1)
#define GAHBCFG_BSTLEN_INCR16 (7 << 1)
#define GAHBCFG_MASKINT 0x1

#define GUSBCFG_TURNAROUND_MASK 0xF
#define GUSBCFG_TURNAROUND_SHIFT 10
#define GUSBCFG_HNPENABLE (1 << 9)
#define GUSBCFG_SRPENABLE (1 << 8)
#define GUSBCFG_PHYIF16BIT (1 << 3)
#define USB_UNKNOWNREG1_START 0x1708

#define GHWCFG2_TKNDEPTH_SHIFT	26
#define GHWCFG2_TKNDEPTH_MASK	0xF
#define GHWCFG2_NUM_ENDPOINTS_SHIFT	10
#define GHWCFG2_NUM_ENDPOINTS_MASK	0xf

#define GHWCFG4_DED_FIFO_EN			(1 << 25)

#define GRSTCTL_AHBIDLE			(1 << 31)
#define GRSTCTL_TXFFLUSH		(1 << 5)
#define GRSTCTL_TXFFNUM_SHIFT	6
#define GRSTCTL_TXFFNUM_MASK	0x1f
#define GRSTCTL_CORESOFTRESET	0x1
#define GRSTCTL_TKNFLUSH		3

#define GINTMSK_NONE        0x0
#define GINTMSK_CURMODE		(1 << 0)
#define GINTMSK_OTG         (1 << 2)
#define GINTMSK_SOF         (1 << 3)
#define GINTMSK_RXSTSQLVL	(1 << 4)
#define GINTMSK_GINNAKEFF   (1 << 6)
#define GINTMSK_GOUTNAKEFF  (1 << 7)
#define GINTMSK_SUSPEND     (1 << 11)
#define GINTMSK_RESET       (1 << 12)
#define GINTMSK_ENUMDONE    (1 << 13)
#define GINTMSK_EPMIS       (1 << 17)
#define GINTMSK_INEP        (1 << 18)
#define GINTMSK_OEP         (1 << 19)
#define GINTMSK_HPRT		(1 << 24)
#define GINTMSK_HCHAN		(1 << 25)
#define GINTMSK_CONIDSTSCHNG (1 << 28)
#define GINTMSK_DISCONNECT  (1 << 29)
#define GINTMSK_RESUME      (1 << 31)

#define GOTGINT_SESENDDET	(1 << 2)

#define GRXSTSR_CH_MASK		0xF
#define GRXSTSR_BCNT_SHIFT	4
#define GRXSTSR_BCNT_MASK	(0x7ff)
#define GRXSTSR_PID_SHIFT	15
#define GRXSTSR_PID_MASK	0x3
#define GRXSTSR_PKTSTS_SHIFT 17
#define GRXSTSR_PKTSTS_MASK	0xf

#define HPRT_PRTCONNSTS		(1 << 0)
#define HPRT_PRTCONNDET		(1 << 1)
#define HPRT_PRTENA			(1 << 2)
#define HPRT_PRTENCHNG		(1 << 3)
#define HPRT_PRTOVRCURRACT	(1 << 4)
#define HPRT_PRTOVRCURRCHNG	(1 << 5)
#define HPRT_PRTRES			(1 << 6)
#define HPRT_PRTSUSP		(1 << 7)
#define HPRT_PRTRST			(1 << 8)
#define HPRT_PRTLNSTS_SHIFT	10
#define HPRT_PRTLNSTS_MASK	0x3
#define HPRT_PRTPWR			(1 << 12)
#define HPRT_PRTTSTCTL_SHIFT 13
#define HPRT_PRTTSTCTL_MASK	0x0xf
#define HPRT_PRTSPD_SHIFT	17
#define HPRT_PRTSPD_MASK	0x3

#define HCCHAR_CHEN			(1 << 31)
#define HCCHAR_CHDIS		(1 << 30)
#define HCCHAR_ODDFRM		(1 << 29)
#define HCCHAR_DEV_SHIFT	22
#define HCCHAR_DEV_MASK		(0x7f)
#define HCCHAR_MC_SHIFT		20
#define HCCHAR_MC_MASK		0x3
#define HCCHAR_EPTYPE_SHIFT	18
#define HCCHAR_EPTYPE_MASK	0x3
#define HCCHAR_LOWSPD		(1 << 17)
#define HCCHAR_EP_DIR		(1 << 15)
#define HCCHAR_EP_SHIFT		11
#define HCCHAR_EP_MASK		0xf
#define HCCHAR_MPS_SHIFT	0
#define HCCHAR_MPS_MASK		(0x7ff)

#define HCINTMSK_XFERCOMPL	(1 << 0)
#define HCINTMSK_CHHLTD		(1 << 1)
#define HCINTMSK_AHBERR		(1 << 2)
#define HCINTMSK_STALL		(1 << 3)
#define HCINTMSK_NAK		(1 << 4)
#define HCINTMSK_ACK		(1 << 5)
#define HCINTMSK_NYET		(1 << 6)
#define HCINTMSK_XACTERR	(1 << 7)
#define HCINTMSK_BBLERR		(1 << 8)
#define HCINTMSK_FRMOVRUN	(1 << 9)
#define HCINTMSK_DATATGLERR	(1 << 10)

#define HCTSIZ_XFER_MASK	(0x7ffff)
#define HCTSIZ_XFER_SHIFT	0
#define HCTSIZ_PKTCNT_MASK	(0x3ff)
#define HCTSIZ_PKTCNT_SHIFT	19
#define HCTSIZ_PID_MASK		0x3
#define HCTSIZ_PID_SHIFT	29
#define HCTSIZ_DOPING		(1 << 31)

#define FIFO_DEPTH_SHIFT 16

#define GNPTXFSTS_GET_TXQSPCAVAIL(x) GET_BITS(x, 16, 8)

#define GHWCFG4_DED_FIFO_EN         (1 << 25)

#define DAINT_ALL                   0xFFFFFFFF
#define DAINT_NONE                  0
#define DAINT_OUT_SHIFT             16
#define DAINT_IN_SHIFT              0

#define DCTL_SFTDISCONNECT			0x2
#define DCTL_PROGRAMDONE			(1 << 11)
#define DCTL_CGOUTNAK				(1 << 10)
#define DCTL_SGOUTNAK				(1 << 9)
#define DCTL_CGNPINNAK				(1 << 8)
#define DCTL_SGNPINNAK				(1 << 7)

#define DSTS_GET_SPEED(x) GET_BITS(x, 1, 2)

#define DCFG_NZSTSOUTHSHK           (1 << 2)
#define DCFG_EPMSCNT                (1 << 18)
#define DCFG_HISPEED                0x0
#define DCFG_FULLSPEED              0x1
#define DCFG_DEVICEADDR_UNSHIFTED_MASK 0x7F
#define DCFG_DEVICEADDR_SHIFT 4
#define DCFG_DEVICEADDRMSK (DCFG_DEVICEADDR_UNSHIFTED_MASK << DCFG_DEVICEADDR_SHIFT)
#define DCFG_ACTIVE_EP_COUNT_MASK	0x1f
#define DCFG_ACTIVE_EP_COUNT_SHIFT	18

#define DOEPTSIZ0_SUPCNT_MASK 0x3
#define DOEPTSIZ0_SUPCNT_SHIFT 29
#define DOEPTSIZ0_PKTCNT_MASK 0x1
#define DEPTSIZ0_XFERSIZ_MASK 0x7F
#define DIEPTSIZ_MC_MASK 0x3
#define DIEPTSIZ_MC_SHIFT 29
#define DEPTSIZ_PKTCNT_MASK 0x3FF
#define DEPTSIZ_PKTCNT_SHIFT 19
#define DEPTSIZ_XFERSIZ_MASK 0x1FFFF

// ENDPOINT_DIRECTIONS register has two bits per endpoint. 0, 1 for endpoint 0. 1, 2 for end point 1, etc.
#define USB_EP_DIRECTION(ep) (USBDirection)(2-((GET_REG(USB + GHWCFG1) >> ((ep) * 2)) & 0x3))
#define USB_ENDPOINT_DIRECTIONS_BIDIR 0
#define USB_ENDPOINT_DIRECTIONS_IN 1
#define USB_ENDPOINT_DIRECTIONS_OUT 2

#define USB_START_DELAYUS 10000
#define USB_SFTDISCONNECT_DELAYUS 4000
#define USB_ONOFFSTART_DELAYUS 100
#define USB_RESETWAITFINISH_DELAYUS 1000
#define USB_SFTCONNECT_DELAYUS 250
#define USB_PROGRAMDONE_DELAYUS 10

#define USB_EPCON_ENABLE		(1 << 31)
#define USB_EPCON_DISABLE		(1 << 30)
#define USB_EPCON_SETD0PID		(1 << 28)
#define USB_EPCON_SETNAK		(1 << 27)
#define USB_EPCON_CLEARNAK		(1 << 26)
#define USB_EPCON_TXFNUM_MASK	0xf
#define USB_EPCON_TXFNUM_SHIFT	22
#define USB_EPCON_STALL			(1 << 21)
#define USB_EPCON_TYPE_MASK		0x3
#define USB_EPCON_TYPE_SHIFT	18
#define USB_EPCON_NAKSTS		(1 << 17)
#define USB_EPCON_ACTIVE		(1 << 15)
#define USB_EPCON_NEXTEP_MASK	0xF
#define USB_EPCON_NEXTEP_SHIFT	11
#define USB_EPCON_MPS_MASK		0x7FF

#define USB_EPINT_INEPNakEff 0x40
#define USB_EPINT_INTknEPMis 0x20
#define USB_EPINT_INTknTXFEmp 0x10
#define USB_EPINT_TimeOUT 0x8
#define USB_EPINT_AHBErr 0x4
#define USB_EPINT_EPDisbld 0x2
#define USB_EPINT_XferCompl 0x1

#define USB_EPINT_Back2BackSetup (1 << 6)
#define USB_EPINT_OUTTknEPDis 0x10
#define USB_EPINT_SetUp 0x8
#define USB_EPINT_EpDisbld 0x1
#define USB_EPINT_NONE 0
#define USB_EPINT_ALL 0xFFFFFFFF

#define USB_2_0 0x0200

#define USB_HIGHSPEED 0
#define USB_FULLSPEED 1
#define USB_LOWSPEED 2
#define USB_FULLSPEED_48_MHZ 3

#define USB_CONTROLEP 0

struct _synopsys_usb_state;

typedef struct _synopsys_usb_channel_state
{
	struct _synopsys_usb_state *parent;
	int index;

	uint32_t hcchar, hcsplt;
	uint32_t hcintsts, hcintmsk;
	uint32_t hctsiz, hcdma;

	int pid, real_pid;
	int fifo_start, fifo_end;
	int in, out;
	int total_done;

	USBPacket packet;
} synopsys_usb_channel_state;

typedef struct _synopsys_usb_ep_state
{
	struct _synopsys_usb_state *parent;
	int index;
	int in;

	uint32_t control;
	uint32_t tx_size;
	uint32_t fifo;
	uint32_t interrupt_status;

	target_phys_addr_t dma_address;
	target_phys_addr_t dma_buffer;

} synopsys_usb_ep_state;

typedef struct _synopsys_usb_state
{
	SysBusDevice busdev;
	USBBus usb;
	USBPort usb_port;
	USBDevice *usb_device;
	qemu_irq irq;
	QEMUTimer *sof_timer;
	MemoryRegion *iomem;

	char *server_host;
	uint32_t server_port;
	//tcp_usb_state_t tcp_state;

	uint32_t pcgcctl;

	uint32_t gsnpsid;
	uint32_t ghwcfg1;
	uint32_t ghwcfg2;
	uint32_t ghwcfg3;
	uint32_t ghwcfg4;

	uint32_t glpmctl;

	uint32_t gahbcfg;
	uint32_t gusbcfg;

	uint32_t grxfsiz;
	uint32_t gnptxfsiz;

	uint32_t gotgctl;
	uint32_t gotgint;
	uint32_t grstctl;
	uint32_t gintmsk;
	uint32_t gintsts;

	uint32_t grxstsr;
	
	uint32_t hptxfsiz;
	uint32_t dptxfsiz[USB_NUM_FIFOS];

	// Host Regs (0x400)

	uint32_t hcfg;
	uint32_t hfir;
	uint32_t hfnum;
	uint32_t hptxsts;
	uint32_t haintsts;
	uint32_t haintmsk;
	uint32_t hprt;

	synopsys_usb_channel_state channels[USB_NUM_CHANNELS];

	// Device regs (0x800)

	uint32_t dctl;
	uint32_t dcfg;
	uint32_t dsts;

	uint32_t daintmsk;
	uint32_t daintsts;
	uint32_t diepmsk;
	uint32_t doepmsk;

	synopsys_usb_ep_state in_eps[USB_NUM_ENDPOINTS];
	synopsys_usb_ep_state out_eps[USB_NUM_ENDPOINTS];

	uint32_t fifos[FIFO_SIZE * (USB_NUM_FIFOS+1)];
	int rxfifo_start, rxfifo_end;

} synopsys_usb_state;

static inline uint32_t mod32(uint32_t *_reg, uint32_t _clr, uint32_t _set)
{
	return (*_reg = ((*_reg) &~ _clr) | _set);
}

static inline uint32_t update32(uint32_t *_reg, uint32_t _clr, uint32_t _set, uint32_t _val)
{
	uint32_t set = _val & _set;
	uint32_t clr = _clr & _val;
	return mod32(_reg, clr | _set, set);
}

static inline size_t synopsys_usb_tx_fifo_start(synopsys_usb_state *_state, uint32_t _fifo)
{
	if(_fifo == 0)
		return _state->gnptxfsiz >> 16;
	else
		return _state->dptxfsiz[_fifo-1] >> 16;
}

static inline size_t synopsys_usb_tx_fifo_size(synopsys_usb_state *_state, uint32_t _fifo)
{
	if(_fifo == 0)
		return _state->gnptxfsiz & 0xFFFF;
	else
		return _state->dptxfsiz[_fifo-1] & 0xFFFF;
}

//
// Simulation
//

static void synopsys_usb_update_irq(synopsys_usb_state *_state)
{
	_state->daintsts = 0;
	_state->gintsts &=~ (
		GINTMSK_OEP | GINTMSK_INEP | GINTMSK_OTG
		| GINTMSK_HCHAN | GINTMSK_HPRT);

	if(_state->gotgint)
		_state->gintsts |= GINTMSK_OTG;
	
	if(_state->hprt & HPRT_PRTCONNDET)
		_state->gintsts |= GINTMSK_HPRT;

	int i;

	if(_state->gintsts & GINTMSK_CURMODE)
	{
		_state->haintsts = 0;

		for(i = 0; i < USB_NUM_CHANNELS; i++)
		{
			synopsys_usb_channel_state *ch = &_state->channels[i];
			if(ch->hcintsts & ch->hcintmsk)
				_state->haintsts |= (1 << i);
		}

		if(_state->haintsts & _state->haintmsk)
			_state->gintsts |= GINTMSK_HCHAN;
	}
	else
	{
		_state->daintsts = 0;

		for(i = 0; i < USB_NUM_ENDPOINTS; i++)
		{
			if(_state->out_eps[i].interrupt_status & _state->doepmsk)
				_state->daintsts |= 1 << (i+DAINT_OUT_SHIFT);

			if(_state->in_eps[i].interrupt_status & _state->diepmsk)
				_state->daintsts |= 1 << (i+DAINT_IN_SHIFT);
		}

		if(_state->daintsts & _state->daintmsk & 0xFFFF)
			_state->gintsts |= GINTMSK_INEP;
		if(_state->daintsts & _state->daintmsk & 0xFFFF0000)
			_state->gintsts |= GINTMSK_OEP;
	}

	// TODO: proper power interface
	if(/*(_state->pcgcctl & 3) == 0 &&*/ _state->gintmsk & _state->gintsts)
	{
		//printf("USB: IRQ triggered 0x%08x & 0x%08x.\n",
		//		_state->gintsts, _state->gintmsk);
		qemu_irq_raise(_state->irq);
	}
	else
		qemu_irq_lower(_state->irq);
}

static void synopsys_usb_update_channel(synopsys_usb_state *_state,
		int _ch);
static void synopsys_usb_channel_complete(synopsys_usb_state *_state,
		int _ch, int _ret)
{
	synopsys_usb_channel_state *ch = &_state->channels[_ch];

#ifdef DEBUG_USB
	printf("USB %d xfer done: %d.\n",
			_ch, _ret);
#endif

	if(_ret >= 0)
	{
		uint32_t xfer = ch->hctsiz & HCTSIZ_XFER_MASK;
		uint32_t xferw = (_ret+3)/4;
		uint32_t left = xfer;
		
		if(_ret >= 0)
		{
#ifdef DEBUG_USB
			if(ch->pid == USB_TOKEN_SETUP)
				printf("USB: setup done.\n");
#endif

			if(ch->out)
				_ret = xfer;
			else
				ch->total_done += _ret;

			left -= _ret;			
			xferw = (_ret+3)/4;
		}
		else
			xferw = 0;

		mod32(&ch->hctsiz,
				HCTSIZ_XFER_MASK,
				left);

		if(!ch->in)
		{
			ch->fifo_start += ((xfer+3)/4);
			if(ch->fifo_end > FIFO_SIZE)
				ch->fifo_end -= FIFO_SIZE;
		}
		else
		{
			int fstart = ch->fifo_start;

#ifdef DEBUG_USB
			printf("USB: IN packet = ");
#endif

			int q;
			for(q = 0; q < xferw; q++)
			{
				if(_state->rxfifo_end+1 == _state->rxfifo_start)
				{
					printf("USB: rxfifo overrun.\n");
					break;
				}

				uint32_t val = _state->fifos[_ch*FIFO_SIZE + fstart];
				_state->fifos[FIFO_SIZE*16 + _state->rxfifo_end] = val;

				_state->rxfifo_end++;
				if(_state->rxfifo_end > FIFO_SIZE)
					_state->rxfifo_end -= FIFO_SIZE;

				fstart++;
				if(fstart > FIFO_SIZE)
					fstart++;

#ifdef DEBUG_USB
				printf("0x%08x ", val);
#endif
			}

#ifdef DEBUG_USB
			printf("\n");
#endif

			if(!ch->total_done && (ch->hcchar & HCCHAR_CHEN))
			{
				synopsys_usb_update_channel(_state, _ch);
				return;
			}

			_state->grxstsr = (ch->index & GRXSTSR_CH_MASK)
				| ((ch->total_done & GRXSTSR_BCNT_MASK) << GRXSTSR_BCNT_SHIFT)
				| ((ch->real_pid & GRXSTSR_PID_MASK) << GRXSTSR_PID_SHIFT)
				| ((0x2 & GRXSTSR_PKTSTS_MASK) << GRXSTSR_PKTSTS_SHIFT);
			ch->total_done = 0;
			_state->gintsts |= GINTMSK_RXSTSQLVL;

			if(ch->hcchar & HCCHAR_CHEN)
			{
				// We don't want to set xfercompl just yet.
				synopsys_usb_update_irq(_state);
				return;
			}
				
		}

		ch->hcintsts |= HCINTMSK_XFERCOMPL;
	}
	else
	{
	
		switch(_ret)
		{
		case USB_RET_NAK:
			ch->hcintsts |= HCINTMSK_NAK;
			break;

		case USB_RET_BABBLE:
			ch->hcintsts |= HCINTMSK_BBLERR;
			break;

		default: // STALL
			ch->hcintsts |= HCINTMSK_STALL;
			break;
		}

		if(ch->in)
		{
			_state->grxstsr = (ch->index & GRXSTSR_CH_MASK)
				| ((ch->real_pid & GRXSTSR_PID_MASK) << GRXSTSR_PID_SHIFT)
				| ((0x7 & GRXSTSR_PKTSTS_MASK) << GRXSTSR_PKTSTS_SHIFT);

			_state->gintsts |= GINTMSK_RXSTSQLVL;
		}

	}	

	ch->hcintsts |= HCINTMSK_CHHLTD;
	synopsys_usb_update_irq(_state);
}

static void synopsys_usb_update_sof(synopsys_usb_state *_state)
{
	qemu_mod_timer_ns(_state->sof_timer,
			qemu_get_clock_ns(vm_clock) + _state->hfir*1000);
}

static void synopsys_usb_complete_xfer(synopsys_usb_state *_state,
		int _ch)
{
	synopsys_usb_channel_state *ch = &_state->channels[_ch];
	void *fifo = &_state->fifos[FIFO_SIZE*_ch + ch->fifo_start];
	uint32_t tsiz = ch->hctsiz;
	uint32_t chr = ch->hcchar;
	uint32_t xferlen = (tsiz >> HCTSIZ_XFER_SHIFT) & HCTSIZ_XFER_MASK;
	uint32_t pktcnt = (tsiz >> HCTSIZ_PKTCNT_SHIFT) & HCTSIZ_PKTCNT_MASK;
	uint32_t mps = (chr >> HCCHAR_MPS_SHIFT) & HCCHAR_MPS_MASK;
	uint32_t ep = (chr >> HCCHAR_EP_SHIFT) & HCCHAR_EP_MASK;
	uint32_t addr = (chr >> HCCHAR_DEV_SHIFT) & HCCHAR_DEV_MASK;
	USBDevice *dev = usb_find_device(&_state->usb_port, addr);

	if(xferlen > mps)
		xferlen = mps;

#ifdef DEBUG_USB
	if(ch->out)
	{
		int q;
		printf("USB: OUT packet: ");

		for(q = 0; q < (xferlen+3)/4; q++)
		{
			printf("0x%08x ", _state->fifos[
						(FIFO_SIZE*_ch + ch->fifo_start + q)%FIFO_SIZE]);
		}
		printf("\n");
	}
#endif

	if(pktcnt > 0)
		pktcnt--;

	mod32(&ch->hctsiz,
			HCTSIZ_PKTCNT_MASK << HCTSIZ_PKTCNT_SHIFT,
			pktcnt << HCTSIZ_PKTCNT_SHIFT);

	if(pktcnt == 0)
		ch->hcchar &=~ HCCHAR_CHEN;

#ifdef DEBUG_USB
	printf("USB-%d (%d) %p xfer: %d pid=%d, pktcnt=%d, xferlen=%d.\n",
			_ch, addr, dev, ep, ch->pid, pktcnt, xferlen);
#endif

	USBEndpoint *epp = usb_ep_get(dev, ch->pid, ep);
	if(!epp)
	{
		synopsys_usb_channel_complete(_state, _ch, USB_RET_STALL);
		return;
	}
			
	usb_packet_setup(&ch->packet, ch->pid, epp);

	uint32_t xferwords = (xferlen+3)/4;
	if(ch->fifo_start + xferwords > FIFO_SIZE)
	{
		// split
		uint32_t start = FIFO_SIZE-ch->fifo_start;
		usb_packet_addbuf(&ch->packet, fifo, start*4);
		usb_packet_addbuf(&ch->packet,
				&_state->fifos[_ch*FIFO_SIZE], xferwords-start);
	}
	else
		usb_packet_addbuf(&ch->packet, fifo, xferlen);

	int ret = usb_handle_packet(dev, &ch->packet);

	if(ret == USB_RET_ASYNC)
		return;

	synopsys_usb_channel_complete(_state, _ch, ret);
}

static void synopsys_usb_write_fifo(synopsys_usb_state *_state,
		unsigned _addr, uint32_t _val)
{
	int chan = _addr / FIFO_SIZE;
	synopsys_usb_channel_state *ch = &_state->channels[chan];

#ifdef DEBUG_USB
	printf("%s: %d, 0x%08x = 0x%08x, %d.\n",
			__func__, chan,
			_addr, _val, ch->hcchar & HCCHAR_CHEN);
#endif

	int fifo_space = ch->fifo_start - ch->fifo_end - 1;
	if(fifo_space <= 0)
		fifo_space += FIFO_SIZE;
		
	uint32_t xfer = ((ch->hctsiz & HCTSIZ_XFER_MASK) + 3)/4;

	if(fifo_space-1  < 0)
		fprintf(stderr, "USB: FIFO overflow.\n");
	else
	{
		uint32_t start = chan*FIFO_SIZE;
		_state->fifos[start + ch->fifo_end] = (uint32_t)_val;

		ch->fifo_end++;
		if(ch->fifo_end >= FIFO_SIZE)
			ch->fifo_end -= FIFO_SIZE;
			
		if((ch->hcchar & HCCHAR_CHEN)
				&& FIFO_SIZE-fifo_space-1+1 >= xfer)
			synopsys_usb_complete_xfer(_state, chan);
	}
}

static uint32_t synopsys_usb_read_fifo(synopsys_usb_state *_state,
		unsigned _addr)
{
	uint32_t ret;

	if(_state->rxfifo_start == _state->rxfifo_end)
		ret = 0;
	else
	{
		ret = _state->fifos[FIFO_SIZE*16
				+ _state->rxfifo_start];
		_state->rxfifo_start++;
		if(_state->rxfifo_start > FIFO_SIZE)
			_state->rxfifo_start -= FIFO_SIZE;
	}

#ifdef DEBUG_USB
	printf("%s: 0x%08x.\n", __func__, ret);
#endif

	return ret;
}

static void synopsys_usb_sof(void *_opaque)
{
	synopsys_usb_state *state = _opaque;

	if(state->gintsts & GINTMSK_CURMODE)
		state->hfnum++;
	
	state->gintsts |= GINTMSK_SOF;

	synopsys_usb_update_sof(state);
	synopsys_usb_update_irq(state);
}

static void synopsys_usb_update_channel(synopsys_usb_state *_state,
		int _ch)
{
	synopsys_usb_channel_state *ch = &_state->channels[_ch];

	if(ch->hcchar & HCCHAR_CHDIS)
	{
		ch->hcchar &=~ (HCCHAR_CHDIS | HCCHAR_CHEN);
		
		// TODO: find the bug!
		_state->rxfifo_start = _state->rxfifo_end = 0;
	}
	else if(ch->hcchar & HCCHAR_CHEN)
	{
		uint32_t pid = (ch->hctsiz >> HCTSIZ_PID_SHIFT) & HCTSIZ_PID_MASK;

		ch->in = 0;
		ch->out = 0;
		ch->real_pid = pid;

		if(pid == 3)
		{
			ch->out = 1;
			ch->pid = USB_TOKEN_SETUP;
		}
		else if(ch->hcchar & HCCHAR_EP_DIR) // IN
		{
			ch->in = 1;
			ch->pid = USB_TOKEN_IN;
		}
		else
		{
			ch->out = 1;
			ch->pid = USB_TOKEN_OUT;
		}

#ifdef DEBUG_USB
		printf("USB: %d beginning transfer.\n", _ch);
#endif
		if(!ch->out || !(ch->hctsiz & HCTSIZ_XFER_MASK))
			synopsys_usb_complete_xfer(_state, _ch);
		else if(ch->hcdma)
		{
			uint32_t xferlen = (ch->hctsiz >> HCTSIZ_XFER_SHIFT) & HCTSIZ_XFER_MASK;

			int i;
			for(i = 0; i < (xferlen+3)/4; i++)
			{
				synopsys_usb_write_fifo(_state, _ch*FIFO_SIZE, ldl_phys(ch->hcdma));
				ch->hcdma += 4;
			}
		}
	}
}

static void synopsys_usb_update_ep(synopsys_usb_state *_state,
		synopsys_usb_ep_state *_ep)
{
	if(_ep->control & USB_EPCON_SETNAK)
	{
		_ep->control |= USB_EPCON_NAKSTS;
		_ep->interrupt_status |= USB_EPINT_INEPNakEff;
		_ep->control &=~ USB_EPCON_SETNAK;
	}

	if(_ep->control & USB_EPCON_DISABLE)
	{
		_ep->interrupt_status |= USB_EPINT_EPDisbld;
		_ep->control &=~ (USB_EPCON_DISABLE | USB_EPCON_ENABLE);
	}
}

static void synopsys_usb_update_in_ep(synopsys_usb_state *_state, uint8_t _ep)
{
	synopsys_usb_ep_state *eps = &_state->in_eps[_ep];
	synopsys_usb_update_ep(_state, eps);

	if(eps->control & USB_EPCON_ENABLE)
		printf("USB: IN transfer queued on %d.\n", _ep);
}

static void synopsys_usb_update_out_ep(synopsys_usb_state *_state, uint8_t _ep)
{
	synopsys_usb_ep_state *eps = &_state->out_eps[_ep];
	synopsys_usb_update_ep(_state, eps);

	if(eps->control & USB_EPCON_ENABLE)
		printf("USB: OUT transfer queued on %d.\n", _ep);
}

static void synopsys_usb_setup_port(synopsys_usb_state *_state, int _mode, int _in)
{
	if(!_in)
	{
		mod32(&_state->gotgctl,
				GOTGCTL_ASESSIONVALID
				| GOTGCTL_BSESSIONVALID, 0);
		mod32(&_state->gintsts, 0, GINTMSK_DISCONNECT);
	}
	else
	{

		if(_mode)
		{
			// Host
			if(!(_state->gintsts & GINTMSK_CURMODE))
				_state->gintsts |= (GINTMSK_CONIDSTSCHNG | GINTMSK_CURMODE);

			mod32(&_state->gotgctl,
					GOTGCTL_CONIDSTS | GOTGCTL_BSESSIONVALID,
					GOTGCTL_ASESSIONVALID);
			_state->hprt |= HPRT_PRTCONNDET | HPRT_PRTCONNSTS;
		}
		else
		{
			// Device
			if(_state->gintsts & GINTMSK_CURMODE)
				mod32(&_state->gintsts,
						GINTMSK_CURMODE, GINTMSK_CONIDSTSCHNG);

			mod32(&_state->gotgctl,
					GOTGCTL_ASESSIONVALID,
					GOTGCTL_BSESSIONVALID | GOTGCTL_CONIDSTS);
		}
	}

	synopsys_usb_update_irq(_state);
}

static void synopsys_usb_attach(USBPort *_port)
{
	synopsys_usb_state *state = container_of(_port,
			synopsys_usb_state, usb_port);

#ifdef DEBUG_USB
	printf("%s, %p.\n", __func__, _port->dev);
#endif

	state->usb_device = _port->dev;
	synopsys_usb_setup_port(state, 1, 1);
}

static void synopsys_usb_detach(USBPort *_port)
{
	synopsys_usb_state *state = container_of(_port,
			synopsys_usb_state, usb_port);

#ifdef DEBUG_USB
	printf("%s.\n", __func__);
#endif

	state->usb_device = NULL;
	synopsys_usb_setup_port(state, 1, 0);
}

static void synopsys_usb_child_detach(USBPort *_port,
		USBDevice *_child)
{
#ifdef DEBUG_USB
	printf("%s.\n", __func__);
#endif
}

static void synopsys_usb_wakeup_port(USBPort *_port)
{
#ifdef DEBUG_USB
	printf("%s.\n", __func__);
#endif
}

static void synopsys_usb_complete(USBPort *_port,
		USBPacket *_p)
{
	synopsys_usb_channel_state *ch = container_of(_p,
			synopsys_usb_channel_state, packet);

#ifdef DEBUG_USB
	printf("%s.\n", __func__);
#endif
	
	synopsys_usb_channel_complete(ch->parent,
			ch->index, _p->result);
}

static struct USBPortOps port_ops = {
	.attach = synopsys_usb_attach,
	.detach = synopsys_usb_detach,
	.child_detach = synopsys_usb_child_detach,
	
	.wakeup = synopsys_usb_wakeup_port,
	
	.complete = synopsys_usb_complete,
};

static int synopsys_usb_register_companion(USBBus *_bus,
		USBPort *_ports[], uint32_t _portcount, uint32_t _fp)
{
#ifdef DEBUG_USB
	printf("%s.\n", __func__);
#endif
	return 0;
}

static void synopsys_usb_wakeup_endpoint(USBBus *_bus,
		USBEndpoint *_ep)
{
#ifdef DEBUG_USB
	printf("%s.\n", __func__);
#endif
}

static struct USBBusOps bus_ops = {
	.register_companion = synopsys_usb_register_companion,
	.wakeup_endpoint = synopsys_usb_wakeup_endpoint,
};

//
// IO
//

/*static int synopsys_usb_tcp_callback(tcp_usb_state_t *_state,
 			void *_arg, tcp_usb_header_t *_hdr, char *_buffer)
{
	synopsys_usb_state *state = _arg;

	_hdr->addr = (state->dcfg & DCFG_DEVICEADDRMSK) >> DCFG_DEVICEADDR_SHIFT;

	if(_hdr->flags & tcp_usb_reset)
	{
		state->gintsts |= GINTMSK_RESET;
		synopsys_usb_update_irq(state);
		return 0;
	}

	if(_hdr->flags & tcp_usb_enumdone)
	{
		state->gintsts |= GINTMSK_ENUMDONE;
	}

	int ret;
	uint8_t ep = _hdr->ep & 0x7f;
	if(_hdr->ep & USB_DIR_IN)
	{
		synopsys_usb_ep_state *eps = &state->in_eps[ep];

		if(eps->control & USB_EPCON_STALL)
		{
			eps->control &=~ USB_EPCON_STALL; // Should this be EP0 only
			printf("USB: Stall.\n");
			ret = USB_RET_STALL;
		}
		else if(eps->control & USB_EPCON_ENABLE)
		{
			// Do IN transfer!
			eps->control &=~ USB_EPCON_ENABLE;

			size_t sz = eps->tx_size & DEPTSIZ_XFERSIZ_MASK;
			size_t amtDone = sz;
			if(amtDone > _hdr->length)
				amtDone = _hdr->length;

			if(eps->fifo >= USB_NUM_FIFOS)
				hw_error("usb_synopsys: USB transfer on non-existant FIFO %d!\n", eps->fifo);

			size_t txfz = synopsys_usb_tx_fifo_size(state, eps->fifo);
			if(amtDone > txfz)
				amtDone = txfz;

			size_t txfs = synopsys_usb_tx_fifo_start(state, eps->fifo);
			if(txfs + txfz > sizeof(state->fifos))
				hw_error("usb_synopsys: USB transfer would overflow FIFO buffer!\n");

			printf("USB: Starting IN transfer on EP %d (%d)...\n", ep, amtDone);

			if(amtDone > 0)
			{
				if(eps->dma_address)
				{
					cpu_physical_memory_read(eps->dma_address, &state->fifos[txfs], amtDone);
					eps->dma_address += amtDone;
				}

				memcpy(_buffer, (char*)&state->fifos[txfs], amtDone);
			}

			printf("USB: IN transfer complete!\n");

			eps->tx_size = (eps->tx_size &~ DEPTSIZ_XFERSIZ_MASK)
							| ((sz-amtDone) & DEPTSIZ_XFERSIZ_MASK);
			eps->interrupt_status |= USB_EPINT_XferCompl;

			ret = amtDone;
		}
		else
			ret = USB_RET_NAK;
	}
	else // OUT
	{	
		synopsys_usb_ep_state *eps = &state->out_eps[ep];
		
		if(eps->control & USB_EPCON_STALL)
		{
			eps->control &=~ USB_EPCON_STALL; // Should this be EP0 only
			printf("USB: Stall.\n");
			ret = USB_RET_STALL;
		}
		else if(eps->control & USB_EPCON_ENABLE)
		{
			// Do OUT transfer!
			eps->control &=~ USB_EPCON_ENABLE;

			size_t sz = eps->tx_size & DEPTSIZ_XFERSIZ_MASK;
			size_t amtDone = sz;
			if(amtDone > _hdr->length)
				amtDone = _hdr->length;

			size_t rxfz = state->grxfsiz;
			if(amtDone > rxfz)
				amtDone = rxfz;

			if(rxfz > sizeof(state->fifos))
				hw_error("usb_synopsys: USB transfer would overflow FIFO buffer!\n");

			printf("USB: Starting OUT transfer on EP %d (%d)...\n", ep, amtDone);

			if(amtDone > 0)
			{
				memcpy((char*)state->fifos, _buffer, amtDone);

				if(eps->dma_address)
				{
					printf("USB: DMA copying to 0x%08x.\n", eps->dma_address);
					cpu_physical_memory_write(eps->dma_address, state->fifos, amtDone);
					eps->dma_address += amtDone;
				}
			}

			printf("USB: OUT transfer complete!\n");

			if(_hdr->flags & tcp_usb_setup)
			{
				printf("USB: Setup %02x %02x %02x %02x %02x %02x %02x %02x\n",
						state->fifos[0],
						state->fifos[1],
						state->fifos[2],
						state->fifos[3],
						state->fifos[4],
						state->fifos[5],
						state->fifos[6],
						state->fifos[7]);

				eps->interrupt_status |= USB_EPINT_SetUp;
			}
			else
				eps->interrupt_status |= USB_EPINT_XferCompl;

			eps->tx_size = (eps->tx_size &~ DEPTSIZ_XFERSIZ_MASK)
							| ((sz-amtDone) & DEPTSIZ_XFERSIZ_MASK);

			ret = amtDone;
		}
		else
			ret = USB_RET_NAK;
	}

	synopsys_usb_update_irq(state);
	return ret;
	}*/

static uint32_t synopsys_usb_channel_read(synopsys_usb_state *_state,
		uint8_t _ch, target_phys_addr_t _addr)
{
	synopsys_usb_channel_state *ch = &_state->channels[_ch];

	switch(_addr)
	{
	case 0x00: // HCCHAR
		return ch->hcchar;

	case 0x04: // HCSPLT
		return ch->hcsplt;

	case 0x08: // HCINT
		return ch->hcintsts;

	case 0x0c: // HCINTMSK
		return ch->hcintmsk;

	case 0x10: // HCTSIZ
		return ch->hctsiz;

	case 0x14: // HCDMA
		return ch->hcdma;

	default:
		hw_error(DEVICE_NAME ": bad channel read at 0x%08x.\n",
				_addr);
		return 0;
	}
}

static uint32_t synopsys_usb_in_ep_read(synopsys_usb_state *_state,
		uint8_t _ep, target_phys_addr_t _addr)
{
	if(_ep >= USB_NUM_ENDPOINTS)
	{
		hw_error("usb_synopsys: Tried to read from disabled EP %d.\n", _ep);
		return 0;
	}

    switch (_addr)
	{
    case 0x00:
        return _state->in_eps[_ep].control;

    case 0x08:
        return _state->in_eps[_ep].interrupt_status;

    case 0x10:
        return _state->in_eps[_ep].tx_size;

    case 0x14:
        return _state->in_eps[_ep].dma_address;

    case 0x1C:
        return _state->in_eps[_ep].dma_buffer;

    default:
        hw_error("usb_synopsys: bad ep read offset 0x" TARGET_FMT_plx "\n", _addr);
		break;
    }

	return 0;
}

static uint32_t synopsys_usb_out_ep_read(synopsys_usb_state *_state, int _ep, target_phys_addr_t _addr)
{
	if(_ep >= USB_NUM_ENDPOINTS)
	{
		hw_error("usb_synopsys: Tried to read from disabled EP %d.\n", _ep);
		return 0;
	}

    switch (_addr)
	{
    case 0x00:
        return _state->out_eps[_ep].control;

    case 0x08:
        return _state->out_eps[_ep].interrupt_status;

    case 0x10:
        return _state->out_eps[_ep].tx_size;

    case 0x14:
        return _state->out_eps[_ep].dma_address;

    case 0x1C:
        return _state->out_eps[_ep].dma_buffer;

    default:
        hw_error("usb_synopsys: bad ep read offset 0x" TARGET_FMT_plx "\n", _addr);
		break;
    }

	return 0;
}

static uint64_t synopsys_usb_read(void *_arg,
		target_phys_addr_t _addr, unsigned _sz)
{
	synopsys_usb_state *state = _arg;
	
	//printf("USB: Read 0x%08x.\n", _addr);

	switch(_addr)
	{
	case PCGCCTL:
		return state->pcgcctl;

	case GOTGCTL:
		return state->gotgctl;

	case GOTGINT:
		return state->gotgint;

	case GRSTCTL:
		return state->grstctl;

	case GSNPSID:
		return state->gsnpsid;

	case GHWCFG1:
		return state->ghwcfg1;

	case GHWCFG2:
		return state->ghwcfg2;

	case GHWCFG3:
		return state->ghwcfg3;

	case GHWCFG4:
		return state->ghwcfg4;

	case GLPMCTL:
		return state->glpmctl;

	case GAHBCFG:
		return state->gahbcfg;

	case GUSBCFG:
		return state->gusbcfg;

	case GINTMSK:
		return state->gintmsk;

	case GINTSTS:
		return state->gintsts;

		// Host Regs

	case HCFG:
		return state->hcfg;

	case HFIR:
		return state->hfir;

	case HFNUM:
		return state->hfnum;

	case HPTXSTS:
		return state->hptxsts;
		
	case HAINTSTS:
		return state->haintsts;

	case HAINTMSK:
		return state->haintmsk;

	case HPRT:
		return state->hprt;

	case USB_HREGS...(USB_HREGS + USB_HREGS_SIZE - 4):
		_addr -= USB_HREGS;
		return synopsys_usb_channel_read(state, _addr >> 5, _addr & 0x1f);

		// Device Regs

	case DIEPMSK:
		return state->diepmsk;

	case DOEPMSK:
		return state->doepmsk;

	case DAINTMSK:
		return state->daintmsk;
	
	case DAINTSTS:
		return state->daintsts;

	case DCTL:
		return state->dctl;

	case DCFG:
		return state->dcfg;

	case DSTS:
		return state->dsts;

	case GRXSTSP:
		state->gintsts &=~ GINTMSK_RXSTSQLVL;
		synopsys_usb_update_irq(state);

		{
			uint32_t ret = state->grxstsr;
			state->grxstsr = 0;
			
			if(state->gintsts & GINTMSK_CURMODE)
			{
				// If we're in host, try and resume another channel.
				int i;
				for(i = 0; i < USB_NUM_CHANNELS; i++)
				{
					synopsys_usb_channel_state *ch = 
						&state->channels[i];
					if(ch->hcchar & HCCHAR_CHEN)
					{
						synopsys_usb_update_channel(state, i);
						break;
					}
				}
			}

			return ret;
		}
		break;
		
	case GRXSTSR:
		return state->grxstsr;

	case GNPTXFSTS:
		return 0xFFFFFFF;

	case GRXFSIZ:
		return state->grxfsiz;

	case GNPTXFSIZ:
		return state->gnptxfsiz;

	case HPTXFSIZ:
		return state->hptxfsiz;

	case DIEPTXF(1) ... DIEPTXF(USB_NUM_FIFOS+1):
		_addr -= DIEPTXF(1);
		_addr >>= 2;
		return state->dptxfsiz[_addr];

	case USB_INREGS ... (USB_INREGS + USB_EPREGS_SIZE - 4):
		_addr -= USB_INREGS;
		return synopsys_usb_in_ep_read(state, _addr >> 5, _addr & 0x1f);

	case USB_OUTREGS ... (USB_OUTREGS + USB_EPREGS_SIZE - 4):
		_addr -= USB_OUTREGS;
		return synopsys_usb_out_ep_read(state, _addr >> 5, _addr & 0x1f);

	case USB_FIFO_START ... USB_FIFO_END-4:
		_addr -= USB_FIFO_START;
		
		if(state->gintsts & GINTMSK_CURMODE)
			return synopsys_usb_read_fifo(state, _addr/4);
		else
			return *((uint32_t*)(&state->fifos[_addr]));

	default:
		fprintf(stderr, "synopsys_usb: Unhandled read address 0x%08x!\n", _addr);
		//hw_error("USB: Unhandled read address 0x%08x!\n", _addr);
	}

	return 0;
}

static void synopsys_usb_channel_write(synopsys_usb_state *_state,
		int _ch, target_phys_addr_t _addr, uint32_t _val)
{
	synopsys_usb_channel_state *ch = &_state->channels[_ch];

	//printf("%s-%d: 0x%08x = 0x%08x.\n", __func__, _ch, _addr, _val);

	switch(_addr)
	{
	case 0x00: // HCCHAR
		ch->hcchar = _val;
		synopsys_usb_update_channel(_state, _ch);
		break;

	case 0x04: // HCSPLT
		ch->hcsplt = _val;
		break;

	case 0x08: // HCINT
		ch->hcintsts &=~ _val;
		synopsys_usb_update_irq(_state);
		break;

	case 0x0c: // HCINTMSK
		ch->hcintmsk = _val;
		break;

	case 0x10: // HCTSIZ
		ch->hctsiz = _val;
		break;

	case 0x14: // HCDMA
		ch->hcdma = _val;
		break;

	default:
		hw_error(DEVICE_NAME ": bad channel read at 0x%08x.\n",
				_addr);
	}
}

static void synopsys_usb_in_ep_write(synopsys_usb_state *_state,
		int _ep, target_phys_addr_t _addr, uint32_t _val)
{
	if(_ep >= USB_NUM_ENDPOINTS)
	{
		fprintf(stderr, "usb_synopsys: Wrote to disabled EP %d.\n", _ep);
		return;
	}

    switch (_addr)
	{
    case 0x00:
		_state->in_eps[_ep].control = _val;
		synopsys_usb_update_in_ep(_state, _ep);
		return;

    case 0x08:
        _state->in_eps[_ep].interrupt_status &=~ _val;
		synopsys_usb_update_irq(_state);
		return;

    case 0x10:
        _state->in_eps[_ep].tx_size = _val;
		return;

    case 0x14:
        _state->in_eps[_ep].dma_address = _val;
		return;

    case 0x1C:
        _state->in_eps[_ep].dma_buffer = _val;
		return;

    default:
        fprintf(stderr, "usb_synopsys: bad ep write offset 0x" TARGET_FMT_plx "\n", _addr);
		break;
    }
}

static void synopsys_usb_out_ep_write(synopsys_usb_state *_state, int _ep, target_phys_addr_t _addr, uint32_t _val)
{
	if(_ep >= USB_NUM_ENDPOINTS)
	{
		fprintf(stderr, "usb_synopsys: Wrote to disabled EP %d.\n", _ep);
		return;
	}

    switch (_addr)
	{
	case 0x00:
        _state->out_eps[_ep].control = _val;
		synopsys_usb_update_out_ep(_state, _ep);
		return;

    case 0x08:
        _state->out_eps[_ep].interrupt_status &=~ _val;
		synopsys_usb_update_irq(_state);
		return;

    case 0x10:
        _state->out_eps[_ep].tx_size = _val;
		return;

    case 0x14:
        _state->out_eps[_ep].dma_address = _val;
		return;

    case 0x1C:
        _state->out_eps[_ep].dma_buffer = _val;
		return;

    default:
        hw_error("usb_synopsys: bad ep write offset 0x" TARGET_FMT_plx "\n", _addr);
		break;
    }
}

static void synopsys_usb_write(void *_arg,
		target_phys_addr_t _addr, uint64_t _val, unsigned _sz)
{
	synopsys_usb_state *state = _arg;
	
	//printf("USB: Write 0x%08x to 0x%08x.\n", _val, _addr);

	switch(_addr)
	{
	case PCGCCTL:
		return;

	case GOTGCTL:
		state->gotgctl = _val;
		break;

	case GOTGINT:
		state->gotgint &=~ _val;
		synopsys_usb_update_irq(state);
		return;

	case GRSTCTL:
		if(_val & GRSTCTL_CORESOFTRESET)
		{
			state->grstctl = GRSTCTL_CORESOFTRESET;

			// Do reset stuff
			/*if(state->server_host)
			{
				tcp_usb_cleanup(&state->tcp_state);
				tcp_usb_init(&state->tcp_state, synopsys_usb_tcp_callback, NULL, state);

				printf("Connecting to USB server at %s:%d...\n",
						state->server_host, state->server_port);

				int ret = tcp_usb_connect(&state->tcp_state, state->server_host, state->server_port);
				if(ret < 0)
					hw_error("Failed to connect to USB server (%d).\n", ret);

				printf("Connected to USB server.\n");
				}*/

			state->grstctl &= ~GRSTCTL_CORESOFTRESET;
			state->grstctl |= GRSTCTL_AHBIDLE;
			state->gintsts |= GINTMSK_RESET;
			state->hprt |= HPRT_PRTENA;
			synopsys_usb_update_irq(state);

			if(state->usb_device)
				synopsys_usb_setup_port(state, 1, 1);
		}
		else if(_val == 0)
			state->grstctl = _val;

		return;

	case GINTMSK:
		state->gintmsk = _val;
		synopsys_usb_update_irq(state);
		break;

	case GINTSTS:
		state->gintsts &=~ (_val & 0xf072fc0a); // WC bits
		synopsys_usb_update_irq(state);
		return;

		// Host Regs

	case HCFG:
		state->hcfg = _val;
		return;

	case HFIR:
		state->hfir = _val;
		return;

	case HFNUM:
		state->hfnum = _val;
		return;

	case HPTXSTS:
		state->hptxsts = _val;
		return;
		
	case HAINTSTS:
		state->haintsts &=~ _val;
		synopsys_usb_update_irq(state);
		return;

	case HAINTMSK:
		state->haintmsk = _val;
		synopsys_usb_update_irq(state);
		return;

	case HPRT:
		update32(&state->hprt,
				0x102e, 0xf1c0,
				_val);

		if((_val & HPRT_PRTRST) && state->usb_device)
		{
			usb_port_reset(&state->usb_port);
			mod32(&state->hprt,
					HPRT_PRTCONNDET | (HPRT_PRTSPD_MASK << HPRT_PRTSPD_SHIFT),
					HPRT_PRTENA | (state->usb_device->speed << HPRT_PRTSPD_SHIFT));

			state->gintsts &=~ GINTMSK_DISCONNECT;
		}

		synopsys_usb_update_irq(state);
		return;

	case USB_HREGS...(USB_HREGS + USB_HREGS_SIZE - 4):
		_addr -= USB_HREGS;
        return synopsys_usb_channel_write(state, _addr >> 5, _addr & 0x1f, _val);

		// Device Regs

	case DOEPMSK:
		state->doepmsk = _val;
		synopsys_usb_update_irq(state);
		return;

	case DIEPMSK:
		state->diepmsk = _val;
		synopsys_usb_update_irq(state);
		return;

	case DAINTMSK:
		state->daintmsk = _val;
		synopsys_usb_update_irq(state);
		return;
	
	case DAINTSTS:
		state->daintsts &=~ _val;
		synopsys_usb_update_irq(state);
		return;

	case GLPMCTL:
		state->glpmctl = _val;
		return;

	case GAHBCFG:
		state->gahbcfg = _val;
		return;

	case GUSBCFG:
		state->gusbcfg = _val;
		return;

	case DCTL:
		if((_val & DCTL_SGNPINNAK) != (state->dctl & DCTL_SGNPINNAK)
				&& (_val & DCTL_SGNPINNAK))
		{
			state->gintsts |= GINTMSK_GINNAKEFF;
			_val &=~ DCTL_SGNPINNAK;
		}

		if((_val & DCTL_SGOUTNAK) != (state->dctl & DCTL_SGOUTNAK)
				&& (_val & DCTL_SGOUTNAK))
		{
			state->gintsts |= GINTMSK_GOUTNAKEFF;
			_val &=~ DCTL_SGOUTNAK;
		}

		state->dctl = _val;
		synopsys_usb_update_irq(state);
		return;

	case DCFG:
		printf("USB: dcfg = 0x%08x.\n", (uint32_t)_val);
		state->dcfg = _val;
		return;

	case GRXFSIZ:
		state->grxfsiz = _val;
		return;

	case GNPTXFSIZ:
		state->gnptxfsiz = _val;
		return;

	case HPTXFSIZ:
		state->hptxfsiz = _val;
		return;

	case DIEPTXF(1) ... DIEPTXF(USB_NUM_FIFOS+1):
		_addr -= DIEPTXF(1);
		_addr >>= 2;
		state->dptxfsiz[_addr] = _val;
		return;

	case USB_INREGS ... (USB_INREGS + USB_EPREGS_SIZE - 4):
		_addr -= USB_INREGS;
		synopsys_usb_in_ep_write(state, _addr >> 5, _addr & 0x1f, _val);
		return;

	case USB_OUTREGS ... (USB_OUTREGS + USB_EPREGS_SIZE - 4):
		_addr -= USB_OUTREGS;
		synopsys_usb_out_ep_write(state, _addr >> 5, _addr & 0x1f, _val);
		return;

	case USB_FIFO_START ... USB_FIFO_END-4:
		_addr -= USB_FIFO_START;

		if(state->gintsts & GINTMSK_CURMODE)
			synopsys_usb_write_fifo(state, (_addr/4), _val);
		else
			*((uint32_t*)(&state->fifos[_addr])) = _val;
		return;

	default:
		fprintf(stderr, "USB: Unhandled write address 0x%08x!\n", _addr);
	}
}

static const MemoryRegionOps usb_ops = {
	.read = synopsys_usb_read,
	.write = synopsys_usb_write,
};

static void synopsys_usb_initial_reset(DeviceState *dev)
{
	synopsys_usb_state *state =
		FROM_SYSBUS(synopsys_usb_state, sysbus_from_qdev(dev));

	// Global regs
	state->pcgcctl = 3;

	state->gsnpsid = 0x4f542fff; // Derived from dwc_otg driver

	// Values from iPad1G.
	state->ghwcfg1 = 0x00000264;
	state->ghwcfg2 = 0x228f60d0;
	state->ghwcfg3 = 0x07e800e8;
	state->ghwcfg4 = 0xdbf08030;

	state->gahbcfg = 0;
	state->gusbcfg = 0;

	state->grstctl = GRSTCTL_AHBIDLE;

	// Host regs
	state->hcfg		= 0x00200000;
	state->hfir		= 0x000017d7;
	state->hfnum	= 0;
	state->hptxsts	= 0;
	state->haintsts	= 0;
	state->haintmsk	= 0;

	// Device regs
	state->dctl = 0;
	state->dcfg = 0;
	state->dsts = 0;

	state->gotgctl = 0;
	state->gotgint = 0;

	state->gintmsk = 0;
	state->gintsts = 0;

	state->daintmsk = 0;
	state->daintsts = 0;

	state->diepmsk = 0;
	state->doepmsk = 0;

	// FIFO sizes
	state->hptxfsiz 	= (0x300 << 16) | 0x500;
	state->hptxsts		= (8 << 16) | 0x500;
	state->grxfsiz 		= 0x100;
	state->gnptxfsiz 	= (0x100 << 16) | 0x100;

	uint32_t counter = 0x200;
	int i;
	for(i = 0; i < USB_NUM_FIFOS; i++)
	{
		state->dptxfsiz[i] = (counter << 16) | 0x100;
		counter += 0x100;
	}

	// EPs
	for(i = 0; i < USB_NUM_ENDPOINTS; i++)
	{
		synopsys_usb_ep_state *in = &state->in_eps[i];
		in->control = 0;
		in->dma_address = 0;
		in->fifo = 0;
		in->tx_size = 0;

		synopsys_usb_ep_state *out = &state->out_eps[i];
		out->control = 0;
		out->dma_address = 0;
		out->fifo = 0;
		out->tx_size = 0;
	}

	if(state->usb_device)
		synopsys_usb_setup_port(state, 1, 1);

	synopsys_usb_update_irq(state);
}

static int synopsys_usb_init(SysBusDevice *dev)
{
	synopsys_usb_state *state =
		FROM_SYSBUS(synopsys_usb_state, dev);

	state->iomem = g_new(MemoryRegion, 1);
	state->sof_timer = qemu_new_timer(vm_clock,
			1, synopsys_usb_sof, state);

    state->rxfifo_start = state->rxfifo_end = 0;

	//tcp_usb_init(&state->tcp_state, NULL, NULL, NULL);

	memory_region_init_io(state->iomem, &usb_ops,
			state, "synopsys_usb", 0x100000);
    sysbus_init_mmio(dev, state->iomem);
    sysbus_init_irq(dev, &state->irq);

	int i;
	for(i = 0; i < USB_NUM_CHANNELS; i++)
	{
		synopsys_usb_channel_state *ch = &state->channels[i];

		memset(ch, 0, sizeof(*ch));

		ch->parent = state;
		ch->index = i;

		ch->fifo_start = ch->fifo_end = 0;

		usb_packet_init(&ch->packet);
	}

	for(i = 0; i < USB_NUM_ENDPOINTS; i++)
	{
		synopsys_usb_ep_state *ep = &state->in_eps[i];
		ep->parent = state;
		ep->index = i;
		ep->in = 1;
	}

	for(i = 0; i < USB_NUM_ENDPOINTS; i++)
	{
		synopsys_usb_ep_state *ep = &state->out_eps[i];
		ep->parent = state;
		ep->index = i;
		ep->in = 0;
	}

	synopsys_usb_initial_reset(&state->busdev.qdev);

	usb_bus_new(&state->usb, &bus_ops, &state->busdev.qdev);
	usb_register_port(&state->usb, &state->usb_port,
			state, 0, &port_ops, 
			USB_SPEED_MASK_LOW | USB_SPEED_MASK_FULL
			| USB_SPEED_MASK_HIGH);

	synopsys_usb_update_sof(state);
	return 0;
}

static void synopsys_usb_class_init(ObjectClass *_klass, void *_data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(_klass);
    sdc->init = synopsys_usb_init;
}

static TypeInfo synopsys_usb_info = {
	.name = DEVICE_NAME,
	.parent = TYPE_SYS_BUS_DEVICE,
    .class_init = synopsys_usb_class_init,
	.instance_size = sizeof(synopsys_usb_state),
};

static void synopsys_usb_register(void)
{
    type_register_static(&synopsys_usb_info);
}
type_init(synopsys_usb_register);
