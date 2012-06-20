/*
 * Copyright 2011 Google Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 *
 *
 * SDHCI (SD Host Controler Interface) emulation
 */

#include "blockdev.h"
#include "sysbus.h"
#include "pci.h"
#include "sd.h"

/* from Linux sources : drivers/mmc/host/sdhci.h */
/*
 * Controller registers
 */

#define SDHCI_DMA_ADDRESS       0x00

#define SDHCI_BLOCK_SIZE        0x04

#define SDHCI_BLOCK_COUNT       0x06

#define SDHCI_ARGUMENT          0x08

#define SDHCI_TRANSFER_MODE     0x0C
#define  SDHCI_TRNS_DMA         0x01
#define  SDHCI_TRNS_BLK_CNT_EN  0x02
#define  SDHCI_TRNS_ACMD12      0x04
#define  SDHCI_TRNS_READ        0x10
#define  SDHCI_TRNS_MULTI       0x20

#define SDHCI_COMMAND           0x0E
#define  SDHCI_CMD_RESP_MASK    0x03
#define  SDHCI_CMD_CRC          0x08
#define  SDHCI_CMD_INDEX        0x10
#define  SDHCI_CMD_DATA         0x20

#define  SDHCI_CMD_RESP_NONE    0x00
#define  SDHCI_CMD_RESP_LONG    0x01
#define  SDHCI_CMD_RESP_SHORT   0x02
#define  SDHCI_CMD_RESP_SHORT_BUSY 0x03

#define SDHCI_RESPONSE          0x10

#define SDHCI_BUFFER            0x20

#define SDHCI_PRESENT_STATE     0x24
#define  SDHCI_CMD_INHIBIT      0x00000001
#define  SDHCI_DATA_INHIBIT     0x00000002
#define  SDHCI_DOING_WRITE      0x00000100
#define  SDHCI_DOING_READ       0x00000200
#define  SDHCI_SPACE_AVAILABLE  0x00000400
#define  SDHCI_DATA_AVAILABLE   0x00000800
#define  SDHCI_CARD_PRESENT     0x00010000
#define  SDHCI_WRITE_PROTECT    0x00080000

#define SDHCI_HOST_CONTROL      0x28
#define  SDHCI_CTRL_LED         0x01
#define  SDHCI_CTRL_4BITBUS     0x02
#define  SDHCI_CTRL_HISPD       0x04
#define  SDHCI_CTRL_DMA_MASK    0x18
#define   SDHCI_CTRL_SDMA       0x00
#define   SDHCI_CTRL_ADMA1      0x08
#define   SDHCI_CTRL_ADMA32     0x10
#define   SDHCI_CTRL_ADMA64     0x18
#define   SDHCI_CTRL_8BITBUS    0x20

#define SDHCI_POWER_CONTROL     0x29
#define  SDHCI_POWER_ON         0x01
#define  SDHCI_POWER_180        0x0A
#define  SDHCI_POWER_300        0x0C
#define  SDHCI_POWER_330        0x0E

#define SDHCI_BLOCK_GAP_CONTROL 0x2A

#define SDHCI_WAKE_UP_CONTROL   0x2B
#define  SDHCI_WAKE_ON_INT      0x01
#define  SDHCI_WAKE_ON_INSERT   0x02
#define  SDHCI_WAKE_ON_REMOVE   0x04

#define SDHCI_CLOCK_CONTROL     0x2C
#define  SDHCI_DIVIDER_SHIFT    8
#define  SDHCI_DIVIDER_HI_SHIFT 6
#define  SDHCI_DIV_MASK 0xFF
#define  SDHCI_DIV_MASK_LEN     8
#define  SDHCI_DIV_HI_MASK      0x300
#define  SDHCI_CLOCK_CARD_EN    0x0004
#define  SDHCI_CLOCK_INT_STABLE 0x0002
#define  SDHCI_CLOCK_INT_EN     0x0001

#define SDHCI_TIMEOUT_CONTROL   0x2E

#define SDHCI_SOFTWARE_RESET    0x2F
#define  SDHCI_RESET_ALL        0x01
#define  SDHCI_RESET_CMD        0x02
#define  SDHCI_RESET_DATA       0x04

#define SDHCI_INT_STATUS        0x30
#define SDHCI_INT_ENABLE        0x34
#define SDHCI_SIGNAL_ENABLE     0x38
#define  SDHCI_INT_RESPONSE     0x00000001
#define  SDHCI_INT_DATA_END     0x00000002
#define  SDHCI_INT_DMA_END      0x00000008
#define  SDHCI_INT_SPACE_AVAIL  0x00000010
#define  SDHCI_INT_DATA_AVAIL   0x00000020
#define  SDHCI_INT_CARD_INSERT  0x00000040
#define  SDHCI_INT_CARD_REMOVE  0x00000080
#define  SDHCI_INT_CARD_INT     0x00000100
#define  SDHCI_INT_ERROR        0x00008000
#define  SDHCI_INT_TIMEOUT      0x00010000
#define  SDHCI_INT_CRC          0x00020000
#define  SDHCI_INT_END_BIT      0x00040000
#define  SDHCI_INT_INDEX        0x00080000
#define  SDHCI_INT_DATA_TIMEOUT 0x00100000
#define  SDHCI_INT_DATA_CRC     0x00200000
#define  SDHCI_INT_DATA_END_BIT 0x00400000
#define  SDHCI_INT_BUS_POWER    0x00800000
#define  SDHCI_INT_ACMD12ERR    0x01000000
#define  SDHCI_INT_ADMA_ERROR   0x02000000

#define  SDHCI_INT_NORMAL_MASK  0x00007FFF
#define  SDHCI_INT_ERROR_MASK   0xFFFF8000

#define SDHCI_ACMD12_ERR        0x3C

/* 3E-3F reserved */

#define SDHCI_CAPABILITIES      0x40
#define  SDHCI_TIMEOUT_CLK_MASK 0x0000003F
#define  SDHCI_TIMEOUT_CLK_SHIFT 0
#define  SDHCI_TIMEOUT_CLK_UNIT 0x00000080
#define  SDHCI_CLOCK_BASE_MASK  0x00003F00
#define  SDHCI_CLOCK_V3_BASE_MASK       0x0000FF00
#define  SDHCI_CLOCK_BASE_SHIFT 8
#define  SDHCI_MAX_BLOCK_MASK   0x00030000
#define  SDHCI_MAX_BLOCK_SHIFT  16
#define  SDHCI_CAN_DO_8BIT      0x00040000
#define  SDHCI_CAN_DO_ADMA2     0x00080000
#define  SDHCI_CAN_DO_ADMA1     0x00100000
#define  SDHCI_CAN_DO_HISPD     0x00200000
#define  SDHCI_CAN_DO_SDMA      0x00400000
#define  SDHCI_CAN_VDD_330      0x01000000
#define  SDHCI_CAN_VDD_300      0x02000000
#define  SDHCI_CAN_VDD_180      0x04000000
#define  SDHCI_CAN_64BIT        0x10000000
#define  SDHCI_SLOT_TYPE_EMBED  0x40000000

#define SDHCI_CAPABILITIES_1    0x44

/* 44-47 reserved for more caps */

#define SDHCI_MAX_CURRENT       0x48

/* 4C-4F reserved for more max current */

#define SDHCI_SET_ACMD12_ERROR  0x50
#define SDHCI_SET_INT_ERROR     0x52

#define SDHCI_ADMA_ERROR        0x54

/* 55-57 reserved */

#define SDHCI_ADMA_ADDRESS      0x58

/* 60-FB reserved */

#define SDHCI_SLOT_INT_STATUS   0xFC

#define SDHCI_HOST_VERSION      0xFE
#define  SDHCI_VENDOR_VER_MASK  0xFF00
#define  SDHCI_VENDOR_VER_SHIFT 8
#define  SDHCI_SPEC_VER_MASK    0x00FF
#define  SDHCI_SPEC_VER_SHIFT   0
#define   SDHCI_SPEC_100        0
#define   SDHCI_SPEC_200        1
#define   SDHCI_SPEC_300        2

/* ADMA descriptor flags */
#define ADMA_VALID   (1<<0)
#define ADMA_END     (1<<1)
#define ADMA_INT     (1<<2)

#define ADMA_OP_MASK (3<<4)
#define ADMA_OP_NOP  (0<<4)
#define ADMA_OP_RSV  (1<<4)
#define ADMA_OP_TRAN (2<<4)
#define ADMA_OP_LINK (3<<4)

/* re-write a part of a variable using a mask
 * to emulate the "byte-enable" behaviour in hardware
 */
#define MASKED_WRITE(var, val, mask) do {\
        var = (var & ~(mask)) | ((val) & (mask));\
    } while (0)

typedef struct {
    union {
        SysBusDevice busdev;
        PCIDevice pcidev;
    };
    BlockDriverState *bs;
    uint32_t sdma_address;
    uint16_t block_size;
    uint16_t block_count;
    uint32_t arg;
    uint16_t transfer_mode;
    uint32_t response[4];
    uint32_t clock;
    uint32_t host_control;
    uint32_t int_enable;
    uint32_t int_status;
    uint32_t adma_address;
    uint8_t adma_error;
    SDState *sd;
    qemu_irq irq;
} sdhci_state;

static void sdhci_reset(DeviceState *d);

static void sdhci_set_irq(sdhci_state *s)
{
    qemu_set_irq(s->irq, !!(s->int_status & s->int_enable));
}

static void sdhci_dma_transfer(sdhci_state *s)
{
    int b;
    struct adma_desc {
        uint16_t flags;
        uint16_t size;
        uint32_t addr;
    } desc;
    uint16_t xfer_size;
    uint32_t total_size, remaining_size;

    s->adma_error = 0;

    if (s->host_control & SDHCI_CTRL_ADMA32) {
        cpu_physical_memory_read(s->adma_address, (void *)&desc, sizeof(desc));
    } else { /* use SDMA */
        /* Generate hardcoded descriptor to emulate ADMA */
        desc.addr = s->sdma_address;
        desc.size = s->block_count * (s->block_size & 0xfff);
        desc.flags = ADMA_VALID | ADMA_END | ADMA_OP_TRAN;
        /* TODO: manage SDMA buffer boundary */
    }
    /* 0 in the size field means 65536 bytes */
    remaining_size = desc.size ? desc.size : 65536;

    s->int_status |= SDHCI_INT_DATA_END;

    total_size = s->block_count*(s->block_size & 0xfff);
    while (total_size) {
        while (!(desc.flags & ADMA_VALID) || !remaining_size ||
                ((desc.flags & ADMA_OP_MASK) != ADMA_OP_TRAN)) {
            if ((desc.flags & ADMA_END) || !(desc.flags & ADMA_VALID)) {
                /* Abort ADMA transfer */
                s->int_status |= SDHCI_INT_ADMA_ERROR;
                s->adma_error = 1 /* ST_FDS */;
                s->block_count = total_size / s->block_size;
                return;
            }
            if ((desc.flags & ADMA_OP_MASK) == ADMA_OP_LINK) {
                s->adma_address = desc.addr;
            } else {
                s->adma_address += sizeof(struct adma_desc);
            }
            cpu_physical_memory_read(s->adma_address, (void *)&desc,
                                     sizeof(desc));
            /* 0 in the size field means 65536 bytes */
            remaining_size = desc.size ? desc.size : 65536;
            /* ADMA_INT flag not implemented : it seems to be debug only */
        }

        xfer_size = MIN((s->block_size & 0xfff), remaining_size);
        for (b = 0; b < xfer_size; b++, desc.addr++) {
            if (s->transfer_mode & SDHCI_TRNS_READ) {
                uint8_t data = sd_read_data(s->sd);
                cpu_physical_memory_write(desc.addr, &data, 1);
            } else {
                uint8_t data;
                cpu_physical_memory_read(desc.addr, &data, 1);
                sd_write_data(s->sd, data);
            }
        }
        remaining_size -= xfer_size;
        total_size -= xfer_size;
    }
    s->block_count = 0;
}

static void sdhci_command(sdhci_state *s, uint32_t cmd)
{
    SDRequest request;
    int len;
    uint8_t r[16];

    if (!s->sd) { /* nothing beyond the controller */
        s->int_status |= SDHCI_INT_TIMEOUT;
        sdhci_set_irq(s);
        return;
    }

    request.cmd = (cmd>>8) & 0xff;
    request.arg = s->arg;
    len = sd_do_command(s->sd, &request, r);
    if (len == 0) {
        if (cmd & SDHCI_CMD_RESP_MASK) {
            /* no response expected */
            s->int_status |= SDHCI_INT_INDEX;
        } else {
            /* error */
            s->int_status |= SDHCI_INT_RESPONSE;
        }
    } else {
        if (len == 4) {
            s->response[0] = (r[0]<<24) | (r[1]<<16) | (r[2]<<8) | r[3];
        } else if (len == 16) {
            s->response[0] = (r[11]<<24) | (r[12]<<16) | (r[13]<<8) | r[14];
            s->response[1] =  (r[7]<<24) |  (r[8]<<16) |  (r[9]<<8) | r[10];
            s->response[2] =  (r[3]<<24) |  (r[4]<<16) |  (r[5]<<8) |  r[6];
            s->response[3] =  (0xcc<<24) |  (r[0]<<16) |  (r[1]<<8) |  r[2];
        }
        s->int_status |= SDHCI_INT_RESPONSE;
        if ((cmd & 3) == SDHCI_CMD_RESP_SHORT_BUSY) {
            /* the command will trigger the busy (DAT[0]) line ON then OFF
             * this will raise the Data END interrupt when done.
             */
            s->int_status |= SDHCI_INT_DATA_END;
        }
        if ((s->transfer_mode & SDHCI_TRNS_DMA) && (cmd & SDHCI_CMD_DATA)) {
            sdhci_dma_transfer(s);
        }
    }
    sdhci_set_irq(s);
}

static uint64_t sdhci_readl(void *opaque, target_phys_addr_t offset)
{
    sdhci_state *s = opaque;

    if ((offset >= 0x100) && (offset < 0x200)) {
        fprintf(stderr, "sdhci: unsupported vendor read @" TARGET_FMT_plx "\n",
                offset);
        return 0;
    }

    switch (offset) {
    case SDHCI_DMA_ADDRESS:
        return s->sdma_address;
    case SDHCI_BLOCK_SIZE /* +SDHCI_BLOCK_COUNT */:
        return s->block_size | (s->block_count << 16);
    case SDHCI_ARGUMENT:
        return s->arg;
    case SDHCI_TRANSFER_MODE /* +SDHCI_COMMAND */:
        return s->transfer_mode;
    case SDHCI_RESPONSE:
    case SDHCI_RESPONSE+4:
    case SDHCI_RESPONSE+8:
    case SDHCI_RESPONSE+12:
        return s->response[(offset-SDHCI_RESPONSE)/sizeof(uint32_t)];
    case SDHCI_BUFFER:
        /* TODO: implement PIO mode */
        return 0;
    case SDHCI_PRESENT_STATE:
        return (s->sd ? 0x00170000 : 0) |
               (s->bs && bdrv_is_read_only(s->bs) ? 0 : 0x00080000);
    case SDHCI_HOST_CONTROL:
        /*+SDHCI_POWER_CONTROL +SDHCI_BLOCK_GAP_CONTROL +SDHCI_WAKE_UP_CONTROL*/
        return s->host_control;
    case SDHCI_CLOCK_CONTROL /* +SDHCI_TIMEOUT_CONTROL +SDHCI_SOFTWARE_RESET*/:
        return s->clock | (0<<24 /* reset done */);
    case SDHCI_INT_STATUS:
        return s->int_status;
    case SDHCI_INT_ENABLE:
        return s->int_enable;
    case SDHCI_SIGNAL_ENABLE:
        return 0;
    case SDHCI_ACMD12_ERR:
        return 0;
    case SDHCI_CAPABILITIES:
        return SDHCI_SLOT_TYPE_EMBED | SDHCI_CAN_VDD_330 |
               SDHCI_CAN_DO_ADMA1 | SDHCI_CAN_DO_HISPD | SDHCI_CAN_DO_SDMA |
               SDHCI_CAN_DO_8BIT | SDHCI_CAN_DO_ADMA2 |
               (52 << SDHCI_CLOCK_BASE_SHIFT) |
               SDHCI_TIMEOUT_CLK_UNIT /* MHz */ | (52<<SDHCI_TIMEOUT_CLK_SHIFT);
    case SDHCI_CAPABILITIES_1:
        return 0;
    case SDHCI_MAX_CURRENT:
        return 0;
    case SDHCI_SET_ACMD12_ERROR /* +SDHCI_SET_INT_ERROR */:
         hw_error("sdhci_read: read only register at " TARGET_FMT_plx "\n",
                  offset);
    case SDHCI_ADMA_ERROR:
        return s->adma_error;
    case SDHCI_ADMA_ADDRESS:
        return s->adma_address;
    case SDHCI_SLOT_INT_STATUS /* +SDHCI_HOST_VERSION */:
        return 0 | (SDHCI_SPEC_200 << 16);
    default:
        hw_error("sdhci_read: Bad offset " TARGET_FMT_plx "\n", offset);
    }

    return 0;
}

static uint64_t sdhci_read(void *_opaque,
		target_phys_addr_t _offset, unsigned _sz)
{
	int off = (_offset & 3);
	uint64_t data = sdhci_readl(_opaque, _offset &~ 3);
	
	return data >> (off << 3);
}

static void sdhci_write_masked(void *opaque, target_phys_addr_t offset,
                               uint32_t value, uint32_t mask)
{
    sdhci_state *s = opaque;

    if ((offset >= 0x100) && (offset < 0x200)) {
        fprintf(stderr, "sdhci: unsupported vendor write at "TARGET_FMT_plx"\n",
                offset);
        return;
    }

    switch (offset) {
    case SDHCI_DMA_ADDRESS:
        MASKED_WRITE(s->sdma_address, value, mask);
        break;
    case SDHCI_BLOCK_SIZE /* +SDHCI_BLOCK_COUNT */:
        MASKED_WRITE(s->block_size, value & 0x7fff, mask);
        MASKED_WRITE(s->block_count, value >> 16, mask >> 16);
        break;
    case SDHCI_ARGUMENT:
        MASKED_WRITE(s->arg, value, mask);
        break;
    case SDHCI_TRANSFER_MODE /* +SDHCI_COMMAND */:
        MASKED_WRITE(s->transfer_mode, value & 0x3f, mask);
        if (mask & 0xffff0000) {
            sdhci_command(s, value >> 16);
        }
        break;
    case SDHCI_BUFFER:
        /* TODO: implement PIO mode */
        break;
    case SDHCI_HOST_CONTROL:
        /*+SDHCI_POWER_CONTROL +SDHCI_BLOCK_GAP_CONTROL +SDHCI_WAKE_UP_CONTROL*/
        MASKED_WRITE(s->host_control, value, mask);
        if ((mask >> 8) & SDHCI_POWER_ON) {
            if (s->sd) {
                sd_enable(s->sd, ((value & mask) >> 8) & SDHCI_POWER_ON);
            }
        }
        break;
    case SDHCI_CLOCK_CONTROL /* +SDHCI_TIMEOUT_CONTROL +SDHCI_SOFTWARE_RESET */:
        if (((value & mask) >> 24) & SDHCI_RESET_ALL) {
            sdhci_reset(opaque);
        }
        if (((value & mask) >> 24) & SDHCI_RESET_CMD) {
            s->int_status &= ~SDHCI_INT_RESPONSE;
            sdhci_set_irq(s);
        }
        if (((value & mask) >> 24) & SDHCI_RESET_DATA) {
            s->adma_error = 0;
            s->int_status &= ~(SDHCI_INT_DATA_END | SDHCI_INT_DMA_END);
            sdhci_set_irq(s);
        }
        MASKED_WRITE(s->clock, (value & 0xfffff) | SDHCI_CLOCK_INT_STABLE,
                     mask);
        break;
    case SDHCI_INT_STATUS:
        s->int_status &= ~(value & mask);
        sdhci_set_irq(s);
        break;
    case SDHCI_INT_ENABLE:
        MASKED_WRITE(s->int_enable, value, mask);
        sdhci_set_irq(s);
        break;
    case SDHCI_SIGNAL_ENABLE:
        break;
    case SDHCI_SET_ACMD12_ERROR /* +SDHCI_SET_INT_ERROR */:
        break;
    case SDHCI_ADMA_ADDRESS:
        MASKED_WRITE(s->adma_address, value, mask);
        break;
    case SDHCI_SLOT_INT_STATUS /* +SDHCI_HOST_VERSION */:
        break;
    /* Read only registers */
    case SDHCI_RESPONSE:
    case SDHCI_RESPONSE+4:
    case SDHCI_RESPONSE+8:
    case SDHCI_RESPONSE+12:
    case SDHCI_PRESENT_STATE:
    case SDHCI_ACMD12_ERR:
    case SDHCI_CAPABILITIES:
    case SDHCI_CAPABILITIES_1:
    case SDHCI_MAX_CURRENT:
    case SDHCI_ADMA_ERROR:
        hw_error("sdhci_write: Read only register at " TARGET_FMT_plx "\n",
                 offset);
    default:
        hw_error("sdhci_write: Bad offset " TARGET_FMT_plx "\n", offset);
    }
}

static void sdhci_write(void *opaque, target_phys_addr_t offset,
		uint64_t value, unsigned _sz)
{
	int off = (offset & 3)*8;
	offset &=~ 3;

    sdhci_write_masked(opaque, offset, value, ((1 << _sz)-1) << off);
}

static const MemoryRegionOps sdhci_ops = {
	.read = sdhci_read,
	.write = sdhci_write,
};

static void sdhci_reset(DeviceState *d)
{
    sdhci_state *s = container_of(d, sdhci_state, busdev.qdev);

    if (!s->bs) {
        DriveInfo *inf = drive_get_next(IF_SD);
        s->bs = inf ? inf->bdrv : NULL;
    }
    if (!s->sd && s->bs) {
        s->sd = sd_init(s->bs, 0);
    }

    s->sdma_address = 0;
    s->block_size = 0;
    s->block_count = 0;
    s->arg = 0;
    s->transfer_mode = 0;
    s->clock = SDHCI_CLOCK_INT_STABLE;
    s->int_enable = 0;
    s->int_status = 0;
    s->adma_address = 0;
    s->adma_error = 0;
}

static const VMStateDescription sdhci_vmstate = {
    .name = "sdhci",
    .version_id = 0,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(sdma_address, sdhci_state),
        VMSTATE_UINT16(block_size, sdhci_state),
        VMSTATE_UINT16(block_count, sdhci_state),
        VMSTATE_UINT32(arg, sdhci_state),
        VMSTATE_UINT16(transfer_mode, sdhci_state),
        VMSTATE_UINT32_ARRAY(response, sdhci_state, 4),
        VMSTATE_UINT32(clock, sdhci_state),
        VMSTATE_UINT32(int_enable, sdhci_state),
        VMSTATE_UINT32(int_status, sdhci_state),
        VMSTATE_UINT32(adma_address, sdhci_state),
        VMSTATE_UINT8(adma_error, sdhci_state),
        VMSTATE_END_OF_LIST()
    }
};

static int sdhci_mm_init(SysBusDevice *dev)
{
    sdhci_state *s = FROM_SYSBUS(sdhci_state, dev);
	MemoryRegion *iomem = g_new(MemoryRegion, 1);
	memory_region_init_io(iomem, &sdhci_ops, s, "sdhci", 0x200);
    sysbus_init_mmio(dev, iomem);
    sysbus_init_irq(dev, &s->irq);

    return 0;
}

static void sdhci_mm_class_init(ObjectClass *_klass, void *_obj)
{
	SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(_klass);
	sdc->init = sdhci_mm_init;
}

static TypeInfo sdhci_mm_info = {
	.name 			= "sdhci",
	.parent 		= TYPE_SYS_BUS_DEVICE,
	.instance_size	= sizeof(sdhci_state),
    .class_init 	= sdhci_mm_class_init,

/*    .qdev.name  = "sdhci",
    .qdev.size  = sizeof(sdhci_state),
    .qdev.vmsd  = &sdhci_vmstate,
    .qdev.reset = sdhci_reset,
    .qdev.props = (Property[]) {
        DEFINE_PROP_DRIVE("block", sdhci_state, bs),
        DEFINE_PROP_END_OF_LIST(),
		}*/
};

static int sdhci_pci_init(PCIDevice *dev)
{
    sdhci_state *s = DO_UPCAST(sdhci_state, pcidev, dev);
    uint8_t *pci_conf = s->pcidev.config;
	MemoryRegion *iomem = g_new(MemoryRegion, 1);

    pci_conf[PCI_CLASS_PROG] = 0x01; /* Standard Host supported DMA */
    pci_conf[PCI_INTERRUPT_PIN] = 1; /* interrupt pin 0 */
    pci_conf[0x40] = 0x00; /* 1 slot at BAR0 */

	memory_region_init_io(iomem, &sdhci_ops, s, "sdhci", 0x100);
    pci_register_bar(&s->pcidev, 0, 0, iomem);
    s->irq = s->pcidev.irq[0];
    return 0;
}

static void sdhci_pci_class_init(ObjectClass *_klass, void *_obj)
{
	PCIDeviceClass *k = PCI_DEVICE_CLASS(_klass);
	k->init 		= sdhci_pci_init;
	k->vendor_id 	= PCI_VENDOR_ID_QEMU;
	k->device_id	= PCI_DEVICE_ID_SDHCI;
	k->class_id		= PCI_CLASS_SYSTEM_SDHCI;
}

static TypeInfo sdhci_pci_info = {
	.name 			= "sdhci_pci",
	.parent			= TYPE_PCI_DEVICE,
	.instance_size	= sizeof(sdhci_state),
	.class_init		= sdhci_pci_class_init,

/*    .qdev.name    = "sdhci_pci",
    .qdev.size    = sizeof(sdhci_state),
    .qdev.vmsd    = &sdhci_vmstate,
    .qdev.reset   = sdhci_reset,
    .qdev.props = (Property[]) {
        DEFINE_PROP_DRIVE("block", sdhci_state, bs),
        DEFINE_PROP_END_OF_LIST(),
    },
    .init         = sdhci_pci_init,
    .vendor_id    = PCI_VENDOR_ID_QEMU,
    .device_id    = PCI_DEVICE_ID_SDHCI,
    .class_id     = PCI_CLASS_SYSTEM_SDHCI,*/
};

static void sdhci_register(void)
{
	type_register_static(&sdhci_mm_info);
	type_register_static(&sdhci_pci_info);
}

type_init(sdhci_register)
