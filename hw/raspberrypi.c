#include "hw.h"
#include "boards.h"
#include "bcm2708.h"
#include "devices.h"
#include "arm-misc.h"
#include "loader.h"

//
//#define OLD_BOOT
//

static struct bcm2708_state rpi_bcm2708;
#ifndef OLD_BOOT
static struct arm_boot_info rpi_boot;
#endif

static void rpi_init(ram_addr_t _ram_size,
		const char *_boot_device,
		const char *_kernel,
		const char *_cmdline,
		const char *_initrd,
		const char *_cpu)
{
	ARMCPU *env;
	MemoryRegion *sysmem = get_system_memory();
	MemoryRegion *ram = g_new(MemoryRegion, 1);

	if(!_cpu)
		_cpu = "arm1176";
	env = cpu_arm_init(_cpu);
	
	if(!env)
	{
		fprintf(stderr, "Unable to find CPU definition.\n");
		exit(1);
	}
	
	memory_region_init_ram(ram, "rpi.ram", _ram_size);
	vmstate_register_ram_global(ram);

	// SDRAM is mapped in at 0x0
	// (RAM is POP so configure it here, rather than
	//  in BCM2708 code.)
	memory_region_add_subregion(sysmem, 0, ram);

	// Do SoC setup.
	bcm2708_init(&rpi_bcm2708, env, sysmem);

#ifdef OLD_BOOT
	if(!_kernel)
	{
		fprintf(stderr, "Kernel image not specified.\n");
		exit(1);
	}

	int ret = load_image_targphys(_kernel, 0, _ram_size);
	if(ret < 0)
	{
		fprintf(stderr, "Failed to load kernel.\n");
		exit(1);
	}
#else

	rpi_boot.ram_size 			= _ram_size - (128*1024*1024); // TODO: parameter.
	rpi_boot.kernel_filename 	= _kernel;
	rpi_boot.kernel_cmdline		= _cmdline;
	rpi_boot.initrd_filename	= _initrd;
	arm_load_kernel(env, &rpi_boot);
#endif
}

static QEMUMachine raspberrypi_machine = {
	.name = "raspberrypi",
	.desc = "Raspberry Pi",
	.init = rpi_init,
};

static void rpi_mach_init(void)
{
	qemu_register_machine(&raspberrypi_machine);
}

machine_init(rpi_mach_init);
