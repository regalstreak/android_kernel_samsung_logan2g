obj-y += io.o irq.o timer.o devices.o arch_asm.o arch_init.o globalregs.o \
         pm_debug.o arch_lock.o \
         dma.o reserve.o pinmap.o adi.o adie_irq.o gpio_eic.o adc.o efuse.o \
         watchdog.o common.o dcdc_efficiency.o


obj-y += glb.o
obj-$(CONFIG_NKERNEL) += clock_vlx.o
obj-y += clock.o

obj-y += dcdc_cal.o
obj-$(CONFIG_SPRD_DCDC_DEBUG) += dcdc_debug.o

obj-$(CONFIG_SMP)		+= platsmp.o headsmp.o

ifeq ($(CONFIG_NKERNEL),y)
obj-$(CONFIG_PM)		+= pm_vlx.o
else
obj-$(CONFIG_PM)		+= pm.o 
endif
obj-$(CONFIG_PM)		+= pm_sc8825.o standby.o pm_debug.o emc_repower.o emc_earlysuspend.o 
CFLAGS_emc_repower.o += -fPIC

obj-$(CONFIG_CPU_FREQ) += cpufreq-sc8825.o

obj-$(CONFIG_SIPC)		+= sipc.o

obj-$(CONFIG_HOTPLUG_CPU)	+= hotplug.o
obj-$(CONFIG_MACH_SP8825_FPGA) += board_sp8825_fpga.o
obj-$(CONFIG_MACH_SP8825_OPENPHONE) += board_sp8825_openphone.o
obj-$(CONFIG_MACH_SP8825EA) += board_sp8825ea.o
obj-$(CONFIG_MACH_SP8825EB) += board_sp8825eb.o
obj-$(CONFIG_MACH_SP6825GA) += board_sp6825ga.o
obj-$(CONFIG_MACH_SP8810i1) += board_sp8810i1/
obj-$(CONFIG_MACH_LOGAN) += board_logan.o board_logan_wifi.o	
obj-$(CONFIG_MACH_KYLEVE) += board_kyleve.o board_garda_wifi.o
obj-$(CONFIG_MACH_GARDA) += board_garda.o board_garda_wifi.o
obj-$(CONFIG_MACH_CS02) += board_cs02.o board_cs02_wifi.o
obj-$(CONFIG_MACH_SP8825GA_OPENPHONE) += board_sp8825ga_openphone.o

obj-$(CONFIG_MACH_SP6825GB) += ar6302g_adapter.o
obj-$(CONFIG_SC8825_VIBRATOR) += vibrator.o
obj-$(CONFIG_SPRD_VIBRATOR) += vibrator.o
ifeq ($(CONFIG_BCMDHD),m)
obj-y  += dhd_adapter.o
endif
obj-$(CONFIG_BCMDHD89)  += dhd_adapter.o
obj-$(CONFIG_BT_BCM4330) += board-bluetooth-bcm4330.o
ifeq ($(CONFIG_RTL8723AS),m)
	obj-y  += rtl8723_adapter.o
endif
obj-$(CONFIG_ANDROID_RAM_CONSOLE) += ram_console.o
obj-$(CONFIG_SPRD_SC8825_MEMFREQ_ONDEMAND)    += memfreq_ondemand.o
obj-$(CONFIG_SPRD_MEM_POOL)    += sprd_mem_pool.o
obj-$(CONFIG_SEC_DEBUG) += sec_debug.o sec_gaf.o sec_getlog.o sec-common.o sec-reboot.o sys_debug.o
obj-$(CONFIG_CPU_FREQ_SC8825) += cpufreq-sc8825.o
obj-y += ss_musb_info.o

sprdboarddirs := $(patsubst %,arch/arm/mach-sc8825/%,$(sprdboard-y))
KBUILD_CPPFLAGS += $(patsubst %,-I%,$(sprdboarddirs))
