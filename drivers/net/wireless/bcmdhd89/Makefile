# bcmdhd

DHDCFLAGS += -Wall -Wstrict-prototypes -Dlinux -DBCMDRIVER                     \
	-DBCMDONGLEHOST -DUNRELEASEDCHIP -DBCMDMA32 -DWLBTAMP -DBCMFILEIMAGE  \
	-DDHDTHREAD -DDHD_GPL -DDHD_SCHED -DDHD_DEBUG -DSDTEST -DBDC -DTOE    \
	-DDHD_BCMEVENTS -DSHOW_EVENTS -DDONGLEOVERLAYS -DBCMDBG               \
	-DCUSTOMER_HW2 -DCUSTOM_OOB_GPIO_NUM=142 -DOOB_INTR_ONLY -DCONFIG_WIFI_CONTROL_FUNC \
	-DMMC_SDIO_ABORT -DBCMSDIO -DBCMLXSDMMC -DBCMPLATFORM_BUS -DWLP2P     \
	-DNEW_COMPAT_WIRELESS -DWIFI_ACT_FRAME -DARP_OFFLOAD_SUPPORT          \
	-DKEEP_ALIVE -DCSCAN -DGET_CUSTOM_MAC_ENABLE -DPKT_FILTER_SUPPORT     \
	-DEMBEDDED_PLATFORM -DENABLE_INSMOD_NO_FW_LOAD -DPNO_SUPPORT          \
	-DOEM_ANDROID -DLINUX -DWL_ENABLE_P2P_IF -DWL_CFG80211_STA_EVENT	  \
	-Idrivers/net/wireless/bcmdhd89 -Idrivers/net/wireless/bcmdhd89/include

DHDOFILES = aiutils.o bcmsdh_sdmmc_linux.o dhd_linux.o siutils.o bcmutils.o   \
	dhd_linux_sched.o bcmwifi.o dhd_sdio.o bcmevent.o dhd_bta.o hndpmu.o  \
	bcmsdh.o dhd_cdc.o bcmsdh_linux.o dhd_common.o linux_osl.o            \
	dhd_custom_gpio.o sbutils.o wldev_common.o wl_android.o dhd_cfg80211.o

ifneq ($(CONFIG_WLAN_SDIO),)
DHDOFILES += bcmsdh_sdmmc.o
DHDCFLAGS += -DHW_OOB       
else
DHDOFILES += bcmspibrcm.o bcmsdspi_linux.o  bcmpcispi.o
DHDCFLAGS += -DSPRD_SPI -DBCMSDIOH_SPI -DBCMSPI
endif

obj-$(CONFIG_BCMDHD89) += bcmdhd.o
bcmdhd-objs += $(DHDOFILES)
ifneq ($(CONFIG_WIRELESS_EXT),)
bcmdhd-objs += wl_iw.o
DHDCFLAGS += -DSOFTAP
endif
ifneq ($(CONFIG_CFG80211),)
bcmdhd-objs += wl_cfg80211.o wl_cfgp2p.o wl_linux_mon.o
DHDCFLAGS += -DWL_CFG80211
endif
EXTRA_CFLAGS = $(DHDCFLAGS)
ifeq ($(CONFIG_BCMDHD89),m)
EXTRA_LDFLAGS += --strip-debug
endif
