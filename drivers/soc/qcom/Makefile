# When adding new entries keep the list in alphabetical order
CFLAGS_scm.o :=$(call as-instr,.arch_extension sec,-DREQUIRES_SEC=1)
CFLAGS_ipc_router_hsic_xprt.o := -fno-strict-aliasing
CFLAGS_ipc_router_smd_xprt.o := -fno-strict-aliasing

obj-y		+= qdsp6v2/

obj-$(CONFIG_MAXIMUM_CURRENT_THROTTLING) += mct.o
obj-$(CONFIG_MSM_BAM_DMUX) += bam_dmux.o
obj-$(CONFIG_MSM_IPC_ROUTER_HSIC_XPRT) += ipc_router_hsic_xprt.o
obj-$(CONFIG_MSM_IPC_ROUTER_SMD_XPRT) += ipc_router_smd_xprt.o
obj-$(CONFIG_MSM_MEMORY_DUMP) += memory_dump.o
obj-$(CONFIG_MSM_QMI_INTERFACE) += qmi_interface.o

obj-$(CONFIG_MSM_RUN_QUEUE_STATS) += msm_rq_stats.o

obj-$(CONFIG_MSM_SCM) += scm.o

obj-$(CONFIG_MSM_SMEM) += smem.o smem_debug.o
obj-$(CONFIG_MSM_SMEM_LOGGING) += smem_log.o
obj-$(CONFIG_MSM_WATCHDOG_V2) += watchdog_v2.o
obj-$(CONFIG_MEM_SHARE_QMI_SERVICE)		+= memshare/
obj-y		+= socinfo.o
obj-$(CONFIG_QCOM_EARLY_RANDOM)	+= early_random.o

obj-$(CONFIG_ASMP) 	      += autosmp.o
obj-$(CONFIG_BLU_PLUG)        += blu_plug.o
obj-$(CONFIG_BRICKED_HOTPLUG) += bricked_hotplug.o
obj-$(CONFIG_INTELLI_PLUG)    += intelli_plug.o
obj-$(CONFIG_MAKO_HOTPLUG)    += mako_hotplug.o
obj-$(CONFIG_MSM_SLEEPER)     += msm-sleeper.o
obj-$(CONFIG_ZEN_DECISION)    += msm_zen_decision.o
obj-$(CONFIG_STATE_HELPER)    += state_helper.o
obj-$(CONFIG_THUNDERPLUG_CONTROL) += thunderplug.o

obj-$(CONFIG_MSM_LIMITER)     += msm_limiter.o
obj-$(CONFIG_STATE_NOTIFIER)  += state_notifier.o
