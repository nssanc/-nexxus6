CFLAGS_KERNEL := -fno-graphite \
          	-fno-graphite-identity \
          	-fno-loop-flatten \
          	-fno-tree-loop-linear \
          	-fno-loop-interchange \
          	-fno-loop-strip-mine \
          	-fno-loop-block \
          	-fno-loop-nest-optimize \
          	-fno-loop-unroll-and-jam \
          	-fno-loop-parallelize-all \
          	-ftree-parallelize-loops=0 \
          	-fno-openmp \
		-Wno-unused-function

obj-$(CONFIG_PM_DEVFREQ)	        += devfreq.o
obj-$(CONFIG_ARCH_MSM_KRAIT)		+= krait-l2pm.o
obj-$(CONFIG_ADRENO_IDLER)		+= adreno_idler.o
obj-$(CONFIG_SIMPLE_GPU_ALGORITHM)	+= simple_gpu_algorithm.o
obj-$(CONFIG_DEVFREQ_GOV_SIMPLE_ONDEMAND) += governor_simpleondemand.o
obj-$(CONFIG_DEVFREQ_GOV_MSM_ADRENO_TZ)	+= governor_msm_adreno_tz.o
obj-$(CONFIG_DEVFREQ_GOV_PERFORMANCE)	+= governor_performance.o
obj-$(CONFIG_DEVFREQ_GOV_MSM_CPUFREQ)	+= governor_msm_cpufreq.o
obj-$(CONFIG_DEVFREQ_GOV_MSM_BW_HWMON)	+= governor_bw_hwmon.o
obj-$(CONFIG_DEVFREQ_GOV_MSM_CACHE_HWMON) += governor_cache_hwmon.o

# DEVFREQ Drivers
obj-$(CONFIG_ARM_EXYNOS4_BUS_DEVFREQ)	+= exynos4_bus.o
