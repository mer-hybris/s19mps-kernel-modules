#
# snd-soc-sprd-codec-sc2730-power.ko
#
# Makefile: for external make invocation
#
# Note:
# - Build arguments(ARCH CROSS_COMPILE O BSP_MAKE_EXTRA_ARGS
#   INSTALL_MOD_PATH etc.) are passed by parent make
# - obj-m objects configuration is moved to Kbuild file used by
#   Kernel kbuild system
# - Please use these predefined name for keeping code:
#   KO_MODULE_NAME KO_MODULE_OUT KO_MODULE_KBUILD
# - For more kernel module building, please refer to kernel doc:
#   Documentation/kbuild/modules.txt
#

export KO_MODULE_NAME := snd-soc-sprd-codec-sc2730-power
KO_MODULE_OUT := $(BSP_MODULES_OUT)/$(KO_MODULE_NAME)
KO_MODULE_KBUILD := $(CURDIR)/Kbuild
export KBUILD_EXTRA_SYMBOLS += $(BSP_MODULES_OUT)/snd-soc-sprd-card/Module.symvers
ifeq ($(BOARD_AUDIO_SMARTPA), AW87XXX)
export KBUILD_EXTRA_SYMBOLS += $(BSP_MODULES_OUT)/smp_aw87xxx/Module.symvers
endif
.PHONY: modules modules_install clean

modules:
	@mkdir -p $(KO_MODULE_OUT) && ln -snf $(KO_MODULE_KBUILD) $(KO_MODULE_OUT)/Kbuild
	@ln -snf $(CURDIR) $(KO_MODULE_OUT)/source
	$(MAKE) -C $(BSP_KERNEL_PATH) M=$(KO_MODULE_OUT) src=$(CURDIR) $@

modules_install:
	$(MAKE) -C $(BSP_KERNEL_PATH) M=$(KO_MODULE_OUT) $@

# Remove the out directory wholly
clean:
	@#$(MAKE) -C $(BSP_KERNEL_PATH) M=$(KO_MODULE_OUT) src=$(CURDIR) $@
	rm -rf $(KO_MODULE_OUT)
