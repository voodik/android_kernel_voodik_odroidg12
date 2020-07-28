# Overlays for the Odroid platform

dtbo-$(CONFIG_ARCH_MESON64_ODROIDN2) += spi0.dtbo		\
					i2c0.dtbo		\
					i2c1.dtbo		\
					uart0.dtbo		\
					uart1.dtbo		\
					hktft32.dtbo		\
					hktft35.dtbo		\
					ads7846.dtbo		\
					sx865x-i2c1.dtbo	\
					saradc.dtbo		\
					irblaster.dtbo		\
					odroidn2-opp.dtbo	\
					odroidn2_plus-opp.dtbo	\
					onewire.dtbo		\
					can0.dtbo		\
					fanspeed-full.dtbo

targets	+= $(dtbo-y)
always	:= $(dtbo-y)
