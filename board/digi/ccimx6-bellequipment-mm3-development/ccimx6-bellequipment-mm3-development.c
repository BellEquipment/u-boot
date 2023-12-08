/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2013-2018 Digi International, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 * Author: Jason Liu <r64343@freescale.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <common.h>
#include <errno.h>
#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#ifdef CONFIG_OF_LIBFDT
#include <fdt_support.h>
#endif
#include <fsl_esdhc.h>
#include <fuse.h>
#include <micrel.h>
#include <miiphy.h>
#include <mmc.h>
#include <netdev.h>
#include <command.h>
#ifdef CONFIG_SYS_I2C_MXC
#include <i2c.h>
#include <asm/imx-common/mxc_i2c.h>
#endif
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/iomux-v3.h>
#include "../ccimx6/ccimx6.h"
#include "../common/carrier_board.h"
#include "../common/helper.h"
#include "../common/hwid.h"
#include "../common/trustfence.h"
#include "../../../drivers/net/fec_mxc.h"

DECLARE_GLOBAL_DATA_PTR;

static int phy_addr;
unsigned int board_version = CARRIERBOARD_VERSION_UNDEFINED;
unsigned int board_id = CARRIERBOARD_ID_UNDEFINED;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |            \
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |               \
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define GPI_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |            \
	PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_MED |               \
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST)

static iomux_v3_cfg_t const uart4_pads[] = {
	MX6_PAD_KEY_COL0__UART4_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_KEY_ROW0__UART4_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

#ifdef CONFIG_CONSOLE_ENABLE_GPIO
static iomux_v3_cfg_t const ext_gpios_pads[] = {
	/* Put the external GPIOs in a known safe state */
	MX6_PAD_NANDF_D2__GPIO2_IO02     | MUX_PAD_CTRL(GPI_PAD_CTRL), // USER_LED0 - Power Indicator
	MX6_PAD_NANDF_D3__GPIO2_IO03     | MUX_PAD_CTRL(GPI_PAD_CTRL), // USER_LED1 - Communications
	MX6_PAD_NANDF_D4__GPIO2_IO04     | MUX_PAD_CTRL(GPI_PAD_CTRL), // USER_LED2 - Heart Beat
	MX6_PAD_GPIO_18__GPIO7_IO13      | MUX_PAD_CTRL(GPI_PAD_CTRL), // GSM On/Off
	MX6_PAD_GPIO_19__GPIO4_IO05      | MUX_PAD_CTRL(GPI_PAD_CTRL), // GSM Reset
	MX6_PAD_EIM_DA10__GPIO3_IO10     | MUX_PAD_CTRL(GPI_PAD_CTRL), // GSM VUSB Enable
	MX6_PAD_NANDF_RB0__GPIO6_IO10    | MUX_PAD_CTRL(GPI_PAD_CTRL), // Communications Inhibit
	MX6_PAD_SD3_DAT2__GPIO7_IO06     | MUX_PAD_CTRL(GPI_PAD_CTRL), // Communication Power Cycle
	MX6_PAD_GPIO_9__GPIO1_IO09       | MUX_PAD_CTRL(GPI_PAD_CTRL), // BT_DISABLE_N
	MX6_PAD_NANDF_D5__GPIO2_IO05     | MUX_PAD_CTRL(GPI_PAD_CTRL), // IO Controller Reset
	MX6_PAD_NANDF_D7__GPIO2_IO07     | MUX_PAD_CTRL(GPI_PAD_CTRL), // EXP_GPIO_2
	MX6_PAD_EIM_CS1__GPIO2_IO24      | MUX_PAD_CTRL(GPI_PAD_CTRL), // GPIO_3 - Test Point
	MX6_PAD_EIM_EB1__GPIO2_IO29      | MUX_PAD_CTRL(GPI_PAD_CTRL), // Iridium On/Off
	MX6_PAD_EIM_D23__GPIO3_IO23      | MUX_PAD_CTRL(GPI_PAD_CTRL), // Capacitor Good GSM
	MX6_PAD_EIM_D27__GPIO3_IO27      | MUX_PAD_CTRL(GPI_PAD_CTRL), // Capacitor Filter GSM
	MX6_PAD_CSI0_DATA_EN__GPIO5_IO20 | MUX_PAD_CTRL(GPI_PAD_CTRL), // Power Fail Status Output for GSM
	MX6_PAD_EIM_A25__GPIO5_IO02      | MUX_PAD_CTRL(GPI_PAD_CTRL), // 5V System Good GSM
	MX6_PAD_EIM_EB0__GPIO2_IO28      | MUX_PAD_CTRL(GPI_PAD_CTRL), // UC GSM Power Monitor
	MX6_PAD_NANDF_D6__GPIO2_IO06     | MUX_PAD_CTRL(GPI_PAD_CTRL), // Iridium Network
	MX6_PAD_EIM_DA15__GPIO3_IO15     | MUX_PAD_CTRL(GPI_PAD_CTRL), // Capacitor Good Iridium
	MX6_PAD_EIM_LBA__GPIO2_IO27      | MUX_PAD_CTRL(GPI_PAD_CTRL), // Capacitor Filter Iridium
	MX6_PAD_EIM_D28__GPIO3_IO28      | MUX_PAD_CTRL(GPI_PAD_CTRL), // Power Fail Status Output for Iridium
	MX6_PAD_EIM_D29__GPIO3_IO29      | MUX_PAD_CTRL(GPI_PAD_CTRL), // 5V System Good Iridium
	MX6_PAD_SD3_DAT3__GPIO7_IO07     | MUX_PAD_CTRL(GPI_PAD_CTRL), // Power Good Communication
	MX6_PAD_EIM_CS0__GPIO2_IO23      | MUX_PAD_CTRL(GPI_PAD_CTRL), // IO IRQ
};

static void setup_iomux_ext_gpios(void)
{
	imx_iomux_v3_setup_multiple_pads(ext_gpios_pads,
					 ARRAY_SIZE(ext_gpios_pads));
}
#endif /* CONFIG_CONSOLE_ENABLE_GPIO */

static iomux_v3_cfg_t const ksz8061_pads[] = {
	/* Micrel KSZ8061 PHY reset */
	MX6_PAD_EIM_OE__GPIO2_IO25 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const usb_rst_pads[] = {
	/* USB hub reset line */
	MX6_PAD_EIM_DA10__GPIO3_IO10 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#ifdef CONFIG_SYS_I2C_MXC
int setup_pmic_voltages_carrierboard(void)
{
#ifdef CONFIG_I2C_MULTI_BUS
	if (i2c_set_bus_num(CONFIG_PMIC_I2C_BUS))
                return -1;
#endif

	if (i2c_probe(CONFIG_PMIC_I2C_ADDR)) {
		printf("ERR: cannot access the PMIC\n");
		return -1;
	}

#if defined(CONFIG_FEC_MXC)
	/* Both NVCC_ENET and NVCC_RGMII come from LDO4 (2.5V) */
	/* Config LDO4 voltages A and B at 2.5V, then enable VLDO4 */
	if (pmic_write_reg(DA9063_VLDO4_A_ADDR, 0x50) ||
	    pmic_write_reg(DA9063_VLDO4_B_ADDR, 0x50) ||
	    pmic_write_bitfield(DA9063_VLDO4_CONT_ADDR, 0x1, 0, 0x1))
		printf("Could not configure VLDO4\n");
#endif
	/* PMIC GPIO11 is the LVDS0 backlight which is low level
	 * enabled. If left with default configuration (input) or when
	 * coming from power-off, the backlight may be enabled and draw
	 * too much power from the 5V source when this source is
	 * enabled, which may cause a voltage drop on the 5V line and
	 * hang the I2C bus where the touch controller is attached.
	 * To prevent this, configure GPIO11 as output and set it
	 * high, to make sure the backlight is disabled when the 5V is
	 * enabled.
	 * This also configures it as active-low when acting as PWM.
	 */
	if (pmic_write_bitfield(DA9063_GPIO10_11_ADDR, 0x3, 4, 0x3))
		printf("Could not configure GPIO11\n");
	if (pmic_write_bitfield(DA9063_GPIO_MODE8_15_ADDR, 0x1, 3, 0x1))
		printf("Could not set GPIO11 high\n");

	/* Similarly, do the same with PMIC_GPIO15 (LVDS1 backlight)
	 * This also configures it as active-low when acting as PWM.
	 */
	if (pmic_write_bitfield(DA9063_GPIO14_15_ADDR, 0x3, 4, 0x3))
		printf("Could not configure GPIO11\n");
	if (pmic_write_bitfield(DA9063_GPIO_MODE8_15_ADDR, 0x1, 7, 0x1))
		printf("Could not set GPIO11 high\n");

	/* PWR_EN on the ccimx6sbc enables the +5V suppy and comes
	 * from PMIC_GPIO7. Set this GPIO high to enable +5V supply.
	 */
	if (pmic_write_bitfield(DA9063_GPIO6_7_ADDR, 0x3, 4, 0x3))
		printf("Could not configure GPIO7\n");
	if (pmic_write_bitfield(DA9063_GPIO_MODE0_7_ADDR, 0x1, 7, 0x1))
		printf("Could not enable PWR_EN\n");

	return 0;
}
#endif /* CONFIG_SYS_I2C_MXC */

static void setup_board_enet(void)
{
	int phy_reset_gpio;

	/* 100BaseT ENET (Micrel PHY) */
	phy_reset_gpio = IMX_GPIO_NR(2, 25);
	phy_addr = CONFIG_ENET_PHYADDR_MICREL;
	imx_iomux_v3_setup_multiple_pads(ksz8061_pads, ARRAY_SIZE(ksz8061_pads));
	/* Assert PHY reset */
	gpio_request(phy_reset_gpio, "ENET PHY Reset");
	gpio_direction_output(phy_reset_gpio , 0);
	/* Need 10ms to guarantee stable voltages */
	udelay(10 * 1000);
	/* Deassert PHY reset */
	gpio_set_value(phy_reset_gpio, 1);
	/* The spec says the FEC needs to wait 100us before
	 * accessing the MIIM (MDC/MDIO). Experimantation has
	 * proved that more time is needed, we use double the
	 * spec requirement */
	udelay(200);
}

int board_get_enet_phy_addr(void)
{
	return phy_addr;
}

static int mx6_rgmii_rework(struct phy_device *phydev)
{
	char *phy_mode;

	/*
	 * Micrel PHY KSZ9031 has four MMD registers to configure the clock skew
	 * of different signals. In U-Boot we're having Ethernet issues on
	 * certain boards which work fine in Linux. We examined these MMD clock
	 * skew registers in Linux which have different values than the reset
	 * defaults:
	 * 			Reset default		Linux
	 * ------------------------------------------------------------------
	 *  Control data pad	0077 (no skew)		0000 (-0.42 ns)
	 *  RX data pad		7777 (no skew)		0000 (-0.42 ns)
	 *  TX data pad		7777 (no skew)		7777 (no skew)
	 *  Clock pad		3def (no skew)		03ff (+0.96 ns)
	 *
	 *  Setting the skews used in Linux solves the issues in U-Boot.
	 */

	/* control data pad skew - devaddr = 0x02, register = 0x04 */
	ksz9031_phy_extended_write(phydev, 0x02,
				   MII_KSZ9031_EXT_RGMII_CTRL_SIG_SKEW,
				   MII_KSZ9031_MOD_DATA_NO_POST_INC, 0x0000);
	/* rx data pad skew - devaddr = 0x02, register = 0x05 */
	ksz9031_phy_extended_write(phydev, 0x02,
				   MII_KSZ9031_EXT_RGMII_RX_DATA_SKEW,
				   MII_KSZ9031_MOD_DATA_NO_POST_INC, 0x0000);
	/* tx data pad skew - devaddr = 0x02, register = 0x05 */
	ksz9031_phy_extended_write(phydev, 0x02,
				   MII_KSZ9031_EXT_RGMII_TX_DATA_SKEW,
				   MII_KSZ9031_MOD_DATA_NO_POST_INC, 0x7777);
	/* gtx and rx clock pad skew - devaddr = 0x02, register = 0x08 */
	ksz9031_phy_extended_write(phydev, 0x02,
				   MII_KSZ9031_EXT_RGMII_CLOCK_SKEW,
				   MII_KSZ9031_MOD_DATA_NO_POST_INC, 0x03ff);

	phy_mode = getenv("phy_mode");
	if (!strcmp("master", phy_mode)) {
		unsigned short reg;

		/*
		 * Micrel PHY KSZ9031 takes up to 5 seconds to autonegotiate
		 * with Gigabit switches. This time can be reduced by forcing
		 * the PHY to work as master during master-slave negotiation.
		 * Forcing master mode may cause autonegotiation to fail if
		 * the other end is also forced as master, or using a direct
		 * cable connection.
		 */
		reg = phy_read(phydev, MDIO_DEVAD_NONE, MII_CTRL1000);
		reg |= MSTSLV_MANCONFIG_ENABLE | MSTSLV_MANCONFIG_MASTER;
		phy_write(phydev, MDIO_DEVAD_NONE, MII_CTRL1000, reg);
	}

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	mx6_rgmii_rework(phydev);
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));
}

/* This comes from ccimx6.c */
#define ENET_NORMAL_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED   |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define ENET_CLK_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_LOW   |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST    | PAD_CTL_HYS)

static iomux_v3_cfg_t const enet_pads_100[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO     | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC       | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	MX6_PAD_ENET_TXD0__ENET_TX_DATA0 | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	MX6_PAD_ENET_TXD1__ENET_TX_DATA1 | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	MX6_PAD_ENET_RXD0__ENET_RX_DATA0 | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	MX6_PAD_ENET_RXD1__ENET_RX_DATA1 | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	MX6_PAD_GPIO_16__ENET_REF_CLK    | MUX_PAD_CTRL(ENET_CLK_PAD_CTRL),
	MX6_PAD_ENET_RX_ER__ENET_RX_ER   | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	MX6_PAD_ENET_TX_EN__ENET_TX_EN   | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	MX6_PAD_ENET_CRS_DV__ENET_RX_EN  | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
};

/* altered from 'setup_iomux_enet' in 'ccimx6.c' */
static void setup_iomux_enet_bell_mm3 ( void )
{
	/* 10/100 ENET */
	setup_iomux_enet();
	imx_iomux_v3_setup_multiple_pads(enet_pads_100, ARRAY_SIZE(enet_pads_100));
}

int board_eth_init(struct bd_info *bis)
{
	if (is_mx6dqp()) {
		int ret;

		debug("Initialising enet board\n");

		/* Ref: IMX6DQ6SDLHDG (Hardware Development Guide) */
		setup_iomux_enet_bell_mm3();
		setup_board_enet();


		/* Get ENET reference clock from external clock (Ethernet PHY)
		 * GPR1[21] = 0
		 */
		imx_iomux_set_gpr_register(1, 21, 1, 0);

		struct anatop_regs __iomem *anatop = (struct anatop_regs __iomem *)ANATOP_BASE_ADDR;
		u32 reg = readl(&anatop->pll_enet);
		s32 timeout = 100000;

		/* Setup the Analog PLLs (Disable PLL) */
		reg &= ~BM_ANADIG_PLL_ENET_DIV_SELECT;

		/* Setup clock rate to 50MHz */
		reg |= BF_ANADIG_PLL_ENET_DIV_SELECT(ENET_50MHZ);

		if ((reg & BM_ANADIG_PLL_ENET_POWERDOWN) ||
			(!(reg & BM_ANADIG_PLL_ENET_LOCK))) {
			reg &= ~BM_ANADIG_PLL_ENET_POWERDOWN;
			writel(reg, &anatop->pll_enet);
			while (timeout--) {
				/* Wait for PLL to lock */
				if (readl(&anatop->pll_enet) & BM_ANADIG_PLL_ENET_LOCK)
					break;
			}
			if (timeout < 0)
				printf("Error fec anatop clock settings, timeout!\n");
		}

		/* Enable FEC clock */
		reg &= ~BM_ANADIG_PLL_ENET_ENABLE;
		reg &= ~BM_ANADIG_PLL_ENET_BYPASS;
		writel(reg, &anatop->pll_enet);
	}

	return cpu_eth_init(bis);
}

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC2_BASE_ADDR:
		ret = 1; /* uSD/uSDHC2 does not connect CD. Assume present */
		break;
	case USDHC4_BASE_ADDR:
		if (board_has_emmc())
			ret = 1;
		break;
	}

	return ret;
}

#ifdef CONFIG_USB_EHCI_MX6
#ifndef CONFIG_DM_USB
int board_ehci_hcd_init(int port)
{
	int usb_rst_gpio = IMX_GPIO_NR(3, 10);

	switch (port) {
	case 0:
		/* USB OTG */
		break;
	case 1:
		/* USB hub reset line IOMUX */
		imx_iomux_v3_setup_multiple_pads(usb_rst_pads,
						ARRAY_SIZE(usb_rst_pads));

		/* Reset USB hub */
		gpio_request(usb_rst_gpio, "USB hub reset");
		gpio_direction_output(usb_rst_gpio, 0);
		mdelay(2);
		gpio_set_value(usb_rst_gpio, 1);
		break;
	default:
		printf("MXC USB port %d not supported\n", port);
		return -EINVAL;
	}

	return 0;
}
#endif
#endif

int board_early_init_f(void)
{
	setup_iomux_uart();

#ifdef CONFIG_CONSOLE_DISABLE
	gd->flags |= (GD_FLG_DISABLE_CONSOLE | GD_FLG_SILENT);
#endif
	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	ccimx6_init();

	board_version = get_carrierboard_version();
	board_id = get_carrierboard_id();

#ifdef CONFIG_CMD_SATA
	setup_iomux_sata();
#endif

	return 0;
}

#define DEVELOPMENT_VERSION (0)

int checkboard(void)
{
	print_ccimx6_info();
	print_carrierboard_info();
	printf("Boot device: %s\n", get_boot_device_name());
	printf("Config:      %s\n", "Bell MM3 Development Board");
	printf("Version:     %d\n", DEVELOPMENT_VERSION);

	return 0;
}

static int board_fixup(void)
{
	/* Mask the CHG_WAKE interrupt. This pin should be grounded
	 * if unused. */
	if (pmic_write_bitfield(DA9063_IRQ_MASK_B_ADDR, 0x1, 0, 0x1)) {
		printf("Failed to mask CHG_WAKE. Spurious wake up events may occur\n");
		return -1;
	}

	if (board_version <= 1) {
		/* Mask the PMIC_GPIO7 interrupt which is N/C on the SBCv1. */
		if (pmic_write_bitfield(DA9063_GPIO6_7_ADDR, 0x1, 0x7, 0x1)) {
			printf("Failed to mask PMIC_GPIO7.\n");
			return -1;
		}
	}

	return 0;
}

void platform_default_environment(void)
{
	char cmd[80];

	som_default_environment();

	/* Set $board_version variable if defined in OTP bits */
	if (board_version > 0) {
		sprintf(cmd, "setenv -f board_version %d", board_version);
		run_command(cmd, 0);
	}

	/* Set $board_id variable if defined in OTP bits */
	if (board_id > 0) {
		sprintf(cmd, "setenv -f board_id %d", board_id);
		run_command(cmd, 0);
	}
}

int board_late_init(void)
{
	int ret;

	int ext_gpios[] =  {
		IMX_GPIO_NR(2, 2),	// 0 - USER_LED0 - Power Indicator
		IMX_GPIO_NR(2, 3),	// 1 - USER_LED1 - Communications
		IMX_GPIO_NR(2, 4),	// 2 - USER_LED2 - Heart Beat

		/* Communications board signals */
		// Output(s)
		IMX_GPIO_NR(6, 10),	// 3 - Communications Inhibit
		IMX_GPIO_NR(7, 6),	// 4 - Communication Power Cycle
		// Input(s)
		IMX_GPIO_NR(7, 7),	// 5 - Power Good Communication

		/* GSM signals */
		// Output(s)
		IMX_GPIO_NR(7, 13),	// 6 - GSM On/Off
		IMX_GPIO_NR(4, 5),	// 7 - GSM Reset
		IMX_GPIO_NR(3, 10),	// 8 - GSM VUSB Enable
		// Input(s)
		IMX_GPIO_NR(3, 23),	//  9 - Capacitor Good GSM
		IMX_GPIO_NR(3, 27),	// 10 - Capacitor Filter GSM
		IMX_GPIO_NR(5, 20),	// 11 - Power Fail Status Output for GSM
		IMX_GPIO_NR(5, 2),	// 12 - 5V System Good GSM
		IMX_GPIO_NR(2, 28),	// 13 - UC GSM Power Monitor

		/* Iridium signals */
		// Output(s)
		IMX_GPIO_NR(2, 29),	// 14 - Iridium On/Off
		// Input(s)
		IMX_GPIO_NR(2, 6),	// 15 - Iridium Network
		IMX_GPIO_NR(3, 15),	// 16 - Capacitor Good Iridium
		IMX_GPIO_NR(2, 27),	// 17 - Capacitor Filter Iridium
		IMX_GPIO_NR(3, 28),	// 18 - Power Fail Status Output for Iridium
		IMX_GPIO_NR(3, 29),	// 19 - 5V System Good Iridium

		/* Misc */
		// Outputs
		IMX_GPIO_NR(2, 5),	// 20 - IO Controller Reset
		IMX_GPIO_NR(2, 7),	// 21 - EXP_GPIO_2
		IMX_GPIO_NR(2, 24),	// 22 - GPIO_3 - Test Point
		IMX_GPIO_NR(1, 9),	// 23 - BT_DISABLE_N
		// Inputs
		IMX_GPIO_NR(2, 23)	// 24 - IO IRQ
	};

	int gpio_pin;
	char gpioTag[32];
	bool gpio_state = true;

	/* Switch on the power LED, and off the other LEDs */
	{
		snprintf(gpioTag, sizeof(gpioTag), "Power LED");
		gpio_pin = ext_gpios[0];
		gpio_request(gpio_pin, gpioTag);
		gpio_direction_output(gpio_pin , 1);
		if ( IS_ERR_VALUE(gpio_get_value(gpio_pin)) )
		{
			printf("Power LED failed\n");
			gpio_state = false;
		}

		for ( int i = 1 ; i < 3 ; i++ )
		{
			snprintf(gpioTag, sizeof(gpioTag), "LED %s", (i==1)?"Comms":"Heart Beat");
			gpio_pin = ext_gpios[i];
			gpio_request(gpio_pin, gpioTag);
			gpio_direction_output(gpio_pin , 0);
			if ( IS_ERR_VALUE(gpio_get_value(gpio_pin)) )
			{
				printf("%s LED failed\n", (i==1)?"Comms":"Heart Beat");
				gpio_state = false;
			}
		}
	}

	/* Communications board signals */
	{
		// Activate communications board
		snprintf(gpioTag, sizeof(gpioTag), "CommsInh");
		gpio_pin = ext_gpios[3];
		gpio_request(gpio_pin, gpioTag);
		gpio_direction_output(gpio_pin , 0);
		if ( IS_ERR_VALUE(gpio_get_value(gpio_pin)) )
		{
			printf("Setting Comms Ihb failed\n");
			gpio_state = false;
		}

		// Pull the Backup Boost Converter pin low
		snprintf(gpioTag, sizeof(gpioTag), "CommsInh");
		gpio_pin = ext_gpios[4];
		gpio_request(gpio_pin, gpioTag);
		gpio_direction_output(gpio_pin , 0);
		if ( IS_ERR_VALUE(gpio_get_value(gpio_pin)) )
		{
			printf("Setting Comms Ihb failed\n");
			gpio_state = false;
		}

		// Set the Power Good Communication pin as input
		snprintf(gpioTag, sizeof(gpioTag), "Power Good Comms");
		gpio_pin = ext_gpios[5];
		gpio_request(gpio_pin, gpioTag);
		gpio_direction_input(gpio_pin);
	}

	/* GSM signals */
	{
		// Activate GSM modem
		// PWR_ON low time to switch-on the module is 50us, max is 80us
		snprintf(gpioTag, sizeof(gpioTag), "GSM_PWR_ON");
		gpio_pin = ext_gpios[6];
		gpio_request(gpio_pin, gpioTag);
		gpio_direction_output(gpio_pin , 1);
		if ( IS_ERR_VALUE(gpio_get_value(gpio_pin)) )
		{
			printf("Setting GSM Power On failed\n");
			gpio_state = false;
		}
		udelay(60);
		gpio_direction_output(gpio_pin , 0);

		// Pull the reset pin high
		snprintf(gpioTag, sizeof(gpioTag), "GSM_RST");
		gpio_pin = ext_gpios[7];
		gpio_request(gpio_pin, gpioTag);
		gpio_direction_output(gpio_pin , 0);
		if ( IS_ERR_VALUE(gpio_get_value(gpio_pin)) )
		{
			printf("Setting GSM reset failed\n");
			gpio_state = false;
		}

		// Pull the GSM USB detect pin high
		snprintf(gpioTag, sizeof(gpioTag), "GSM_USB_EN") ;
		gpio_pin = ext_gpios[8];
		gpio_request(gpio_pin, gpioTag);
		gpio_direction_output(gpio_pin , 0);
		if ( IS_ERR_VALUE(gpio_get_value(gpio_pin)) )
		{
			printf("Setting GSM USB Enable failed\n");
			gpio_state = false;
		}

		// Set other pins as inputs
		for ( int i = 9 ; i < 14 ; i++ )
		{
			snprintf(gpioTag, sizeof(gpioTag), "GSM %d", i) ;
			gpio_pin = ext_gpios[i];
			gpio_request(gpio_pin, gpioTag);
			gpio_direction_input(gpio_pin);
		}
	}

	/* Iridium signals */
	{
		// Activate Iridium modem
		// A high on the On/Off pin switches the modem on
		snprintf(gpioTag, sizeof(gpioTag), "IRI_PWR_ON") ;
		gpio_pin = ext_gpios[14];
		gpio_request(gpio_pin, gpioTag);
		gpio_direction_output(gpio_pin , 1);
		if ( IS_ERR_VALUE(gpio_get_value(gpio_pin)) )
		{
			printf("Setting Iridium Power On failed\n");
			gpio_state = false;
		}

		// Set other pins as inputs
		for ( int i = 15 ; i < 20 ; i++ )
		{
			snprintf(gpioTag, sizeof(gpioTag), "Iridium %d", i) ;
			gpio_pin = ext_gpios[i];
			gpio_request(gpio_pin, gpioTag);
			gpio_direction_input(gpio_pin);
		}
	}

	/* IO Controller signals */
	{
		// Activate IO Controller
		// A high on the IO Controller will reset the TC233
		snprintf(gpioTag, sizeof(gpioTag), "IO_CTRL") ;
		gpio_pin = ext_gpios[20];
		gpio_request(gpio_pin, gpioTag);
		gpio_direction_output(gpio_pin, 1);
		if (IS_ERR_VALUE(gpio_get_value(gpio_pin)))
		{
			printf("Resetting the TC233 failed\n");
			gpio_state = false;
		}
		// Wait 20msec for power to stebilise
		mdelay(20);

		// Release the reset on the TC233
		gpio_direction_output(gpio_pin, 1);
		if (IS_ERR_VALUE(gpio_get_value(gpio_pin)))
		{
			printf("Setting the TC233 failed\n");
			gpio_state = false;
		}

		snprintf(gpioTag, sizeof(gpioTag), "BT_DISABLE_N");
		gpio_pin = ext_gpios[23];
		gpio_request(gpio_pin, gpioTag);
		gpio_direction_output(gpio_pin, 1);
		if ( IS_ERR_VALUE(gpio_get_value(gpio_pin)) )
		{
			printf("Setting Bluetooth Disable failed\n");
			gpio_state = false;
		}
	}

	printf("MM3 GPIOs: %s\n", ((gpio_state)?"Initialised":"Initialisation failed"));

	/* SOM late init */
	ret = ccimx6_late_init();
	if ( !ret )
	{
		ret = board_fixup();
	}
	/* Set default dynamic variables */
	platform_default_environment();

	return ret;
}

#if defined(CONFIG_OF_BOARD_SETUP)
/* Platform function to modify the FDT as needed */
int ft_board_setup(void *blob, bd_t *bd)
{
	fdt_fixup_hwid(blob);
	fdt_fixup_ccimx6(blob);
	fdt_fixup_carrierboard(blob);

	return 0;
}
#endif /* CONFIG_OF_BOARD_SETUP */

/* board specific configuration for spurious wakeup */
void board_spurious_wakeup(void)
{
	/* Disable the 5V regulator on the ccimx6sbc before going
	 * to power down
	 */
	if (pmic_write_bitfield(DA9063_GPIO_MODE0_7_ADDR, 0x1, 7, 0x0))
		printf("Could not disable PWR_EN\n");
}
