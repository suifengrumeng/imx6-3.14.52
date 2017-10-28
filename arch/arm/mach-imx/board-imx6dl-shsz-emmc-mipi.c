/*
 * Copyright 2011-2015 Freescale Semiconductor, Inc.
 * Copyright 2011 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/can/platform/flexcan.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pm_opp.h>
#include <linux/pci.h>
#include <linux/phy.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/micrel_phy.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/of_net.h>
#include <linux/fec.h>
#include <linux/netdevice.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/system_misc.h>

#include "common.h"
#include "cpuidle.h"
#include "hardware.h"

static struct fec_platform_data fec_pdata;
static struct flexcan_platform_data flexcan_pdata[2];
static int flexcan_en_gpio;
static int flexcan_stby_gpio;
static int flexcan0_en;
static int flexcan1_en;

static void imx6q_fec_sleep_enable(int enabled)
{
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr)) {
		if (enabled)
			regmap_update_bits(gpr, IOMUXC_GPR13,
					   IMX6Q_GPR13_ENET_STOP_REQ,
					   IMX6Q_GPR13_ENET_STOP_REQ);
		else
			regmap_update_bits(gpr, IOMUXC_GPR13,
					   IMX6Q_GPR13_ENET_STOP_REQ, 0);
	} else
		pr_err("failed to find fsl,imx6q-iomux-gpr regmap\n");
}

static void __init imx6q_enet_plt_init(void)
{
	struct device_node *np;

	np = of_find_node_by_path("/soc/aips-bus@02100000/ethernet@02188000");
	if (np && of_get_property(np, "fsl,magic-packet", NULL))
		fec_pdata.sleep_mode_enable = imx6q_fec_sleep_enable;
}

static void mx6q_flexcan_switch(void)
{
	if (flexcan0_en || flexcan1_en) {
		/*
		 * The transceiver TJA1041A on sabreauto RevE baseboard will
		 * fail to transit to Normal state if EN/STBY is high by default
		 * after board power up. So we set the EN/STBY initial state to low
		 * first then to high to guarantee the state transition successfully.
		 */
		gpio_set_value_cansleep(flexcan_en_gpio, 0);
		gpio_set_value_cansleep(flexcan_stby_gpio, 0);

		gpio_set_value_cansleep(flexcan_en_gpio, 1);
		gpio_set_value_cansleep(flexcan_stby_gpio, 1);
	} else {
		/*
		 * avoid to disable CAN xcvr if any of the CAN interfaces
		 * are down. XCRV will be disabled only if both CAN2
		 * interfaces are DOWN.
		*/
		gpio_set_value_cansleep(flexcan_en_gpio, 0);
		gpio_set_value_cansleep(flexcan_stby_gpio, 0);
	}
}

static void imx6q_flexcan0_switch_auto(int enable)
{
	flexcan0_en = enable;
	mx6q_flexcan_switch();
}

static void imx6q_flexcan1_switch_auto(int enable)
{
	flexcan1_en = enable;
	mx6q_flexcan_switch();
}

static int __init imx6q_flexcan_fixup_auto(void)
{
	struct device_node *np;

	np = of_find_node_by_path("/soc/aips-bus@02000000/can@02090000");
	if (!np)
		return -ENODEV;

	flexcan_en_gpio = of_get_named_gpio(np, "trx-en-gpio", 0);
	flexcan_stby_gpio = of_get_named_gpio(np, "trx-stby-gpio", 0);
	if (gpio_is_valid(flexcan_en_gpio) && gpio_is_valid(flexcan_stby_gpio) &&
		!gpio_request_one(flexcan_en_gpio, GPIOF_DIR_OUT, "flexcan-trx-en") &&
		!gpio_request_one(flexcan_stby_gpio, GPIOF_DIR_OUT, "flexcan-trx-stby")) {
		/* flexcan 0 & 1 are using the same GPIOs for transceiver */
		flexcan_pdata[0].transceiver_switch = imx6q_flexcan0_switch_auto;
		flexcan_pdata[1].transceiver_switch = imx6q_flexcan1_switch_auto;
	}

	return 0;
}

/* For imx6q sabrelite board: set KSZ9021RN RGMII pad skew */
static int ksz9021rn_phy_fixup(struct phy_device *phydev)
{
	if (IS_BUILTIN(CONFIG_PHYLIB)) {
		/* min rx data delay */
		phy_write(phydev, MICREL_KSZ9021_EXTREG_CTRL,
			0x8000 | MICREL_KSZ9021_RGMII_RX_DATA_PAD_SCEW);
		phy_write(phydev, MICREL_KSZ9021_EXTREG_DATA_WRITE, 0x0000);

		/* max rx/tx clock delay, min rx/tx control delay */
		phy_write(phydev, MICREL_KSZ9021_EXTREG_CTRL,
			0x8000 | MICREL_KSZ9021_RGMII_CLK_CTRL_PAD_SCEW);
		phy_write(phydev, MICREL_KSZ9021_EXTREG_DATA_WRITE, 0xf0f0);
		phy_write(phydev, MICREL_KSZ9021_EXTREG_CTRL,
			MICREL_KSZ9021_RGMII_CLK_CTRL_PAD_SCEW);
	}

	return 0;
}

static void mmd_write_reg(struct phy_device *dev, int device, int reg, int val)
{
	phy_write(dev, 0x0d, device);
	phy_write(dev, 0x0e, reg);
	phy_write(dev, 0x0d, (1 << 14) | device);
	phy_write(dev, 0x0e, val);
}

static int ksz9031rn_phy_fixup(struct phy_device *dev)
{
	/*
	 * min rx data delay, max rx/tx clock delay,
	 * min rx/tx control delay
	 */
	mmd_write_reg(dev, 2, 4, 0);
	mmd_write_reg(dev, 2, 5, 0);
	mmd_write_reg(dev, 2, 8, 0x003ff);

	return 0;
}

/*
 * fixup for PLX PEX8909 bridge to configure GPIO1-7 as output High
 * as they are used for slots1-7 PERST#
 */
static void ventana_pciesw_early_fixup(struct pci_dev *dev)
{
	u32 dw;

	if (!of_machine_is_compatible("gw,ventana"))
		return;

	if (dev->devfn != 0)
		return;

	pci_read_config_dword(dev, 0x62c, &dw);
	dw |= 0xaaa8; // GPIO1-7 outputs
	pci_write_config_dword(dev, 0x62c, dw);

	pci_read_config_dword(dev, 0x644, &dw);
	dw |= 0xfe;   // GPIO1-7 output high
	pci_write_config_dword(dev, 0x644, dw);

	msleep(100);
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_PLX, 0x8609, ventana_pciesw_early_fixup);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_PLX, 0x8606, ventana_pciesw_early_fixup);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_PLX, 0x8604, ventana_pciesw_early_fixup);

static int ar8031_phy_fixup(struct phy_device *dev)
{
	u16 val;

	/* Set RGMII IO voltage to 1.8V */
	phy_write(dev, 0x1d, 0x1f);
	phy_write(dev, 0x1e, 0x8);

	/* disable phy AR8031 SmartEEE function. */
	phy_write(dev, 0xd, 0x3);
	phy_write(dev, 0xe, 0x805d);
	phy_write(dev, 0xd, 0x4003);
	val = phy_read(dev, 0xe);
	val &= ~(0x1 << 8);
	phy_write(dev, 0xe, val);

	/* To enable AR8031 output a 125MHz clk from CLK_25M */
	phy_write(dev, 0xd, 0x7);
	phy_write(dev, 0xe, 0x8016);
	phy_write(dev, 0xd, 0x4007);

	val = phy_read(dev, 0xe);
	val &= 0xffe3;
	val |= 0x18;
	phy_write(dev, 0xe, val);

	/* introduce tx clock delay */
	phy_write(dev, 0x1d, 0x5);
	val = phy_read(dev, 0x1e);
	val |= 0x0100;
	phy_write(dev, 0x1e, val);

	return 0;
}

#define PHY_ID_AR8031	0x004dd074

static int ar8035_phy_fixup(struct phy_device *dev)
{
	u16 val;

	/* Ar803x phy SmartEEE feature cause link status generates glitch,
	 * which cause ethernet link down/up issue, so disable SmartEEE
	 */
	phy_write(dev, 0xd, 0x3);
	phy_write(dev, 0xe, 0x805d);
	phy_write(dev, 0xd, 0x4003);

	val = phy_read(dev, 0xe);
	phy_write(dev, 0xe, val & ~(1 << 8));

	/*
	 * Enable 125MHz clock from CLK_25M on the AR8031.  This
	 * is fed in to the IMX6 on the ENET_REF_CLK (V22) pad.
	 * Also, introduce a tx clock delay.
	 *
	 * This is the same as is the AR8031 fixup.
	 */
	ar8031_phy_fixup(dev);

	/*check phy power*/
	val = phy_read(dev, 0x0);
	if (val & BMCR_PDOWN)
		phy_write(dev, 0x0, val & ~BMCR_PDOWN);

	return 0;
}

#define PHY_ID_AR8035 0x004dd072

static void __init imx6q_enet_phy_init(void)
{
	if (IS_BUILTIN(CONFIG_PHYLIB)) {
		phy_register_fixup_for_uid(PHY_ID_KSZ9021, MICREL_PHY_ID_MASK,
				ksz9021rn_phy_fixup);
		phy_register_fixup_for_uid(PHY_ID_KSZ9031, MICREL_PHY_ID_MASK,
				ksz9031rn_phy_fixup);
		phy_register_fixup_for_uid(PHY_ID_AR8031, 0xffffffff,
				ar8031_phy_fixup);
		phy_register_fixup_for_uid(PHY_ID_AR8035, 0xffffffef,
				ar8035_phy_fixup);
	}
}

static void __init imx6q_1588_init(void)
{
	struct device_node *np;
	struct clk *ptp_clk;
	struct regmap *gpr;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-fec");
	if (!np) {
		pr_warn("%s: failed to find fec node\n", __func__);
		return;
	}

	ptp_clk = of_clk_get(np, 2);
	if (IS_ERR(ptp_clk)) {
		pr_warn("%s: failed to get ptp clock\n", __func__);
		goto put_node;
	}

	/*
	 * If enet_ref from ANATOP/CCM is the PTP clock source, we need to
	 * set bit IOMUXC_GPR1[21].  Or the PTP clock must be from pad
	 * (external OSC), and we need to clear the bit.
	 */
	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr))
		regmap_update_bits(gpr, IOMUXC_GPR1,
				IMX6Q_GPR1_ENET_CLK_SEL_MASK,
				IMX6Q_GPR1_ENET_CLK_SEL_ANATOP);
	else
		pr_err("failed to find fsl,imx6q-iomux-gpr regmap\n");

	clk_put(ptp_clk);
put_node:
	of_node_put(np);
}

static void __init imx6q_csi_mux_init(void)
{
	/*
	 * MX6Q SabreSD board:
	 * IPU1 CSI0 connects to parallel interface.
	 * Set GPR1 bit 19 to 0x1.
	 *
	 * MX6DL SabreSD board:
	 * IPU1 CSI0 connects to parallel interface.
	 * Set GPR13 bit 0-2 to 0x4.
	 * IPU1 CSI1 connects to MIPI CSI2 virtual channel 1.
	 * Set GPR13 bit 3-5 to 0x1.
	 */
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr)) {
		if (of_machine_is_compatible("fsl,imx6q-sabresd") ||
			of_machine_is_compatible("fsl,imx6q-sabreauto"))
			regmap_update_bits(gpr, IOMUXC_GPR1, 1 << 19, 1 << 19);
		else if (of_machine_is_compatible("fsl,imx6dl-sabresd") ||
			 of_machine_is_compatible("fsl,imx6dl-sabreauto"))
			regmap_update_bits(gpr, IOMUXC_GPR13, 0x3F, 0x0C);
	} else {
		pr_err("%s(): failed to find fsl,imx6q-iomux-gpr regmap\n",
		       __func__);
	}
}

static void __init imx6q_enet_clk_sel(void)
{
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr))
		regmap_update_bits(gpr, IOMUXC_GPR5,
				   IMX6Q_GPR5_ENET_TX_CLK_SEL, IMX6Q_GPR5_ENET_TX_CLK_SEL);
	else
		pr_err("failed to find fsl,imx6q-iomux-gpr regmap\n");
}

static inline void imx6q_enet_init(void)
{
	imx6_enet_mac_init("fsl,imx6q-fec");
	imx6q_enet_phy_init();
	imx6q_1588_init();
	if (cpu_is_imx6q() && imx_get_soc_revision() == IMX_CHIP_REVISION_2_0)
		imx6q_enet_clk_sel();
	imx6q_enet_plt_init();
}

/* Add auxdata to pass platform data */
static const struct of_dev_auxdata imx6q_auxdata_lookup[] __initconst = {
	OF_DEV_AUXDATA("fsl,imx6q-flexcan", 0x02090000, NULL, &flexcan_pdata[0]),
	OF_DEV_AUXDATA("fsl,imx6q-flexcan", 0x02094000, NULL, &flexcan_pdata[1]),
	OF_DEV_AUXDATA("fsl,imx6q-fec", 0x02188000, NULL, &fec_pdata),
	{ /* sentinel */ }
};

static void __init imx6q_init_machine(void)
{
	struct device *parent;

	if (cpu_is_imx6q() && imx_get_soc_revision() == IMX_CHIP_REVISION_2_0)
		imx_print_silicon_rev("i.MX6QP", IMX_CHIP_REVISION_1_0);
	else
		imx_print_silicon_rev(cpu_is_imx6dl() ? "i.MX6DL" : "i.MX6Q",
				 imx_get_soc_revision());

	mxc_arch_reset_init_dt();

	parent = imx_soc_device_init();
	if (parent == NULL)
		pr_warn("failed to initialize soc device\n");

	of_platform_populate(NULL, of_default_bus_match_table,
					imx6q_auxdata_lookup, parent);

	imx6q_enet_init();
	imx_anatop_init();
	imx6q_csi_mux_init();
	cpu_is_imx6q() ?  imx6q_pm_init() : imx6dl_pm_init();
}

#define OCOTP_CFG3			0x440
#define OCOTP_CFG3_SPEED_SHIFT		16
#define OCOTP_CFG3_SPEED_1P2GHZ		0x3
#define OCOTP_CFG3_SPEED_996MHZ		0x2
#define OCOTP_CFG3_SPEED_852MHZ		0x1

static void __init imx6q_opp_check_speed_grading(struct device *cpu_dev)
{
	struct device_node *np;
	void __iomem *base;
	u32 val;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-ocotp");
	if (!np) {
		pr_warn("failed to find ocotp node\n");
		return;
	}

	base = of_iomap(np, 0);
	if (!base) {
		pr_warn("failed to map ocotp\n");
		goto put_node;
	}

	/*
	 * SPEED_GRADING[1:0] defines the max speed of ARM:
	 * 2b'11: 1200000000Hz;
	 * 2b'10: 996000000Hz;
	 * 2b'01: 852000000Hz; -- i.MX6Q Only, exclusive with 996MHz.
	 * 2b'00: 792000000Hz;
	 * We need to set the max speed of ARM according to fuse map.
	 */
	val = readl_relaxed(base + OCOTP_CFG3);
	val >>= OCOTP_CFG3_SPEED_SHIFT;
	val &= 0x3;

	if (val != OCOTP_CFG3_SPEED_1P2GHZ)
		if (dev_pm_opp_disable(cpu_dev, 1200000000))
			pr_warn("failed to disable 1.2 GHz OPP\n");
	if (val < OCOTP_CFG3_SPEED_996MHZ)
		if (dev_pm_opp_disable(cpu_dev, 996000000))
			pr_warn("failed to disable 996 MHz OPP\n");
	if (cpu_is_imx6q()) {
		if (val != OCOTP_CFG3_SPEED_852MHZ)
			if (dev_pm_opp_disable(cpu_dev, 852000000))
				pr_warn("failed to disable 852 MHz OPP\n");
	}

	if (IS_ENABLED(CONFIG_MX6_VPU_352M)) {
		if (dev_pm_opp_disable(cpu_dev, 396000000))
			pr_warn("failed to disable 396MHz OPP\n");
		pr_info("remove 396MHz OPP for VPU running at 352MHz!\n");
	}

put_node:
	of_node_put(np);
}

static void __init imx6q_opp_init(void)
{
	struct device_node *np;
	struct device *cpu_dev = get_cpu_device(0);

	if (!cpu_dev) {
		pr_warn("failed to get cpu0 device\n");
		return;
	}
	np = of_node_get(cpu_dev->of_node);
	if (!np) {
		pr_warn("failed to find cpu0 node\n");
		return;
	}

	if (of_init_opp_table(cpu_dev)) {
		pr_warn("failed to init OPP table\n");
		goto put_node;
	}

	imx6q_opp_check_speed_grading(cpu_dev);

put_node:
	of_node_put(np);
}

static struct platform_device imx6q_cpufreq_pdev = {
	.name = "imx6q-cpufreq",
};






static void SPI_Send_Data(unsigned int dat)
{
	unsigned int i, n = 0x0;

	n |= (0x72 << 16);
	dat &= 0xffff;
	n |= dat;

	//SPI_CS(0);
	gpio_set_value(SPI_CS, 0);
	udelay(10);

	for (i = 0; i < 24; i++)	//data
	{
		if (n & 0x800000)
			//SPI_DO(1);
			gpio_set_value(SPI_DO, 1);
		else
			//SPI_DO(0);
			gpio_set_value(SPI_DO, 0);
		n <<= 1;
		//SPI_CLK(0);
		gpio_set_value(SPI_CLK, 0);
		udelay(10);
		//SPI_CLK(1);
		gpio_set_value(SPI_CLK, 1);
		udelay(10);
	}
	//SPI_CS(1);
	gpio_set_value(SPI_CS, 1);
}
static unsigned int SPI_Read_Data(void)
{
	unsigned int i, n = 0, val = 0;

	n = 0x73;

	//SPI_CS(0);
	gpio_set_value(SPI_CS, 0);
	udelay(10);

	for (i = 0; i < 8; i++)
	{
		if (n & 0x80)
			//SPI_DO(1);
			gpio_set_value(SPI_DO, 1);
		else
			//SPI_DO(0);
			gpio_set_value(SPI_DO, 0);
		n <<= 1;
		//SPI_CLK(0);
		gpio_set_value(SPI_CLK, 0);
		udelay(10);
		//SPI_CLK(1);
		gpio_set_value(SPI_CLK, 1);
		udelay(10);
	}
	udelay(1);
	for (i = 0; i < 16; i++)
	{
		//SPI_CLK(1);
		gpio_set_value(SPI_CLK, 1);
		udelay(10);
		//SPI_CLK(0);
		gpio_set_value(SPI_CLK, 0);
		val <<= 1;
		//if (SPI_DI)
		if (gpio_get_value(SPI_DI))
			val |= 1;
		udelay(10);
	}
	//SPI_CS(1);
	gpio_set_value(SPI_CS, 1);
	return val;
}
static void SPI_Send_Cmd(unsigned int cmd)
{
	unsigned int i, n = 0;

	n |= (0x70 << 16);
	cmd &= 0xffff;
	n |= cmd;

	//SPI_CS(0);
	gpio_set_value(SPI_CS, 0);
	udelay(10);

	for (i = 0; i < 24; i++)	//cmd
	{
		if (n & 0x800000)
			//SPI_DO(1);
			gpio_set_value(SPI_DO, 1);
		else
			//SPI_DO(0);
			gpio_set_value(SPI_DO, 0);
		n <<= 1;
		//SPI_CLK(0);
		gpio_set_value(SPI_CLK, 0);
		udelay(10);
		//SPI_CLK(1);
		gpio_set_value(SPI_CLK, 1);
		udelay(10);
	}
	//SPI_CS(1);
	gpio_set_value(SPI_CS, 1);
}
void ssd2805_write(unsigned int number)
{
	SPI_Send_Cmd(0xb7);
	if (number == 1)
		SPI_Send_Data(0x0343);
	else
		SPI_Send_Data(0x0650);


	SPI_Send_Cmd(0xbd);
	SPI_Send_Data(0x0000);

	SPI_Send_Cmd(0xbc);
	SPI_Send_Data(number);

	SPI_Send_Cmd(0xbf);
}

unsigned int ssd2805_read(unsigned int reg)
{
	unsigned int ret;
	SPI_Send_Cmd(0xb7);
	SPI_Send_Data(0x00382);  //0250 !!!!!!!

	SPI_Send_Cmd(0xbb);
	SPI_Send_Data(0x0004);

	SPI_Send_Cmd(0xc1);
	SPI_Send_Data(0x0001);

	SPI_Send_Cmd(0xc0);
	SPI_Send_Data(0x0001);

	SPI_Send_Cmd(0xbc);
	SPI_Send_Data(1);

	SPI_Send_Cmd(0xbf);
	SPI_Send_Data(reg);

	//udelay(5);
	udelay(10);

	SPI_Send_Cmd(0xff);
	ret = SPI_Read_Data();

	return ret;
}

static void SPI_Recv_Cmd(unsigned int cmd)
{
	unsigned int i, n = 0;

	SPI_Send_Cmd(cmd);

	n |= (0x73 << 16);
	cmd &= 0xffff;
	n |= cmd;

	//SPI_CS(0);
	gpio_set_value(SPI_CS, 0);
	udelay(10);

	for (i = 0; i < 8; i++)	//cmd
	{
		if (n & 0x800000)
			//SPI_DO(1);
			gpio_set_value(SPI_DO, 1);
		else
			//SPI_DO(0);
			gpio_set_value(SPI_DO, 0);
		n <<= 1;
		//SPI_CLK(0);
		gpio_set_value(SPI_CLK, 0);
		udelay(10);
		//SPI_CLK(1);
		gpio_set_value(SPI_CLK, 1);
		udelay(10);
	}

	n = 0;
	for (i = 0; i < 16; i++)	//data
	{
		if (gpio_get_value(SPI_DI))
		{
			n |= (1 << i);
		}
		//SPI_CLK(0);
		gpio_set_value(SPI_CLK, 0);
		udelay(10);
		//SPI_CLK(1);
		gpio_set_value(SPI_CLK, 1);
		udelay(10);
	}
	//SPI_CS(1);
	gpio_set_value(SPI_CS, 1);
	printk("---->Device Identification Register: %x= %x\n", cmd, n);
}


static void ILI9488_LG35_Reset(void) 
{
	// VCI=2.8V 
	/************* Reset LCD Driver ***************
	LCD_nRESET = 1; 
	mdelay(1); // Delay 1ms 
	LCD_nRESET = 0; 
	mdelay(10); // Delay 10ms // This delay time is necessary 
	LCD_nRESET = 1; 
	mdelay(120); // Delay 120 ms 
	 */

	if (gpio_request(ILI9488_LG35_GPIO_RESET, "lcd reset\n") < 0)
	{
		pr_err("Failed to request gpio for lcd reset");
		return;
	}
	gpio_direction_output(ILI9488_LG35_GPIO_RESET, 1);
	mdelay(1); // Delay 1ms 
	gpio_set_value(ILI9488_LG35_GPIO_RESET, 0);
	mdelay(10); //  Delay 10ms // This delay time is necessary 
	gpio_set_value(ILI9488_LG35_GPIO_RESET, 1);
	mdelay(120); // Delay 120 ms 
}


static void TFT_SSD2805_init_code(void) 
{
	ILI9488_LG35_Reset();

	SPI_Send_Cmd(0xb9);
	SPI_Send_Data(0x0000);	//disable PLL


	SPI_Send_Cmd(0xba);		//422
	SPI_Send_Data(0x822c);

	SPI_Send_Cmd(0xbb);		//LP clock = fout/LPD/8 = fout/6/8
	SPI_Send_Data(0x0006);

	SPI_Send_Cmd(0xb9);		//enable PLL
	SPI_Send_Data(0x0001);

	mdelay(4);

	SPI_Send_Cmd(0xb1);
	//	SPI_Send_Data((VPW << 8 )|HPS);
	SPI_Send_Data(0x0104);

	SPI_Send_Cmd(0xb2);
	//	SPI_Send_Data(((VPW+FPS))|(HPS+LPS));
	SPI_Send_Data(0x2518);

	SPI_Send_Cmd(0xb3);
	//	SPI_Send_Data((FPS)|LPS);
	SPI_Send_Data(0x1416);

	SPI_Send_Cmd(0xb4);
	//SPI_Send_Data(COL);
	SPI_Send_Data(480);

	SPI_Send_Cmd(0xb5);
	//SPI_Send_Data(ROW);
	SPI_Send_Data(854);

	SPI_Send_Cmd(0xde);		//2 Lane mode
	SPI_Send_Data(0x0001);

	SPI_Send_Cmd(0xb8);     //VC setting
	SPI_Send_Data(0x0000);
	/**************************** add ***********************/
	//	SPI_Send_Cmd(0xc4);
	//	SPI_Send_Data(0x0001);

	//SPI_Send_Cmd(0xca);
	//SPI_Send_Data(0x2003);

	SPI_Send_Cmd(0xcb);
	SPI_Send_Data(0x1a1f);

	SPI_Send_Cmd(0xcc);
	SPI_Send_Data(0x1a1f);

	SPI_Send_Cmd(0xbc);
	SPI_Send_Data(0x0000);

	SPI_Send_Cmd(0xbd);
	SPI_Send_Data(0x0000);

	SPI_Send_Cmd(0xc9);
	SPI_Send_Data(0x2302);
	mdelay(100);

	SPI_Send_Cmd(0xb7);
	SPI_Send_Data(0x0210);

	SPI_Send_Cmd(0xbc);
	SPI_Send_Data(0x0002);

	SPI_Send_Cmd(0xbd);
	SPI_Send_Data(0x0000);

	SPI_Send_Cmd(0xbe);
	SPI_Send_Data(0x021c);

	SPI_Send_Cmd(0xd6);
	SPI_Send_Data(0x0004);		//RGB


	SPI_Send_Cmd(0xb6);
	SPI_Send_Data(0x000b);


	/***************************** add ************************/
	ssd2805_write(6);
	SPI_Send_Data(0xFFFF);
	//	SPI_Send_Data(0xFF);
	SPI_Send_Data(0x0698);
	//	SPI_Send_Data(0x06);
	SPI_Send_Data(0x0104);
	//	SPI_Send_Data(0x01);
	ssd2805_write(2); SPI_Send_Data(0x1808);// SPI_Send_Data(0x18);                // output SDA
	ssd2805_write(2); SPI_Send_Data(0x0121);// SPI_Send_Data(0x01);                // DE = 1 Active  Display Function Control

	ssd2805_write(2); SPI_Send_Data(0x0130);// SPI_Send_Data(0x01);                // 480 X 854
	ssd2805_write(2); SPI_Send_Data(0x0031);// SPI_Send_Data(0x00);                // Column  inversion

	ssd2805_write(2); SPI_Send_Data(0x8750);// SPI_Send_Data(0x87);                // VGMP
	ssd2805_write(2); SPI_Send_Data(0x8751);// SPI_Send_Data(0x87);                // VGMN
	ssd2805_write(2); SPI_Send_Data(0x0760);// SPI_Send_Data(0x07);                // SDTI
	ssd2805_write(2); SPI_Send_Data(0x0061);// SPI_Send_Data(0x00);               // CRTI
	ssd2805_write(2); SPI_Send_Data(0x0762);// SPI_Send_Data(0x07);                // EQTI
	ssd2805_write(2); SPI_Send_Data(0x0063);// SPI_Send_Data(0x00);               // PCTI
	ssd2805_write(2); SPI_Send_Data(0x1540);// SPI_Send_Data(0x15);                // VGH/VGL
	ssd2805_write(2); SPI_Send_Data(0x5541);// SPI_Send_Data(0x55);                // DDVDH/DDVDL  Clamp
	ssd2805_write(2); SPI_Send_Data(0x0342);// SPI_Send_Data(0x03);                // VGH/VGL
	ssd2805_write(2); SPI_Send_Data(0x8a43);// SPI_Send_Data(0x8a);                // VGH Clamp 16V
	ssd2805_write(2); SPI_Send_Data(0x8644);// SPI_Send_Data(0x86);                // VGL Clamp -10V
	ssd2805_write(2); SPI_Send_Data(0x5546);// SPI_Send_Data(0x55);
	ssd2805_write(2); SPI_Send_Data(0x0023);// SPI_Send_Data(0x00);

	ssd2805_write(2); SPI_Send_Data(0x0753);// SPI_Send_Data(0x07);                  //Flicker 10min-02 3min-7
											//	printk("ssd2828 read 0x08 = 0x%x\n", ssd2805_read(0x08));
											//	printk("ssd2828 read 0x21 = 0x%x\n", ssd2805_read(0x21));
											//	printk("ssd2828 read 0x30 = 0x%x\n", ssd2805_read(0x30));
											//	printk("ssd2828 read 0x31 = 0x%x\n", ssd2805_read(0x31));
											//	printk("ssd2828 read 0x50 = 0x%x\n", ssd2805_read(0x50));
											//	printk("ssd2828 read 0x51 = 0x%x\n", ssd2805_read(0x51));
											//	printk("ssd2828 read 0x60 = 0x%x\n", ssd2805_read(0x60));
											//	printk("ssd2828 read 0x61 = 0x%x\n", ssd2805_read(0x61));
											//	printk("ssd2828 read 0x62 = 0x%x\n", ssd2805_read(0x62));
											//	printk("ssd2828 read 0x63 = 0x%x\n", ssd2805_read(0x63));
											//	printk("ssd2828 read 0x40 = 0x%x\n", ssd2805_read(0x40));
											//	printk("ssd2828 read 0x41 = 0x%x\n", ssd2805_read(0x41));
											//	printk("ssd2828 read 0x42 = 0x%x\n", ssd2805_read(0x42));
											//	printk("ssd2828 read 0x43 = 0x%x\n", ssd2805_read(0x43));
											//	printk("ssd2828 read 0x44 = 0x%x\n", ssd2805_read(0x44));
											//	printk("ssd2828 read 0x46 = 0x%x\n", ssd2805_read(0x46));
											//	printk("ssd2828 read 0x23 = 0x%x\n", ssd2805_read(0x23));
											//++++++++++++++++++ Gamma Setting ++++++++++++++++++//
	ssd2805_write(6);
	SPI_Send_Data(0xFFFF);
	//	SPI_Send_Data(0xFF);
	SPI_Send_Data(0x0698);
	//	SPI_Send_Data(0x06);
	SPI_Send_Data(0x0104);
	//	SPI_Send_Data(0x01);     // Change to Page 1
	ssd2805_write(2); SPI_Send_Data(0x00A0);// SPI_Send_Data(0x00);  // Gamma 0
	ssd2805_write(2); SPI_Send_Data(0x10A1);// SPI_Send_Data(0x10);  // Gamma 4
	ssd2805_write(2); SPI_Send_Data(0x1AA2);// SPI_Send_Data(0x1a);  // Gamma 8
	ssd2805_write(2); SPI_Send_Data(0x0FA3);// SPI_Send_Data(0x0f);  // Gamma 16
	ssd2805_write(2); SPI_Send_Data(0x08A4);// SPI_Send_Data(0x08);  // Gamma 24
	ssd2805_write(2); SPI_Send_Data(0x0EA5);// SPI_Send_Data(0x0e);  // Gamma 52
	ssd2805_write(2); SPI_Send_Data(0x08A6);// SPI_Send_Data(0x08);  // Gamma 80
	ssd2805_write(2); SPI_Send_Data(0x06A7);// SPI_Send_Data(0x06);  // Gamma 108
	ssd2805_write(2); SPI_Send_Data(0x08A8);// SPI_Send_Data(0x08);  // Gamma 147
	ssd2805_write(2); SPI_Send_Data(0x0BA9);// SPI_Send_Data(0x0b);  // Gamma 175
	ssd2805_write(2); SPI_Send_Data(0x10AA);// SPI_Send_Data(0x10);  // Gamma 203
	ssd2805_write(2); SPI_Send_Data(0x07AB);// SPI_Send_Data(0x07);  // Gamma 231
	ssd2805_write(2); SPI_Send_Data(0x0DAC);// SPI_Send_Data(0x0D);  // Gamma 239
	ssd2805_write(2); SPI_Send_Data(0x12AD);// SPI_Send_Data(0x12);  // Gamma 247
	ssd2805_write(2); SPI_Send_Data(0x0BAE);// SPI_Send_Data(0x0b);  // Gamma 251
	ssd2805_write(2); SPI_Send_Data(0x07AF);// SPI_Send_Data(0x07);  // Gamma 255
											///==============Nagitive
	ssd2805_write(2); SPI_Send_Data(0x04C0);// SPI_Send_Data(0x04);  // Gamma 0    255
	ssd2805_write(2); SPI_Send_Data(0x10C1);// SPI_Send_Data(0x10);  // Gamma 4    251
	ssd2805_write(2); SPI_Send_Data(0x1aC2);// SPI_Send_Data(0x1a);  // Gamma 8    247
	ssd2805_write(2); SPI_Send_Data(0x0fC3);// SPI_Send_Data(0x0f);  // Gamma 16   239
	ssd2805_write(2); SPI_Send_Data(0x08C4);// SPI_Send_Data(0x08);  // Gamma 24   231
	ssd2805_write(2); SPI_Send_Data(0x0eC5);// SPI_Send_Data(0x0e);  // Gamma 52   203
	ssd2805_write(2); SPI_Send_Data(0x08C6);// SPI_Send_Data(0x08);  // Gamma 80   175
	ssd2805_write(2); SPI_Send_Data(0x06C7);// SPI_Send_Data(0x06);  // Gamma 108  147
	ssd2805_write(2); SPI_Send_Data(0x08C8);// SPI_Send_Data(0x08);  // Gamma 147  108
	ssd2805_write(2); SPI_Send_Data(0x0bC9);// SPI_Send_Data(0x0b);  // Gamma 175  80
	ssd2805_write(2); SPI_Send_Data(0x10CA);// SPI_Send_Data(0x10);  // Gamma 203  52
	ssd2805_write(2); SPI_Send_Data(0x07CB);// SPI_Send_Data(0x07);  // Gamma 231  24
	ssd2805_write(2); SPI_Send_Data(0x0DCC);// SPI_Send_Data(0x0D);  // Gamma 239  16
	ssd2805_write(2); SPI_Send_Data(0x12CD);// SPI_Send_Data(0x12);  // Gamma 247  8
	ssd2805_write(2); SPI_Send_Data(0x0bCE);// SPI_Send_Data(0x0b);  // Gamma 251  4
	ssd2805_write(2); SPI_Send_Data(0x00CF);// SPI_Send_Data(0x00);  // Gamma 255  0

											//*************************************************************************
											//****************************** Page 6 Command ***************************
											//*************************************************************************
	ssd2805_write(6);
	SPI_Send_Data(0xFFFF);
	//	SPI_Send_Data(0xFF);
	SPI_Send_Data(0x0698);
	//	SPI_Send_Data(0x06);
	SPI_Send_Data(0x0604);
	//	SPI_Send_Data(0x06);     // Change to Page 6
	ssd2805_write(2); SPI_Send_Data(0x2100);// SPI_Send_Data(0x21);
	ssd2805_write(2); SPI_Send_Data(0x0601);// SPI_Send_Data(0x06);
	ssd2805_write(2); SPI_Send_Data(0xa002);// SPI_Send_Data(0xA0);
	ssd2805_write(2); SPI_Send_Data(0x0203);// SPI_Send_Data(0x02);
	ssd2805_write(2); SPI_Send_Data(0x0104);// SPI_Send_Data(0x01);
	ssd2805_write(2); SPI_Send_Data(0x0105);// SPI_Send_Data(0x01);
	ssd2805_write(2); SPI_Send_Data(0x8006);// SPI_Send_Data(0x80);
	ssd2805_write(2); SPI_Send_Data(0x0407);// SPI_Send_Data(0x04);
	ssd2805_write(2); SPI_Send_Data(0x0008);// SPI_Send_Data(0x00);
	ssd2805_write(2); SPI_Send_Data(0x8009);// SPI_Send_Data(0x80);
	ssd2805_write(2); SPI_Send_Data(0x000A);// SPI_Send_Data(0x00);
	ssd2805_write(2); SPI_Send_Data(0x000B);// SPI_Send_Data(0x00);
	ssd2805_write(2); SPI_Send_Data(0x330C);// SPI_Send_Data(0x33);
	ssd2805_write(2); SPI_Send_Data(0x330D);// SPI_Send_Data(0x33);
	ssd2805_write(2); SPI_Send_Data(0x090E);// SPI_Send_Data(0x09);
	ssd2805_write(2); SPI_Send_Data(0x000F);// SPI_Send_Data(0x00);
	ssd2805_write(2); SPI_Send_Data(0xFF10);// SPI_Send_Data(0xFF);
	ssd2805_write(2); SPI_Send_Data(0xF011);// SPI_Send_Data(0xF0);
	ssd2805_write(2); SPI_Send_Data(0x0012);// SPI_Send_Data(0x00);
	ssd2805_write(2); SPI_Send_Data(0x0013);// SPI_Send_Data(0x00);
	ssd2805_write(2); SPI_Send_Data(0x0014);// SPI_Send_Data(0x00);
	ssd2805_write(2); SPI_Send_Data(0xC015);// SPI_Send_Data(0xC0);
	ssd2805_write(2); SPI_Send_Data(0x0816);// SPI_Send_Data(0x08);
	ssd2805_write(2); SPI_Send_Data(0x0017);// SPI_Send_Data(0x00);
	ssd2805_write(2); SPI_Send_Data(0x0018);// SPI_Send_Data(0x00);
	ssd2805_write(2); SPI_Send_Data(0x0019);// SPI_Send_Data(0x00);
	ssd2805_write(2); SPI_Send_Data(0x001A);// SPI_Send_Data(0x00);
	ssd2805_write(2); SPI_Send_Data(0x001B);// SPI_Send_Data(0x00);
	ssd2805_write(2); SPI_Send_Data(0x001C);// SPI_Send_Data(0x00);
	ssd2805_write(2); SPI_Send_Data(0x001D);// SPI_Send_Data(0x00);
	ssd2805_write(2); SPI_Send_Data(0x0120);// SPI_Send_Data(0x01);
	ssd2805_write(2); SPI_Send_Data(0x2321);// SPI_Send_Data(0x23);
	ssd2805_write(2); SPI_Send_Data(0x4522);// SPI_Send_Data(0x45);
	ssd2805_write(2); SPI_Send_Data(0x6723);// SPI_Send_Data(0x67);
	ssd2805_write(2); SPI_Send_Data(0x0124);// SPI_Send_Data(0x01);
	ssd2805_write(2); SPI_Send_Data(0x2325);// SPI_Send_Data(0x23);
	ssd2805_write(2); SPI_Send_Data(0x4526);// SPI_Send_Data(0x45);
	ssd2805_write(2); SPI_Send_Data(0x6727);// SPI_Send_Data(0x67);
	ssd2805_write(2); SPI_Send_Data(0x1230);// SPI_Send_Data(0x12);
	ssd2805_write(2); SPI_Send_Data(0x2231);// SPI_Send_Data(0x22);
	ssd2805_write(2); SPI_Send_Data(0x2232);// SPI_Send_Data(0x22);
	ssd2805_write(2); SPI_Send_Data(0x2233);// SPI_Send_Data(0x22);
	ssd2805_write(2); SPI_Send_Data(0x8734);// SPI_Send_Data(0x87);
	ssd2805_write(2); SPI_Send_Data(0x9635);// SPI_Send_Data(0x96);
	ssd2805_write(2); SPI_Send_Data(0xAA36);// SPI_Send_Data(0xAA);
	ssd2805_write(2); SPI_Send_Data(0xDB37);// SPI_Send_Data(0xDB);
	ssd2805_write(2); SPI_Send_Data(0xCC38);// SPI_Send_Data(0xCC);
	ssd2805_write(2); SPI_Send_Data(0xBD39);// SPI_Send_Data(0xBD);
	ssd2805_write(2); SPI_Send_Data(0x783A);// SPI_Send_Data(0x78);
	ssd2805_write(2); SPI_Send_Data(0x693B);// SPI_Send_Data(0x69);
	ssd2805_write(2); SPI_Send_Data(0x223C);// SPI_Send_Data(0x22);
	ssd2805_write(2); SPI_Send_Data(0x223D);// SPI_Send_Data(0x22);
	ssd2805_write(2); SPI_Send_Data(0x223E);// SPI_Send_Data(0x22);
	ssd2805_write(2); SPI_Send_Data(0x223F);// SPI_Send_Data(0x22);
	ssd2805_write(2); SPI_Send_Data(0x2240);// SPI_Send_Data(0x22);
											//*************************************************************************
	ssd2805_write(6);
	SPI_Send_Data(0xFFFF);
	//	SPI_Send_Data(0xFF);
	SPI_Send_Data(0x0698);
	//	SPI_Send_Data(0x06);
	SPI_Send_Data(0x0704);
	//	SPI_Send_Data(0x07);     // Change to Page 7
	ssd2805_write(2); SPI_Send_Data(0x1306);// SPI_Send_Data(0x13);
	ssd2805_write(2); SPI_Send_Data(0x7702);// SPI_Send_Data(0x77);
	ssd2805_write(2); SPI_Send_Data(0x1D18);// SPI_Send_Data(0x1D);        //VREG1/2OUT ENABLE
											//*************************************************************************
	ssd2805_write(6);
	SPI_Send_Data(0xFFFF);
	//	SPI_Send_Data(0xFF);
	SPI_Send_Data(0x0698);
	//	SPI_Send_Data(0x06);
	SPI_Send_Data(0x0004);
	//	SPI_Send_Data(0x00);     // Change to Page 0
	//ssd2805_write(2); SPI_Send_Data(0x0836);// SPI_Send_Data(0x00);
	ssd2805_write(1);SPI_Send_Data(0x08);
	mdelay(50);
	ssd2805_write(1); SPI_Send_Data(0x11);// SPI_Send_Data(0x00);// Sleep-Out
	mdelay(50);
	ssd2805_write(1); SPI_Send_Data(0x29);// SPI_Send_Data(0x00);                 // Display On	
	mdelay(100);

	//	printk("ssd2828 read 0x0a = 0x%x\n", ssd2805_read(0x0A));
	//	printk("ssd2828 read 0x0d = 0x%x\n", ssd2805_read(0x0d));
	//	printk("ssd2828 read 0x0e = 0x%x\n", ssd2805_read(0x0e));
	//	printk("ssd2828 read 0x05 = 0x%x\n", ssd2805_read(0x05));


	SPI_Send_Cmd(0xB7);
	SPI_Send_Data(0x0250);
	SPI_Send_Cmd(0xbd);
	SPI_Send_Data(0x0000);
	SPI_Send_Cmd(0xbc);
	SPI_Send_Data(0x0000);
	SPI_Send_Cmd(0x11);
	//	SPI_Send_Data(0x11); 
	mdelay(500);


	SPI_Send_Cmd(0xB7);
	SPI_Send_Data(0x0250);
	SPI_Send_Cmd(0xbd);
	SPI_Send_Data(0x0000);
	SPI_Send_Cmd(0xbc);
	SPI_Send_Data(0x0000);
	SPI_Send_Cmd(0x29);
	//	SPI_Send_Data(0x29);  

	mdelay(500);

	SPI_Send_Cmd(0xB7);
	SPI_Send_Data(0x0259); //30B
	mdelay(200);    

} 











#define SPI_CLK                  IMX_GPIO_NR(0, 7)
#define SPI_DI                   IMX_GPIO_NR(1, 8)
#define SPI_DO                   IMX_GPIO_NR(1, 9)
#define SPI_CS                   IMX_GPIO_NR(0, 13)
#define ILI9488_LG35_GPIO_RESET  IMX_GPIO_NR(1, 13)
#define GPIO_LCD_BACKLIGHT       IMX_GPIO_NR(3, 17)


static void lcd_gpio_init(void)
{
	int status = 0;
	
	status = gpio_request(SPI_CLK, "lcd spi clk\n");
	if (status < 0)
	{
		pr_err("Failed to request gpio for lcd spi clk");
		return;
	}
	gpio_direction_output(SPI_CLK, 0);
	
	status = gpio_request(SPI_DO, "lcd spi data\n");
	if (status < 0)
	{
		pr_err("Failed to request gpio for lcd spi data");
		return;
	}
	gpio_direction_output(SPI_DO, 0);	

	status = gpio_request(SPI_DI, "lcd spi data\n");
	if (status < 0)
	{
		pr_err("Failed to request gpio for lcd spi data");
		return;
	}
	gpio_direction_input(SPI_DI);
	
	
	status = gpio_request(SPI_CS, "lcd spi cs\n");
	if (status < 0)
	{
		pr_err("Failed to request gpio for lcd spi cs");
		return;
	}
	gpio_direction_output(SPI_CS, 0);

	status = gpio_request(GPIO_LCD_BACKLIGHT, "lcd backlight\n");

	if (status < 0)
	{
		pr_err("Failed to request gpio for lcd backlight");
		return;
	}
	gpio_direction_output(GPIO_LCD_BACKLIGHT, 0);
}
















static void spi1_init(void)
{
	lcd_gpio_init();
	gpio_set_value(GPIO_LCD_BACKLIGHT, 0);
	TFT_SSD2805_init_code();
	gpio_set_value(GPIO_LCD_BACKLIGHT, 1);
	
	return;
}






static void __init imx6q_init_late(void)
{
	/*
	 * WAIT mode is broken on TO 1.0 and 1.1, so there is no point
	 * to run cpuidle on them.
	 */
	if ((cpu_is_imx6q() && imx_get_soc_revision() > IMX_CHIP_REVISION_1_1)
		|| (cpu_is_imx6dl() && imx_get_soc_revision() >
		IMX_CHIP_REVISION_1_0))
		imx6q_cpuidle_init();

	if (IS_ENABLED(CONFIG_ARM_IMX6Q_CPUFREQ)) {
		imx6q_opp_init();
		platform_device_register(&imx6q_cpufreq_pdev);
	}


	spi1_init();	


	if (of_machine_is_compatible("fsl,imx6q-sabreauto")
		|| of_machine_is_compatible("fsl,imx6dl-sabreauto"))
		imx6q_flexcan_fixup_auto();
}

static void __init imx6q_map_io(void)
{
	debug_ll_io_init();
	imx_scu_map_io();
	imx6_pm_map_io();
#ifdef CONFIG_CPU_FREQ
	imx_busfreq_map_io();
#endif
}

static void __init imx6q_init_irq(void)
{
	imx_init_revision_from_anatop();
	imx_init_l2cache();
	imx_src_init();
	imx_gpc_init();
	irqchip_init();
}

static const char *imx6q_dt_compat[] __initdata = {
	"fsl,imx6dl",
	"fsl,imx6q",
	NULL,
};

DT_MACHINE_START(IMX6Q, "Freescale i.MX6 Quad/DualLite (Device Tree)")
	/*
	 * i.MX6Q/DL maps system memory at 0x10000000 (offset 256MiB), and
	 * GPU has a limit on physical address that it accesses, which must
	 * be below 2GiB.
	 */
	.dma_zone_size	= (SZ_2G - SZ_256M),
	.smp		= smp_ops(imx_smp_ops),
	.map_io		= imx6q_map_io,
	.init_irq	= imx6q_init_irq,
	.init_machine	= imx6q_init_machine,
	.init_late      = imx6q_init_late,
	.dt_compat	= imx6q_dt_compat,
	.restart	= mxc_restart,
MACHINE_END
