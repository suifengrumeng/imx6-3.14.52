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


/*****************************************************************/

#define	VBPD  6
#define	VFPD  2
#define	VSPW  10   
#define	HBPD  10
#define	HFPD  10
#define	HSPW  10


#define SPI_CLK                  IMX_GPIO_NR(5, 22)
#define SPI_DO                   IMX_GPIO_NR(5, 24)
#define SPI_DI                   IMX_GPIO_NR(5, 23)
#define SPI_CS                   IMX_GPIO_NR(5, 25)
#define ILI9488_LG35_GPIO_RESET  IMX_GPIO_NR(1, 15)
#define SPI_RES  IMX_GPIO_NR(1, 15)
#define KK  IMX_GPIO_NR(1, 10)
#define GPIO_LCD_BACKLIGHT       IMX_GPIO_NR(1, 21)


#define		LCD_SDA_CLR		    gpio_set_value(SPI_DO, 0)
#define		LCD_SDA_SET		    gpio_set_value(SPI_DO, 1)
#define		LCD_SCL_CLR			gpio_set_value(SPI_CLK, 0)
#define		LCD_SCL_SET		    gpio_set_value(SPI_CLK, 1)
#define		LCD_RST_CLR			gpio_set_value(SPI_RES, 0)
#define		LCD_RST_SET			gpio_set_value(SPI_RES, 1)
#define		LCD_CS_CLR			gpio_set_value(SPI_CS, 0)
#define		LCD_CS_SET			gpio_set_value(SPI_CS, 1)



static void lcd_gpio_init(void)
{

	int status = 0;

	status = gpio_request(GPIO_LCD_BACKLIGHT, "lcd backlight\n");

	if (status < 0)
	{
		pr_err("Failed to request gpio for lcd backlight");
		return;
	}
	gpio_direction_output(GPIO_LCD_BACKLIGHT, 0);
}


void	LCD_SPI_Init(void)
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



	status = gpio_request(SPI_RES, "lcd reset\n");
	if (status < 0)
	{
		pr_err("Failed to request gpio for lcd reset");
		return;
	}
	gpio_direction_output(SPI_RES, 0);
/*
	status = gpio_request(KK, "lcd reset\n");
	if (status < 0)
	{
		pr_err("Failed to request gpio for lcd reset");
		return;
	}
	gpio_direction_output(KK, 0);
*/
}


static	void  spi_delay(void)
{
  volatile	int i;
  for(i=0;i<10;i++);
}

void W_C(int data)
{


// RS=0
   int i;
/*
   LCD_CS_SET;
   LCD_SDA_SET;
   LCD_SCL_SET;
   spi_delay();
*/
   LCD_CS_CLR;
   spi_delay();
   LCD_SDA_CLR;
   spi_delay();
   LCD_SCL_CLR;
   spi_delay();
   LCD_SCL_SET;spi_delay();

   for(i=0;i<8;i++)
   {

   if (data & 0x80)
      {LCD_SDA_SET;spi_delay();}
   else
      {LCD_SDA_CLR;spi_delay();}

    data<<= 1;
    LCD_SCL_CLR;
   spi_delay();
    LCD_SCL_SET;
  spi_delay();

    }
    LCD_CS_SET;spi_delay();
  
}


void W_D(int data)
{


//RS=1
   int i;
/*
   LCD_CS_SET;
   LCD_SDA_SET;
   LCD_SCL_SET;
   spi_delay();
*/

   LCD_CS_CLR;
   spi_delay();
  LCD_SDA_SET;//BIT8=0 COMM
  spi_delay();
   LCD_SCL_CLR;
  spi_delay();
   LCD_SCL_SET;spi_delay();

   for(i=0;i<8;i++)
   {

   if (data & 0x80)
      {LCD_SDA_SET;spi_delay();}
   else
      {LCD_SDA_CLR;spi_delay();}

    data<<= 1;
    LCD_SCL_CLR;
  spi_delay();
    LCD_SCL_SET;
   spi_delay();

    }
    LCD_CS_SET;spi_delay();
   
 }


void writed16(int data)
{
 
 
//RS=1
   int i;

   LCD_CS_CLR;
 
 
 spi_delay();
  LCD_SDA_SET;//BIT8=0 COMM
 spi_delay();
   LCD_SCL_CLR;
 spi_delay();
   LCD_SCL_SET;spi_delay();

   for(i=0;i<8;i++)
   {

   if (data & 0x80)
     { LCD_SDA_SET;spi_delay();}
   else
      {LCD_SDA_CLR;spi_delay();}

    data<<= 1;
    LCD_SCL_CLR;
  spi_delay();
    LCD_SCL_SET;
  spi_delay();

    }
    LCD_CS_SET;spi_delay();
 
 
}


 void GP_COMMAD_PA(int a)
{
 	W_C(0xBC);
	writed16(a);
	writed16(a>>8);
	W_C(0xBF);
}





unsigned short  SPI_READ(void)
{
    unsigned char  cmd,rdT;
    unsigned short reValue;
    unsigned int   kk;

    LCD_CS_CLR;

    LCD_SDA_CLR;            
    LCD_SCL_CLR;
    LCD_SCL_SET;


    cmd = 0xB0;
    LCD_SCL_CLR;
    for(kk=0;kk<8;kk++)
    {
        if((cmd&0x80)==0x80) 
            LCD_SDA_SET;
        else         
            LCD_SDA_CLR;

        LCD_SCL_SET;
        LCD_SCL_CLR;
        cmd = cmd<<1;    
    }

    LCD_SDA_CLR;        
    LCD_SCL_CLR;
    LCD_SCL_SET;


    cmd = 0xFA;
    LCD_SCL_CLR;
    for(kk=0;kk<8;kk++)
    {
        if((cmd&0x80)==0x80) 
            LCD_SDA_SET;
        else        
            LCD_SDA_CLR;

        LCD_SCL_SET;
        LCD_SCL_CLR;
        cmd = cmd<<1;   
    }   

    rdT=0;
    for(kk=0;kk<8;kk++)
    {
        rdT = rdT<<1;
        LCD_SCL_SET;
        if(gpio_get_value(SPI_DI)) 
                rdT |= 0x01;
        LCD_SCL_CLR;            
    }

    reValue = rdT;

    rdT=0;
    for(kk=0;kk<8;kk++)
    {
        rdT = rdT<<1;
        LCD_SCL_SET;
        if(gpio_get_value(SPI_DI)) 
            rdT |= 0x01;
        LCD_SCL_CLR;            
    }

    reValue += (rdT<<8);

    LCD_CS_SET;

    return reValue;         
}
void Wr_com_data16(unsigned char c, unsigned short value)
{
    LCD_CS_CLR;
    W_C(c);
    W_D(value&0xff);
    W_D((value>>8)&0xff);   
    LCD_CS_SET; 
}






void   SPI_READ_ID(void)
{
    int a;
    Wr_com_data16(0xd4, 0x00FA);
    a=SPI_READ();
    if(a == 0x2828)
    {
        printk(" \n");
        printk("The SSD2828 ID: 0x%x  ^-^  successful !! \n",a);
        printk(" \n");
    }
    else
    {
        printk(" \n");
        printk("The SSD2828 ID: 0x%x  -_-! failing !!!! \n",a);
        printk(" \n");
    }
}


void LCD_Init(void)
{  
	LCD_SPI_Init();  
	//// Reset LCD Driver////
	LCD_RST_SET;
	mdelay(50); // Delay 1ms
    LCD_RST_CLR;
    mdelay(50); // Delay 10ms // This Delay time is necessary
    LCD_RST_SET;
    mdelay(100); // Delay 50 ms

			//SSD2825_Initial
						        W_C(0xb7);
						        W_D(0x50);//50=TX_CLK 70=PCLK
						        W_D(0x00);   //Configuration Register

						        W_C(0xb8);
						        W_D(0x00);
						        W_D(0x00);   //VC(Virtual ChannelID) Control Register

						        W_C(0xb9);
						        W_D(0x00);//1=PLL disable
						        W_D(0x00);
			                               //TX_CLK/MS should be between 5Mhz to100Mhz
						        W_C(0xBA);//PLL=(TX_CLK/MS)*NS 8228=480M 4428=240M  061E=120M 4214=240M 821E=360M 8219=300M
						        W_D(0x14);//D7-0=NS(0x01 : NS=1)
						        W_D(0x42);//D15-14=PLL范围 00=62.5-125 01=126-250 10=251-500 11=501-1000  DB12-8=MS(01:MS=1)

						        W_C(0xBB);//LP Clock Divider LP clock = 400MHz / LPD / 8 = 240 / 8 / 4 = 7.5MHz
						        W_D(0x03);//D5-0=LPD=0x1 – Divide by 2
						        W_D(0x00);

						        W_C(0xb9);
						       	W_D(0x01);//1=PLL disable
						       	W_D(0x00);
						        //MIPI lane configuration
						            	W_C(0xDE);//通道数
						            	W_D(0x00);//11=4LANE 10=3LANE 01=2LANE 00=1LANE
						            	W_D(0x00);

						        W_C(0xc9);
						        W_D(0x02);
						        W_D(0x23);   //p1: HS-Data-zero  p2: HS-Data- prepare  --> 8031 issue
						        mdelay(100);
					 //////////////////////////////////////////////////////////////////////
					    				    	W_C(0xB7);
					    				    	W_D(0x50);//10=TX_CLK 30=PCLK
					    				    	W_D(0x02);

					    				    	W_C(0xBD);
					    				    	W_D(0x00);
					    				    	W_D(0x00);
					///////////////////////////////////////////////////////////////////////

GP_COMMAD_PA(2);W_D(0x01);W_D(0x00);
GP_COMMAD_PA(2);W_D(0xb0);W_D(0x00);

GP_COMMAD_PA(6);W_D(0xb3);W_D(0x14);W_D(0x08);W_D(0x00);W_D(0x22);W_D(0x00);
GP_COMMAD_PA(2);W_D(0xb4);W_D(0x0c);
GP_COMMAD_PA(3);W_D(0xb6);W_D(0x3a);W_D(0xd3);
GP_COMMAD_PA(2);W_D(0x51);W_D(0xe6);
GP_COMMAD_PA(2);W_D(0x53);W_D(0x2c);					
					
					
					
					
					    				    	W_C(0xBC);
					    				    	W_D(0x01);
					    				    	W_D(0x00);

					    				    	W_C(0xBF);
					    				    	W_D(0x11);     //
					    				    	//W_D(0x00);     //
					    				    	mdelay(100);







					    				    	W_C(0xBC);
					    				    	W_D(0x01);
					    				    	W_D(0x00);
					    				    	W_C(0xBF);
					    				    	W_D(0x29);    //  Display On
					    				    	mdelay(100);




					    				    					    	//SSD2825_Initial
					    				    					    	W_C(0xb7);
					    				    					    	W_D(0x50);
					    				    					    	W_D(0x00);   //Configuration Register

					    				    					    	W_C(0xb8);
					    				    					    	W_D(0x00);
					    				    					    	W_D(0x00);   //VC(Virtual ChannelID) Control Register

					    				    					    	W_C(0xb9);
					    				    					    	W_D(0x00);//1=PLL disable
					    				    					    	W_D(0x00);

					    				    					    	W_C(0xBA);//PLL=(TX_CLK/MS)*NS 8228=480M 4428=240M  061E=120M 4214=240M 821E=360M 8219=300M 8225=444M 8224=432
					    				    					    	W_D(0x4a);//D7-0=NS(0x01 : NS=1)       3f
					    				    					    	W_D(0xc2);//D15-14=PLL范围 00=62.5-125 01=126-250 10=251-500 11=501-1000  DB12-8=MS(01:MS=1)

					    				    					    	W_C(0xBB);//LP Clock Divider LP clock = 400MHz / LPD / 8 = 480 / 8/ 8 = 7MHz
					    				    					    	W_D(0x07);//D5-0=LPD=0x1 – Divide by 2
					    				    					    	W_D(0x00);

					    				    					    	W_C(0xb9);
					    				    					    	W_D(0x01);//1=PLL disable
					    				    					    	W_D(0x00);

					    				    					    	W_C(0xc9);
					    				    					    	W_D(0x02);
					    				    					    	W_D(0x23);   //p1: HS-Data-zero  p2: HS-Data- prepare  --> 8031 issue
					    				    					    	mdelay(100);

					    				    					    	W_C(0xCA);
					    				    					    	W_D(0x01);//CLK Prepare
					    				    					    	W_D(0x23);//Clk Zero

					    				    					    	W_C(0xCB); //local_write_reg(addr=0xCB,data=0x0510)
					    				    					    	W_D(0x10); //Clk Post
					    				    					    	W_D(0x05); //Clk Per

					    				    					    	W_C(0xCC); //local_write_reg(addr=0xCC,data=0x100A)
					    				    					    	W_D(0x05); //HS Trail
					    				    					    	W_D(0x10); //Clk Trail

					    				    					    	W_C(0xd0); //local_write_reg(addr=0xCC,data=0x100A)
					    				    					    	W_D(0x00); //HS Trail
					    				    					    	W_D(0x00); //Clk Trail

					    				    					    	//RGB interface configuration
					    				    						    W_C(0xB1);
					    				    							W_D(HSPW);//HSPW 07
					    				    							W_D(VSPW);//VSPW 05

					    				    							W_C(0xB2);
					    				    							W_D(HBPD);//HBPD 0x64=100
					    				    							W_D(VBPD);//VBPD 8 减小下移

					    				    							W_C(0xB3);
					    				    							W_D(HFPD);//HFPD 8
					    				    							W_D(VFPD);//VFPD 10

					    				    							W_C(0xB4);//Horizontal active period 720=02D0
					    				    							W_D(0xb0);//013F=319 02D0=720
					    				    							W_D(0x04);

					    				    							W_C(0xB5);//Vertical active period 1280=0500
					    				    							W_D(0x80);//01DF=479 0500=1280
					    				    							W_D(0x07);

					    				    					    	W_C(0xB6);//RGB CLK  16BPP=00 18BPP=01
					    				    					    	W_D(0x07);//D7=0 D6=0 D5=0  D1-0=11 – 24bpp
					    				    					    	W_D(0x20);//D15=VS D14=HS D13=CLK D12-9=NC D8=0=Video with blanking packet. 00-F0



					    				    					    	//MIPI lane configuration
					    				    					    	W_C(0xDE);//通道数
					    				    					    	W_D(0x03);//11=4LANE 10=3LANE 01=2LANE 00=1LANE
					    				    					    	W_D(0x00);

					    				    					    	W_C(0xD6);//  05=BGR  04=RGB
					    				    					    	W_D(0x05);//D0=0=RGB 1:BGR D1=1=Most significant byte sent first
					    				    					    	W_D(0x00);


					    				    					    	W_C(0xB7);
					    				    					    	W_D(0x4B);
					    				    					    	W_D(0x02);



}

static void spi1_init(void)
{
    unsigned short id;
	lcd_gpio_init();
	gpio_set_value(GPIO_LCD_BACKLIGHT, 0);
    LCD_Init();
	gpio_set_value(GPIO_LCD_BACKLIGHT, 1);

    SPI_READ_ID();

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

	if (of_machine_is_compatible("fsl,imx6q-sabreauto")
		|| of_machine_is_compatible("fsl,imx6dl-sabreauto"))
		imx6q_flexcan_fixup_auto();


   	spi1_init();	

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
