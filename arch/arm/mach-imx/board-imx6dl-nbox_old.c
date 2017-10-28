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
    int status;
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


/**********************NBOX-INPUT-TS***************************/
#define TS_INPUT_IRQ	IMX_GPIO_NR(1, 0)
#define TS_INPUT_RESET	IMX_GPIO_NR(1, 1)


    status = gpio_request(TS_INPUT_IRQ, "input ts irq\n");
	if (status < 0)
	{
		pr_err("Failed to request gpio for input ts irq");
		return;
	}
	gpio_direction_output(TS_INPUT_IRQ, 0);

    status = gpio_request(TS_INPUT_RESET, "input ts reset\n");
	if (status < 0)
	{
		pr_err("Failed to request gpio for input ts reset");
		return;
	}
	gpio_direction_output(TS_INPUT_RESET, 0);

/************************************************************/


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







/********usb reset*************************/
#define USBHUB_RESET 	IMX_GPIO_NR(1, 15)
#define USB_POWER 	    IMX_GPIO_NR(1, 11)

void usb_reset(void)
{
	int status = 0;
	
    printk("usbhub_reset____________________________________________!\n");

	status = gpio_request(USBHUB_RESET, "lcd spi clk\n");
	if (status < 0)
	{
		pr_err("Failed to request gpio for lcd spi clk");
		return;
	}
	gpio_direction_output(USBHUB_RESET, 0);
	
    gpio_set_value(USBHUB_RESET, 1);	
    mdelay(500);
	gpio_set_value(USBHUB_RESET, 0);	
    mdelay(500);
    gpio_set_value(USBHUB_RESET, 1);	


	status = gpio_request(USB_POWER, "lcd spi clk\n");
	if (status < 0)
	{
		pr_err("Failed to request gpio for lcd spi clk");
		return;
	}
	gpio_direction_output(USB_POWER, 0);
	   
    printk("usb_power____________________________________________!\n");
    gpio_set_value(USB_POWER, 1);	
    
}


/*******lcd_spi****************************************/
#define ILI9488_LG35_GPIO_RESET 	IMX_GPIO_NR(1, 18)
//#define ILI9488_LG35_GPIO_BACKLIGHT IMX_GPIO_NR(4, 10)
#define ILI9488_LG35_GPIO_SPI_CS	IMX_GPIO_NR(4, 9)
#define ILI9488_LG35_GPIO_SPI_CLK	IMX_GPIO_NR(5, 22)
#define ILI9488_LG35_GPIO_SPI_DATA	IMX_GPIO_NR(5, 23)


static void lcd_gpio_init(void)
{
	int status = 0;
	
	status = gpio_request(ILI9488_LG35_GPIO_SPI_CLK, "lcd spi clk\n");
	if (status < 0)
	{
		pr_err("Failed to request gpio for lcd spi clk");
		return;
	}
	gpio_direction_output(ILI9488_LG35_GPIO_SPI_CLK, 0);
	
	status = gpio_request(ILI9488_LG35_GPIO_SPI_DATA, "lcd spi data\n");
	if (status < 0)
	{
		pr_err("Failed to request gpio for lcd spi data");
		return;
	}
	gpio_direction_output(ILI9488_LG35_GPIO_SPI_DATA, 0);	
	
	
	status = gpio_request(ILI9488_LG35_GPIO_SPI_CS, "lcd spi cs\n");
	if (status < 0)
	{
		pr_err("Failed to request gpio for lcd spi cs");
		return;
	}
	gpio_direction_output(ILI9488_LG35_GPIO_SPI_CS, 0);

}


static void lcd_gpio_clear(void)
{
	gpio_free(ILI9488_LG35_GPIO_SPI_CLK);	
	gpio_free(ILI9488_LG35_GPIO_SPI_DATA);	
	gpio_free(ILI9488_LG35_GPIO_SPI_CS);	
	//gpio_free(ILI9488_LG35_GPIO_RESET);	//cannot free, or else will reset lcd
}

static void fn043_spi_write_cmd_byte(unsigned int i)
{ 
    unsigned char n,m;
    
    gpio_set_value(ILI9488_LG35_GPIO_SPI_CS, 0);    
    gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 0);
    gpio_set_value(ILI9488_LG35_GPIO_SPI_DATA, 0);
    gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 1);
     
    m=0x80;
    for(n=0; n<8; n++)
    {           
           if(i&m)

            {    
                gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 0);
                gpio_set_value(ILI9488_LG35_GPIO_SPI_DATA, 1);
                gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 1);
            }
            else
            {     
                gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 0);
                gpio_set_value(ILI9488_LG35_GPIO_SPI_DATA, 0);
                gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 1);
            }            
            m=m>>1;

           
     }  
     
     gpio_set_value(ILI9488_LG35_GPIO_SPI_CS, 1);
} 


static void fn043_spi_write_cmd_double_byte(unsigned int i)
{ 
    unsigned short n,m;
    
    gpio_set_value(ILI9488_LG35_GPIO_SPI_CS, 0);    
    gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 0);
    gpio_set_value(ILI9488_LG35_GPIO_SPI_DATA, 0);
    gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 1);
     
    m=0x8000;
    for(n=0; n<16; n++)
    {           
           if(i&m)
            {    
                gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 0);
                gpio_set_value(ILI9488_LG35_GPIO_SPI_DATA, 1);
                gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 1);
            }
            else
            {     
                gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 0);
                gpio_set_value(ILI9488_LG35_GPIO_SPI_DATA, 0);
                gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 1);
            }            
            m=m>>1;

           
     }  
     
     gpio_set_value(ILI9488_LG35_GPIO_SPI_CS, 1);
} 




static void fn043_spi_write_data_byte(unsigned int i)
{ 
    unsigned char n,m;
    
    gpio_set_value(ILI9488_LG35_GPIO_SPI_CS, 0);    
    gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 0);
    gpio_set_value(ILI9488_LG35_GPIO_SPI_DATA, 1);
    gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 1);
     
    m=0x80;
    for(n=0; n<8; n++)
    {           
           if(i&m)

            {    
                gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 0);
                gpio_set_value(ILI9488_LG35_GPIO_SPI_DATA, 1);
                gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 1);
            }
            else
            {     
                gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 0);
                gpio_set_value(ILI9488_LG35_GPIO_SPI_DATA, 0);
                gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 1);
            }            
            m=m>>1;

           
     }  
     
     gpio_set_value(ILI9488_LG35_GPIO_SPI_CS, 1);
} 

static void TFT_FN043_Reset(void)
{

    if (gpio_request(ILI9488_LG35_GPIO_RESET, "lcd reset\n") < 0)
    {
        pr_err("Failed to request gpio for lcd reset");
        return;
    }
    gpio_direction_output(ILI9488_LG35_GPIO_RESET, 1);
    mdelay(100); // Delay 1ms 
    gpio_set_value(ILI9488_LG35_GPIO_RESET, 0);
    mdelay(200); //  Delay 10ms // This delay time is necessary 
    gpio_set_value(ILI9488_LG35_GPIO_RESET, 1);
    mdelay(200); // Delay 120 ms 
}




static void TFT_STD4802_Reset(void)
{
    // VCI=2.8V 
    /************* Reset LCD Driver ***************
      548     LCD_nRESET = 1; 
      549     mdelay(1); // Delay 1ms 
      550     LCD_nRESET = 0; 
      551     mdelay(10); // Delay 10ms // This delay time is necessary 
      552     LCD_nRESET = 1; 
      553     mdelay(120); // Delay 120 ms 
      554      */

    if (gpio_request(ILI9488_LG35_GPIO_RESET, "lcd reset\n") < 0)
    {
        pr_err("Failed to request gpio for lcd reset");
        return;
    }
    gpio_direction_output(ILI9488_LG35_GPIO_RESET, 1);
    mdelay(100); // Delay 1ms 
    gpio_set_value(ILI9488_LG35_GPIO_RESET, 0);
    mdelay(200); //  Delay 10ms // This delay time is necessary 
    gpio_set_value(ILI9488_LG35_GPIO_RESET, 1);
    mdelay(200); // Delay 120 ms 
}


static void lcd_spi_send_byte(unsigned char i){
    uint8_t n;
    uint8_t m;
    m = 0x80;
    for (n = 0; n < 8; n++) {
    if (i & m) {
            gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 0);
            gpio_set_value(ILI9488_LG35_GPIO_SPI_DATA, 1);
            gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 1);
        } else {
                gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 0);
                gpio_set_value(ILI9488_LG35_GPIO_SPI_DATA, 0);
                gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 1);
            }
        m = m >> 1;
    }
}

static void lcd_spi_write_cmd_byte(unsigned short i){
    gpio_set_value(ILI9488_LG35_GPIO_SPI_CS, 0);
    lcd_spi_send_byte(0x20);
    lcd_spi_send_byte(i>>8);
    gpio_set_value(ILI9488_LG35_GPIO_SPI_CS, 1);
    udelay(1000);
    gpio_set_value(ILI9488_LG35_GPIO_SPI_CS, 0);
    lcd_spi_send_byte(0x00);
    lcd_spi_send_byte(i);
    gpio_set_value(ILI9488_LG35_GPIO_SPI_CS, 1);
}

static void lcd_spi_write_data_byte(unsigned char i)
{
     gpio_set_value(ILI9488_LG35_GPIO_SPI_CS, 0);
     lcd_spi_send_byte(0x40);
     lcd_spi_send_byte(i);
     gpio_set_value(ILI9488_LG35_GPIO_SPI_CS, 1);
}




static void TFT_FN043_init_code(void)
{
 
	TFT_FN043_Reset();   //reset

	//Enable Page1
	lcd_spi_write_cmd_byte(0xF000);lcd_spi_write_data_byte(0x55);
	lcd_spi_write_cmd_byte(0xF001);lcd_spi_write_data_byte(0xAA);
	lcd_spi_write_cmd_byte(0xF002);lcd_spi_write_data_byte(0x52);
	lcd_spi_write_cmd_byte(0xF003);lcd_spi_write_data_byte(0x08);
	lcd_spi_write_cmd_byte(0xF004);lcd_spi_write_data_byte(0x01);
	
	// AVDD: manual, 6V (0x44: 2.5xVCI0)
	lcd_spi_write_cmd_byte(0xB600);lcd_spi_write_data_byte(0x34); 	
	lcd_spi_write_cmd_byte(0xB601);lcd_spi_write_data_byte(0x34); 	
	
	
	lcd_spi_write_cmd_byte(0xB602);lcd_spi_write_data_byte(0x34);
	
	lcd_spi_write_cmd_byte(0xB000);lcd_spi_write_data_byte(0x0C);  
	
	//Default žé05	6v
	lcd_spi_write_cmd_byte(0xB001);lcd_spi_write_data_byte(0x0C);
	lcd_spi_write_cmd_byte(0xB002);lcd_spi_write_data_byte(0x0C);
	
	// AVEE: manual, -6V (0x34: -2.5xVCI)
	lcd_spi_write_cmd_byte(0xB700);lcd_spi_write_data_byte(0x24); 	
	
	
	lcd_spi_write_cmd_byte(0xB701);lcd_spi_write_data_byte(0x24); 	
	
	
	lcd_spi_write_cmd_byte(0xB702);lcd_spi_write_data_byte(0x24);
	
	lcd_spi_write_cmd_byte(0xB100);lcd_spi_write_data_byte(0x0C);  
	
	//Default žé05	-6v 		   
	lcd_spi_write_cmd_byte(0xB101);lcd_spi_write_data_byte(0x0C); 	
	
	
	lcd_spi_write_cmd_byte(0xB102);lcd_spi_write_data_byte(0x0C);
	
	//Power Control for VCL
	lcd_spi_write_cmd_byte(0xB800);lcd_spi_write_data_byte(0x34);
	lcd_spi_write_cmd_byte(0xB200);lcd_spi_write_data_byte(0x00); //		-2.5v
	
	
	
	// VGH: Clamp Enable, 2*AVDD-AVEE, 11V (0x00);
	
	lcd_spi_write_cmd_byte(0xBF00);lcd_spi_write_data_byte(0x01);
	lcd_spi_write_cmd_byte(0xB900);lcd_spi_write_data_byte(0x34); 	
	
	
	lcd_spi_write_cmd_byte(0xB901);lcd_spi_write_data_byte(0x34); 	
	
	
	lcd_spi_write_cmd_byte(0xB902);lcd_spi_write_data_byte(0x34);  
	
	lcd_spi_write_cmd_byte(0xB300);lcd_spi_write_data_byte(0x08);
	lcd_spi_write_cmd_byte(0xB301);lcd_spi_write_data_byte(0x08);
	lcd_spi_write_cmd_byte(0xB302);lcd_spi_write_data_byte(0x08);
	
	// VGL(LVGL):
	lcd_spi_write_cmd_byte(0xBA00);lcd_spi_write_data_byte(0x14);
	lcd_spi_write_cmd_byte(0xBA01);lcd_spi_write_data_byte(0x14);
	lcd_spi_write_cmd_byte(0xBA02);lcd_spi_write_data_byte(0x14);
	
	// VGL_REG(VGLO):-10V
	lcd_spi_write_cmd_byte(0xB500);lcd_spi_write_data_byte(0x08);
	lcd_spi_write_cmd_byte(0xB501);lcd_spi_write_data_byte(0x08);
	lcd_spi_write_cmd_byte(0xB502);lcd_spi_write_data_byte(0x08);
	
	// VGMP/VGSP:	  4.8v
	lcd_spi_write_cmd_byte(0xBC00);lcd_spi_write_data_byte(0x00); 
	lcd_spi_write_cmd_byte(0xBC01);lcd_spi_write_data_byte(0x80);
	lcd_spi_write_cmd_byte(0xBC02);lcd_spi_write_data_byte(0x00); 
	
	// VGMN/VGSN  -4.8v
	lcd_spi_write_cmd_byte(0xBD00);lcd_spi_write_data_byte(0x00);
	lcd_spi_write_cmd_byte(0xBD01);lcd_spi_write_data_byte(0x80);
	lcd_spi_write_cmd_byte(0xBD02);lcd_spi_write_data_byte(0x00);
	
	// VCOM=-0.1
	lcd_spi_write_cmd_byte(0xBE00);lcd_spi_write_data_byte(0x00);
	lcd_spi_write_cmd_byte(0xBE01);lcd_spi_write_data_byte(0x2F);

#if 0
	
	//R+							
	lcd_spi_write_cmd_byte(0xD100);lcd_spi_write_data_byte(0x00);
	lcd_spi_write_cmd_byte(0xD101);lcd_spi_write_data_byte(0x37);
	lcd_spi_write_cmd_byte(0xD102);lcd_spi_write_data_byte(0x00);
	lcd_spi_write_cmd_byte(0xD103);lcd_spi_write_data_byte(0x53);
	lcd_spi_write_cmd_byte(0xD104);lcd_spi_write_data_byte(0x00);
	lcd_spi_write_cmd_byte(0xD105);lcd_spi_write_data_byte(0x79);
	lcd_spi_write_cmd_byte(0xD106);lcd_spi_write_data_byte(0x00);
	lcd_spi_write_cmd_byte(0xD107);lcd_spi_write_data_byte(0x97);
	lcd_spi_write_cmd_byte(0xD108);lcd_spi_write_data_byte(0x00);
	lcd_spi_write_cmd_byte(0xD109);lcd_spi_write_data_byte(0xB1);
	lcd_spi_write_cmd_byte(0xD10A);lcd_spi_write_data_byte(0x00);
	lcd_spi_write_cmd_byte(0xD10B);lcd_spi_write_data_byte(0xD5);
	lcd_spi_write_cmd_byte(0xD10C);lcd_spi_write_data_byte(0x00);
	lcd_spi_write_cmd_byte(0xD10D);lcd_spi_write_data_byte(0xF4);
	lcd_spi_write_cmd_byte(0xD10E);lcd_spi_write_data_byte(0x01);
	lcd_spi_write_cmd_byte(0xD10F);lcd_spi_write_data_byte(0x23);
	lcd_spi_write_cmd_byte(0xD110);lcd_spi_write_data_byte(0x01);
	lcd_spi_write_cmd_byte(0xD111);lcd_spi_write_data_byte(0x49);
	lcd_spi_write_cmd_byte(0xD112);lcd_spi_write_data_byte(0x01);
	lcd_spi_write_cmd_byte(0xD113);lcd_spi_write_data_byte(0x87);
	lcd_spi_write_cmd_byte(0xD114);lcd_spi_write_data_byte(0x01);
	lcd_spi_write_cmd_byte(0xD115);lcd_spi_write_data_byte(0xB6);
	lcd_spi_write_cmd_byte(0xD116);lcd_spi_write_data_byte(0x02);
	lcd_spi_write_cmd_byte(0xD117);lcd_spi_write_data_byte(0x00);
	lcd_spi_write_cmd_byte(0xD118);lcd_spi_write_data_byte(0x02);
	lcd_spi_write_cmd_byte(0xD119);lcd_spi_write_data_byte(0x3B);
	lcd_spi_write_cmd_byte(0xD11A);lcd_spi_write_data_byte(0x02);
	lcd_spi_write_cmd_byte(0xD11B);lcd_spi_write_data_byte(0x3D);
	lcd_spi_write_cmd_byte(0xD11C);lcd_spi_write_data_byte(0x02);
	lcd_spi_write_cmd_byte(0xD11D);lcd_spi_write_data_byte(0x75);
	lcd_spi_write_cmd_byte(0xD11E);lcd_spi_write_data_byte(0x02);
	lcd_spi_write_cmd_byte(0xD11F);lcd_spi_write_data_byte(0xB1);
	lcd_spi_write_cmd_byte(0xD120);lcd_spi_write_data_byte(0x02);
	lcd_spi_write_cmd_byte(0xD121);lcd_spi_write_data_byte(0xD5);
	lcd_spi_write_cmd_byte(0xD122);lcd_spi_write_data_byte(0x03);
	lcd_spi_write_cmd_byte(0xD123);lcd_spi_write_data_byte(0x09);
	lcd_spi_write_cmd_byte(0xD124);lcd_spi_write_data_byte(0x03);
	lcd_spi_write_cmd_byte(0xD125);lcd_spi_write_data_byte(0x28);
	lcd_spi_write_cmd_byte(0xD126);lcd_spi_write_data_byte(0x03);
	lcd_spi_write_cmd_byte(0xD127);lcd_spi_write_data_byte(0x52);
	lcd_spi_write_cmd_byte(0xD128);lcd_spi_write_data_byte(0x03);
	lcd_spi_write_cmd_byte(0xD129);lcd_spi_write_data_byte(0x6B);
	lcd_spi_write_cmd_byte(0xD12A);lcd_spi_write_data_byte(0x03);
	lcd_spi_write_cmd_byte(0xD12B);lcd_spi_write_data_byte(0x8D);
	lcd_spi_write_cmd_byte(0xD12C);lcd_spi_write_data_byte(0x03);
	lcd_spi_write_cmd_byte(0xD12D);lcd_spi_write_data_byte(0xA2);
	lcd_spi_write_cmd_byte(0xD12E);lcd_spi_write_data_byte(0x03);
	lcd_spi_write_cmd_byte(0xD12F);lcd_spi_write_data_byte(0xBB);
	lcd_spi_write_cmd_byte(0xD130);lcd_spi_write_data_byte(0x03);
	lcd_spi_write_cmd_byte(0xD131);lcd_spi_write_data_byte(0xC1);
	lcd_spi_write_cmd_byte(0xD132);lcd_spi_write_data_byte(0x03);
	lcd_spi_write_cmd_byte(0xD133);lcd_spi_write_data_byte(0xC1);
	
	
	
	
	//G+							
	lcd_spi_write_cmd_byte(0xD200);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD201);lcd_spi_write_data_byte(0x37); 	
	
	
	lcd_spi_write_cmd_byte(0xD202);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD203);lcd_spi_write_data_byte(0x53); 	
	
	
	lcd_spi_write_cmd_byte(0xD204);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD205);lcd_spi_write_data_byte(0x79); 	
	
	
	lcd_spi_write_cmd_byte(0xD206);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD207);lcd_spi_write_data_byte(0x97); 	
	
	
	lcd_spi_write_cmd_byte(0xD208);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD209);lcd_spi_write_data_byte(0xB1); 	
	
	
	lcd_spi_write_cmd_byte(0xD20A);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD20B);lcd_spi_write_data_byte(0xD5); 	
	
	
	lcd_spi_write_cmd_byte(0xD20C);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD20D);lcd_spi_write_data_byte(0xF4); 	
	
	
	lcd_spi_write_cmd_byte(0xD20E);lcd_spi_write_data_byte(0x01); 	
	
	
	lcd_spi_write_cmd_byte(0xD20F);lcd_spi_write_data_byte(0x23); 	
	
	
	lcd_spi_write_cmd_byte(0xD210);lcd_spi_write_data_byte(0x01);   


	lcd_spi_write_cmd_byte(0xD211);lcd_spi_write_data_byte(0x49); 	
	
	
	lcd_spi_write_cmd_byte(0xD212);lcd_spi_write_data_byte(0x01); 	
	
	
	lcd_spi_write_cmd_byte(0xD213);lcd_spi_write_data_byte(0x87); 	
	
	
	lcd_spi_write_cmd_byte(0xD214);lcd_spi_write_data_byte(0x01); 	
	
	
	lcd_spi_write_cmd_byte(0xD215);lcd_spi_write_data_byte(0xB6); 	
	
	
	lcd_spi_write_cmd_byte(0xD216);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD217);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD218);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD219);lcd_spi_write_data_byte(0x3B); 	
	
	
	lcd_spi_write_cmd_byte(0xD21A);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD21B);lcd_spi_write_data_byte(0x3D); 	
	
	
	lcd_spi_write_cmd_byte(0xD21C);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD21D);lcd_spi_write_data_byte(0x75); 	
	
	
	lcd_spi_write_cmd_byte(0xD21E);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD21F);lcd_spi_write_data_byte(0xB1); 	
	
	
	lcd_spi_write_cmd_byte(0xD220);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD221);lcd_spi_write_data_byte(0xD5); 	
	
	
	lcd_spi_write_cmd_byte(0xD222);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD223);lcd_spi_write_data_byte(0x09); 	
	
	
	lcd_spi_write_cmd_byte(0xD224);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD225);lcd_spi_write_data_byte(0x28); 	
	
	
	lcd_spi_write_cmd_byte(0xD226);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD227);lcd_spi_write_data_byte(0x52); 	
	
	
	lcd_spi_write_cmd_byte(0xD228);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD229);lcd_spi_write_data_byte(0x6B); 	
	
	
	lcd_spi_write_cmd_byte(0xD22A);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD22B);lcd_spi_write_data_byte(0x8D); 	
	
	
	lcd_spi_write_cmd_byte(0xD22C);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD22D);lcd_spi_write_data_byte(0xA2); 	
	
	
	lcd_spi_write_cmd_byte(0xD22E);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD22F);lcd_spi_write_data_byte(0xBB); 	
	
	
	lcd_spi_write_cmd_byte(0xD230);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD231);lcd_spi_write_data_byte(0xC1); 	
	
	
	lcd_spi_write_cmd_byte(0xD232);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD233);lcd_spi_write_data_byte(0xC1); 	
	
	
	
	
	//B+							
	lcd_spi_write_cmd_byte(0xD300);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD301);lcd_spi_write_data_byte(0x37); 	
	
	
	lcd_spi_write_cmd_byte(0xD302);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD303);lcd_spi_write_data_byte(0x53); 	
	
	
	lcd_spi_write_cmd_byte(0xD304);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD305);lcd_spi_write_data_byte(0x79); 	
	
	
	lcd_spi_write_cmd_byte(0xD306);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD307);lcd_spi_write_data_byte(0x97); 	
	
	
	lcd_spi_write_cmd_byte(0xD308);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD309);lcd_spi_write_data_byte(0xB1); 	
	
	
	lcd_spi_write_cmd_byte(0xD30A);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD30B);lcd_spi_write_data_byte(0xD5); 	
	
	
	lcd_spi_write_cmd_byte(0xD30C);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD30D);lcd_spi_write_data_byte(0xF4); 	
	
	
	lcd_spi_write_cmd_byte(0xD30E);lcd_spi_write_data_byte(0x01); 	
	
	
	lcd_spi_write_cmd_byte(0xD30F);lcd_spi_write_data_byte(0x23); 	
	
	
	lcd_spi_write_cmd_byte(0xD310);lcd_spi_write_data_byte(0x01); 	
	
	
	lcd_spi_write_cmd_byte(0xD311);lcd_spi_write_data_byte(0x49); 	
	
	
	lcd_spi_write_cmd_byte(0xD312);lcd_spi_write_data_byte(0x01); 	
	
	
	lcd_spi_write_cmd_byte(0xD313);lcd_spi_write_data_byte(0x87); 	
	
	
	lcd_spi_write_cmd_byte(0xD314);lcd_spi_write_data_byte(0x01); 	
	
	
	lcd_spi_write_cmd_byte(0xD315);lcd_spi_write_data_byte(0xB6); 	
	
	
	lcd_spi_write_cmd_byte(0xD316);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD317);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD318);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD319);lcd_spi_write_data_byte(0x3B); 	
	
	
	lcd_spi_write_cmd_byte(0xD31A);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD31B);lcd_spi_write_data_byte(0x3D); 	
	
	
	lcd_spi_write_cmd_byte(0xD31C);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD31D);lcd_spi_write_data_byte(0x75); 	
	
	
	lcd_spi_write_cmd_byte(0xD31E);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD31F);lcd_spi_write_data_byte(0xB1); 	
	
	
	lcd_spi_write_cmd_byte(0xD320);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD321);lcd_spi_write_data_byte(0xD5); 	
	
	
	lcd_spi_write_cmd_byte(0xD322);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD323);lcd_spi_write_data_byte(0x09); 	
	
	
	lcd_spi_write_cmd_byte(0xD324);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD325);lcd_spi_write_data_byte(0x28); 	
	
	
	lcd_spi_write_cmd_byte(0xD326);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD327);lcd_spi_write_data_byte(0x52); 	
	
	
	lcd_spi_write_cmd_byte(0xD328);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD329);lcd_spi_write_data_byte(0x6B); 	
	
	
	lcd_spi_write_cmd_byte(0xD32A);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD32B);lcd_spi_write_data_byte(0x8D); 	
	
	
	lcd_spi_write_cmd_byte(0xD32C);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD32D);lcd_spi_write_data_byte(0xA2); 	
	
	
	lcd_spi_write_cmd_byte(0xD32E);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD32F);lcd_spi_write_data_byte(0xBB); 	
	
	
	lcd_spi_write_cmd_byte(0xD330);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD331);lcd_spi_write_data_byte(0xC1); 	
	
	
	lcd_spi_write_cmd_byte(0xD332);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD333);lcd_spi_write_data_byte(0xC1); 	
	
	
	
	
	//R-							
	lcd_spi_write_cmd_byte(0xD400);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD401);lcd_spi_write_data_byte(0x37); 	
	
	
	lcd_spi_write_cmd_byte(0xD402);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD403);lcd_spi_write_data_byte(0x53); 	
	
	
	lcd_spi_write_cmd_byte(0xD404);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD405);lcd_spi_write_data_byte(0x79); 	
	
	
	lcd_spi_write_cmd_byte(0xD406);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD407);lcd_spi_write_data_byte(0x97); 	
	
	
	lcd_spi_write_cmd_byte(0xD408);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD409);lcd_spi_write_data_byte(0xB1); 	
	
	
	lcd_spi_write_cmd_byte(0xD40A);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD40B);lcd_spi_write_data_byte(0xD5); 	
	
	
	lcd_spi_write_cmd_byte(0xD40C);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD40D);lcd_spi_write_data_byte(0xF4); 	
	
	
	lcd_spi_write_cmd_byte(0xD40E);lcd_spi_write_data_byte(0x01); 	
	
	
	lcd_spi_write_cmd_byte(0xD40F);lcd_spi_write_data_byte(0x23); 	
	
	
	lcd_spi_write_cmd_byte(0xD410);lcd_spi_write_data_byte(0x01); 	
	
	
	lcd_spi_write_cmd_byte(0xD411);lcd_spi_write_data_byte(0x49); 	
	
	
	lcd_spi_write_cmd_byte(0xD412);lcd_spi_write_data_byte(0x01); 	
	
	
	lcd_spi_write_cmd_byte(0xD413);lcd_spi_write_data_byte(0x87); 	
	
	
	lcd_spi_write_cmd_byte(0xD414);lcd_spi_write_data_byte(0x01); 	
	
	
	lcd_spi_write_cmd_byte(0xD415);lcd_spi_write_data_byte(0xB6); 	
	
	
	lcd_spi_write_cmd_byte(0xD416);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD417);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD418);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD419);lcd_spi_write_data_byte(0x3B); 	
	
	
	lcd_spi_write_cmd_byte(0xD41A);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD41B);lcd_spi_write_data_byte(0x3D); 	
	
	
	lcd_spi_write_cmd_byte(0xD41C);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD41D);lcd_spi_write_data_byte(0x75); 	
	
	
	lcd_spi_write_cmd_byte(0xD41E);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD41F);lcd_spi_write_data_byte(0xB1); 	
	
	
	lcd_spi_write_cmd_byte(0xD420);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD421);lcd_spi_write_data_byte(0xD5); 	
	
	
	lcd_spi_write_cmd_byte(0xD422);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD423);lcd_spi_write_data_byte(0x09); 	
	
	
	lcd_spi_write_cmd_byte(0xD424);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD425);lcd_spi_write_data_byte(0x28); 	
	
	
	lcd_spi_write_cmd_byte(0xD426);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD427);lcd_spi_write_data_byte(0x52); 	
	
	
	lcd_spi_write_cmd_byte(0xD428);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD429);lcd_spi_write_data_byte(0x6B); 	
	
	
	lcd_spi_write_cmd_byte(0xD42A);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD42B);lcd_spi_write_data_byte(0x8D); 	
	
	
	lcd_spi_write_cmd_byte(0xD42C);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD42D);lcd_spi_write_data_byte(0xA2); 	
	
	
	lcd_spi_write_cmd_byte(0xD42E);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD42F);lcd_spi_write_data_byte(0xBB); 	
	
	
	lcd_spi_write_cmd_byte(0xD430);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD431);lcd_spi_write_data_byte(0xC1); 	
	
	
	lcd_spi_write_cmd_byte(0xD432);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD433);lcd_spi_write_data_byte(0xC1); 	
	
	
	
	
	//G-							
	lcd_spi_write_cmd_byte(0xD500);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD501);lcd_spi_write_data_byte(0x37); 	
	
	
	lcd_spi_write_cmd_byte(0xD502);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD503);lcd_spi_write_data_byte(0x53); 	
	
	
	lcd_spi_write_cmd_byte(0xD504);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD505);lcd_spi_write_data_byte(0x79); 	
	
	
	lcd_spi_write_cmd_byte(0xD506);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD507);lcd_spi_write_data_byte(0x97); 	
	
	
	lcd_spi_write_cmd_byte(0xD508);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD509);lcd_spi_write_data_byte(0xB1); 	
	
	
	lcd_spi_write_cmd_byte(0xD50A);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD50B);lcd_spi_write_data_byte(0xD5); 	
	
	
	lcd_spi_write_cmd_byte(0xD50C);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD50D);lcd_spi_write_data_byte(0xF4); 	
	
	
	lcd_spi_write_cmd_byte(0xD50E);lcd_spi_write_data_byte(0x01); 	
	
	
	lcd_spi_write_cmd_byte(0xD50F);lcd_spi_write_data_byte(0x23); 	
	
	
	lcd_spi_write_cmd_byte(0xD510);lcd_spi_write_data_byte(0x01); 	
	
	
	lcd_spi_write_cmd_byte(0xD511);lcd_spi_write_data_byte(0x49); 	
	
	
	lcd_spi_write_cmd_byte(0xD512);lcd_spi_write_data_byte(0x01); 	
	
	
	lcd_spi_write_cmd_byte(0xD513);lcd_spi_write_data_byte(0x87); 	
	
	
	lcd_spi_write_cmd_byte(0xD514);lcd_spi_write_data_byte(0x01); 	
	
	
	lcd_spi_write_cmd_byte(0xD515);lcd_spi_write_data_byte(0xB6); 	
	
	
	lcd_spi_write_cmd_byte(0xD516);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD517);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD518);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD519);lcd_spi_write_data_byte(0x3B); 	
	
	
	lcd_spi_write_cmd_byte(0xD51A);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD51B);lcd_spi_write_data_byte(0x3D); 	
	
	
	lcd_spi_write_cmd_byte(0xD51C);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD51D);lcd_spi_write_data_byte(0x75); 	
	
	
	lcd_spi_write_cmd_byte(0xD51E);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD51F);lcd_spi_write_data_byte(0xB1); 	
	
	
	lcd_spi_write_cmd_byte(0xD520);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD521);lcd_spi_write_data_byte(0xD5); 	
	
	
	lcd_spi_write_cmd_byte(0xD522);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD523);lcd_spi_write_data_byte(0x09); 	
	
	
	lcd_spi_write_cmd_byte(0xD524);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD525);lcd_spi_write_data_byte(0x28); 	
	
	
	lcd_spi_write_cmd_byte(0xD526);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD527);lcd_spi_write_data_byte(0x52); 	
	
	
	lcd_spi_write_cmd_byte(0xD528);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD529);lcd_spi_write_data_byte(0x6B); 	
	
	
	lcd_spi_write_cmd_byte(0xD52A);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD52B);lcd_spi_write_data_byte(0x8D);   

	lcd_spi_write_cmd_byte(0xD52C);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD52D);lcd_spi_write_data_byte(0xA2); 	
	
	
	lcd_spi_write_cmd_byte(0xD52E);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD52F);lcd_spi_write_data_byte(0xBB); 	
	
	
	lcd_spi_write_cmd_byte(0xD530);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD531);lcd_spi_write_data_byte(0xC1); 	
	
	
	lcd_spi_write_cmd_byte(0xD532);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD533);lcd_spi_write_data_byte(0xC1); 	
	
	
	
	
	//B-							
	lcd_spi_write_cmd_byte(0xD600);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD601);lcd_spi_write_data_byte(0x37); 	
	
	
	lcd_spi_write_cmd_byte(0xD602);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD603);lcd_spi_write_data_byte(0x53); 	
	
	
	lcd_spi_write_cmd_byte(0xD604);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD605);lcd_spi_write_data_byte(0x79); 	
	
	
	lcd_spi_write_cmd_byte(0xD606);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD607);lcd_spi_write_data_byte(0x97); 	
	
	
	lcd_spi_write_cmd_byte(0xD608);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD609);lcd_spi_write_data_byte(0xB1); 	
	
	
	lcd_spi_write_cmd_byte(0xD60A);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD60B);lcd_spi_write_data_byte(0xD5); 	
	
	
	lcd_spi_write_cmd_byte(0xD60C);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD60D);lcd_spi_write_data_byte(0xF4); 	
	
	
	lcd_spi_write_cmd_byte(0xD60E);lcd_spi_write_data_byte(0x01); 	
	
	
	lcd_spi_write_cmd_byte(0xD60F);lcd_spi_write_data_byte(0x23); 	
	
	
	lcd_spi_write_cmd_byte(0xD610);lcd_spi_write_data_byte(0x01); 	
	
	
	lcd_spi_write_cmd_byte(0xD611);lcd_spi_write_data_byte(0x49); 	
	
	
	lcd_spi_write_cmd_byte(0xD612);lcd_spi_write_data_byte(0x01); 	
	
	
	lcd_spi_write_cmd_byte(0xD613);lcd_spi_write_data_byte(0x87); 	
	
	
	lcd_spi_write_cmd_byte(0xD614);lcd_spi_write_data_byte(0x01); 	
	
	
	lcd_spi_write_cmd_byte(0xD615);lcd_spi_write_data_byte(0xB6); 	
	
	
	lcd_spi_write_cmd_byte(0xD616);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD617);lcd_spi_write_data_byte(0x00); 	
	
	
	lcd_spi_write_cmd_byte(0xD618);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD619);lcd_spi_write_data_byte(0x3B); 	
	
	
	lcd_spi_write_cmd_byte(0xD61A);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD61B);lcd_spi_write_data_byte(0x3D); 	
	
	
	lcd_spi_write_cmd_byte(0xD61C);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD61D);lcd_spi_write_data_byte(0x75); 	
	
	
	lcd_spi_write_cmd_byte(0xD61E);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD61F);lcd_spi_write_data_byte(0xB1); 	
	
	
	lcd_spi_write_cmd_byte(0xD620);lcd_spi_write_data_byte(0x02); 	
	
	
	lcd_spi_write_cmd_byte(0xD621);lcd_spi_write_data_byte(0xD5); 	
	
	
	lcd_spi_write_cmd_byte(0xD622);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD623);lcd_spi_write_data_byte(0x09); 	
	
	
	lcd_spi_write_cmd_byte(0xD624);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD625);lcd_spi_write_data_byte(0x28); 	
	
	
	lcd_spi_write_cmd_byte(0xD626);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD627);lcd_spi_write_data_byte(0x52); 	
	
	
	lcd_spi_write_cmd_byte(0xD628);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD629);lcd_spi_write_data_byte(0x6B); 	
	
	
	lcd_spi_write_cmd_byte(0xD62A);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD62B);lcd_spi_write_data_byte(0x8D); 	
	
	
	lcd_spi_write_cmd_byte(0xD62C);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD62D);lcd_spi_write_data_byte(0xA2); 	
	
	
	lcd_spi_write_cmd_byte(0xD62E);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD62F);lcd_spi_write_data_byte(0xBB); 	
	
	
	lcd_spi_write_cmd_byte(0xD630);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD631);lcd_spi_write_data_byte(0xC1); 	
	
	
	lcd_spi_write_cmd_byte(0xD632);lcd_spi_write_data_byte(0x03); 	
	
	
	lcd_spi_write_cmd_byte(0xD633);lcd_spi_write_data_byte(0xC1);

#endif
	
	//Enable Page0
	lcd_spi_write_cmd_byte(0xF000);lcd_spi_write_data_byte(0x55);
	lcd_spi_write_cmd_byte(0xF001);lcd_spi_write_data_byte(0xAA);
	lcd_spi_write_cmd_byte(0xF002);lcd_spi_write_data_byte(0x52);
	lcd_spi_write_cmd_byte(0xF003);lcd_spi_write_data_byte(0x08);
	lcd_spi_write_cmd_byte(0xF004);lcd_spi_write_data_byte(0x00);
	
	// RGB Internal mode
	//lcd_spi_write_cmd_byte(0xB300);lcd_spi_write_data_byte(0x01);
	
	// RGB I/F Setting
	lcd_spi_write_cmd_byte(0xB000);lcd_spi_write_data_byte(0x00);
	lcd_spi_write_cmd_byte(0xB001);lcd_spi_write_data_byte(0x05);
	lcd_spi_write_cmd_byte(0xB002);lcd_spi_write_data_byte(0x02);
	lcd_spi_write_cmd_byte(0xB003);lcd_spi_write_data_byte(0x05);
	lcd_spi_write_cmd_byte(0xB004);lcd_spi_write_data_byte(0x02);
	
	// SDT:
	lcd_spi_write_cmd_byte(0xB600);lcd_spi_write_data_byte(0x05);
	
	// Gate EQ:
	lcd_spi_write_cmd_byte(0xB700);lcd_spi_write_data_byte(0x70);
	lcd_spi_write_cmd_byte(0xB701);lcd_spi_write_data_byte(0x70);
	
	//// Source EQ: 
	lcd_spi_write_cmd_byte(0xB800);lcd_spi_write_data_byte(0x01);
	lcd_spi_write_cmd_byte(0xB801);lcd_spi_write_data_byte(0x05);
	lcd_spi_write_cmd_byte(0xB802);lcd_spi_write_data_byte(0x05);
	lcd_spi_write_cmd_byte(0xB803);lcd_spi_write_data_byte(0x05);
	
	// Inversion: Column inversion (NVT)
	lcd_spi_write_cmd_byte(0xBC00);lcd_spi_write_data_byte(0x00);
	lcd_spi_write_cmd_byte(0xBC01);lcd_spi_write_data_byte(0x00);
	lcd_spi_write_cmd_byte(0xBC02);lcd_spi_write_data_byte(0x00);
	
	// BOE's Setting (default)
	lcd_spi_write_cmd_byte(0xCC00);lcd_spi_write_data_byte(0x03);
	lcd_spi_write_cmd_byte(0xCC01);lcd_spi_write_data_byte(0x50);
	lcd_spi_write_cmd_byte(0xCC02);lcd_spi_write_data_byte(0x50);
	
	// Display Timing:
	lcd_spi_write_cmd_byte(0xBD00);lcd_spi_write_data_byte(0x01);
	lcd_spi_write_cmd_byte(0xBD01);lcd_spi_write_data_byte(0x00);
	lcd_spi_write_cmd_byte(0xBD02);lcd_spi_write_data_byte(0x07);
	lcd_spi_write_cmd_byte(0xBD03);lcd_spi_write_data_byte(0x31);
	lcd_spi_write_cmd_byte(0xBD04);lcd_spi_write_data_byte(0x00);

	lcd_spi_write_cmd_byte(0xFF00);lcd_spi_write_data_byte(0xAA);
	lcd_spi_write_cmd_byte(0xFF01);lcd_spi_write_data_byte(0x55);
	lcd_spi_write_cmd_byte(0xFF02);lcd_spi_write_data_byte(0x25);
	lcd_spi_write_cmd_byte(0xFF03);lcd_spi_write_data_byte(0x01);
	
//	lcd_spi_write_cmd_byte(0xF304);lcd_spi_write_data_byte(0x11);
//	lcd_spi_write_cmd_byte(0xF306);lcd_spi_write_data_byte(0x10);
//	lcd_spi_write_cmd_byte(0xF408);lcd_spi_write_data_byte(0x00);
	
	
	lcd_spi_write_cmd_byte(0x3a00);lcd_spi_write_data_byte(0x77);
	
	lcd_spi_write_cmd_byte(0x1100);	
	mdelay(120);
	
	lcd_spi_write_cmd_byte(0x2900);	


}






 
static void TFT_STD4802_init_code(void) 
{
     TFT_STD4802_Reset();
/* 
     ILI9488_LG35_spi_write_cmd_byte(0x11);    //Sleep out
     mdelay(120);
     ILI9488_LG35_spi_write_cmd_byte(0x29);    //Sleep out
     mdelay(25);
*/   
     
    lcd_spi_write_cmd_byte(0xFF00); lcd_spi_write_data_byte(0x48);
    lcd_spi_write_cmd_byte(0xFF01); lcd_spi_write_data_byte(0x02);
    lcd_spi_write_cmd_byte(0xFF02); lcd_spi_write_data_byte(0x01);
 
    lcd_spi_write_cmd_byte(0xFF80); lcd_spi_write_data_byte(0x48);
    lcd_spi_write_cmd_byte(0xFF81); lcd_spi_write_data_byte(0x02);
 
    lcd_spi_write_cmd_byte(0x5100); lcd_spi_write_data_byte(0xF0);
 
    lcd_spi_write_cmd_byte(0xC5B1); lcd_spi_write_data_byte(0x00);
 
    lcd_spi_write_cmd_byte(0xC4B0); lcd_spi_write_data_byte(0x02);
    lcd_spi_write_cmd_byte(0xC4B1); lcd_spi_write_data_byte(0x08);
    lcd_spi_write_cmd_byte(0xC4B2); lcd_spi_write_data_byte(0x05);
    lcd_spi_write_cmd_byte(0xC4B3); lcd_spi_write_data_byte(0x00);
 
    lcd_spi_write_cmd_byte(0xC090); lcd_spi_write_data_byte(0x00);
    lcd_spi_write_cmd_byte(0xC091); lcd_spi_write_data_byte(0x0F);
    lcd_spi_write_cmd_byte(0xC092); lcd_spi_write_data_byte(0x00);
    lcd_spi_write_cmd_byte(0xC093); lcd_spi_write_data_byte(0x15);
    lcd_spi_write_cmd_byte(0xC094); lcd_spi_write_data_byte(0x00);
    lcd_spi_write_cmd_byte(0xC095); lcd_spi_write_data_byte(0x17);
 
    lcd_spi_write_cmd_byte(0xC582); lcd_spi_write_data_byte(0x01);
 
    lcd_spi_write_cmd_byte(0xC590); lcd_spi_write_data_byte(0x47);
 
    lcd_spi_write_cmd_byte(0xD800); lcd_spi_write_data_byte(0x48);
    lcd_spi_write_cmd_byte(0xD801); lcd_spi_write_data_byte(0x48);
 
    lcd_spi_write_cmd_byte(0xD900); lcd_spi_write_data_byte(0xB0);
 
    lcd_spi_write_cmd_byte(0xB391); lcd_spi_write_data_byte(0x10);//C0
    lcd_spi_write_cmd_byte(0xB392); lcd_spi_write_data_byte(0x25);//25
 
    lcd_spi_write_cmd_byte(0xC181); lcd_spi_write_data_byte(0x77);
 
    lcd_spi_write_cmd_byte(0x3A81); lcd_spi_write_data_byte(0x55);
 
    lcd_spi_write_cmd_byte(0xFF90); lcd_spi_write_data_byte(0x02);
 
    lcd_spi_write_cmd_byte(0xFF93); lcd_spi_write_data_byte(0x20);
    lcd_spi_write_cmd_byte(0xE100); lcd_spi_write_data_byte(0x00);
    lcd_spi_write_cmd_byte(0xE101); lcd_spi_write_data_byte(0x05);
    lcd_spi_write_cmd_byte(0xE102); lcd_spi_write_data_byte(0x09);
    lcd_spi_write_cmd_byte(0xE103); lcd_spi_write_data_byte(0x04);
    lcd_spi_write_cmd_byte(0xE104); lcd_spi_write_data_byte(0x02);
    lcd_spi_write_cmd_byte(0xE105); lcd_spi_write_data_byte(0x0B);
    lcd_spi_write_cmd_byte(0xE106); lcd_spi_write_data_byte(0x0A);
    lcd_spi_write_cmd_byte(0xE107); lcd_spi_write_data_byte(0x09);
    lcd_spi_write_cmd_byte(0xE108); lcd_spi_write_data_byte(0x05);
    lcd_spi_write_cmd_byte(0xE109); lcd_spi_write_data_byte(0x08);
    lcd_spi_write_cmd_byte(0xE10A); lcd_spi_write_data_byte(0x10);
    lcd_spi_write_cmd_byte(0xE10B); lcd_spi_write_data_byte(0x05);
    lcd_spi_write_cmd_byte(0xE10C); lcd_spi_write_data_byte(0x06);
    lcd_spi_write_cmd_byte(0xE10D); lcd_spi_write_data_byte(0x11);
    lcd_spi_write_cmd_byte(0xE10E); lcd_spi_write_data_byte(0x09);
    lcd_spi_write_cmd_byte(0xE10F); lcd_spi_write_data_byte(0x01);
 
    lcd_spi_write_cmd_byte(0xE200); lcd_spi_write_data_byte(0x00);
    lcd_spi_write_cmd_byte(0xE201); lcd_spi_write_data_byte(0x05);
    lcd_spi_write_cmd_byte(0xE202); lcd_spi_write_data_byte(0x09);
    lcd_spi_write_cmd_byte(0xE203); lcd_spi_write_data_byte(0x04);
    lcd_spi_write_cmd_byte(0xE204); lcd_spi_write_data_byte(0x02);
    lcd_spi_write_cmd_byte(0xE205); lcd_spi_write_data_byte(0x0B);
    lcd_spi_write_cmd_byte(0xE206); lcd_spi_write_data_byte(0x0A);
    lcd_spi_write_cmd_byte(0xE207); lcd_spi_write_data_byte(0x09);
    lcd_spi_write_cmd_byte(0xE208); lcd_spi_write_data_byte(0x05);
    lcd_spi_write_cmd_byte(0xE209); lcd_spi_write_data_byte(0x08);
    lcd_spi_write_cmd_byte(0xE20A); lcd_spi_write_data_byte(0x10);
    lcd_spi_write_cmd_byte(0xE20B); lcd_spi_write_data_byte(0x05);
    lcd_spi_write_cmd_byte(0xE20C); lcd_spi_write_data_byte(0x06);
    lcd_spi_write_cmd_byte(0xE20D); lcd_spi_write_data_byte(0x11);
    lcd_spi_write_cmd_byte(0xE20E); lcd_spi_write_data_byte(0x09);
    lcd_spi_write_cmd_byte(0xE20F); lcd_spi_write_data_byte(0x01);
 
    lcd_spi_write_cmd_byte(0xFF80); lcd_spi_write_data_byte(0x00);
    lcd_spi_write_cmd_byte(0xFF81); lcd_spi_write_data_byte(0x00);
 
    lcd_spi_write_cmd_byte(0xFF00); lcd_spi_write_data_byte(0xFF);
    lcd_spi_write_cmd_byte(0xFF01); lcd_spi_write_data_byte(0xFF);
    lcd_spi_write_cmd_byte(0xFF02); lcd_spi_write_data_byte(0xFF);
 
    lcd_spi_write_cmd_byte(0x3500); lcd_spi_write_data_byte(0x00);
 
    lcd_spi_write_cmd_byte(0x3600); lcd_spi_write_data_byte(0x00);     //00
 
    lcd_spi_write_cmd_byte(0x1100);
    mdelay(120);
 
    lcd_spi_write_cmd_byte(0x2900);
    mdelay(20);

} 












static void ILI9488_LG35_init(void)
{
	lcd_gpio_init();
   
    TFT_FN043_init_code();
	
//    TFT_STD4802_init_code();
    
    lcd_gpio_clear();
	
	printk("LDSH------------------------------------------------:ILI9488_LG35_init\n");
	return;
}



/***************late*******************************/


static void __init imx6q_init_late(void)
{
	/*
	 * WAIT mode is broken on TO 1.0 and 1.1, so there is no point
	 * to run cpuidle on them.
	 */

    usb_reset();    //usb_reset

    ILI9488_LG35_init();   //lcd

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
