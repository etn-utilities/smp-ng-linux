// SPDX-License-Identifier: GPL-2.0+
//
// MXC GPIO support. (c) 2008 Daniel Mack <daniel@caiaq.de>
// Copyright 2008 Juergen Beisert, kernel@pengutronix.de
//
// Based on code from Freescale Semiconductor,
// Authors: Daniel Mack, Juergen Beisert.
// Copyright (C) 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/syscore_ops.h>
#include <linux/gpio/driver.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/bug.h>
#ifdef CONFIG_GPIO_MXC_PAD_WAKEUP
#include <linux/firmware/imx/sci.h>

#define IMX_SC_PAD_FUNC_GET_WAKEUP	9
#define IMX_SC_PAD_FUNC_SET_WAKEUP	4
#define IMX_SC_PAD_WAKEUP_OFF		0
#endif

#ifdef CONFIG_GPIO_MXC_PAD_WAKEUP
struct mxc_gpio_pad_wakeup {
	u32 pin_id;
	u32 type;
	u32 line;
};

struct imx_sc_msg_gpio_get_pad_wakeup {
	struct imx_sc_rpc_msg hdr;
	union {
		struct req_pad {
			u16 pad;
		} __packed req;
		struct resp_wakeup {
			u8 wakeup;
		} resp;
	} data;
} __packed __aligned(4);

struct imx_sc_msg_gpio_set_pad_wakeup {
	struct imx_sc_rpc_msg hdr;
	u16 pad;
	u8 wakeup;
} __packed __aligned(4);

#endif

/* device type dependent stuff */
struct mxc_gpio_hwdata {
	unsigned dr_reg;
	unsigned gdir_reg;
	unsigned psr_reg;
	unsigned icr1_reg;
	unsigned icr2_reg;
	unsigned imr_reg;
	unsigned isr_reg;
	int edge_sel_reg;
	unsigned low_level;
	unsigned high_level;
	unsigned rise_edge;
	unsigned fall_edge;
};

struct mxc_gpio_reg_saved {
	u32 icr1;
	u32 icr2;
	u32 imr;
	u32 gdir;
	u32 edge_sel;
	u32 dr;
};

struct mxc_gpio_port {
	struct list_head node;
	void __iomem *base;
	struct clk *clk;
	int irq;
	int irq_high;
	struct irq_domain *domain;
	struct gpio_chip gc;
	struct device *dev;
	u32 both_edges;
	struct mxc_gpio_reg_saved gpio_saved_reg;
	bool power_off;
	const struct mxc_gpio_hwdata *hwdata;
	bool gpio_ranges;
#ifdef CONFIG_GPIO_MXC_PAD_WAKEUP
	u32 pad_wakeup_num;
	struct mxc_gpio_pad_wakeup pad_wakeup[32];
#endif

	struct clk *clkRdc;
	struct clk *clkSem;
	void __iomem *pRDC;
	void __iomem *pSEMA42;
};

#ifdef CONFIG_GPIO_MXC_PAD_WAKEUP
static struct imx_sc_ipc *gpio_ipc_handle;
#endif

static struct mxc_gpio_hwdata imx1_imx21_gpio_hwdata = {
	.dr_reg		= 0x1c,
	.gdir_reg	= 0x00,
	.psr_reg	= 0x24,
	.icr1_reg	= 0x28,
	.icr2_reg	= 0x2c,
	.imr_reg	= 0x30,
	.isr_reg	= 0x34,
	.edge_sel_reg	= -EINVAL,
	.low_level	= 0x03,
	.high_level	= 0x02,
	.rise_edge	= 0x00,
	.fall_edge	= 0x01,
};

static struct mxc_gpio_hwdata imx31_gpio_hwdata = {
	.dr_reg		= 0x00,
	.gdir_reg	= 0x04,
	.psr_reg	= 0x08,
	.icr1_reg	= 0x0c,
	.icr2_reg	= 0x10,
	.imr_reg	= 0x14,
	.isr_reg	= 0x18,
	.edge_sel_reg	= -EINVAL,
	.low_level	= 0x00,
	.high_level	= 0x01,
	.rise_edge	= 0x02,
	.fall_edge	= 0x03,
};

static struct mxc_gpio_hwdata imx35_gpio_hwdata = {
	.dr_reg		= 0x00,
	.gdir_reg	= 0x04,
	.psr_reg	= 0x08,
	.icr1_reg	= 0x0c,
	.icr2_reg	= 0x10,
	.imr_reg	= 0x14,
	.isr_reg	= 0x18,
	.edge_sel_reg	= 0x1c,
	.low_level	= 0x00,
	.high_level	= 0x01,
	.rise_edge	= 0x02,
	.fall_edge	= 0x03,
};

#define GPIO_DR			(port->hwdata->dr_reg)
#define GPIO_GDIR		(port->hwdata->gdir_reg)
#define GPIO_PSR		(port->hwdata->psr_reg)
#define GPIO_ICR1		(port->hwdata->icr1_reg)
#define GPIO_ICR2		(port->hwdata->icr2_reg)
#define GPIO_IMR		(port->hwdata->imr_reg)
#define GPIO_ISR		(port->hwdata->isr_reg)
#define GPIO_EDGE_SEL		(port->hwdata->edge_sel_reg)

#define GPIO_INT_LOW_LEV	(port->hwdata->low_level)
#define GPIO_INT_HIGH_LEV	(port->hwdata->high_level)
#define GPIO_INT_RISE_EDGE	(port->hwdata->rise_edge)
#define GPIO_INT_FALL_EDGE	(port->hwdata->fall_edge)
#define GPIO_INT_BOTH_EDGES	0x4


///////////////////////////////////////
///////////////////////////////////////
// RDC access protection add-on
// Code based on NXP/VARISCITE var-mcuxpresso RDC


typedef enum _rdc_master
{
    kRDC_Master_A53                 = 0U,          /**< ARM Cortex-A53 RDC Master */
    kRDC_Master_M4                  = 1U,          /**< ARM Cortex-M4 RDC Master */
    kRDC_Master_PCIE1               = 2U,          /**< PCIE1 RDC Master */
    kRDC_Master_PCIE2               = 3U,          /**< PCIE2 RDC Master */
    kRDC_Master_VPU                 = 4U,          /**< VPU RDC Master */
    kRDC_Master_LCDIF               = 5U,          /**< LCDIF RDC Master */
    kRDC_Master_CSI1                = 6U,          /**< CSI1 PORT RDC Master */
    kRDC_Master_CSI2                = 7U,          /**< CSI2 RDC Master */
    kRDC_Master_Coresight           = 8U,          /**< CORESIGHT RDC Master */
    kRDC_Master_DAP                 = 9U,          /**< DAP RDC Master */
    kRDC_Master_CAAM                = 10U,         /**< CAAM RDC Master */
    kRDC_Master_SDMA1_PERIPH        = 11U,         /**< SDMA1 PERIPHERAL RDC Master */
    kRDC_Master_SDMA1_BURST         = 12U,         /**< SDMA1 BURST RDC Master */
    kRDC_Master_APBHDMA             = 13U,         /**< APBH DMA RDC Master */
    kRDC_Master_RAWNAND             = 14U,         /**< RAW NAND RDC Master */
    kRDC_Master_USDHC1              = 15U,         /**< USDHC1 RDC Master */
    kRDC_Master_USDHC2              = 16U,         /**< USDHC2 RDC Master */
    kRDC_Master_DP                  = 17U,         /**< DP RDC Master */
    kRDC_Master_GPU                 = 18U,         /**< GPU RDC Master */
    kRDC_Master_USB1                = 19U,         /**< USB1 RDC Master */
    kRDC_Master_USB2                = 20U,         /**< USB2 RDC Master */
    kRDC_Master_TESTPORT            = 21U,         /**< TESTPORT RDC Master */
    kRDC_Master_ENET1TX             = 22U,         /**< ENET1 TX RDC Master */
    kRDC_Master_ENET1RX             = 23U,         /**< ENET1 RX RDC Master */
    kRDC_Master_SDMA2_PERIPH        = 24U,         /**< SDMA2 PERIPH RDC Master */
    kRDC_Master_SDMA2_BURST         = 24U,         /**< SDMA2 BURST RDC Master */
    kRDC_Master_SDMA2_SPDA2         = 24U,         /**< SDMA2 to SPDA2 RDC Master */
    kRDC_Master_SDMA1_SPBA1         = 26U,         /**< SDMA1 to SPBA1 RDC Master */
} rdc_master_t;


typedef enum _rdc_periph
{
    kRDC_Periph_GPIO1               = 0U,          /**< GPIO1 RDC Peripheral */
    kRDC_Periph_GPIO2               = 1U,          /**< GPIO2 RDC Peripheral */
    kRDC_Periph_GPIO3               = 2U,          /**< GPIO3 RDC Peripheral */
    kRDC_Periph_GPIO4               = 3U,          /**< GPIO4 RDC Peripheral */
    kRDC_Periph_GPIO5               = 4U,          /**< GPIO5 RDC Peripheral */
    kRDC_Periph_ANA_TSENSOR         = 6U,          /**< ANA_TSENSOR RDC Peripheral */
    kRDC_Periph_ANA_OSC             = 7U,          /**< ANA_OSC RDC Peripheral */
    kRDC_Periph_WDOG1               = 8U,          /**< WDOG1 RDC Peripheral */
    kRDC_Periph_WDOG2               = 9U,          /**< WDOG2 RDC Peripheral */
    kRDC_Periph_WDOG3               = 10U,         /**< WDOG3 RDC Peripheral */
    kRDC_Periph_SDMA2               = 12U,         /**< SDMA2 RDC Peripheral */
    kRDC_Periph_GPT1                = 13U,         /**< GPT1 RDC Peripheral */
    kRDC_Periph_GPT2                = 14U,         /**< GPT2 RDC Peripheral */
    kRDC_Periph_GPT3                = 15U,         /**< GPT3 RDC Peripheral */
    kRDC_Periph_ROMCP               = 17U,         /**< ROMCP RDC Peripheral */
    kRDC_Periph_LCDIF               = 18U,         /**< LCDIF RDC Peripheral */
    kRDC_Periph_IOMUXC              = 19U,         /**< IOMUXC RDC Peripheral */
    kRDC_Periph_IOMUXC_GPR          = 20U,         /**< IOMUXC_GPR RDC Peripheral */
    kRDC_Periph_OCOTP_CTRL          = 21U,         /**< OCOTP_CTRL RDC Peripheral */
    kRDC_Periph_ANA_PLL             = 22U,         /**< ANA_PLL RDC Peripheral */
    kRDC_Periph_SNVS_HP             = 23U,         /**< SNVS_HP GPR RDC Peripheral */
    kRDC_Periph_CCM                 = 24U,         /**< CCM RDC Peripheral */
    kRDC_Periph_SRC                 = 25U,         /**< SRC RDC Peripheral */
    kRDC_Periph_GPC                 = 26U,         /**< GPC RDC Peripheral */
    kRDC_Periph_SEMAPHORE1          = 27U,         /**< SEMAPHORE1 RDC Peripheral */
    kRDC_Periph_SEMAPHORE2          = 28U,         /**< SEMAPHORE2 RDC Peripheral */
    kRDC_Periph_RDC                 = 29U,         /**< RDC RDC Peripheral */
    kRDC_Periph_CSU                 = 30U,         /**< CSU RDC Peripheral */
    kRDC_Periph_DC_MST0             = 32U,         /**< DC_MST0 RDC Peripheral */
    kRDC_Periph_DC_MST1             = 33U,         /**< DC_MST1 RDC Peripheral */
    kRDC_Periph_DC_MST2             = 34U,         /**< DC_MST2 RDC Peripheral */
    kRDC_Periph_DC_MST3             = 35U,         /**< DC_MST3 RDC Peripheral */
    kRDC_Periph_HDMI_SEC            = 36U,         /**< HDMI_SEC RDC Peripheral */
    kRDC_Periph_PWM1                = 38U,         /**< PWM1 RDC Peripheral */
    kRDC_Periph_PWM2                = 39U,         /**< PWM2 RDC Peripheral */
    kRDC_Periph_PWM3                = 40U,         /**< PWM3 RDC Peripheral */
    kRDC_Periph_PWM4                = 41U,         /**< PWM4 RDC Peripheral */
    kRDC_Periph_SYS_COUNTER_RD      = 42U,         /**< System counter read RDC Peripheral */
    kRDC_Periph_SYS_COUNTER_CMP     = 43U,         /**< System counter compare RDC Peripheral */
    kRDC_Periph_SYS_COUNTER_CTRL    = 44U,         /**< System counter control RDC Peripheral */
    kRDC_Periph_HDMI_CTRL           = 45U,         /**< HDMI_CTRL RDC Peripheral */
    kRDC_Periph_GPT6                = 46U,         /**< GPT6 RDC Peripheral */
    kRDC_Periph_GPT5                = 47U,         /**< GPT5 RDC Peripheral */
    kRDC_Periph_GPT4                = 48U,         /**< GPT4 RDC Peripheral */
    kRDC_Periph_TZASC               = 56U,         /**< TZASC RDC Peripheral */
    kRDC_Periph_MTR                 = 59U,         /**< MTR RDC Peripheral */
    kRDC_Periph_PERFMON1            = 60U,         /**< PERFMON1 RDC Peripheral */
    kRDC_Periph_PERFMON2            = 61U,         /**< PERFMON2 RDC Peripheral */
    kRDC_Periph_PLATFORM_CTRL       = 62U,         /**< PLATFORM_CTRL RDC Peripheral */
    kRDC_Periph_QOSC                = 63U,         /**< QOSC RDC Peripheral */
    kRDC_Periph_MIPI_PHY            = 64U,         /**< MIPI_PHY RDC Peripheral */
    kRDC_Periph_MIPI_DSI            = 65U,         /**< MIPI_DSI RDC Peripheral */
    kRDC_Periph_I2C1                = 66U,         /**< I2C1 RDC Peripheral */
    kRDC_Periph_I2C2                = 67U,         /**< I2C2 RDC Peripheral */
    kRDC_Periph_I2C3                = 68U,         /**< I2C3 RDC Peripheral */
    kRDC_Periph_I2C4                = 69U,         /**< I2C4 RDC Peripheral */
    kRDC_Periph_UART4               = 70U,         /**< UART4 RDC Peripheral */
    kRDC_Periph_MIPI_CSI1           = 71U,         /**< MIPI_CSI1 RDC Peripheral */
    kRDC_Periph_MIPI_CSI_PHY1       = 72U,         /**< MIPI_CSI_PHY1 RDC Peripheral */
    kRDC_Periph_CSI1                = 73U,         /**< CSI1 RDC Peripheral */
    kRDC_Periph_MU_A                = 74U,         /**< MU_A RDC Peripheral */
    kRDC_Periph_MU_B                = 75U,         /**< MU_B RDC Peripheral */
    kRDC_Periph_SEMAPHORE_HS        = 76U,         /**< SEMAPHORE_HS RDC Peripheral */
    kRDC_Periph_SAI1                = 78U,         /**< SAI1 RDC Peripheral */
    kRDC_Periph_SAI6                = 80U,         /**< SAI6 RDC Peripheral */
    kRDC_Periph_SAI5                = 81U,         /**< SAI5 RDC Peripheral */
    kRDC_Periph_SAI4                = 82U,         /**< SAI4 RDC Peripheral */
    kRDC_Periph_USDHC1              = 84U,         /**< USDHC1 RDC Peripheral */
    kRDC_Periph_USDHC2              = 85U,         /**< USDHC2 RDC Peripheral */
    kRDC_Periph_MIPI_CSI2           = 86U,         /**< MIPI_CSI2 RDC Peripheral */
    kRDC_Periph_MIPI_CSI_PHY2       = 87U,         /**< MIPI_CSI_PHY2 RDC Peripheral */
    kRDC_Periph_CSI2                = 88U,         /**< CSI2 RDC Peripheral */
    kRDC_Periph_SPBA2               = 90U,         /**< SPBA2 RDC Peripheral */
    kRDC_Periph_QSPI                = 91U,         /**< QSPI RDC Peripheral */
    kRDC_Periph_SDMA1               = 93U,         /**< SDMA1 RDC Peripheral */
    kRDC_Periph_ENET1               = 94U,         /**< ENET1 RDC Peripheral */
    kRDC_Periph_SPDIF1              = 97U,         /**< SPDIF1 RDC Peripheral */
    kRDC_Periph_ECSPI1              = 98U,         /**< ECSPI1 RDC Peripheral */
    kRDC_Periph_ECSPI2              = 99U,         /**< ECSPI2 RDC Peripheral */
    kRDC_Periph_ECSPI3              = 100U,        /**< ECSPI3 RDC Peripheral */
    kRDC_Periph_UART1               = 102U,        /**< UART1 RDC Peripheral */
    kRDC_Periph_UART3               = 104U,        /**< UART3 RDC Peripheral */
    kRDC_Periph_UART2               = 105U,        /**< UART2 RDC Peripheral */
    kRDC_Periph_SPDIF2              = 106U,        /**< SPDIF2 RDC Peripheral */
    kRDC_Periph_SAI2                = 107U,        /**< SAI2 RDC Peripheral */
    kRDC_Periph_SAI3                = 108U,        /**< SAI3 RDC Peripheral */
    kRDC_Periph_SPBA1               = 111U,        /**< SPBA1 RDC Peripheral */
    kRDC_Periph_CAAM                = 114U,        /**< CAAM RDC Peripheral */
} rdc_periph_t;


/** RDC - Register Layout Typedef */
typedef struct
{
    u32     VIR; /**< Version Information, offset: 0x0 */
    u8      RESERVED_0[32];
    u32     STAT;    /**< Status, offset: 0x24 */
    u32     INTCTRL; /**< Interrupt and Control, offset: 0x28 */
    u32     INTSTAT; /**< Interrupt Status, offset: 0x2C */
    u8      RESERVED_1[464];
    u32     MDA[40]; /**< Master Domain Assignment, array offset: 0x200, array step: 0x4 */
    u8      RESERVED_2[352];
    u32     PDAP[112]; /**< Peripheral Domain Access Permissions, array offset: 0x400, array step: 0x4 */
    u8      RESERVED_3[576];
    struct
    {                       /* offset: 0x800, array step: 0x10 */
        u32     MRSA; /**< Memory Region Start Address, array offset: 0x800, array step: 0x10 */
        u32     MREA; /**< Memory Region End Address, array offset: 0x804, array step: 0x10 */
        u32     MRC;  /**< Memory Region Control, array offset: 0x808, array step: 0x10 */
        u32     MRVS; /**< Memory Region Violation Status, array offset: 0x80C, array step: 0x10 */
    } MR[77];
} RDC_Type;


/*
 * Master index:
 * All masters excluding ARM core: 0
 * A53 core: 1
 * M4 core: 6
 * SDMA 3
 */
#define MASTER_INDEX 1
#define DOMAIN_ID 0


typedef struct _rdc_domain_assignment
{
    u32 domainId : 2U; /*!< Domain ID.                  */
    u32 : 29U;         /*!< Reserved.                   */
    u32 lock : 1U;     /*!< Lock the domain assignment. */
} rdc_domain_assignment_t;


typedef union
{
    rdc_domain_assignment_t _mda;
    u32 _u32;
} rdc_mda_reg_t;


typedef struct _rdc_periph_access_config
{
    rdc_periph_t periph; /*!< Peripheral name.                 */
    bool lock;           /*!< Lock the permission until reset. */
    bool enableSema;     /*!< Enable semaphore or not, when enabled, master should
                              call @ref RDC_SEMA42_Lock to lock the semaphore gate
                              accordingly before access the peripheral. */
    u16 policy;     /*!< Access policy.                   */
} rdc_periph_access_config_t;


void RDC_SEMA42_Lock(void *pSem)
{
    u8  masterIndex = MASTER_INDEX;
    u8  domainId = DOMAIN_ID;
    //u32 tryCnt = 0;
    u8  regGate;


    if (!pSem) {
        return;
    }

    ++masterIndex;
    regGate = (u8)((domainId << 4) | masterIndex);

    while (regGate != readb(pSem))
    {
        /* Wait for unlocked status. */
        if (0U == (readb(pSem) & 0x0f)) 
        {
            /* Lock the gate. */
            writeb(masterIndex, pSem);
        }

        #if 0
        tryCnt++;
        if (tryCnt > 100000) {
            printk(KERN_INFO "%s  (%d)  %s:  gate locked  pGate: %08x  %08x\n"
                , __FILE__ , __LINE__, __func__
                , (u32)pSem
                , readb(pSem)
            );
            tryCnt = 0;
            break;
        }
        #endif
    }
}


void RDC_SEMA42_Unlock(void *pSem)
{
    if (pSem) {
        writeb(0, pSem);
    }
}


void RDC_SetMasterDomainAssignment(RDC_Type *base, rdc_master_t master, const rdc_domain_assignment_t *domainAssignment)
{
    rdc_mda_reg_t mda;

    mda._mda = *domainAssignment;
    //base->MDA[master] = mda._u32;
	writel(mda._u32, &base->MDA[master]);
}


void rdc_Init(struct platform_device *pdev, struct mxc_gpio_port *port)
{
    int i1;

    port->pRDC = 0;
    port->pSEMA42 = 0;

    for (i1=0; i1<pdev->num_resources; i1++) {
        // imx_rdc reg
        if (strncmp(pdev->resource[i1].name, "rdc_base", 8) == 0) {
            port->pRDC = devm_platform_ioremap_resource(pdev, i1);
            printk(KERN_INFO "%s  (%d)  %s:  %.20s (%08x)  resIdx:%d  %.20s  %08x  %08x\n"
                , __FILE__ , __LINE__, __func__
                , pdev->name
                , (u32)port->base
                , i1
                , pdev->resource[i1].name
                , pdev->resource[i1].start
                , pdev->resource[i1].end
            );
        }

        // sema42 reg
        if (strncmp(pdev->resource[i1].name, "sem_base", 8) == 0) {
            port->pSEMA42 = devm_platform_ioremap_resource(pdev, i1);
            printk(KERN_INFO "%s  (%d)  %s:  %.20s (%08x)  resIdx:%d  %.20s  %08x  %08x  %08x\n"
                , __FILE__ , __LINE__, __func__
                , pdev->name
                , (u32)port->base
                , i1
                , pdev->resource[i1].name
                , pdev->resource[i1].start
                , pdev->resource[i1].end
                , (u32)port->pSEMA42
            );
        }
    }

    if (port->pRDC) {
        rdc_domain_assignment_t assignment;
        assignment.domainId = 0;
        RDC_SetMasterDomainAssignment(port->pRDC, kRDC_Master_A53, &assignment);
    }
}


static void reg_write8(unsigned long data, void __iomem *reg, void *pSem)
{
    RDC_SEMA42_Lock(pSem);
	writeb(data, reg);
    RDC_SEMA42_Unlock(pSem);
}

static unsigned long reg_read8(void __iomem *reg, void *pSem)
{
    unsigned long retVal;

    RDC_SEMA42_Lock(pSem);
	retVal = readb(reg);
    RDC_SEMA42_Unlock(pSem);
    return retVal;
}

static void reg_write16(unsigned long data, void __iomem *reg, void *pSem)
{
    RDC_SEMA42_Lock(pSem);
	writew(data, reg);
    RDC_SEMA42_Unlock(pSem);
}

static unsigned long reg_read16(void __iomem *reg, void *pSem)
{
    unsigned long retVal;

    RDC_SEMA42_Lock(pSem);
	retVal = readw(reg);
    RDC_SEMA42_Unlock(pSem);
    return retVal;
}

static void reg_write32(unsigned long data, void __iomem *reg, void *pSem)
{
    RDC_SEMA42_Lock(pSem);
	writel(data, reg);
    RDC_SEMA42_Unlock(pSem);
}

static unsigned long reg_read32(void __iomem *reg, void *pSem)
{
    unsigned long retVal;

    RDC_SEMA42_Lock(pSem);
	retVal = readl(reg);
    RDC_SEMA42_Unlock(pSem);
    return retVal;
}

static void reg_write32_bgpio(void __iomem *reg, unsigned long data, void __iomem *pSem)
{
    reg_write32(data, reg, pSem);
}

static unsigned long reg_read32_bgpio(void __iomem *reg, void __iomem *pSem)
{
    return reg_read32(reg, pSem);
}


static void reg_write32Gc(unsigned int data, void __iomem *reg,  void __iomem *pSem)
{
    reg_write32(data, reg, pSem);
}

static unsigned int reg_read32Gc(void __iomem *reg, void __iomem *pSem)
{
    return (unsigned int)reg_read32(reg, pSem);
}

// End RDC add-on
///////////////////////////////////////
///////////////////////////////////////


static const struct platform_device_id mxc_gpio_devtype[] = {
	{
		.name = "imx1-gpio",
		.driver_data = IMX1_GPIO,
	}, {
		.name = "imx21-gpio",
		.driver_data = IMX21_GPIO,
	}, {
		.name = "imx31-gpio",
		.driver_data = IMX31_GPIO,
	}, {
		.name = "imx35-gpio",
		.driver_data = IMX35_GPIO,
	}, {
		/* sentinel */
	}
};

static const struct of_device_id mxc_gpio_dt_ids[] = {
	{ .compatible = "fsl,imx1-gpio", .data =  &imx1_imx21_gpio_hwdata },
	{ .compatible = "fsl,imx21-gpio", .data = &imx1_imx21_gpio_hwdata },
	{ .compatible = "fsl,imx31-gpio", .data = &imx31_gpio_hwdata },
	{ .compatible = "fsl,imx35-gpio", .data = &imx35_gpio_hwdata },
	{ .compatible = "fsl,imx7d-gpio", .data = &imx35_gpio_hwdata },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mxc_gpio_dt_ids);

/*
 * MX2 has one interrupt *for all* gpio ports. The list is used
 * to save the references to all ports, so that mx2_gpio_irq_handler
 * can walk through all interrupt status registers.
 */
static LIST_HEAD(mxc_gpio_ports);

/* Note: This driver assumes 32 GPIOs are handled in one register */

static int gpio_set_irq_type(struct irq_data *d, u32 type)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct mxc_gpio_port *port = gc->private;
	u32 bit, val;
	u32 gpio_idx = d->hwirq;
	int edge;
	void __iomem *reg = port->base;

	port->both_edges &= ~(1 << gpio_idx);
	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		edge = GPIO_INT_RISE_EDGE;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		edge = GPIO_INT_FALL_EDGE;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		if (GPIO_EDGE_SEL >= 0) {
			edge = GPIO_INT_BOTH_EDGES;
		} else {
			val = port->gc.get(&port->gc, gpio_idx);
			if (val) {
				edge = GPIO_INT_LOW_LEV;
				pr_debug("mxc: set GPIO %d to low trigger\n", gpio_idx);
			} else {
				edge = GPIO_INT_HIGH_LEV;
				pr_debug("mxc: set GPIO %d to high trigger\n", gpio_idx);
			}
			port->both_edges |= 1 << gpio_idx;
		}
		break;
	case IRQ_TYPE_LEVEL_LOW:
		edge = GPIO_INT_LOW_LEV;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		edge = GPIO_INT_HIGH_LEV;
		break;
	default:
		return -EINVAL;
	}

	if (GPIO_EDGE_SEL >= 0) {
		val = reg_read32(port->base + GPIO_EDGE_SEL, port->pSEMA42);
		if (edge == GPIO_INT_BOTH_EDGES)
			reg_write32(val | (1 << gpio_idx),
				port->base + GPIO_EDGE_SEL, port->pSEMA42);
		else
			reg_write32(val & ~(1 << gpio_idx),
				port->base + GPIO_EDGE_SEL, port->pSEMA42);
	}

	if (edge != GPIO_INT_BOTH_EDGES) {
		reg += GPIO_ICR1 + ((gpio_idx & 0x10) >> 2); /* lower or upper register */
		bit = gpio_idx & 0xf;
		val = reg_read32(reg, port->pSEMA42) & ~(0x3 << (bit << 1));
		reg_write32(val | (edge << (bit << 1)), reg, port->pSEMA42);
	}

	reg_write32(1 << gpio_idx, port->base + GPIO_ISR, port->pSEMA42);

	return 0;
}

static void mxc_flip_edge(struct mxc_gpio_port *port, u32 gpio)
{
	void __iomem *reg = port->base;
	u32 bit, val;
	int edge;

	reg += GPIO_ICR1 + ((gpio & 0x10) >> 2); /* lower or upper register */
	bit = gpio & 0xf;
	val = reg_read32(reg, port->pSEMA42);
	edge = (val >> (bit << 1)) & 3;
	val &= ~(0x3 << (bit << 1));
	if (edge == GPIO_INT_HIGH_LEV) {
		edge = GPIO_INT_LOW_LEV;
		pr_debug("mxc: switch GPIO %d to low trigger\n", gpio);
	} else if (edge == GPIO_INT_LOW_LEV) {
		edge = GPIO_INT_HIGH_LEV;
		pr_debug("mxc: switch GPIO %d to high trigger\n", gpio);
	} else {
		pr_err("mxc: invalid configuration for GPIO %d: %x\n",
		       gpio, edge);
		return;
	}
	reg_write32(val | (edge << (bit << 1)), reg, port->pSEMA42);
}

/* handle 32 interrupts in one status register */
static void mxc_gpio_irq_handler(struct mxc_gpio_port *port, u32 irq_stat)
{
	while (irq_stat != 0) {
		int irqoffset = fls(irq_stat) - 1;

		if (port->both_edges & (1 << irqoffset))
			mxc_flip_edge(port, irqoffset);

		generic_handle_domain_irq(port->domain, irqoffset);

		irq_stat &= ~(1 << irqoffset);
	}
}

/* MX1 and MX3 has one interrupt *per* gpio port */
static void mx3_gpio_irq_handler(struct irq_desc *desc)
{
	u32 irq_stat;
	struct mxc_gpio_port *port = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);

	chained_irq_enter(chip, desc);

	irq_stat = reg_read32(port->base + GPIO_ISR, port->pSEMA42) & reg_read32(port->base + GPIO_IMR, port->pSEMA42);

	mxc_gpio_irq_handler(port, irq_stat);

	chained_irq_exit(chip, desc);
}

/* MX2 has one interrupt *for all* gpio ports */
static void mx2_gpio_irq_handler(struct irq_desc *desc)
{
	u32 irq_msk, irq_stat;
	struct mxc_gpio_port *port;
	struct irq_chip *chip = irq_desc_get_chip(desc);

	chained_irq_enter(chip, desc);

	/* walk through all interrupt status registers */
	list_for_each_entry(port, &mxc_gpio_ports, node) {
		irq_msk = reg_read32(port->base + GPIO_IMR, port->pSEMA42);
		if (!irq_msk)
			continue;

		irq_stat = reg_read32(port->base + GPIO_ISR, port->pSEMA42) & irq_msk;
		if (irq_stat)
			mxc_gpio_irq_handler(port, irq_stat);
	}
	chained_irq_exit(chip, desc);
}

#ifdef CONFIG_GPIO_MXC_PAD_WAKEUP
static int mxc_gpio_get_pad_wakeup(struct mxc_gpio_port *port)
{
	struct imx_sc_msg_gpio_get_pad_wakeup msg;
	struct imx_sc_rpc_msg *hdr = &msg.hdr;
	u8 wakeup_type;
	int ret;
	int i;

	for (i = 0; i < port->pad_wakeup_num; i++) {

		hdr->ver = IMX_SC_RPC_VERSION;
		hdr->svc = IMX_SC_RPC_SVC_PAD;
		hdr->func = IMX_SC_PAD_FUNC_GET_WAKEUP;
		hdr->size = 2;

		/* get original pad type */
		wakeup_type = port->pad_wakeup[i].type;
		msg.data.req.pad = port->pad_wakeup[i].pin_id;
		ret = imx_scu_call_rpc(gpio_ipc_handle, &msg, true);
		if (ret) {
			dev_err(port->gc.parent, "get pad wakeup failed, ret %d\n", ret);
			return ret;
		}
		wakeup_type = msg.data.resp.wakeup;
		/* return wakeup gpio pin's line */
		if (wakeup_type != port->pad_wakeup[i].type)
			return port->pad_wakeup[i].line;
	}

	return -EINVAL;
}

static void mxc_gpio_set_pad_wakeup(struct mxc_gpio_port *port, bool enable)
{
	struct imx_sc_msg_gpio_set_pad_wakeup msg;
	struct imx_sc_rpc_msg *hdr = &msg.hdr;
	int ret;
	int i;

	for (i = 0; i < port->pad_wakeup_num; i++) {

		hdr->ver = IMX_SC_RPC_VERSION;
		hdr->svc = IMX_SC_RPC_SVC_PAD;
		hdr->func = IMX_SC_PAD_FUNC_SET_WAKEUP;
		hdr->size = 2;

		msg.pad = port->pad_wakeup[i].pin_id;
		msg.wakeup = enable ? port->pad_wakeup[i].type : IMX_SC_PAD_WAKEUP_OFF;
		ret = imx_scu_call_rpc(gpio_ipc_handle, &msg, true);
		if (ret) {
			dev_err(port->gc.parent, "set pad wakeup failed, ret %d\n", ret);
			return;
		}
	}
}

static void mxc_gpio_handle_pad_wakeup(struct mxc_gpio_port *port, int line)
{
	struct irq_desc *desc = irq_to_desc(port->irq);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u32 irq_stat;

	/* skip invalid line */
	if (line > 31) {
		dev_err(port->gc.parent, "invalid wakeup line %d\n", line);
		return;
	}

	dev_info(port->gc.parent, "wakeup by pad, line %d\n", line);

	chained_irq_enter(chip, desc);

	irq_stat = (1 << line);

	mxc_gpio_irq_handler(port, irq_stat);

	chained_irq_exit(chip, desc);
}
#endif

/*
 * Set interrupt number "irq" in the GPIO as a wake-up source.
 * While system is running, all registered GPIO interrupts need to have
 * wake-up enabled. When system is suspended, only selected GPIO interrupts
 * need to have wake-up enabled.
 * @param  irq          interrupt source number
 * @param  enable       enable as wake-up if equal to non-zero
 * @return       This function returns 0 on success.
 */
static int gpio_set_wake_irq(struct irq_data *d, u32 enable)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct mxc_gpio_port *port = gc->private;
	u32 gpio_idx = d->hwirq;
	int ret;

	if (enable) {
		if (port->irq_high && (gpio_idx >= 16))
			ret = enable_irq_wake(port->irq_high);
		else
			ret = enable_irq_wake(port->irq);
	} else {
		if (port->irq_high && (gpio_idx >= 16))
			ret = disable_irq_wake(port->irq_high);
		else
			ret = disable_irq_wake(port->irq);
	}

	return ret;
}

static int mxc_gpio_irq_reqres(struct irq_data *d)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct mxc_gpio_port *port = gc->private;

	if (gpiochip_lock_as_irq(&port->gc, d->hwirq)) {
		dev_err(port->gc.parent,
			"unable to lock HW IRQ %lu for IRQ\n",
			d->hwirq);
		return -EINVAL;
	}

	return 0;
}

static void mxc_gpio_irq_relres(struct irq_data *d)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct mxc_gpio_port *port = gc->private;

	gpiochip_unlock_as_irq(&port->gc, d->hwirq);
}

static int mxc_gpio_init_gc(struct mxc_gpio_port *port, int irq_base,
			    struct device *dev)
{
	struct irq_chip_generic *gc;
	struct irq_chip_type *ct;
	int rv;

	gc = devm_irq_alloc_generic_chip(port->dev, "gpio-mxc", 1, irq_base,
					 port->base, handle_level_irq);
	if (!gc)
		return -ENOMEM;
	gc->private = port;

	ct = gc->chip_types;
	ct->chip.parent_device = dev;
	ct->chip.irq_ack = irq_gc_ack_set_bit;
	ct->chip.irq_mask = irq_gc_mask_clr_bit;
	ct->chip.irq_unmask = irq_gc_mask_set_bit;
	ct->chip.irq_set_type = gpio_set_irq_type;
	ct->chip.irq_set_wake = gpio_set_wake_irq;
	ct->chip.flags = IRQCHIP_MASK_ON_SUSPEND | IRQCHIP_ENABLE_WAKEUP_ON_SUSPEND;
	ct->chip.irq_request_resources = mxc_gpio_irq_reqres;
	ct->chip.irq_release_resources = mxc_gpio_irq_relres,
	ct->regs.ack = GPIO_ISR;
	ct->regs.mask = GPIO_IMR;

	rv = devm_irq_setup_generic_chip(port->dev, gc, IRQ_MSK(32),
					 IRQ_GC_INIT_NESTED_LOCK,
					 IRQ_NOREQUEST, 0);

    #if 1
    //This must be done after devm_irq_setup_generic_chip
    gc->pSem = port->pSEMA42;
    gc->reg_writelSem = reg_write32Gc;
    gc->reg_readlSem = reg_read32Gc;
    #endif

	return rv;
}

static int mxc_gpio_to_irq(struct gpio_chip *gc, unsigned offset)
{
	struct mxc_gpio_port *port = gpiochip_get_data(gc);

	return irq_find_mapping(port->domain, offset);
}

static int mxc_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	struct mxc_gpio_port *port = gpiochip_get_data(chip);
	int ret;

	if (port->gpio_ranges) {
		ret = gpiochip_generic_request(chip, offset);
		if (ret)
			return ret;
	}

	ret = pm_runtime_get_sync(chip->parent);
	return ret < 0 ? ret : 0;
}

static void mxc_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	struct mxc_gpio_port *port = gpiochip_get_data(chip);

	if (port->gpio_ranges)
		gpiochip_generic_free(chip, offset);
	pm_runtime_put(chip->parent);
}

static int mxc_gpio_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct mxc_gpio_port *port;
	int irq_count;
	int irq_base;
	int err;
#ifdef CONFIG_GPIO_MXC_PAD_WAKEUP
	int i;
#endif

	port = devm_kzalloc(&pdev->dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	port->dev = &pdev->dev;

	port->hwdata = device_get_match_data(&pdev->dev);

	port->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(port->base))
		return PTR_ERR(port->base);

    rdc_Init(pdev, port);

	irq_count = platform_irq_count(pdev);
	if (irq_count < 0)
		return irq_count;

	if (irq_count > 1) {
		port->irq_high = platform_get_irq_optional(pdev, 1);
		if (port->irq_high < 0)
			port->irq_high = 0;
	}

	port->irq = platform_get_irq(pdev, 0);
	if (port->irq < 0)
		return port->irq;

	/* the controller clock is optional */
	port->clk = devm_clk_get_optional(&pdev->dev, NULL);
	if (IS_ERR(port->clk))
		return PTR_ERR(port->clk);

	err = clk_prepare_enable(port->clk);
	if (err) {
		dev_err(&pdev->dev, "Unable to enable clock.\n");
		return err;
	}

    // Add RDC controller clock
	port->clkRdc = devm_clk_get_optional(&pdev->dev, "rdc");
	if (IS_ERR(port->clkRdc))
		return PTR_ERR(port->clkRdc);

	err = clk_prepare_enable(port->clkRdc);
	if (err) {
		dev_err(&pdev->dev, "Unable to enable clock clkRdc.\n");
		return err;
	}

    // Add SEMA42 controller clock
	port->clkSem = devm_clk_get_optional(&pdev->dev, "sem");
	if (IS_ERR(port->clkSem))
		return PTR_ERR(port->clkSem);

	err = clk_prepare_enable(port->clkSem);
	if (err) {
		dev_err(&pdev->dev, "Unable to enable clock clkSem.\n");
		return err;
	}

#ifdef CONFIG_GPIO_MXC_PAD_WAKEUP
	/*
	 * parse pad wakeup info from dtb, each pad has to provide
	 * <pin_id, type, line>, these info should be put in each
	 * gpio node and with a "pad-wakeup-num" to indicate the
	 * total lines are with pad wakeup enabled.
	 */
	if (!of_property_read_u32(np, "pad-wakeup-num", &port->pad_wakeup_num)) {
		if (port->pad_wakeup_num != 0) {
			if (!gpio_ipc_handle) {
				err = imx_scu_get_handle(&gpio_ipc_handle);
				if (err)
					return err;
			}
			for (i = 0; i < port->pad_wakeup_num; i++) {
				of_property_read_u32_index(np, "pad-wakeup",
					i * 3 + 0, &port->pad_wakeup[i].pin_id);
				of_property_read_u32_index(np, "pad-wakeup",
					i * 3 + 1, &port->pad_wakeup[i].type);
				of_property_read_u32_index(np, "pad-wakeup",
					i * 3 + 2, &port->pad_wakeup[i].line);
			}
			err = imx_scu_irq_group_enable(IMX_SC_IRQ_GROUP_WAKE, IMX_SC_IRQ_PAD, true);
			if (err)
				dev_warn(&pdev->dev, "Enable irq failed, GPIO pad wakeup NOT supported\n");
		}
	}
#endif

	if (of_device_is_compatible(np, "fsl,imx7d-gpio"))
		port->power_off = true;

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	err = pm_runtime_get_sync(&pdev->dev);
	if (err < 0)
		goto out_pm_dis;

	/* disable the interrupt and clear the status */
	reg_write32(0, port->base + GPIO_IMR, port->pSEMA42);
	reg_write32(~0, port->base + GPIO_ISR, port->pSEMA42);

	if (of_device_is_compatible(np, "fsl,imx21-gpio")) {
		/*
		 * Setup one handler for all GPIO interrupts. Actually setting
		 * the handler is needed only once, but doing it for every port
		 * is more robust and easier.
		 */
		irq_set_chained_handler(port->irq, mx2_gpio_irq_handler);
	} else {
		/* setup one handler for each entry */
		irq_set_chained_handler_and_data(port->irq,
						 mx3_gpio_irq_handler, port);
		if (port->irq_high > 0)
			/* setup handler for GPIO 16 to 31 */
			irq_set_chained_handler_and_data(port->irq_high,
							 mx3_gpio_irq_handler,
							 port);
	}

    port->gc.pSem = port->pSEMA42;
    port->gc.read_regSem = reg_read32_bgpio;
    port->gc.write_regSem = reg_write32_bgpio;
	err = bgpio_init(&port->gc, &pdev->dev, 4,
			 port->base + GPIO_PSR,
			 port->base + GPIO_DR, NULL,
			 port->base + GPIO_GDIR, NULL,
			 BGPIOF_READ_OUTPUT_REG_SET);
	if (err)
		goto out_bgio;

	if (of_property_read_bool(np, "gpio_ranges"))
		port->gpio_ranges = true;
	else
		port->gpio_ranges = false;

	port->gc.request = mxc_gpio_request;
	port->gc.free = mxc_gpio_free;
	port->gc.parent = &pdev->dev;
	port->gc.to_irq = mxc_gpio_to_irq;
	port->gc.base = (pdev->id < 0) ? of_alias_get_id(np, "gpio") * 32 :
					     pdev->id * 32;

	err = devm_gpiochip_add_data(&pdev->dev, &port->gc, port);
	if (err)
		goto out_bgio;

	irq_base = devm_irq_alloc_descs(&pdev->dev, -1, 0, 32, numa_node_id());
	if (irq_base < 0) {
		err = irq_base;
		goto out_bgio;
	}

	port->domain = irq_domain_add_legacy(np, 32, irq_base, 0,
					     &irq_domain_simple_ops, NULL);
	if (!port->domain) {
		err = -ENODEV;
		goto out_bgio;
	}

	/* gpio-mxc can be a generic irq chip */
	err = mxc_gpio_init_gc(port, irq_base, &pdev->dev);
	if (err < 0)
		goto out_irqdomain_remove;

	list_add_tail(&port->node, &mxc_gpio_ports);

	platform_set_drvdata(pdev, port);
	pm_runtime_put(&pdev->dev);

	return 0;

out_pm_dis:
	pm_runtime_disable(&pdev->dev);
	clk_disable_unprepare(port->clk);
	clk_disable_unprepare(port->clkRdc);
	clk_disable_unprepare(port->clkSem);
out_irqdomain_remove:
	irq_domain_remove(port->domain);
out_bgio:
	clk_disable_unprepare(port->clk);
	clk_disable_unprepare(port->clkRdc);
	clk_disable_unprepare(port->clkSem);
	dev_info(&pdev->dev, "%s failed with errno %d\n", __func__, err);
	return err;
}

static void mxc_gpio_save_regs(struct mxc_gpio_port *port)
{
	if (!port->power_off)
		return;

	port->gpio_saved_reg.icr1 = reg_read32(port->base + GPIO_ICR1, port->pSEMA42);
	port->gpio_saved_reg.icr2 = reg_read32(port->base + GPIO_ICR2, port->pSEMA42);
	port->gpio_saved_reg.imr = reg_read32(port->base + GPIO_IMR, port->pSEMA42);
	port->gpio_saved_reg.gdir = reg_read32(port->base + GPIO_GDIR, port->pSEMA42);
	port->gpio_saved_reg.edge_sel = reg_read32(port->base + GPIO_EDGE_SEL, port->pSEMA42);
	port->gpio_saved_reg.dr = reg_read32(port->base + GPIO_DR, port->pSEMA42);
}

static void mxc_gpio_restore_regs(struct mxc_gpio_port *port)
{
	if (!port->power_off)
		return;

	reg_write32(port->gpio_saved_reg.icr1, port->base + GPIO_ICR1, port->pSEMA42);
	reg_write32(port->gpio_saved_reg.icr2, port->base + GPIO_ICR2, port->pSEMA42);
	reg_write32(port->gpio_saved_reg.imr, port->base + GPIO_IMR, port->pSEMA42);
	reg_write32(port->gpio_saved_reg.gdir, port->base + GPIO_GDIR, port->pSEMA42);
	reg_write32(port->gpio_saved_reg.edge_sel, port->base + GPIO_EDGE_SEL, port->pSEMA42);
	reg_write32(port->gpio_saved_reg.dr, port->base + GPIO_DR, port->pSEMA42);
}

static int __maybe_unused mxc_gpio_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mxc_gpio_port *port = platform_get_drvdata(pdev);

	mxc_gpio_save_regs(port);
	clk_disable_unprepare(port->clk);
	clk_disable_unprepare(port->clkRdc);
	clk_disable_unprepare(port->clkSem);

	return 0;
}

static int __maybe_unused mxc_gpio_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mxc_gpio_port *port = platform_get_drvdata(pdev);
	int ret;

	ret = clk_prepare_enable(port->clk);
	if (ret)
		return ret;

	ret = clk_prepare_enable(port->clkRdc);
	if (ret)
		return ret;

	ret = clk_prepare_enable(port->clkSem);
	if (ret)
		return ret;

	mxc_gpio_restore_regs(port);

	return 0;
}

static int __maybe_unused mxc_gpio_noirq_suspend(struct device *dev)
{
#ifdef CONFIG_GPIO_MXC_PAD_WAKEUP
	struct platform_device *pdev = to_platform_device(dev);
	struct mxc_gpio_port *port = platform_get_drvdata(pdev);

	mxc_gpio_set_pad_wakeup(port, true);
#endif
	return 0;
}

static int __maybe_unused mxc_gpio_noirq_resume(struct device *dev)
{
#ifdef CONFIG_GPIO_MXC_PAD_WAKEUP
	struct platform_device *pdev = to_platform_device(dev);
	struct mxc_gpio_port *port = platform_get_drvdata(pdev);
	int wakeup_line = mxc_gpio_get_pad_wakeup(port);

	mxc_gpio_set_pad_wakeup(port, false);

	if (wakeup_line >= 0)
		mxc_gpio_handle_pad_wakeup(port, wakeup_line);
#endif
	return 0;
}

static const struct dev_pm_ops mxc_gpio_dev_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(mxc_gpio_noirq_suspend, mxc_gpio_noirq_resume)
	SET_RUNTIME_PM_OPS(mxc_gpio_runtime_suspend, mxc_gpio_runtime_resume, NULL)
};

static int mxc_gpio_syscore_suspend(void)
{
	struct mxc_gpio_port *port;
	int ret;

	/* walk through all ports */
	list_for_each_entry(port, &mxc_gpio_ports, node) {
		ret = clk_prepare_enable(port->clk);
		if (ret)
			return ret;

		ret = clk_prepare_enable(port->clkRdc);
		if (ret)
			return ret;

		ret = clk_prepare_enable(port->clkSem);
		if (ret)
			return ret;


		mxc_gpio_save_regs(port);
		clk_disable_unprepare(port->clk);
		clk_disable_unprepare(port->clkRdc);
		clk_disable_unprepare(port->clkSem);
	}

	return 0;
}

static void mxc_gpio_syscore_resume(void)
{
	struct mxc_gpio_port *port;
	int ret;

	/* walk through all ports */
	list_for_each_entry(port, &mxc_gpio_ports, node) {
		ret = clk_prepare_enable(port->clk);
		if (ret) {
			pr_err("mxc: failed to enable gpio clock %d\n", ret);
			return;
		}

		ret = clk_prepare_enable(port->clkRdc);
		if (ret) {
			pr_err("mxc: failed to enable gpio clock %d\n", ret);
			return;
		}

		ret = clk_prepare_enable(port->clkSem);
		if (ret) {
			pr_err("mxc: failed to enable gpio clock %d\n", ret);
			return;
		}

		mxc_gpio_restore_regs(port);
		clk_disable_unprepare(port->clk);
		clk_disable_unprepare(port->clkRdc); //!!! seems wrong
		clk_disable_unprepare(port->clkSem); //!!! seems wrong
	}
}

static struct syscore_ops mxc_gpio_syscore_ops = {
	.suspend = mxc_gpio_syscore_suspend,
	.resume = mxc_gpio_syscore_resume,
};

static struct platform_driver mxc_gpio_driver = {
	.driver		= {
		.name	= "gpio-mxc",
		.of_match_table = mxc_gpio_dt_ids,
		.suppress_bind_attrs = true,
		.pm = &mxc_gpio_dev_pm_ops,
	},
	.probe		= mxc_gpio_probe,
};

static int __init gpio_mxc_init(void)
{
	register_syscore_ops(&mxc_gpio_syscore_ops);

	return platform_driver_register(&mxc_gpio_driver);
}
subsys_initcall(gpio_mxc_init);

MODULE_AUTHOR("Shawn Guo <shawn.guo@linaro.org>");
MODULE_DESCRIPTION("i.MX GPIO Driver");
MODULE_LICENSE("GPL");
