/*-
 * Copyright 2022 Rob Norris <robn@despairlabs.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/rman.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/fdt/simplebus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/extres/clk/clk_div.h>
#include <dev/extres/clk/clk_fixed.h>
#include <dev/extres/clk/clk_gate.h>
#include <dev/extres/clk/clk_mux.h>
#include <dev/extres/syscon/syscon.h>

#include <contrib/device-tree/include/dt-bindings/clock/starfive-jh7110-clkgen.h>

#include "clkdev_if.h"

struct jh7110_clkgen_softc {
	device_t	dev;

	struct mtx	mtx;

	struct resource	*sys_base;
	struct resource	*stg_base;
	struct resource	*aon_base;

	struct clkdom	*clkdom;
};

#define STARFIVE_CLK_ENABLE_SHIFT	31
#define STARFIVE_CLK_INVERT_SHIFT 	30
#define STARFIVE_CLK_MUX_SHIFT		24
#define STARFIVE_CLK_DIV_SHIFT		0

/*
static const char *parents_cpu_root[]			= { "osc", "pll0_out" };
static const char *parents_perh_root[]			= { "pll0_out", "pll2_out" };
static const char *parents_bus_root[]			= { "osc", "pll2_out" };
static const char *parents_qspi_ref[]			= { "osc", "u0_cdns_qspi_ref_src" };
static const char *parents_gmac5_tx[]			= { "gmac1_gtxclk", "gmac1_rmii_rtx" };
static const char *parents_u0_dw_gmac5_axi64_clk_tx[]	= { "gmac0_gtxclk", "gmac0_rmii_rtx" };
*/

static void
jh7110_div_register(struct jh7110_clkgen_softc *sc,
    int reg, const char *name, const char *parent_name,
    uint32_t width)
{
	int rc;

	struct clk_div_def divdef = {
		.clkdef = {
			.id		= reg,
			.name		= name,
			.parent_names	= (const char *[]) { parent_name },
			.parent_cnt	= 1,
			.flags		= CLK_NODE_STATIC_STRINGS,
		},
		.offset = reg,
		.i_shift = STARFIVE_CLK_DIV_SHIFT,
		.i_width = width,
	};

	rc = clknode_div_register(sc->clkdom, &divdef);
	if (rc != 0)
		panic("jh7110_div_register: clknode_div_register failed: rc=%d; reg=0x%04x; name=%s", rc, reg, name);

	if (bootverbose)
		device_printf(sc->dev, "registered div: reg=0x%04x; name=%s\n", reg, name);
}

static void
jh7110_composite_register(struct jh7110_clkgen_softc *sc,
    int reg, const char *name, const char **parent_names, int num_parents,
    uint32_t mux_width, uint32_t gate_width, uint32_t div_width)
{
	device_printf(sc->dev, "TODO registered composite: reg=0x%04x; name=%s\n", reg, name);
}

static void
jh7110_fix_parent_composite_register(struct jh7110_clkgen_softc *sc,
    int reg, const char *name, const char *parent_name,
    uint32_t mux_width, uint32_t gate_width, uint32_t div_width)
{
	device_printf(sc->dev, "TODO registered fix_parent_composite: reg=0x%04x; name=%s\n", reg, name);
}

static void
jh7110_gate_div_register(struct jh7110_clkgen_softc *sc,
    int reg, const char *name, const char *parent_name,
    uint32_t width)
{
	device_printf(sc->dev, "TODO registered gate_div: reg=0x%04x; name=%s\n", reg, name);
}


/* fixed-frequency clocks. the PLL clocks are fully programmable through
 * starfive,sys-syscon but that's extra complication we don't need right now.
 * these are the reset frequencies */
struct jh7110_fixed_def {
	int		reg;
	const char	*name;
	uint64_t	freq;
};
static struct jh7110_fixed_def jh7110_fixed[] = {
	{ JH7110_PLL0_OUT, "pll0_out", 1250000000 },
	{ JH7110_PLL1_OUT, "pll1_out", 1066000000 },
	{ JH7110_PLL2_OUT, "pll2_out", 1228800000 },
	{ -1 },
};

static void
jh7110_register_fixed(struct jh7110_clkgen_softc *sc)
{
	struct jh7110_fixed_def *def;
	struct clk_fixed_def clkdef;
	int rc;

	for (def = jh7110_fixed; def->reg >= 0; def++) {
		bzero(&clkdef, sizeof(clkdef));
		clkdef.clkdef.id = def->reg;
		clkdef.clkdef.name = def->name;
		clkdef.freq = def->freq;

		rc = clknode_fixed_register(sc->clkdom, &clkdef);
		if (rc != 0)
			panic(
			    "jh7110_fixed_register: "
			    "clknode_fixed_register failed: "
			    "rc=%d; reg=0x%04x; name=%s",
			    rc, def->reg, def->name);

		if (bootverbose)
			device_printf(sc->dev,
			    "registered fixed: reg=0x%04x; name=%s\n",
			    def->reg, def->name);
	}
}


/* fixed-factor clocks. they are all mult=1 div=1, so no need to specify that */
struct jh7110_fixed_factor_def {
	int		reg;
	const char	*name;
	const char	*parent;
};
static struct jh7110_fixed_factor_def jh7110_fixed_factor[] = {
	{ JH7110_AON_APB,                      "aon_apb",                                           "apb_bus_func"                       },
	{ JH7110_RESET1_CTRL_CLK_SRC,          "u1_reset_ctrl_clk_src",                             "osc"                                },
	{ JH7110_DDR_ROOT,                     "ddr_root",                                          "pll1_out"                           },
	{ JH7110_VDEC_ROOT,                    "vdec_root",                                         "pll0_out"                           },
	{ JH7110_VENC_ROOT,                    "venc_root",                                         "pll2_out"                           },
	{ JH7110_VOUT_ROOT,                    "vout_root",                                         "pll2_out"                           },
	{ JH7110_GMACUSB_ROOT,                 "gmacusb_root",                                      "pll0_out"                           },
	{ JH7110_PCLK2_MUX_FUNC_PCLK,          "u2_pclk_mux_func_pclk",                             "apb_bus_func"                       },
	{ JH7110_PCLK2_MUX_BIST_PCLK,          "u2_pclk_mux_bist_pclk",                             "bist_apb"                           },
	{ JH7110_APB_BUS,                      "apb_bus",                                           "u2_pclk_mux_pclk"                   },
	{ JH7110_APB12,                        "apb12",                                             "apb_bus"                            },
	{ JH7110_AXI_CFG1,                     "axi_cfg1",                                          "isp_axi"                            },
	{ JH7110_PLL_WRAP_CRG_GCLK0,           "u0_pll_wrap_crg_gclk0",                             "gclk0"                              },
	{ JH7110_PLL_WRAP_CRG_GCLK1,           "u0_pll_wrap_crg_gclk1",                             "gclk1"                              },
	{ JH7110_PLL_WRAP_CRG_GCLK2,           "u0_pll_wrap_crg_gclk2",                             "gclk2"                              },
	{ JH7110_JTAG2APB_PCLK,                "u0_jtag2apb_pclk",                                  "bist_apb"                           },
	{ JH7110_U7_BUS_CLK,                   "u0_u7mc_sft7110_bus_clk",                           "cpu_bus"                            },
	{ JH7110_U7_IRQ_SYNC_BUS_CLK,          "u0_u7mc_sft7110_irq_sync_bus_clk",                  "cpu_bus"                            },
	{ JH7110_NOC_BUS_CLK2_CPU_AXI,         "u0_sft7110_noc_bus_clk2_cpu_axi",                   "u0_sft7110_noc_bus_clk_cpu_axi"     },
	{ JH7110_NOC_BUS_CLK_APB_BUS,          "u0_sft7110_noc_bus_clk_apb_bus",                    "apb_bus"                            },
	{ JH7110_NOC_BUS_CLK2_APB_BUS,         "u0_sft7110_noc_bus_clk2_apb_bus",                   "u0_sft7110_noc_bus_clk_apb_bus"     },
	{ JH7110_NOC_BUS_CLK2_AXICFG0_AXI,     "u0_sft7110_noc_bus_clk2_axicfg0_axi",               "u0_sft7110_noc_bus_clk_axicfg0_axi" },
	{ JH7110_DDR_CLK_DDRPHY_PLL_BYPASS,    "u0_ddr_sft7110_clk_ddrphy_pll_bypass",              "pll1_out"                           },
	{ JH7110_DDR_CLK_OSC,                  "u0_ddr_sft7110_clk_osc",                            "osc"                                },
	{ JH7110_DDR_CLK_APB,                  "u0_ddr_sft7110_clk_apb",                            "apb12"                              },
	{ JH7110_NOC_BUS_CLK_DDRC,             "u0_sft7110_noc_bus_clk_ddrc",                       "ddr_bus"                            },
	{ JH7110_NOC_BUS_CLK2_DDRC,            "u0_sft7110_noc_bus_clk2_ddrc",                      "u0_sft7110_noc_bus_clk_ddrc"        },
	{ JH7110_SYS_AHB_DEC_CLK_AHB,          "u0_saif_amba_sys_ahb_dec_clk_ahb",                  "ahb0"                               },
	{ JH7110_STG_AHB_DEC_CLK_AHB,          "u0_saif_amba_stg_ahb_dec_clk_ahb",                  "ahb0"                               },
	{ JH7110_NOC_BUS_CLK2_GPU_AXI,         "u0_sft7110_noc_bus_clk2_gpu_axi",                   "u0_sft7110_noc_bus_clk_gpu_axi"     },
	{ JH7110_ISP_TOP_CLK_DVP,              "u0_dom_isp_top_clk_dom_isp_top_clk_dvp",            "dvp_clk"                            },
	{ JH7110_NOC_BUS_CLK2_ISP_AXI,         "u0_sft7110_noc_bus_clk2_isp_axi",                   "u0_sft7110_noc_bus_clk_isp_axi"     },
	{ JH7110_ISP_TOP_CLK_BIST_APB,         "u0_dom_isp_top_clk_dom_isp_top_clk_bist_apb",       "bist_apb"                           },
	{ JH7110_NOC_BUS_CLK2_DISP_AXI,        "u0_sft7110_noc_bus_clk2_disp_axi",                  "u0_sft7110_noc_bus_clk_disp_axi"    },
	{ JH7110_VOUT_TOP_CLK_HDMITX0_BCLK,    "u0_dom_vout_top_clk_dom_vout_top_clk_hdmitx0_bclk", "u0_i2stx_4ch_bclk"                  },
	{ JH7110_VOUT_TOP_U0_HDMI_TX_PIN_WS,   "u0_dom_vout_top_u0_hdmi_tx_pin_ws",                 "u0_i2stx_4ch_lrck"                  },
	{ JH7110_VOUT_TOP_CLK_HDMIPHY_REF,     "u0_dom_vout_top_clk_dom_vout_top_clk_hdmiphy_ref",  "osc"                                },
	{ JH7110_VOUT_TOP_BIST_PCLK,           "u0_dom_vout_top_clk_dom_vout_top_bist_pclk",        "bist_apb"                           },
	{ JH7110_AXIMEM0_128B_CLK_AXI,         "u0_aximem_128b_clk_axi",                            "u0_WAVE511_clk_axi"                 },
	{ JH7110_VDEC_INTSRAM_CLK_VDEC_AXI,    "u0_vdec_intsram_clk_vdec_axi",                      "u0_aximem_128b_clk_axi"             },
	{ JH7110_NOC_BUS_CLK2_VDEC_AXI,        "u0_sft7110_noc_bus_clk2_vdec_axi",                  "u0_sft7110_noc_bus_clk_vdec_axi"    },
	{ JH7110_AXIMEM1_128B_CLK_AXI,         "u1_aximem_128b_clk_axi",                            "u0_wave420l_clk_axi"                },
	{ JH7110_VENC_INTSRAM_CLK_VENC_AXI,    "u0_venc_intsram_clk_venc_axi",                      "u0_wave420l_clk_axi"                },
	{ JH7110_NOC_BUS_CLK2_VENC_AXI,        "u0_sft7110_noc_bus_clk2_venc_axi",                  "u0_sft7110_noc_bus_clk_venc_axi"    },
	{ JH7110_SRAM_CLK_ROM,                 "u0_intmem_rom_sram_clk_rom",                        "u2_aximem_128b_clk_axi"             },
	{ JH7110_NOC_BUS_CLK2_STG_AXI,         "u0_sft7110_noc_bus_clk2_stg_axi",                   "u0_sft7110_noc_bus_clk_stg_axi"     },
	{ JH7110_GMAC5_CLK_RMII,               "u1_dw_gmac5_axi64_clk_rmii",                        "gmac1_rmii_refin"                   },
	{ JH7110_AON_AHB,                      "aon_ahb",                                           "stg_axiahb"                         },
	{ JH7110_SYS_CRG_PCLK,                 "u0_sys_crg_pclk",                                   "apb12"                              },
	{ JH7110_SYS_SYSCON_PCLK,              "u0_sys_syscon_pclk",                                "apb12"                              },
	{ JH7110_SPI0_CLK_CORE,                "u0_ssp_spi_clk_core",                               "u0_ssp_spi_clk_apb"                 },
	{ JH7110_SPI1_CLK_CORE,                "u1_ssp_spi_clk_core",                               "u1_ssp_spi_clk_apb"                 },
	{ JH7110_SPI2_CLK_CORE,                "u2_ssp_spi_clk_core",                               "u2_ssp_spi_clk_apb"                 },
	{ JH7110_SPI3_CLK_CORE,                "u3_ssp_spi_clk_core",                               "u3_ssp_spi_clk_apb"                 },
	{ JH7110_SPI4_CLK_CORE,                "u4_ssp_spi_clk_core",                               "u4_ssp_spi_clk_apb"                 },
	{ JH7110_SPI5_CLK_CORE,                "u5_ssp_spi_clk_core",                               "u5_ssp_spi_clk_apb"                 },
	{ JH7110_SPI6_CLK_CORE,                "u6_ssp_spi_clk_core",                               "u6_ssp_spi_clk_apb"                 },
	{ JH7110_I2C0_CLK_CORE,                "u0_dw_i2c_clk_core",                                "u0_dw_i2c_clk_apb"                  },
	{ JH7110_I2C1_CLK_CORE,                "u1_dw_i2c_clk_core",                                "u1_dw_i2c_clk_apb"                  },
	{ JH7110_I2C2_CLK_CORE,                "u2_dw_i2c_clk_core",                                "u2_dw_i2c_clk_apb"                  },
	{ JH7110_I2C3_CLK_CORE,                "u3_dw_i2c_clk_core",                                "u3_dw_i2c_clk_apb"                  },
	{ JH7110_I2C4_CLK_CORE,                "u4_dw_i2c_clk_core",                                "u4_dw_i2c_clk_apb"                  },
	{ JH7110_I2C5_CLK_CORE,                "u5_dw_i2c_clk_core",                                "u5_dw_i2c_clk_apb"                  },
	{ JH7110_I2C6_CLK_CORE,                "u6_dw_i2c_clk_core",                                "u6_dw_i2c_clk_apb"                  },
	{ JH7110_I2STX_BCLK_MST,               "i2stx_bclk_mst",                                    "i2stx_4ch1_bclk_mst"                },
	{ JH7110_I2STX_LRCK_MST,               "i2stx_lrck_mst",                                    "i2stx_4ch1_lrck_mst"                },
	{ JH7110_I2SRX_BCLK_MST,               "i2srx_bclk_mst",                                    "i2srx_3ch_bclk_mst"                 },
	{ JH7110_I2SRX_LRCK_MST,               "i2srx_lrck_mst",                                    "i2srx_3ch_lrck_mst"                 },
	{ JH7110_PDM_CLK_DMIC0_BCLK_SLV,       "u0_pdm_4mic_clk_dmic0_bclk_slv",                    "u0_i2srx_3ch_bclk"                  },
	{ JH7110_PDM_CLK_DMIC0_LRCK_SLV,       "u0_pdm_4mic_clk_dmic0_lrck_slv",                    "u0_i2srx_3ch_lrck"                  },
	{ JH7110_PDM_CLK_DMIC1_BCLK_SLV,       "u0_pdm_4mic_clk_dmic1_bclk_slv",                    "u0_i2srx_3ch_bclk"                  },
	{ JH7110_PDM_CLK_DMIC1_LRCK_SLV,       "u0_pdm_4mic_clk_dmic1_lrck_slv",                    "u0_i2srx_3ch_lrck"                  },
	{ JH7110_TDM_CLK_MST,                  "tdm_clk_mst",                                       "ahb0"                               },
	{ JH7110_AHB2APB_CLK_AHB,              "u1_ahb2apb_clk_ahb",                                "tdm_internal"                       },
	{ JH7110_P2P_ASYNC_CLK_APBS,           "u1_p2p_async_clk_apbs",                             "apb0"                               },
	{ JH7110_P2P_ASYNC_CLK_APBM,           "u1_p2p_async_clk_apbm",                             "aon_apb"                            },
	{ JH7110_JTAG_DAISY_CHAIN_JTAG_TCK,    "u0_jtag_daisy_chain_JTAG_TCK",                      "jtag_tck_inner"                     },
	{ JH7110_U7_DEBUG_SYSTEMJTAG_JTAG_TCK, "u0_u7mc_sft7110_debug_systemjtag_jtag_TCK",         "u0_jtag_daisy_chain_jtag_tck_0"     },
	{ JH7110_E2_DEBUG_SYSTEMJTAG_TCK,      "u0_e2_sft7110_debug_systemjtag_jtag_TCK",           "u0_jtag_daisy_chain_jtag_tck_1"     },
	{ JH7110_JTAG_CERTIFICATION_TCK,       "u0_jtag_certification_tck",                         "jtag_tck_inner"                     },
	{ JH7110_SEC_SKP_CLK,                  "u0_sec_top_skp_clk",                                "u0_jtag_certification_trng_clk"     },
	{ JH7110_U2_PCLK_MUX_PCLK,             "u2_pclk_mux_pclk",                                  "u2_pclk_mux_func_pclk"              },
	{ -1 },
};

static void
jh7110_register_fixed_factor(struct jh7110_clkgen_softc *sc)
{
	struct jh7110_fixed_factor_def *def;
	struct clk_fixed_def clkdef;
	int rc;

	for (def = jh7110_fixed_factor; def->reg >= 0; def++) {
		bzero(&clkdef, sizeof(clkdef));
		clkdef.clkdef.id = def->reg;
		clkdef.clkdef.name = def->name;
		clkdef.clkdef.parent_names = (const char *[]) { def->parent };
		clkdef.clkdef.parent_cnt = 1;
		clkdef.clkdef.flags = CLK_NODE_STATIC_STRINGS;
		clkdef.mult = 1;
		clkdef.div = 1;

		rc = clknode_fixed_register(sc->clkdom, &clkdef);
		if (rc != 0)
			panic(
			    "jh7110_fixed_factor_register: "
			    "clknode_fixed_register failed: "
			    "rc=%d; reg=0x%04x; name=%s",
			    rc, def->reg, def->name);

		if (bootverbose)
			device_printf(sc->dev,
			    "registered fixed factor: reg=0x%04x; name=%s\n",
			    def->reg, def->name);
	}
}


/* clock gates */
struct jh7110_gate_def {
	int		reg;
	const char	*name;
	const char	*parent;
	int		invert;
};
static struct jh7110_gate_def jh7110_gate[] = {
	{ JH7110_AHB0,                      "ahb0",                                              "stg_axiahb"   },
	{ JH7110_AHB1,                      "ahb1",                                              "stg_axiahb"   },
	{ JH7110_APB0,                      "apb0",                                              "apb_bus"      },
	{ JH7110_MCLK_OUT,                  "mclk_out",                                          "mclk_inner"   },
	{ JH7110_U7_CORE_CLK,               "u0_u7mc_sft7110_core_clk",                          "cpu_core"     },
	{ JH7110_U7_CORE_CLK1,              "u0_u7mc_sft7110_core_clk1",                         "cpu_core"     },
	{ JH7110_U7_CORE_CLK2,              "u0_u7mc_sft7110_core_clk2",                         "cpu_core"     },
	{ JH7110_U7_CORE_CLK3,              "u0_u7mc_sft7110_core_clk3",                         "cpu_core"     },
	{ JH7110_U7_CORE_CLK4,              "u0_u7mc_sft7110_core_clk4",                         "cpu_core"     },
	{ JH7110_U7_DEBUG_CLK,              "u0_u7mc_sft7110_debug_clk",                         "cpu_bus"      },
	{ JH7110_U7_TRACE_CLK0,             "u0_u7mc_sft7110_trace_clk0",                        "cpu_core"     },
	{ JH7110_U7_TRACE_CLK1,             "u0_u7mc_sft7110_trace_clk1",                        "cpu_core"     },
	{ JH7110_U7_TRACE_CLK2,             "u0_u7mc_sft7110_trace_clk2",                        "cpu_core"     },
	{ JH7110_U7_TRACE_CLK3,             "u0_u7mc_sft7110_trace_clk3",                        "cpu_core"     },
	{ JH7110_U7_TRACE_CLK4,             "u0_u7mc_sft7110_trace_clk4",                        "cpu_core"     },
	{ JH7110_U7_TRACE_COM_CLK,          "u0_u7mc_sft7110_trace_com_clk",                     "cpu_bus"      },
	{ JH7110_NOC_BUS_CLK_CPU_AXI,       "u0_sft7110_noc_bus_clk_cpu_axi",                    "cpu_bus"      },
	{ JH7110_NOC_BUS_CLK_AXICFG0_AXI,   "u0_sft7110_noc_bus_clk_axicfg0_axi",                "axi_cfg0"     },
	{ JH7110_DDR_CLK_AXI,               "u0_ddr_sft7110_clk_axi",                            "ddr_bus"      },
	{ JH7110_GPU_CORE_CLK,              "u0_img_gpu_core_clk",                               "gpu_core"     },
	{ JH7110_GPU_SYS_CLK,               "u0_img_gpu_sys_clk",                                "axi_cfg1"     },
	{ JH7110_GPU_CLK_APB,               "u0_img_gpu_clk_apb",                                "apb12"        },
	{ JH7110_NOC_BUS_CLK_GPU_AXI,       "u0_sft7110_noc_bus_clk_gpu_axi",                    "gpu_core"     },
	{ JH7110_ISP_TOP_CLK_ISPCORE_2X,    "u0_dom_isp_top_clk_dom_isp_top_clk_ispcore_2x",     "isp_2x"       },
	{ JH7110_ISP_TOP_CLK_ISP_AXI,       "u0_dom_isp_top_clk_dom_isp_top_clk_isp_axi",        "isp_axi"      },
	{ JH7110_NOC_BUS_CLK_ISP_AXI,       "u0_sft7110_noc_bus_clk_isp_axi",                    "isp_axi"      },
	{ JH7110_AXI_CFG1_DEC_CLK_MAIN,     "u0_axi_cfg1_dec_clk_main",                          "axi_cfg1"     },
	{ JH7110_AXI_CFG1_DEC_CLK_AHB,      "u0_axi_cfg1_dec_clk_ahb",                           "ahb0"         },
	{ JH7110_VOUT_SRC,                  "u0_dom_vout_top_clk_dom_vout_top_clk_vout_src",     "vout_root"    },
	{ JH7110_NOC_BUS_CLK_DISP_AXI,      "u0_sft7110_noc_bus_clk_disp_axi",                   "vout_axi"     },
	{ JH7110_VOUT_TOP_CLK_VOUT_AHB,     "u0_dom_vout_top_clk_dom_vout_top_clk_vout_ahb",     "ahb1"         },
	{ JH7110_VOUT_TOP_CLK_VOUT_AXI,     "u0_dom_vout_top_clk_dom_vout_top_clk_vout_axi",     "vout_axi"     },
	{ JH7110_VOUT_TOP_CLK_HDMITX0_MCLK, "u0_dom_vout_top_clk_dom_vout_top_clk_hdmitx0_mclk", "mclk"         },
	{ JH7110_CODAJ12_CLK_AXI,           "u0_CODAJ12_clk_axi",                                "jpegc_axi"    },
	{ JH7110_CODAJ12_CLK_APB,           "u0_CODAJ12_clk_apb",                                "apb12"        },
	{ JH7110_WAVE511_CLK_AXI,           "u0_WAVE511_clk_axi",                                "vdec_axi"     },
	{ JH7110_WAVE511_CLK_APB,           "u0_WAVE511_clk_apb",                                "apb12"        },
	{ JH7110_VDEC_JPG_ARB_JPGCLK,       "u0_vdec_jpg_arb_jpgclk",                            "jpegc_axi"    },
	{ JH7110_VDEC_JPG_ARB_MAINCLK,      "u0_vdec_jpg_arb_mainclk",                           "vdec_axi"     },
	{ JH7110_NOC_BUS_CLK_VDEC_AXI,      "u0_sft7110_noc_bus_clk_vdec_axi",                   "vdec_axi"     },
	{ JH7110_WAVE420L_CLK_AXI,          "u0_wave420l_clk_axi",                               "venc_axi"     },
	{ JH7110_WAVE420L_CLK_APB,          "u0_wave420l_clk_apb",                               "apb12"        },
	{ JH7110_NOC_BUS_CLK_VENC_AXI,      "u0_sft7110_noc_bus_clk_venc_axi",                   "venc_axi"     },
	{ JH7110_AXI_CFG0_DEC_CLK_MAIN_DIV, "u0_axi_cfg0_dec_clk_main_div",                      "ahb1"         },
	{ JH7110_AXI_CFG0_DEC_CLK_MAIN,     "u0_axi_cfg0_dec_clk_main",                          "axi_cfg0"     },
	{ JH7110_AXI_CFG0_DEC_CLK_HIFI4,    "u0_axi_cfg0_dec_clk_hifi4",                         "hifi4_axi"    },
	{ JH7110_AXIMEM2_128B_CLK_AXI,      "u2_aximem_128b_clk_axi",                            "axi_cfg0"     },
	{ JH7110_QSPI_CLK_AHB,              "u0_cdns_qspi_clk_ahb",                              "ahb1"         },
	{ JH7110_QSPI_CLK_APB,              "u0_cdns_qspi_clk_apb",                              "apb12"        },
	{ JH7110_SDIO0_CLK_AHB,             "u0_dw_sdio_clk_ahb",                                "ahb0"         },
	{ JH7110_SDIO1_CLK_AHB,             "u1_dw_sdio_clk_ahb",                                "ahb0"         },
	{ JH7110_NOC_BUS_CLK_STG_AXI,       "u0_sft7110_noc_bus_clk_stg_axi",                    "nocstg_bus"   },
	{ JH7110_GMAC5_CLK_AHB,             "u1_dw_gmac5_axi64_clk_ahb",                         "ahb0"         },
	{ JH7110_GMAC5_CLK_AXI,             "u1_dw_gmac5_axi64_clk_axi",                         "stg_axiahb"   },
	{ JH7110_GMAC1_GTXC,                "gmac1_gtxc",                                        "gmac1_gtxclk" },
	{ JH7110_GMAC0_GTXC,                "gmac0_gtxc",                                        "gmac0_gtxclk" },
	{ JH7110_SYS_IOMUX_PCLK,            "u0_sys_iomux_pclk",                                 "apb12"        },
	{ JH7110_MAILBOX_CLK_APB,           "u0_mailbox_clk_apb",                                "apb12"        },
	{ JH7110_INT_CTRL_CLK_APB,          "u0_int_ctrl_clk_apb",                               "apb12"        },
	{ JH7110_CAN0_CTRL_CLK_APB,         "u0_can_ctrl_clk_apb",                               "apb12"        },
	{ JH7110_CAN1_CTRL_CLK_APB,         "u1_can_ctrl_clk_apb",                               "apb12"        },
	{ JH7110_PWM_CLK_APB,               "u0_pwm_8ch_clk_apb",                                "apb12"        },
	{ JH7110_DSKIT_WDT_CLK_APB,         "u0_dskit_wdt_clk_apb",                              "apb12"        },
	{ JH7110_DSKIT_WDT_CLK_WDT,         "u0_dskit_wdt_clk_wdt",                              "osc"          },
	{ JH7110_TIMER_CLK_APB,             "u0_si5_timer_clk_apb",                              "apb12"        },
	{ JH7110_TIMER_CLK_TIMER0,          "u0_si5_timer_clk_timer0",                           "osc"          },
	{ JH7110_TIMER_CLK_TIMER1,          "u0_si5_timer_clk_timer1",                           "osc"          },
	{ JH7110_TIMER_CLK_TIMER2,          "u0_si5_timer_clk_timer2",                           "osc"          },
	{ JH7110_TIMER_CLK_TIMER3,          "u0_si5_timer_clk_timer3",                           "osc"          },
	{ JH7110_TEMP_SENSOR_CLK_APB,       "u0_temp_sensor_clk_apb",                            "apb12"        },
	{ JH7110_SPI0_CLK_APB,              "u0_ssp_spi_clk_apb",                                "apb0"         },
	{ JH7110_SPI1_CLK_APB,              "u1_ssp_spi_clk_apb",                                "apb0"         },
	{ JH7110_SPI2_CLK_APB,              "u2_ssp_spi_clk_apb",                                "apb0"         },
	{ JH7110_SPI3_CLK_APB,              "u3_ssp_spi_clk_apb",                                "apb12"        },
	{ JH7110_SPI4_CLK_APB,              "u4_ssp_spi_clk_apb",                                "apb12"        },
	{ JH7110_SPI5_CLK_APB,              "u5_ssp_spi_clk_apb",                                "apb12"        },
	{ JH7110_SPI6_CLK_APB,              "u6_ssp_spi_clk_apb",                                "apb12"        },
	{ JH7110_I2C0_CLK_APB,              "u0_dw_i2c_clk_apb",                                 "apb0"         },
	{ JH7110_I2C1_CLK_APB,              "u1_dw_i2c_clk_apb",                                 "apb0"         },
	{ JH7110_I2C2_CLK_APB,              "u2_dw_i2c_clk_apb",                                 "apb0"         },
	{ JH7110_I2C3_CLK_APB,              "u3_dw_i2c_clk_apb",                                 "apb12"        },
	{ JH7110_I2C4_CLK_APB,              "u4_dw_i2c_clk_apb",                                 "apb12"        },
	{ JH7110_I2C5_CLK_APB,              "u5_dw_i2c_clk_apb",                                 "apb12"        },
	{ JH7110_I2C6_CLK_APB,              "u6_dw_i2c_clk_apb",                                 "apb12"        },
	{ JH7110_UART0_CLK_APB,             "u0_dw_uart_clk_apb",                                "apb0"         },
	{ JH7110_UART0_CLK_CORE,            "u0_dw_uart_clk_core",                               "osc"          },
	{ JH7110_UART1_CLK_APB,             "u1_dw_uart_clk_apb",                                "apb0"         },
	{ JH7110_UART1_CLK_CORE,            "u1_dw_uart_clk_core",                               "osc"          },
	{ JH7110_UART2_CLK_APB,             "u2_dw_uart_clk_apb",                                "apb0"         },
	{ JH7110_UART2_CLK_CORE,            "u2_dw_uart_clk_core",                               "osc"          },
	{ JH7110_UART3_CLK_APB,             "u3_dw_uart_clk_apb",                                "apb0"         },
	{ JH7110_UART4_CLK_APB,             "u4_dw_uart_clk_apb",                                "apb0"         },
	{ JH7110_UART5_CLK_APB,             "u5_dw_uart_clk_apb",                                "apb0"         },
	{ JH7110_PWMDAC_CLK_APB,            "u0_pwmdac_clk_apb",                                 "apb0"         },
	{ JH7110_SPDIF_CLK_APB,             "u0_cdns_spdif_clk_apb",                             "apb0"         },
	{ JH7110_SPDIF_CLK_CORE,            "u0_cdns_spdif_clk_core",                            "mclk"         },
	{ JH7110_I2STX0_4CHCLK_APB,         "u0_i2stx_4ch_clk_apb",                              "apb0"         },
	{ JH7110_I2STX1_4CHCLK_APB,         "u1_i2stx_4ch_clk_apb",                              "apb0"         },
	{ JH7110_I2SRX0_3CH_CLK_APB,        "u0_i2srx_3ch_clk_apb",                              "apb0"         },
	{ JH7110_PDM_CLK_APB,               "u0_pdm_4mic_clk_apb",                               "apb0"         },
	{ JH7110_TDM_CLK_AHB,               "u0_tdm16slot_clk_ahb",                              "ahb0"         },
	{ JH7110_TDM_CLK_APB,               "u0_tdm16slot_clk_apb",                              "apb0"         },

	/* inverted gates */
	{ JH7110_GMAC5_CLK_RX_INV,        "u1_dw_gmac5_axi64_clk_rx_inv", "u1_dw_gmac5_axi64_clk_rx", 1 },
	{ JH7110_GMAC5_CLK_TX_INV,        "u1_dw_gmac5_axi64_clk_tx_inv", "u1_dw_gmac5_axi64_clk_tx", 1 },
	{ JH7110_I2STX_4CH0_BCLK_MST_INV, "i2stx_4ch0_bclk_mst_inv",      "i2stx_4ch0_bclk_mst",      1 },
	{ JH7110_I2STX0_4CHBCLK_N,        "u0_i2stx_4ch_bclk_n",          "u0_i2stx_4ch_bclk",        1 },
	{ JH7110_I2STX_4CH1_BCLK_MST_INV, "i2stx_4ch1_bclk_mst_inv",      "i2stx_4ch1_bclk_mst",      1 },
	{ JH7110_I2STX1_4CHBCLK_N,        "u1_i2stx_4ch_bclk_n",          "u1_i2stx_4ch_bclk",        1 },
	{ JH7110_I2SRX_3CH_BCLK_MST_INV,  "i2srx_3ch_bclk_mst_inv",       "i2srx_3ch_bclk_mst",       1 },
	{ JH7110_I2SRX0_3CH_BCLK_N,       "u0_i2srx_3ch_bclk_n",          "u0_i2srx_3ch_bclk",        1 },
	{ JH7110_TDM_CLK_TDM_N,           "u0_tdm16slot_clk_tdm_n",       "u0_tdm16slot_clk_tdm",     1 },

	{ -1 },
};

static void
jh7110_register_gate(struct jh7110_clkgen_softc *sc)
{
	struct jh7110_gate_def *def;
	struct clk_gate_def clkdef;
	int rc;

	for (def = jh7110_gate; def->reg >= 0; def++) {
		bzero(&clkdef, sizeof(clkdef));
		clkdef.clkdef.id = def->reg;
		clkdef.clkdef.name = def->name;
		clkdef.clkdef.parent_names = (const char *[]) { def->parent };
		clkdef.clkdef.parent_cnt = 1;
		clkdef.clkdef.flags = CLK_NODE_STATIC_STRINGS;
		clkdef.offset = def->reg;
		clkdef.shift = STARFIVE_CLK_ENABLE_SHIFT;
		clkdef.mask = 1;
		clkdef.on_value = def->invert ? 0 : 1;
		clkdef.off_value = def->invert ? 1 : 0;

		rc = clknode_gate_register(sc->clkdom, &clkdef);
		if (rc != 0)
			panic(
			    "jh7110_gate_register: "
			    "clknode_gate_register failed: "
			    "rc=%d; reg=0x%04x; name=%s",
			    rc, def->reg, def->name);

		if (bootverbose)
			device_printf(sc->dev,
			    "registered gate: reg=0x%04x; name=%s\n",
			    def->reg, def->name);
	}
}


/* mux clocks */
struct jh7110_mux_def {
	int		reg;
	const char	*name;
	int		parent_cnt;
	const char	*parent_names[4];
};
static struct jh7110_mux_def jh7110_mux[] = {
	{ JH7110_CPU_ROOT,        "cpu_root",                 2, { "osc", "pll0_out" } },
	{ JH7110_GPU_ROOT,        "gpu_root",                 2, { "pll2_out", "pll1_out" } },
	{ JH7110_BUS_ROOT,        "bus_root",                 2, { "osc", "pll2_out" } },
	{ JH7110_MCLK,            "mclk",                     2, { "mclk_inner", "mclk_ext" } },
	{ JH7110_DDR_BUS,         "ddr_bus",                  4, { "osc_div2", "pll1_div2", "pll1_div4", "pll1_div8" } },
	{ JH7110_GMAC5_CLK_RX,    "u1_dw_gmac5_axi64_clk_rx", 2, { "gmac1_rgmii_rxin", "gmac1_rmii_rtx" } },
	{ JH7110_I2STX0_4CHBCLK,  "u0_i2stx_4ch_bclk",        2, { "i2stx_4ch0_bclk_mst", "i2stx_bclk_ext" } },
	{ JH7110_I2STX0_4CHLRCK,  "u0_i2stx_4ch_lrck",        2, { "i2stx_4ch0_lrck_mst", "i2stx_lrck_ext" } },
	{ JH7110_I2STX1_4CHBCLK,  "u1_i2stx_4ch_bclk",        2, { "i2stx_4ch1_bclk_mst", "i2stx_bclk_ext" } },
	{ JH7110_I2STX1_4CHLRCK,  "u1_i2stx_4ch_lrck",        2, { "i2stx_4ch1_lrck_mst", "i2stx_lrck_ext" } },
	{ JH7110_I2SRX0_3CH_BCLK, "u0_i2srx_3ch_bclk",        2, { "i2srx_3ch_bclk_mst", "i2srx_bclk_ext" } },
	{ JH7110_I2SRX0_3CH_LRCK, "u0_i2srx_3ch_lrck",        2, { "i2srx_3ch_lrck_mst", "i2srx_lrck_ext" } },
	{ JH7110_TDM_CLK_TDM,     "u0_tdm16slot_clk_tdm",     2, { "tdm_internal", "tdm_ext" } },
	{ -1 },
};

static void
jh7110_register_mux(struct jh7110_clkgen_softc *sc)
{
	struct jh7110_mux_def *def;
	struct clk_mux_def clkdef;
	int rc;

	for (def = jh7110_mux; def->reg >= 0; def++) {
		bzero(&clkdef, sizeof(clkdef));
		clkdef.clkdef.id = def->reg;
		clkdef.clkdef.name = def->name;
		clkdef.clkdef.parent_names = def->parent_names;
		clkdef.clkdef.parent_cnt = def->parent_cnt;
		clkdef.clkdef.flags = CLK_NODE_STATIC_STRINGS;
		clkdef.offset = def->reg;
		clkdef.shift = STARFIVE_CLK_MUX_SHIFT;
		clkdef.width = 1;

		rc = clknode_mux_register(sc->clkdom, &clkdef);
		if (rc != 0)
			panic(
			    "jh7110_mux_register: "
			    "clknode_mux_register failed: "
			    "rc=%d; reg=0x%04x; name=%s",
			    rc, def->reg, def->name);

		if (bootverbose)
			device_printf(sc->dev,
			    "registered mux: reg=0x%04x; name=%s\n",
			    def->reg, def->name);
	}
}


static struct ofw_compat_data compat_data[] = {
	{ "starfive,jh7110-clkgen",	1 },
	{ NULL,				0 },
};

static int
jh7110_clkgen_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "StarFive JH7110 clock generator");

	return (BUS_PROBE_DEFAULT);
}

static int
jh7110_clkgen_init(device_t dev, phandle_t node,
		   const char *name, struct resource **base,
		   int *rid)
{
	if (ofw_bus_find_string_index(node, "reg-names", name, rid) != 0) {
		device_printf(dev, "could not locate '%s' clock register\n", name);
		return (ENXIO);
	}
	*base = bus_alloc_resource_any(dev, SYS_RES_MEMORY, rid, RF_ACTIVE);
	if (*base == NULL) {
		device_printf(dev, "could not allocate resource for '%s' clock register\n", name);
		return (ENXIO);
	}
	return (0);
}

static int
jh7110_clkgen_attach(device_t dev)
{
	struct jh7110_clkgen_softc *sc;
	int rc = 0;
	phandle_t node;
	int sys_rid, stg_rid, aon_rid;

	sc = device_get_softc(dev);
	sc->dev = dev;

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	node = ofw_bus_get_node(dev);

	rc = jh7110_clkgen_init(dev, node, "sys", &sc->sys_base, &sys_rid);
	if (rc != 0) goto fail;

	rc = jh7110_clkgen_init(dev, node, "stg", &sc->stg_base, &stg_rid);
	if (rc != 0) goto fail;

	rc = jh7110_clkgen_init(dev, node, "aon", &sc->aon_base, &aon_rid);
	if (rc != 0) goto fail;

	sc->clkdom = clkdom_create(dev);
	if (sc->clkdom == NULL) {
		device_printf(dev, "jh7110_clkgen_attach: could not create clock domain\n");
		rc = ENXIO;
		goto fail;
	}

	jh7110_register_fixed(sc);
	jh7110_register_fixed_factor(sc);
	jh7110_register_gate(sc);
	jh7110_register_mux(sc);

#if 0
static const struct jh7110_clk_data jh7110_clk_sys_data[] __initconst = {
	/*root*/
	JH7110__DIV(JH7110_CPU_CORE, "cpu_core", 7, JH7110_CPU_ROOT),
	JH7110__DIV(JH7110_CPU_BUS, "cpu_bus", 2, JH7110_CPU_CORE),
	JH7110_MDIV(JH7110_PERH_ROOT, "perh_root", 2, PARENT_NUMS_2,
			JH7110_PLL0_OUT,
			JH7110_PLL2_OUT),
	JH7110__DIV(JH7110_NOCSTG_BUS, "nocstg_bus", 3, JH7110_BUS_ROOT),
	JH7110__DIV(JH7110_AXI_CFG0, "axi_cfg0", 3, JH7110_BUS_ROOT),
	JH7110__DIV(JH7110_STG_AXIAHB, "stg_axiahb", 2, JH7110_AXI_CFG0),
	JH7110__DIV(JH7110_APB_BUS_FUNC, "apb_bus_func",
			8, JH7110_STG_AXIAHB),
	JH7110__DIV(JH7110_PLL0_DIV2, "pll0_div2", 2, JH7110_PLL0_OUT),
	JH7110__DIV(JH7110_PLL1_DIV2, "pll1_div2", 2, JH7110_PLL1_OUT),
	JH7110__DIV(JH7110_PLL2_DIV2, "pll2_div2", 2, JH7110_PLL2_OUT),
	JH7110__DIV(JH7110_AUDIO_ROOT, "audio_root", 8, JH7110_PLL2_OUT),
	JH7110__DIV(JH7110_MCLK_INNER, "mclk_inner", 64, JH7110_AUDIO_ROOT),
	JH7110_MDIV(JH7110_ISP_2X, "isp_2x", 8, PARENT_NUMS_2,
			JH7110_PLL2_OUT,
			JH7110_PLL1_OUT),
	JH7110__DIV(JH7110_ISP_AXI, "isp_axi", 4, JH7110_ISP_2X),
	JH7110_GDIV(JH7110_GCLK0, "gclk0", GATE_FLAG_NORMAL,
			62, JH7110_PLL0_DIV2),
	JH7110_GDIV(JH7110_GCLK1, "gclk1", GATE_FLAG_NORMAL,
			62, JH7110_PLL1_DIV2),
	JH7110_GDIV(JH7110_GCLK2, "gclk2", GATE_FLAG_NORMAL,
			62, JH7110_PLL2_DIV2),
	/*u0_u7mc_sft7110*/
	JH7110__DIV(JH7110_U7_RTC_TOGGLE, "u0_u7mc_sft7110_rtc_toggle",
			6, JH7110_OSC),
	//NOC
	//DDRC
	JH7110__DIV(JH7110_OSC_DIV2, "osc_div2", 2, JH7110_OSC),
	JH7110__DIV(JH7110_PLL1_DIV4, "pll1_div4", 2, JH7110_PLL1_DIV2),
	JH7110__DIV(JH7110_PLL1_DIV8, "pll1_div8", 2, JH7110_PLL1_DIV4),
	//GPU
	JH7110__DIV(JH7110_GPU_CORE, "gpu_core", 7, JH7110_GPU_ROOT),
	JH7110_GDIV(JH7110_GPU_RTC_TOGGLE, "u0_img_gpu_rtc_toggle",
			GATE_FLAG_NORMAL, 12, JH7110_OSC),
	//ISP
	//HIFI4
	JH7110__DIV(JH7110_HIFI4_CORE, "hifi4_core", 15, JH7110_BUS_ROOT),
	JH7110__DIV(JH7110_HIFI4_AXI, "hifi4_axi", 2, JH7110_HIFI4_CORE),
	//AXICFG1_DEC
	//VOUT
	JH7110__DIV(JH7110_VOUT_AXI, "vout_axi", 7, JH7110_VOUT_ROOT),
	JH7110__DIV(JH7110_VOUT_TOP_CLK_MIPIPHY_REF,
			"u0_dom_vout_top_clk_dom_vout_top_clk_mipiphy_ref",
			2, JH7110_OSC),
	//JPEGC
	JH7110__DIV(JH7110_JPEGC_AXI, "jpegc_axi", 16, JH7110_VENC_ROOT),
	JH7110_GDIV(JH7110_CODAJ12_CLK_CORE, "u0_CODAJ12_clk_core",
			GATE_FLAG_NORMAL, 16, JH7110_VENC_ROOT),
	//VDEC
	JH7110__DIV(JH7110_VDEC_AXI, "vdec_axi", 7, JH7110_BUS_ROOT),
	JH7110_GDIV(JH7110_WAVE511_CLK_BPU, "u0_WAVE511_clk_bpu",
			GATE_FLAG_NORMAL, 7, JH7110_BUS_ROOT),
	JH7110_GDIV(JH7110_WAVE511_CLK_VCE, "u0_WAVE511_clk_vce",
			GATE_FLAG_NORMAL, 7, JH7110_VDEC_ROOT),
	//VENC
	JH7110__DIV(JH7110_VENC_AXI, "venc_axi", 15, JH7110_VENC_ROOT),
	JH7110_GDIV(JH7110_WAVE420L_CLK_BPU, "u0_wave420l_clk_bpu",
			GATE_FLAG_NORMAL, 15, JH7110_VENC_ROOT),
	JH7110_GDIV(JH7110_WAVE420L_CLK_VCE, "u0_wave420l_clk_vce",
			GATE_FLAG_NORMAL, 15, JH7110_VENC_ROOT),
	//INTMEM
	//QSPI
	JH7110__DIV(JH7110_QSPI_REF_SRC, "u0_cdns_qspi_ref_src",
			16, JH7110_GMACUSB_ROOT),
	JH7110_GMUX(JH7110_QSPI_CLK_REF, "u0_cdns_qspi_clk_ref",
			CLK_IGNORE_UNUSED, PARENT_NUMS_2,
			JH7110_OSC,
			JH7110_QSPI_REF_SRC),
	//SDIO
	JH7110_GDIV(JH7110_SDIO0_CLK_SDCARD, "u0_dw_sdio_clk_sdcard",
			CLK_IGNORE_UNUSED, 15, JH7110_AXI_CFG0),
	JH7110_GDIV(JH7110_SDIO1_CLK_SDCARD, "u1_dw_sdio_clk_sdcard",
			CLK_IGNORE_UNUSED, 15, JH7110_AXI_CFG0),
	//STG
	JH7110__DIV(JH7110_USB_125M, "usb_125m", 15, JH7110_GMACUSB_ROOT),
	//GMAC1
	JH7110__DIV(JH7110_GMAC_SRC, "gmac_src", 7, JH7110_GMACUSB_ROOT),
	JH7110__DIV(JH7110_GMAC1_GTXCLK, "gmac1_gtxclk",
			15, JH7110_GMACUSB_ROOT),
	JH7110__DIV(JH7110_GMAC1_RMII_RTX, "gmac1_rmii_rtx",
			30, JH7110_GMAC1_RMII_REFIN),
	JH7110_GDIV(JH7110_GMAC5_CLK_PTP, "u1_dw_gmac5_axi64_clk_ptp",
			GATE_FLAG_NORMAL, 31, JH7110_GMAC_SRC),
	JH7110_GMUX(JH7110_GMAC5_CLK_TX, "u1_dw_gmac5_axi64_clk_tx",
			GATE_FLAG_NORMAL, PARENT_NUMS_2,
			JH7110_GMAC1_GTXCLK,
			JH7110_GMAC1_RMII_RTX),
	//GMAC0
	JH7110_GDIV(JH7110_GMAC0_GTXCLK, "gmac0_gtxclk",
			GATE_FLAG_NORMAL, 15, JH7110_GMACUSB_ROOT),
	JH7110_GDIV(JH7110_GMAC0_PTP, "gmac0_ptp",
			GATE_FLAG_NORMAL, 31, JH7110_GMAC_SRC),
	JH7110_GDIV(JH7110_GMAC_PHY, "gmac_phy",
			GATE_FLAG_NORMAL, 31, JH7110_GMAC_SRC),
	//SYS MISC
	//CAN
	JH7110_GDIV(JH7110_CAN0_CTRL_CLK_TIMER, "u0_can_ctrl_clk_timer",
			GATE_FLAG_NORMAL, 24, JH7110_OSC),
	JH7110_GDIV(JH7110_CAN0_CTRL_CLK_CAN, "u0_can_ctrl_clk_can",
			GATE_FLAG_NORMAL, 63, JH7110_PERH_ROOT),
	JH7110_GDIV(JH7110_CAN1_CTRL_CLK_TIMER, "u1_can_ctrl_clk_timer",
			GATE_FLAG_NORMAL, 24, JH7110_OSC),
	JH7110_GDIV(JH7110_CAN1_CTRL_CLK_CAN, "u1_can_ctrl_clk_can",
			GATE_FLAG_NORMAL, 63, JH7110_PERH_ROOT),
	//PWM
	//WDT
	//TIMER
	//TEMP SENSOR
	JH7110_GDIV(JH7110_TEMP_SENSOR_CLK_TEMP, "u0_temp_sensor_clk_temp",
			GATE_FLAG_NORMAL, 24, JH7110_OSC),
	//SPI
	//I2C
	//UART
	JH7110_GDIV(JH7110_UART3_CLK_CORE, "u3_dw_uart_clk_core",
			GATE_FLAG_NORMAL, 10, JH7110_PERH_ROOT),
	JH7110_GDIV(JH7110_UART4_CLK_CORE, "u4_dw_uart_clk_core",
			GATE_FLAG_NORMAL, 10, JH7110_PERH_ROOT),
	JH7110_GDIV(JH7110_UART5_CLK_CORE, "u5_dw_uart_clk_core",
			GATE_FLAG_NORMAL, 10, JH7110_PERH_ROOT),
	//PWMDAC
	JH7110_GDIV(JH7110_PWMDAC_CLK_CORE, "u0_pwmdac_clk_core",
			GATE_FLAG_NORMAL, 256, JH7110_AUDIO_ROOT),
	//SPDIF
	//I2STX0_4CH0
	JH7110_GDIV(JH7110_I2STX_4CH0_BCLK_MST, "i2stx_4ch0_bclk_mst",
			GATE_FLAG_NORMAL, 32, JH7110_MCLK),
	JH7110_MDIV(JH7110_I2STX_4CH0_LRCK_MST, "i2stx_4ch0_lrck_mst",
			64, PARENT_NUMS_2,
			JH7110_I2STX_4CH0_BCLK_MST_INV,
			JH7110_I2STX_4CH0_BCLK_MST),
	//I2STX1_4CH0
	JH7110_GDIV(JH7110_I2STX_4CH1_BCLK_MST, "i2stx_4ch1_bclk_mst",
			GATE_FLAG_NORMAL, 32, JH7110_MCLK),
	JH7110_MDIV(JH7110_I2STX_4CH1_LRCK_MST, "i2stx_4ch1_lrck_mst",
			64, PARENT_NUMS_2,
			JH7110_I2STX_4CH1_BCLK_MST_INV,
			JH7110_I2STX_4CH1_BCLK_MST),
	//I2SRX_3CH
	JH7110_GDIV(JH7110_I2SRX_3CH_BCLK_MST, "i2srx_3ch_bclk_mst",
			GATE_FLAG_NORMAL, 32, JH7110_MCLK),
	JH7110_MDIV(JH7110_I2SRX_3CH_LRCK_MST, "i2srx_3ch_lrck_mst",
			64, PARENT_NUMS_2,
			JH7110_I2SRX_3CH_BCLK_MST_INV,
			JH7110_I2SRX_3CH_BCLK_MST),
	//PDM_4MIC
	JH7110_GDIV(JH7110_PDM_CLK_DMIC, "u0_pdm_4mic_clk_dmic",
			GATE_FLAG_NORMAL, 64, JH7110_MCLK),
	//TDM
	JH7110_GDIV(JH7110_TDM_INTERNAL, "tdm_internal",
			GATE_FLAG_NORMAL, 64, JH7110_MCLK),
	JH7110__DIV(JH7110_JTAG_CERTIFICATION_TRNG_CLK,
			"u0_jtag_certification_trng_clk", 4, JH7110_OSC),
};
#endif

#if 0
	/* root */
	jh7110_mux_register(sc, JH7110_CPU_ROOT, "cpu_root", parents_cpu_root, nitems(parents_cpu_root), 1);
	jh7110_div_register(sc, JH7110_CPU_CORE, "cpu_core", "cpu_root", 3);
	jh7110_div_register(sc, JH7110_CPU_BUS, "cpu_bus", "cpu_core", 2);
	jh7110_composite_register(sc, JH7110_PERH_ROOT, "perh_root", parents_perh_root, nitems(parents_perh_root), 1, 0, 2);

	jh7110_mux_register(sc, JH7110_BUS_ROOT, "bus_root", parents_bus_root, nitems(parents_bus_root), 1);
        jh7110_div_register(sc, JH7110_AXI_CFG0, "axi_cfg0", "bus_root", 2);
        jh7110_div_register(sc, JH7110_STG_AXIAHB, "stg_axiahb", "axi_cfg0", 2);
        jh7110_gate_register(sc, JH7110_AHB0, "ahb0", "stg_axiahb");
        jh7110_gate_register(sc, JH7110_AHB1, "ahb1", "stg_axiahb");
        jh7110_div_register(sc, JH7110_APB_BUS_FUNC, "apb_bus_func", "stg_axiahb", 4);
        jh7110_fixed_factor_register(sc, JH7110_U2_PCLK_MUX_PCLK, "u2_pclk_mux_pclk", "u2_pclk_mux_func_pclk", 1, 1);

        jh7110_gate_register(sc, JH7110_APB0, "apb0", "apb_bus");

        /* qspi */
        jh7110_gate_register(sc, JH7110_QSPI_CLK_AHB, "u0_cdns_qspi_clk_ahb", "ahb1");
        jh7110_gate_register(sc, JH7110_QSPI_CLK_APB, "u0_cdns_qspi_clk_apb", "apb12");
        jh7110_div_register(sc, JH7110_QSPI_REF_SRC, "u0_cdns_qspi_ref_src", "gmacusb_root", 5);
        jh7110_composite_register(sc, JH7110_QSPI_CLK_REF, "u0_cdns_qspi_clk_ref", parents_qspi_ref, nitems(parents_qspi_ref), 1, 1, 0);

        /* sdio */
        jh7110_gate_register(sc, JH7110_SDIO0_CLK_AHB, "u0_dw_sdio_clk_ahb", "ahb0");
        jh7110_gate_register(sc, JH7110_SDIO1_CLK_AHB, "u1_dw_sdio_clk_ahb", "ahb0");
        jh7110_fix_parent_composite_register(sc, JH7110_SDIO0_CLK_SDCARD, "u0_dw_sdio_clk_sdcard", "axi_cfg0", 0, 1, 4);
        jh7110_fix_parent_composite_register(sc, JH7110_SDIO1_CLK_SDCARD, "u1_dw_sdio_clk_sdcard", "axi_cfg0", 0, 1, 4);

        /* stg */
        jh7110_div_register(sc, JH7110_USB_125M, "usb_125m", "gmacusb_root", 4);

        /* gmac1 */
        jh7110_gate_register(sc, JH7110_GMAC5_CLK_AHB, "u1_dw_gmac5_axi64_clk_ahb", "ahb0");
        jh7110_gate_register(sc, JH7110_GMAC5_CLK_AXI, "u1_dw_gmac5_axi64_clk_axi", "stg_axiahb");
        jh7110_div_register(sc, JH7110_GMAC_SRC, "gmac_src", "gmacusb_root", 3);
        jh7110_div_register(sc, JH7110_GMAC1_GTXCLK, "gmac1_gtxclk", "gmacusb_root", 4);
        jh7110_gate_register(sc, JH7110_GMAC1_GTXC, "gmac1_gtxc", "gmac1_gtxclk");
        jh7110_div_register(sc, JH7110_GMAC1_RMII_RTX, "gmac1_rmii_rtx", "gmac1_rmii_refin", 5);
        jh7110_gate_div_register(sc, JH7110_GMAC5_CLK_PTP, "u1_dw_gmac5_axi64_clk_ptp", "gmac_src", 5);
        jh7110_composite_register(sc, JH7110_GMAC5_CLK_TX, "u1_dw_gmac5_axi64_clk_tx", parents_gmac5_tx, nitems(parents_gmac5_tx), 1, 1, 0);

        /* gmac0 */
        jh7110_fixed_factor_register(sc, JH7110_AON_AHB, "aon_ahb", "stg_axiahb", 1, 1);
        jh7110_gate_div_register(sc, JH7110_GMAC0_GTXCLK, "gmac0_gtxclk", "gmacusb_root", 4);
        jh7110_gate_div_register(sc, JH7110_GMAC0_PTP, "gmac0_ptp", "gmac_src", 5);
        jh7110_gate_register(sc, JH7110_GMAC0_GTXC, "gmac0_gtxc", "gmac0_gtxclk");

        /* uart0 */
        jh7110_gate_register(sc, JH7110_UART0_CLK_APB, "u0_dw_uart_clk_apb", "apb0");
        jh7110_gate_register(sc, JH7110_UART0_CLK_CORE, "u0_dw_uart_clk_core", "osc");

        /* uart1 */
        jh7110_gate_register(sc, JH7110_UART1_CLK_APB, "u1_dw_uart_clk_apb", "apb0");
        jh7110_gate_register(sc, JH7110_UART1_CLK_CORE, "u1_dw_uart_clk_core", "osc");

        /* uart2 */
        jh7110_gate_register(sc, JH7110_UART2_CLK_APB, "u2_dw_uart_clk_apb", "apb0");
        jh7110_gate_register(sc, JH7110_UART2_CLK_CORE, "u2_dw_uart_clk_core", "osc");

        /* uart3 */
        jh7110_gate_register(sc, JH7110_UART3_CLK_APB, "u3_dw_uart_clk_apb", "apb0");
        jh7110_gate_div_register(sc, JH7110_UART3_CLK_CORE, "u3_dw_uart_clk_core", "perh_root", 8);

        /* uart4 */
        jh7110_gate_register(sc, JH7110_UART4_CLK_APB, "u4_dw_uart_clk_apb", "apb0");
        jh7110_gate_div_register(sc, JH7110_UART4_CLK_CORE, "u4_dw_uart_clk_core", "perh_root", 8);

        /* uart5 */
        jh7110_gate_register(sc, JH7110_UART5_CLK_APB, "u5_dw_uart_clk_apb", "apb0");
        jh7110_gate_div_register(sc, JH7110_UART5_CLK_CORE, "u5_dw_uart_clk_core", "perh_root", 8);

        jh7110_fixed_factor_register(sc, JH7110_STG_APB, "stg_apb", "apb_bus", 1, 1);

        /* usb */
        jh7110_gate_register(sc, JH7110_USB0_CLK_USB_APB, "u0_cdn_usb_clk_usb_apb", "stg_apb");
        jh7110_gate_register(sc, JH7110_USB0_CLK_UTMI_APB, "u0_cdn_usb_clk_utmi_apb", "stg_apb");
        jh7110_gate_register(sc, JH7110_USB0_CLK_AXI, "u0_cdn_usb_clk_axi", "stg_axiahb");
        jh7110_gate_div_register(sc, JH7110_USB0_CLK_LPM, "u0_cdn_usb_clk_lpm", "osc", 2);
        jh7110_gate_div_register(sc, JH7110_USB0_CLK_STB, "u0_cdn_usb_clk_stb", "osc", 3);
        jh7110_gate_register(sc, JH7110_USB0_CLK_APP_125, "u0_cdn_usb_clk_app_125", "usb_125m");
        jh7110_div_register(sc, JH7110_USB0_REFCLK, "u0_cdn_usb_refclk", "osc", 2);

        /* gmac1 */
        jh7110_gate_register(sc, JH7110_U0_GMAC5_CLK_AHB, "u0_dw_gmac5_axi64_clk_ahb", "aon_ahb");
        jh7110_gate_register(sc, JH7110_U0_GMAC5_CLK_AXI, "u0_dw_gmac5_axi64_clk_axi", "aon_ahb");
        jh7110_div_register(sc, JH7110_GMAC0_RMII_RTX, "gmac0_rmii_rtx", "gmac0_rmii_refin", 5);
        jh7110_fixed_factor_register(sc, JH7110_U0_GMAC5_CLK_PTP, "u0_dw_gmac5_axi64_clk_ptp", "gmac0_ptp", 1, 1);
        jh7110_composite_register(sc, JH7110_U0_GMAC5_CLK_TX, "u0_dw_gmac5_axi64_clk_tx", parents_u0_dw_gmac5_axi64_clk_tx, nitems(parents_u0_dw_gmac5_axi64_clk_tx), 1, 1, 0);

        /* otp */
        jh7110_gate_register(sc, JH7110_OTPC_CLK_APB, "u0_otpc_clk_apb", "aon_apb");

        /* i2c */
        jh7110_gate_register(sc, JH7110_I2C5_CLK_APB, "u5_dw_i2c_clk_apb", "apb0");
        jh7110_gate_register(sc, JH7110_I2C5_CLK_CORE, "u5_dw_i2c_clk_core", "osc");
#endif

	rc = clkdom_finit(sc->clkdom);
	if (rc != 0)
		panic("jh7110_clkgen_attach: clkdom_finit failed: rc=%d", rc);

	return (0);

fail:
	if (sc->sys_base != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sys_rid, sc->sys_base);
	if (sc->stg_base != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, stg_rid, sc->stg_base);
	if (sc->aon_base != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, aon_rid, sc->aon_base);

	return (rc);
}

static inline void
jh7110_clkgen_get_bus_addr(device_t dev, int reg,
    bus_space_tag_t *bst, bus_space_handle_t *bsh, bus_addr_t *addr)
{
	struct jh7110_clkgen_softc *sc;
	sc = device_get_softc(dev);

	if (reg < JH7110_CLK_SYS_REG_END) {
		*bst = rman_get_bustag(sc->sys_base);
		*bsh = rman_get_bushandle(sc->sys_base);
		*addr = 4 * reg;
		return;
	}

	if (reg < JH7110_CLK_STG_REG_END) {
		*bst = rman_get_bustag(sc->stg_base);
		*bsh = rman_get_bushandle(sc->stg_base);
		*addr = 4 * (reg - JH7110_CLK_SYS_REG_END);
		return;
	}

	if (reg < JH7110_CLK_REG_END) {
		*bst = rman_get_bustag(sc->aon_base);
		*bsh = rman_get_bushandle(sc->aon_base);
		*addr = 4 * (reg - JH7110_CLK_STG_REG_END);
		return;
	}

	panic("get_bus_addr req for nonexistent clock reg %d", reg);
}

static int
jh7110_clkgen_write_4(device_t dev, bus_addr_t addr, uint32_t val)
{
	bus_space_tag_t bst;
	bus_space_handle_t bsh;

	device_printf(dev, "write_4: reg=0x%04lx\n", addr);

	jh7110_clkgen_get_bus_addr(dev, addr, &bst, &bsh, &addr);
	bus_space_write_4(bst, bsh, addr, val);

	return (0);
}

static int
jh7110_clkgen_read_4(device_t dev, bus_addr_t addr, uint32_t *val)
{
	bus_space_tag_t bst;
	bus_space_handle_t bsh;

	device_printf(dev, "read_4: reg=0x%04lx\n", addr);

	jh7110_clkgen_get_bus_addr(dev, addr, &bst, &bsh, &addr);
	*val = bus_space_read_4(bst, bsh, addr);

	return (0);
}

static int
jh7110_clkgen_modify_4(device_t dev, bus_addr_t addr, uint32_t clr, uint32_t set)
{
	bus_space_tag_t bst;
	bus_space_handle_t bsh;
	uint32_t val;

	device_printf(dev, "modify_4: reg=0x%04lx\n", addr);

	jh7110_clkgen_get_bus_addr(dev, addr, &bst, &bsh, &addr);

	val = bus_space_read_4(bst, bsh, addr);
	val &= ~clr;
	val |= set;
	bus_space_write_4(bst, bsh, addr, val);

	return (0);
}

static void
jh7110_clkgen_lock(device_t dev)
{
	struct jh7110_clkgen_softc *sc;
	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
}

static void
jh7110_clkgen_unlock(device_t dev)
{
	struct jh7110_clkgen_softc *sc;
	sc = device_get_softc(dev);
	mtx_unlock(&sc->mtx);
}

static device_method_t jh7110_clkgen_methods[] = {
	DEVMETHOD(device_probe,		jh7110_clkgen_probe),
	DEVMETHOD(device_attach,	jh7110_clkgen_attach),

	DEVMETHOD(clkdev_write_4,	jh7110_clkgen_write_4),
	DEVMETHOD(clkdev_read_4,	jh7110_clkgen_read_4),
	DEVMETHOD(clkdev_modify_4,	jh7110_clkgen_modify_4),
	DEVMETHOD(clkdev_device_lock,	jh7110_clkgen_lock),
	DEVMETHOD(clkdev_device_unlock,	jh7110_clkgen_unlock),

	DEVMETHOD_END
};

DEFINE_CLASS_0(starfive_jh7110_clkgen, jh7110_clkgen_driver, jh7110_clkgen_methods,
    sizeof(struct jh7110_clkgen_softc));

EARLY_DRIVER_MODULE(starfive_jh7110_clkgen, simplebus, jh7110_clkgen_driver, 0, 0,
    BUS_PASS_TIMER + BUS_PASS_ORDER_MIDDLE);
