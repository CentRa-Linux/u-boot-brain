#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>

void mxs_set_divcpu(uint8_t clkdiv) {
	struct mxs_clkctrl_regs *clkctrl_regs =
		(struct mxs_clkctrl_regs *)MXS_CLKCTRL_BASE;
	uint32_t clkctrl;
	clkctrl = readl(&clkctrl_regs->hw_clkctrl_cpu);
	clkctrl &= ~0x3F;
	clkctrl |= (clkdiv & 0x3F);
	writel(clkctrl, &clkctrl_regs->hw_clkctrl_cpu);
}
