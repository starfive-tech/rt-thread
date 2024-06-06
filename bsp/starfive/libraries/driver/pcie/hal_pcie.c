#include <rtthread.h>
#include <rtdevice.h>

#include "hal_pcie.h"

enum
{
    PCIE_START = -1,
#if defined(BSP_USING_PCIE0)
    PCIE0_IDX,
#endif
#if defined(BSP_USING_PCIE1)
    PCIE1_IDX,
#endif
    PCIE1_CNT
};

static struct pcie pcie_inst[] =
{
#if defined(BSP_USING_PCIE0)
{
	.name		 =  "pcie0",
	.index		 =  0,
},
#endif
#if defined(BSP_USING_PCIE1)
{
	.name		 =  "pcie1",
	.index		 =  1,
},
#endif
};

static void pcie_isr(int vector, void *param)
{
    struct pcie *pcie = (void*)param;

    pcie->irq_handle(pcie->priv);
}

void rt_hw_pcie_init(void)
{
	int i;

	for (i = (PCIE_START + 1); i < PCIE1_CNT; i++) {
		pcie_plat_init(&pcie_inst[i]);
		pcie_init(&pcie_inst[i]);
		rt_hw_interrupt_install(pcie_inst[i].irq, pcie_isr, (void *)&pcie_inst[i], pcie_inst[i].name);

		/* Unmask interrupt. */
		rt_hw_interrupt_umask(pcie_inst[i].irq);

	}
}


