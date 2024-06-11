#include <rtthread.h>

#include "board.h"
#include "pci.h"
#include "hal_pcie.h"

int bus_dev_num; 
int total_bus_num;

struct pci_bus_device bus_device[2];

int pci_scan_bus(struct pcie *pcie, void *pci_ops)
{
    int ret, second_bus;
    struct pci_device *pci_device;
    struct dm_pci_ops *ops = pci_ops;

    pcie->bus_device = &bus_device[bus_dev_num];

    bus_device[bus_dev_num].dev.seq = total_bus_num;
    bus_device[bus_dev_num].dev.type = 0;

    bus_device[bus_dev_num].pci_mem.bus_start = pcie->cfg.mem32_base;
    bus_device[bus_dev_num].pci_mem.size = (1 << pcie->cfg.mem32_log);
    bus_device[bus_dev_num].pci_prefetch.bus_start = pcie->cfg.mem64_base;
    bus_device[bus_dev_num].pci_prefetch.size = pcie->cfg.mem64_log;

    bus_device[bus_dev_num].hose.pci_mem = &bus_device[bus_dev_num].pci_mem;
    bus_device[bus_dev_num].hose.pci_prefetch = &bus_device[bus_dev_num].pci_prefetch;
    bus_device[bus_dev_num].dev.hose = &bus_device[bus_dev_num].hose;
    bus_device[bus_dev_num].dev.type = 0x0; /* bus dev */
    bus_device[bus_dev_num].dev.ops = ops;
    pciauto_config_init(&bus_device[bus_dev_num].hose);

    bus_device[bus_dev_num].dev.priv = pcie->priv;
    pcie->bus_device = &bus_device[bus_dev_num];
    total_bus_num++;

    second_bus = pci_probe(&bus_device[bus_dev_num].dev);

    if (second_bus > 0) {
	pci_device = &bus_device[bus_dev_num].pci_dev[0];
	pci_device->dev.seq = second_bus;
	pci_device->dev.type = 0x1;
	pci_device->dev.hose = &bus_device[bus_dev_num].hose;
	pci_device->dev.ops = ops;
	pci_device->dev.priv = pcie->priv;
	pci_probe(&pci_device->dev);
	total_bus_num++;
    }

    dm_pciauto_postscan_setup_bridge(&bus_device[bus_dev_num].dev,
    	second_bus);
    ret = bus_device[bus_dev_num].dev.seq;

    bus_dev_num++;
 
    return ret;
}

#if 0
void get_bus_first_no(int dev_num)
{
    return bus_device[dev_num].dev.seq;
}
#endif
