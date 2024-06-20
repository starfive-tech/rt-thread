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
    bus_device[bus_dev_num].pci_prefetch.size = (1 << pcie->cfg.mem64_log);

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
	bus_device[bus_dev_num].dev_cnt++;
	total_bus_num++;
    }

    dm_pciauto_postscan_setup_bridge(&bus_device[bus_dev_num].dev,
    	second_bus);
    ret = bus_device[bus_dev_num].dev.seq;

    bus_dev_num++;
 
    return ret;
}

int register_msi_irq(void *arg, void (*handler)(int, void *arg), struct udevice *udev,
				int irq_num)
{
	udev->ops->register_msi(udev->priv, arg, handler, irq_num);

	return 0;
}

int alloc_msi_irq(struct udevice *udev, int irq_num)
{
	return udev->ops->alloc_msi_irq(udev->priv, irq_num);
}

struct udevice *get_match_pci_device(struct pci_device_id *dev_id)
{
	struct pci_bus_device *bus_dev;
	struct pci_device *pci;
	int i, j;

	if (!bus_dev_num)
		return NULL;

	for (i = 0; i < bus_dev_num; i++) {
		bus_dev = &bus_device[i];
		for (j = 0; j < bus_dev->dev_cnt; j++) {
			pci = &bus_dev->pci_dev[j];
			if ((dev_id->vendor == pci->dev.pplat.vendor) &&
				(dev_id->device== pci->dev.pplat.device)) {
				return &pci->dev;
			}
		}
	}

	return NULL;
}

