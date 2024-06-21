#include <rtthread.h>

#include "board.h"
#include "pci.h"
#include "hal_pcie.h"

int bus_dev_num; 
int total_bus_num;

struct pci_bus_device *bus_device[4];

int pci_scan_bus(struct pcie *pcie, void *pci_ops)
{
    int ret, second_bus;
    struct pci_device *pci_device;
    struct pci_bus_device *bdev;
    struct dm_pci_ops *ops = pci_ops;

    bdev = hal_malloc(sizeof(struct pci_bus_device));
    if (!bdev)
	return -1;

    memset(bdev, 0, sizeof(struct pci_bus_device));
    pcie->bus_device = bdev;

    bdev->dev.seq = total_bus_num;
    bdev->dev.type = 0;

    bdev->pci_mem.bus_start = pcie->cfg.mem32_base;
    bdev->pci_mem.size = (1 << pcie->cfg.mem32_log);
    bdev->pci_prefetch.bus_start = pcie->cfg.mem64_base;
    bdev->pci_prefetch.size = (1 << pcie->cfg.mem64_log);

    bdev->hose.pci_mem = &bdev->pci_mem;
    bdev->hose.pci_prefetch = &bdev->pci_prefetch;
    bdev->dev.hose = &bdev->hose;
    bdev->dev.type = 0x0; /* bus dev */
    bdev->dev.ops = ops;
    pciauto_config_init(&bdev->hose);

    bdev->dev.priv = pcie->priv;
    total_bus_num++;

    second_bus = pci_probe(&bdev->dev);

    if (second_bus > 0) {
	pci_device = hal_malloc(sizeof(struct pci_device));
	if (!pci_device) {
		hal_free(bdev);
		return -1;
	}
	memset(pci_device, 0, sizeof(struct pci_device));
	pci_device->dev.seq = second_bus;
	pci_device->dev.type = 0x1;
	pci_device->dev.hose = &bdev->hose;
	pci_device->dev.ops = ops;
	pci_device->dev.priv = pcie->priv;
	ret = pci_probe(&pci_device->dev);
	if (ret < 0) {
		hal_free(pci_device);
		return bdev->dev.seq;
	}
	bdev->pci_dev[bdev->dev_cnt] = pci_device;
	bdev->dev_cnt++;
	total_bus_num++;
    }

    dm_pciauto_postscan_setup_bridge(&bdev->dev,
    	second_bus);
    ret = bdev->dev.seq;

    bus_device[bus_dev_num] = bdev;
    bus_dev_num++;
 
    return ret;
}

int pcie_resource_free(void)
{
    int i, j;

    for(i = 0; i < bus_dev_num; i++) {
	for (j = 0; j < bus_device[i]->dev_cnt; j++) {
		hal_free(bus_device[i]->pci_dev[j]);
	}
	hal_free(bus_device[i]);
	bus_device[i] = NULL;
    }
    bus_dev_num = 0;
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
		bus_dev = bus_device[i];
		for (j = 0; j < bus_dev->dev_cnt; j++) {
			pci = bus_dev->pci_dev[j];
			if ((dev_id->vendor == pci->dev.pplat.vendor) &&
				(dev_id->device== pci->dev.pplat.device)) {
				return &pci->dev;
			}
		}
	}

	return NULL;
}

