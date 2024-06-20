
#include <rtthread.h>
#include <riscv_io.h>

#include "board.h"
#include "pci.h"

#define printf hal_printf

void pcie_intx(struct pci_msi_dev *pdev, int enable)
{
	uint16_t pci_command, new;
	struct udevice *udev = (void *)pdev;

	dm_pci_read_config16(udev, PCI_COMMAND, &pci_command);

	if (enable)
		new = pci_command & ~PCI_COMMAND_INTX_DISABLE;
	else
		new = pci_command | PCI_COMMAND_INTX_DISABLE;

	if (new != pci_command)
		dm_pci_write_config16(udev, PCI_COMMAND, new);
}

void pcie_msix_vec_count(struct pci_msi_dev *dev)
{
	struct udevice *udev = (void *)dev;
	uint16_t control;

	dm_pci_read_config16(udev, dev->msix_cap + PCI_MSIX_FLAGS, &control);
	dev->msix_hwsize = ((control & PCI_MSIX_FLAGS_QSIZE) + 1);
}

static void msix_mask_all(void *base, int tsize, int nvec)
{
	uint32_t ctrl = PCI_MSIX_ENTRY_CTRL_MASKBIT;
	int i;

	base += nvec * PCI_MSIX_ENTRY_SIZE;
	for (i = nvec; i < tsize; i++, base += PCI_MSIX_ENTRY_SIZE)
		sys_writel(ctrl, base + PCI_MSIX_ENTRY_VECTOR_CTRL);
}

void msix_prepare_msi_desc(struct pci_msi_dev *dev, struct msi_desc *desc)
{
	void *addr;

	desc->nvec_used				= 1;
	desc->pci.msi_attrib.is_msix		= 1;
	desc->pci.msi_attrib.is_64		= 1;
	desc->pci.msi_attrib.default_irq	= dev->irq;
	desc->pci.mask_base			= dev->msix_base;
	desc->pci.msi_attrib.can_mask		= 1;

	addr = pci_msix_desc_addr(desc);
	desc->pci.msix_ctrl = sys_readl(addr + PCI_MSIX_ENTRY_VECTOR_CTRL);
}

static int msix_setup_msi_descs(struct pci_msi_dev *dev, int nvec)
{
	int i;
	struct msi_desc desc;

	for (i = 0; i < nvec; i++) {
		desc.msi_index = i;
		msix_prepare_msi_desc(dev, &dev->desc[i]);
	}
	return 0;
}

static inline void pcie_msix_write_vector_ctrl(struct msi_desc *desc, uint32_t ctrl)
{
	void *desc_addr = pci_msix_desc_addr(desc);

	if (desc->pci.msi_attrib.can_mask) {
		sys_writel(ctrl, desc_addr + PCI_MSIX_ENTRY_VECTOR_CTRL);
	}
}

static inline void pcie_write_msg_msi(struct pci_msi_dev *dev, struct msi_desc *desc,
				     struct msi_msg *msg)
{
	struct udevice *udev = (void *)dev;
	int pos = dev->msi_cap;
	uint16_t msgctl;

	dm_pci_read_config16(udev, pos + PCI_MSI_FLAGS, &msgctl);
	msgctl &= ~PCI_MSI_FLAGS_QSIZE;
	msgctl |= desc->pci.msi_attrib.multiple << 4;
	dm_pci_write_config16(udev, pos + PCI_MSI_FLAGS, msgctl);

	dm_pci_write_config32(udev, pos + PCI_MSI_ADDRESS_LO, msg->address_lo);
	if (desc->pci.msi_attrib.is_64) {
		dm_pci_write_config32(udev, pos + PCI_MSI_ADDRESS_HI,  msg->address_hi);
		dm_pci_write_config16(udev, pos + PCI_MSI_DATA_64, msg->data);
	} else {
		dm_pci_write_config16(udev, pos + PCI_MSI_DATA_32, msg->data);
	}
	/* Ensure that the writes are visible in the device */
	dm_pci_read_config16(udev, pos + PCI_MSI_FLAGS, &msgctl);
}

static inline void pcie_write_msg_msix(struct msi_desc *desc, struct msi_msg *msg)
{
	void *base = pci_msix_desc_addr(desc);
	uint32_t ctrl = desc->pci.msix_ctrl;
	int unmasked = !(ctrl & PCI_MSIX_ENTRY_CTRL_MASKBIT);

	/*
	 * The specification mandates that the entry is masked
	 * when the message is modified:
	 *
	 * "If software changes the Address or Data value of an
	 * entry while the entry is unmasked, the result is
	 * undefined."
	 */
	if (unmasked)
		pcie_msix_write_vector_ctrl(desc, ctrl | PCI_MSIX_ENTRY_CTRL_MASKBIT);

	sys_writel(msg->address_lo, base + PCI_MSIX_ENTRY_LOWER_ADDR);
	sys_writel(msg->address_hi, base + PCI_MSIX_ENTRY_UPPER_ADDR);
	sys_writel(msg->data, base + PCI_MSIX_ENTRY_DATA);

	if (unmasked)
		pcie_msix_write_vector_ctrl(desc, ctrl);

	/* Ensure that the writes are visible in the device */
	sys_readl(base + PCI_MSIX_ENTRY_DATA);
}

static void pci_msix_clear_and_set_ctrl(struct pci_msi_dev *dev, uint16_t clear, uint16_t set)
{
	struct udevice *udev = (void *)dev;
	uint16_t ctrl;

	dm_pci_read_config16(udev, dev->msix_cap + PCI_MSIX_FLAGS, &ctrl);
	ctrl &= ~clear;
	ctrl |= set;
	dm_pci_write_config16(udev, dev->msix_cap + PCI_MSIX_FLAGS, ctrl);
}

static void *msix_map_region(struct pci_msi_dev *dev,
				     unsigned int nr_entries)
{
	struct udevice *udev = (void *)dev;
	uint32_t addr0, addr1, table_offset;
	unsigned long flags;
	//void *region;
	uint8_t bir;

	dm_pci_read_config32(udev, dev->msix_cap + PCI_MSIX_TABLE,
			      &table_offset);
	bir = (uint8_t)(table_offset & PCI_MSIX_TABLE_BIR);

	dm_pci_read_config32(udev, PCI_BASE_ADDRESS_0 + bir * 4, &addr0);
	//dm_pci_read_config32(udev, PCI_BASE_ADDRESS_0 + bir * 4 + 4, &addr1);

	return (void *)(unsigned long)(addr0 & ~0xf);
}

static int pci_msi_setup_msi_irqs(struct pci_msi_dev *dev, int nvec) 
{
	struct udevice *udev = (void *)dev;
	struct msi_desc *desc = &dev->desc[0];
	int i;

	for (i = 0; i < nvec; i++) {
		desc->msg.data = i + dev->irq;
		udev->ops->compose_msi(udev->priv, &desc->msg);
		pcie_write_msg_msi(dev, desc, &desc->msg);
	}
}

static int pci_msix_setup_msi_irqs(struct pci_msi_dev *dev, int nvec) 
{
	struct udevice *udev = (void *)dev;
	struct msi_desc *desc;
	int i;

	for (i = 0; i < nvec; i++) {
		desc = &dev->desc[i];
		desc->msg.data = i + dev->irq;
		udev->ops->compose_msi(udev->priv, &desc->msg);
		pcie_write_msg_msix(desc, &desc->msg);
	}
	return 0;
}

static int msix_setup_interrupts(struct pci_msi_dev *dev, int nvec)
{
	int ret;

	//msi_lock_descs(&dev->dev);
	ret = msix_setup_msi_descs(dev, nvec);
	if (ret)
		return ret;

	ret = pci_msix_setup_msi_irqs(dev, nvec);
	if (ret)
		return ret;

	/* Check if all MSI entries honor device restrictions */
#if 0
	ret = msi_verify_entries(dev);
	if (ret)
		goto out_free;

	msix_update_entries(dev, entries);
#endif
	//pci_free_msi_irqs(dev);
	//msi_unlock_descs(&dev->dev);
	//kfree(masks);
	return ret;
}


static int msix_cap_init(struct pci_msi_dev *dev, int nvec)
{
	int ret, tsize;
	uint16_t control;

	/*
	 * Some devices require MSI-X to be enabled before the MSI-X
	 * registers can be accessed.  Mask all the vectors to prevent
	 * interrupts coming in before they're fully set up.
	 */
	pci_msix_clear_and_set_ctrl(dev, 0, PCI_MSIX_FLAGS_MASKALL |
				    PCI_MSIX_FLAGS_ENABLE);

	/* Mark it enabled so setup functions can query it */
	dev->msix_enabled = 1;

#if 0
	dm_pci_read_config16(dev, dev->msix_cap + PCI_MSIX_FLAGS, &control);
	/* Request & Map MSI-X table region */
	tsize = msix_table_size(control);
#endif
	tsize = dev->msix_hwsize;
	dev->msix_base = msix_map_region(dev, dev->msix_hwsize);
	if (!dev->msix_base) {
		hal_printf("msix base null\n");
		return -1;
		//goto out_disable;
	}

	ret = msix_setup_interrupts(dev, nvec);
	if (ret)
		goto out_disable;

	//pr_info("nvec %d", nvec);
	/* Disable INTX */
	pcie_intx(dev, 0);

	/*
	 * Ensure that all table entries are masked to prevent
	 * stale entries from firing in a crash kernel.
	 *
	 * Done late to deal with a broken Marvell NVME device
	 * which takes the MSI-X mask bits into account even
	 * when MSI-X is disabled, which prevents MSI delivery.
	 */
	msix_mask_all(dev->msix_base, tsize, nvec);
	pci_msix_clear_and_set_ctrl(dev, PCI_MSIX_FLAGS_MASKALL, 0);

	return 0;

out_disable:
	dev->msix_enabled = 0;
	pci_msix_clear_and_set_ctrl(dev, PCI_MSIX_FLAGS_MASKALL | PCI_MSIX_FLAGS_ENABLE, 0);

	return -1;
}

int pcie_enable_msix_range(struct pci_msi_dev *dev, int minvec, int maxvec, int flags)
{
	int hwsize, rc, nvec = maxvec;

	if (dev->msi_enabled) {
		hal_printf("can't enable MSI-X (MSI already enabled)\n");
		return -1;
	}
#if 0
	if (WARN_ON_ONCE(dev->msix_enabled))
		return -EINVAL;


	/* Check MSI-X early on irq domain enabled architectures */
	if (!pci_msi_domain_supports(dev, MSI_FLAG_PCI_MSIX, ALLOW_LEGACY))
		return -ENOTSUPP;


	if (!pci_msi_supported(dev, nvec) || dev->current_state != PCI_D0)
		return -EINVAL;
#endif
	pcie_msix_vec_count(dev);

#if 0
	if (hwsize < 0)
		return hwsize;


	if (!pci_msix_validate_entries(dev, entries, nvec))
		return -EINVAL;

	if (hwsize < nvec) {
		/* Keep the IRQ virtual hackery working */
		if (flags & PCI_IRQ_VIRTUAL)
			hwsize = nvec;
		else
			nvec = hwsize;
	}

	if (nvec < minvec)
		return -ENOSPC;

	rc = pci_setup_msi_context(dev);
	if (rc)
		return rc;

	if (!pci_setup_msix_device_domain(dev, hwsize))
		return -ENODEV;
#endif

	for (;;) {
		rc = msix_cap_init(dev, nvec);
		if (rc == 0)
			return nvec;

		if (rc < 0)
			return rc;
		if (rc < minvec)
			return -1;

		nvec = rc;
	}
}

static void pci_msi_set_enable(struct pci_msi_dev *dev, int enable)
{
	struct udevice *udev = (void *)dev;
	uint16_t control;

	dm_pci_read_config16(udev, dev->msi_cap + PCI_MSI_FLAGS, &control);
	control &= ~PCI_MSI_FLAGS_ENABLE;
	if (enable)
		control |= PCI_MSI_FLAGS_ENABLE;
	dm_pci_write_config16(udev, dev->msi_cap + PCI_MSI_FLAGS, control);
}

static int msi_setup_msi_desc(struct pci_msi_dev *dev, int nvec)
{
	struct udevice *udev = (void *)dev;
	struct msi_desc *desc = &dev->desc[0];
	uint16_t control;

	/* MSI Entry Initialization */
	//memset(&desc, 0, sizeof(desc));

	dm_pci_read_config16(udev, dev->msi_cap + PCI_MSI_FLAGS, &control);
	/* Lies, damned lies, and MSIs */
	//if (dev->dev_flags & PCI_DEV_FLAGS_HAS_MSI_MASKING)
		//control |= PCI_MSI_FLAGS_MASKBIT;
	/* Respect XEN's mask disabling */
	//if (pci_msi_ignore_mask)
		//control &= ~PCI_MSI_FLAGS_MASKBIT;

	desc->nvec_used			= nvec;
	desc->pci.msi_attrib.is_64	= !!(control & PCI_MSI_FLAGS_64BIT);
	desc->pci.msi_attrib.can_mask	= !!(control & PCI_MSI_FLAGS_MASKBIT);
	desc->pci.msi_attrib.default_irq	= dev->irq;
	desc->pci.msi_attrib.multi_cap	= (control & PCI_MSI_FLAGS_QMASK) >> 1;
	if (nvec == 1)
		desc->pci.msi_attrib.multiple = 0;
	else if (nvec == 2)
		desc->pci.msi_attrib.multiple = 1;
	else if (nvec <= 4)
		desc->pci.msi_attrib.multiple = 2;
	else if (nvec <= 8)
		desc->pci.msi_attrib.multiple = 3;
	//desc.pci.msi_attrib.multiple	= ilog2(__roundup_pow_of_two(nvec));

	if (control & PCI_MSI_FLAGS_64BIT)
		desc->pci.mask_pos = dev->msi_cap + PCI_MSI_MASK_64;
	else
		desc->pci.mask_pos = dev->msi_cap + PCI_MSI_MASK_32;

	/* Save the initial mask status */
#if 0
	if (desc.pci.msi_attrib.can_mask)
		pci_read_config_dword(dev, desc.pci.mask_pos, &desc.pci.msi_mask);

	return msi_insert_msi_desc(&dev->dev, &desc);
#endif

}

static int msi_cap_init(struct pci_msi_dev *dev, int nvec)
{
	int ret;

	/* Reject multi-MSI early on irq domain enabled architectures */
	//if (nvec > 1 && !pci_msi_domain_supports(dev, MSI_FLAG_MULTI_PCI_MSI, ALLOW_LEGACY))
		//return 1;

	/*
	 * Disable MSI during setup in the hardware, but mark it enabled
	 * so that setup code can evaluate it.
	 */
	pci_msi_set_enable(dev, 0);
	dev->msi_enabled = 1;

	//msi_lock_descs(&dev->dev);
	ret = msi_setup_msi_desc(dev, nvec);
	if (ret)
		goto fail;

#if 0
	/* All MSIs are unmasked by default; mask them all */
	entry = msi_first_desc(&dev->dev, MSI_DESC_ALL);
	pci_msi_mask(entry, msi_multi_mask(entry));
#endif

	/* Configure MSI capability structure */
	ret = pci_msi_setup_msi_irqs(dev, nvec);
	if (ret)
		goto fail;

#if 0
	ret = msi_verify_entries(dev);
	if (ret)
		goto err;
#endif
	/* Set MSI enabled bits	*/
	pcie_intx(dev, 0);
	pci_msi_set_enable(dev, 1);

	//pcibios_free_irq(dev);
	//dev->irq = entry->irq;
	return ret;


	//pci_msi_unmask(entry, msi_multi_mask(entry));
	//pci_free_msi_irqs(dev);
fail:
	dev->msi_enabled = 0;
	return ret;
}


int pci_msi_vec_count(struct pci_msi_dev *dev)
{
	int ret;
	struct udevice *udev = (void *)dev;
	uint16_t msgctl;

	dm_pci_read_config16(udev, dev->msi_cap + PCI_MSI_FLAGS, &msgctl);
	ret = 1 << ((msgctl & PCI_MSI_FLAGS_QMASK) >> 1);

	return ret;
}

int pcie_enable_msi_range(struct pci_msi_dev *dev, int minvec, int maxvec)
{
	int nvec;
	int rc;

	if (dev->current_state != PCI_D0) {
		hal_printf("currect state is not d0\n");
		return -1;
	}
	/* Check whether driver already requested MSI-X IRQs */
	if (dev->msix_enabled) {
		hal_printf("can't enable MSI (MSI-X already enabled)\n");
		return -1;
	}

	nvec = pci_msi_vec_count(dev);
	if (nvec < 0)
		return nvec;
	if (nvec < minvec)
		return -1;

	if (nvec > maxvec)
		nvec = maxvec;
#if 0
	rc = pci_setup_msi_context(dev);
	if (rc)
		return rc;

	if (!pci_setup_msi_device_domain(dev))
		return -ENODEV;
#endif

	for (;;) {
		rc = msi_cap_init(dev, nvec);
		if (rc == 0)
			return nvec;

		if (rc < 0)
			return rc;
		if (rc < minvec)
			return -1;

		nvec = rc;
	}
}

int pcie_alloc_irq_vectors(struct udevice *udev, unsigned int min_vecs,
				   unsigned int max_vecs, unsigned int msi_irq, unsigned int flags)
{
	int nvecs;
	struct pci_msi_dev *dev = &udev->msi_dev;

	dev->irq = msi_irq;

	if (flags & PCI_IRQ_MSIX) {
		nvecs = pcie_enable_msix_range(dev, min_vecs, max_vecs,
						flags);
		if (nvecs > 0)
			return nvecs;
	}

	if (flags & PCI_IRQ_MSI) {
		nvecs = pcie_enable_msi_range(dev, min_vecs, max_vecs);
		if (nvecs > 0)
			return nvecs;
	}

	/* use legacy IRQ if allowed */
	if (flags & PCI_IRQ_LEGACY) {
		if (min_vecs == 1 && dev->irq) {
			/*
			 * Invoke the affinity spreading logic to ensure that
			 * the device driver can adjust queue configuration
			 * for the single interrupt case.
			 */
			pcie_intx(dev, 1);
			return 1;
		}
	}

	return nvecs;
}

void pcie_msi_init(struct pci_msi_dev *dev)
{
	struct udevice *udev = (void *)dev;
	uint16_t ctrl;

	dev->msi_cap = dm_pci_find_capability(udev, PCI_CAP_ID_MSI);

	if (!dev->msi_cap)
		return;

	dm_pci_read_config16(udev, dev->msi_cap + PCI_MSI_FLAGS, &ctrl);
	if (ctrl & PCI_MSI_FLAGS_ENABLE) {
		dm_pci_write_config16(udev, dev->msi_cap + PCI_MSI_FLAGS,
				      ctrl & ~PCI_MSI_FLAGS_ENABLE);
	}

	if (!(ctrl & PCI_MSI_FLAGS_64BIT))
		dev->no_64bit_msi = 1;
}

void pcie_msix_init(struct pci_msi_dev *dev)
{
	uint16_t ctrl;
	struct udevice *udev = (void *)dev;

	dev->msix_cap = dm_pci_find_capability(udev, PCI_CAP_ID_MSIX);
	if (!dev->msix_cap)
		return;

	dm_pci_read_config16(udev, dev->msix_cap + PCI_MSIX_FLAGS, &ctrl);
	if (ctrl & PCI_MSIX_FLAGS_ENABLE) {
		dm_pci_write_config16(udev, dev->msix_cap + PCI_MSIX_FLAGS,
				      ctrl & ~PCI_MSIX_FLAGS_ENABLE);
	}
}

static void pci_pme_active(struct pci_msi_dev *dev, int enable)
{
	uint16_t pmcsr;
	struct udevice *udev = (void *)dev;

	if (!dev->pme_support)
		return;

	dm_pci_read_config16(udev, dev->pm_cap + PCI_PM_CTRL, &pmcsr);
	/* Clear PME_Status by writing 1 to it and enable PME# */
	pmcsr |= PCI_PM_CTRL_PME_STATUS | PCI_PM_CTRL_PME_ENABLE;
	if (!enable)
		pmcsr &= ~PCI_PM_CTRL_PME_ENABLE;

	dm_pci_write_config16(udev, dev->pm_cap + PCI_PM_CTRL, pmcsr);
}

void pcie_pm_init(struct pci_msi_dev *dev)
{
	struct udevice *udev = (void *)dev;
	int pm;
	uint16_t status, pmcsr;
	uint16_t pmc;

	dev->pm_cap = 0;
	dev->pme_support = 0;

	/* find PCI PM capability in list */
	dev->pm_cap = dm_pci_find_capability(udev, PCI_CAP_ID_PM);
	if (!dev->pm_cap)
		return;
	/* Check device's ability to generate PME# */
	dm_pci_read_config16(udev, dev->pm_cap + PCI_PM_PMC, &pmc);

	if ((pmc & PCI_PM_CAP_VER_MASK) > 3) {
		hal_printf("unsupported PM cap regs version (%u)\n",
			pmc & PCI_PM_CAP_VER_MASK);
		return;
	}

	//dev->pm_cap = pm;
	//dev->d3hot_delay = PCI_PM_D3HOT_WAIT;
	//dev->d3cold_delay = PCI_PM_D3COLD_WAIT;
	//dev->bridge_d3 = pci_bridge_d3_possible(dev);
	dev->d3cold_allowed = 1;

	dev->d1_support = 0;
	dev->d2_support = 0;
	//if (!pci_no_d1d2(dev))
	{
		if (pmc & PCI_PM_CAP_D1)
			dev->d1_support = 1;
		if (pmc & PCI_PM_CAP_D2)
			dev->d2_support = 1;

		if (dev->d1_support || dev->d2_support)
			hal_printf("supports%s%s\n",
				   dev->d1_support ? " D1" : "",
				   dev->d2_support ? " D2" : "");
	}

	pmc &= PCI_PM_CAP_PME_MASK;
	if (pmc) {
		hal_printf("PME# supported from%s%s%s%s%s\n",
			 (pmc & PCI_PM_CAP_PME_D0) ? " D0" : "",
			 (pmc & PCI_PM_CAP_PME_D1) ? " D1" : "",
			 (pmc & PCI_PM_CAP_PME_D2) ? " D2" : "",
			 (pmc & PCI_PM_CAP_PME_D3hot) ? " D3hot" : "",
			 (pmc & PCI_PM_CAP_PME_D3cold) ? " D3cold" : "");
		dev->pme_support = (pmc >> PCI_PM_CAP_PME_SHIFT);
		dev->pme_poll = 1;
		/*
		 * Make device's PM flags reflect the wake-up capability, but
		 * let the user space enable it to wake up the system as needed.
		 */
		//device_set_wakeup_capable(&dev->dev, true);
		/* Disable the PME# generation functionality */
		pci_pme_active(dev, 0);
	}

	dm_pci_read_config16(udev, PCI_STATUS, &status);
	if (status & PCI_STATUS_IMM_READY)
		dev->imm_ready = 1;


	dm_pci_read_config16(udev, dev->pm_cap + PCI_PM_CTRL, &pmcsr);

	dev->current_state = pmcsr & PCI_PM_CTRL_STATE_MASK;
	hal_printf("pm current state %d\n", dev->current_state);
}


void pcie_init_capabilities(struct udevice *dev)
{
	struct pci_msi_dev *msi_dev = &dev->msi_dev;

	pcie_msi_init(msi_dev);
	pcie_msix_init(msi_dev);
	pcie_pm_init(msi_dev);
}

