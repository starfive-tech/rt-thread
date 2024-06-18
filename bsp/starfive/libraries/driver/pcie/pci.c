#include <rtthread.h>

#include "board.h"
#include "pci.h"

#define printf hal_printf

int pci_bus_read_config(const struct udevice *bus, pci_dev_t bdf, int offset,
			unsigned long *valuep, enum pci_size_t size)
{
	struct dm_pci_ops *ops;

	ops = pci_get_ops(bus);
	if (!ops->read_config)
		return -1;
	return ops->read_config(bus, bdf, offset, valuep, size);
}

int pci_bus_write_config(struct udevice *bus, pci_dev_t bdf, int offset,
			 unsigned long value, enum pci_size_t size)
{
	struct dm_pci_ops *ops;

	ops = pci_get_ops(bus);
	if (!ops->write_config)
		return -1;
	return ops->write_config(bus, bdf, offset, value, size);
}

int dm_pci_read_config(struct udevice *dev, int offset,
                       unsigned long *valuep, enum pci_size_t size)
{
        //const struct udevice *bus;        
	struct pci_child_plat *pplat = dev_get_parent_plat(dev);

	//return PCI_ADD_BUS(dev_seq(dev), pplat->devfn);

        //for (bus = dev; device_is_on_pci_bus(bus);)
               // bus = bus->parent;
        return pci_bus_read_config(dev, dm_pci_get_bdf(dev), offset, valuep,
                                   size);
}

int dm_pci_write_config(struct udevice *dev, int offset, unsigned long value,
                        enum pci_size_t size)
{
        //struct udevice *bus;

        //for (bus = dev; device_is_on_pci_bus(bus);)
                //bus = bus->parent;
        return pci_bus_write_config(dev, dm_pci_get_bdf(dev), offset, value,
                                    size);
}

int dm_pci_read_config8(struct udevice *dev, int offset, uint8_t *valuep)
{
        unsigned long value;
        int ret;

        ret = dm_pci_read_config(dev, offset, &value, PCI_SIZE_8);
        if (ret)
                return ret;
        *valuep = value;

        return 0;
}

int dm_pci_read_config16(struct udevice *dev, int offset, uint16_t *valuep)
{
        unsigned long value;
        int ret;

        ret = dm_pci_read_config(dev, offset, &value, PCI_SIZE_16);
        if (ret)
                return ret;
        *valuep = value;

        return 0;
}

int dm_pci_read_config32(struct udevice *dev, int offset, uint32_t *valuep)
{
        unsigned long value;
        int ret;

        ret = dm_pci_read_config(dev, offset, &value, PCI_SIZE_32);
        if (ret)
                return ret;
        *valuep = value;

        return 0;
}

int dm_pci_write_config8(struct udevice *dev, int offset, uint8_t value)
{
        return dm_pci_write_config(dev, offset, value, PCI_SIZE_8);
}

int dm_pci_write_config16(struct udevice *dev, int offset, uint16_t value)
{
        return dm_pci_write_config(dev, offset, value, PCI_SIZE_16);
}

int dm_pci_write_config32(struct udevice *dev, int offset, uint32_t value)
{
        return dm_pci_write_config(dev, offset, value, PCI_SIZE_32);
}

unsigned long pci_conv_32_to_size(unsigned long value, unsigned int offset, enum pci_size_t size)
{
        switch (size) {
        case PCI_SIZE_8:
                return (value >> ((offset & 3) * 8)) & 0xff;
        case PCI_SIZE_16:
                return (value >> ((offset & 2) * 8)) & 0xffff;
        default:
                return value;
        }
}

unsigned long pci_conv_size_to_32(unsigned long old, unsigned long value, unsigned int offset,
                          enum pci_size_t size)
{
        unsigned int off_mask;
        unsigned int val_mask, shift;
        unsigned long ldata, mask;

        switch (size) {
        case PCI_SIZE_8:
                off_mask = 3;
                val_mask = 0xff;
                break;
        case PCI_SIZE_16:
                off_mask = 2;
                val_mask = 0xffff;
                break;
        default:
                return value;
        }
        shift = (offset & off_mask) * 8;
        ldata = (value & val_mask) << shift;
        mask = val_mask << shift;
        value = (old & ~mask) | ldata;

        return value;
}


#if 0
static int pci_find_and_bind_driver(struct udevice *parent,
				    struct pci_device_id *find_id,
				    pci_dev_t bdf, struct udevice **devp)
{
	struct pci_driver_entry *start, *entry;
	ofnode node = ofnode_null();
	const char *drv;
	int n_ents;
	int ret;
	char name[30], *str;
	bool bridge;

	*devp = NULL;

	printf("%s: Searching for driver: vendor=%x, device=%x\n", __func__,
	      find_id->vendor, find_id->device);

	/* Determine optional OF node */
	if (ofnode_valid(dev_ofnode(parent)))
		pci_dev_find_ofnode(parent, bdf, &node);

	if (ofnode_valid(node) && !ofnode_is_available(node)) {
		printf("%s: Ignoring disabled device\n", __func__);
		return log_msg_ret("dis", -EPERM);
	}

	start = ll_entry_start(struct pci_driver_entry, pci_driver_entry);
	n_ents = ll_entry_count(struct pci_driver_entry, pci_driver_entry);
	for (entry = start; entry != start + n_ents; entry++) {
		const struct pci_device_id *id;
		struct udevice *dev;
		const struct driver *drv;

		for (id = entry->match;
		     id->vendor || id->subvendor || id->class_mask;
		     id++) {
			if (!pci_match_one_id(id, find_id))
				continue;

			drv = entry->driver;

			/*
			 * In the pre-relocation phase, we only bind devices
			 * whose driver has the DM_FLAG_PRE_RELOC set, to save
			 * precious memory space as on some platforms as that
			 * space is pretty limited (ie: using Cache As RAM).
			 */
			if (!(gd->flags & GD_FLG_RELOC) &&
			    !(drv->flags & DM_FLAG_PRE_RELOC))
				return log_msg_ret("pre", -EPERM);

			/*
			 * We could pass the descriptor to the driver as
			 * plat (instead of NULL) and allow its bind()
			 * method to return -ENOENT if it doesn't support this
			 * device. That way we could continue the search to
			 * find another driver. For now this doesn't seem
			 * necesssary, so just bind the first match.
			 */
			ret = device_bind(parent, drv, drv->name, NULL, node,
					  &dev);
			if (ret)
				goto error;
			printf("%s: Match found: %s\n", __func__, drv->name);
			dev->driver_data = id->driver_data;
			*devp = dev;
			return 0;
		}
	}

	bridge = (find_id->class >> 8) == PCI_CLASS_BRIDGE_PCI;
	/*
	 * In the pre-relocation phase, we only bind bridge devices to save
	 * precious memory space as on some platforms as that space is pretty
	 * limited (ie: using Cache As RAM).
	 */
	if (!(gd->flags & GD_FLG_RELOC) && !bridge &&
	    !pci_need_device_pre_reloc(parent, find_id->vendor,
				       find_id->device))
		return log_msg_ret("notbr", -EPERM);

	/* Bind a generic driver so that the device can be used */
	sprintf(name, "pci_%x:%x.%x", dev_seq(parent), PCI_DEV(bdf),
		PCI_FUNC(bdf));
	str = strdup(name);
	if (!str)
		return -ENOMEM;
	drv = bridge ? "pci_bridge_drv" : "pci_generic_drv";

	ret = device_bind_driver_to_node(parent, drv, str, node, devp);
	if (ret) {
		printf("%s: Failed to bind generic driver: %d\n", __func__, ret);
		free(str);
		return ret;
	}
	printf("%s: No match found: bound generic driver instead\n", __func__);

	return 0;

error:
	printf("%s: No match found: error %d\n", __func__, ret);
	return ret;
}
#endif

static int _dm_pci_find_next_capability(struct udevice *dev, uint8_t pos, int cap)
{
        int ttl = PCI_FIND_CAP_TTL;
        uint8_t id;
        uint16_t ent;

        dm_pci_read_config8(dev, pos, &pos);

        while (ttl--) {
                if (pos < PCI_STD_HEADER_SIZEOF)
                        break;
                pos &= ~3;
                dm_pci_read_config16(dev, pos, &ent);

                id = ent & 0xff;
                if (id == 0xff)
                        break;
                if (id == cap)
                        return pos;
                pos = (ent >> 8);
        }

        return 0;
}

int dm_pci_find_capability(struct udevice *dev, int cap)
{
        uint16_t status;
        uint8_t header_type;
        uint8_t pos;

        dm_pci_read_config16(dev, PCI_STATUS, &status);
        if (!(status & PCI_STATUS_CAP_LIST))
                return 0;

        dm_pci_read_config8(dev, PCI_HEADER_TYPE, &header_type);
        if ((header_type & 0x7f) == PCI_HEADER_TYPE_CARDBUS)
                pos = PCI_CB_CAPABILITY_LIST;
        else
                pos = PCI_CAPABILITY_LIST;

        return _dm_pci_find_next_capability(dev, pos, cap);
}

int dm_pci_hose_probe_bus(struct udevice *bus)
{
        int sub_bus;
        int ret;
        int ea_pos;
        uint8_t reg;

        printf("%s\n", __func__);

        ea_pos = dm_pci_find_capability(bus, PCI_CAP_ID_EA);
        if (ea_pos) {
                dm_pci_read_config8(bus, ea_pos + sizeof(uint32_t) + sizeof(uint8_t),
                                    &reg);
                sub_bus = reg;
        } else {
                sub_bus = dev_seq(bus) + 1;
        }
        printf("%s: bus = %d\n", __func__, sub_bus);
        dm_pciauto_prescan_setup_bridge(bus, sub_bus);

#if 0
        ret = device_probe(bus);
        if (ret) {
                printf("%s: Cannot probe bus %s: %d\n", __func__, bus->name,
                      ret);
                return ret//log_msg_ret("probe", ret);
        }

        if (!ea_pos)
                sub_bus = dev_seq(bus) + 1;
#endif

        //

        return sub_bus;
}

int pci_bind_bus_devices(struct udevice *bus)
{
	unsigned long vendor, device;
	unsigned long header_type;
	pci_dev_t bdf, end;
	int found_multi;
	int ari_off;
	int ret;

	found_multi = 0;
	end = PCI_BDF(dev_seq(bus), PCI_MAX_PCI_DEVICES - 1,
		      PCI_MAX_PCI_FUNCTIONS - 1);
	for (bdf = PCI_BDF(dev_seq(bus), 0, 0); bdf <= end;
	     bdf += PCI_BDF(0, 0, 1)) {
		struct pci_child_plat *pplat;
		unsigned long class;

		if (!PCI_FUNC(bdf))
			found_multi = 0;
		if (PCI_FUNC(bdf) && !found_multi)
			continue;

		/* Check only the first access, we don't expect problems */
		ret = pci_bus_read_config(bus, bdf, PCI_VENDOR_ID, &vendor,
					  PCI_SIZE_16);
		if (ret)
			goto error;

		if (vendor == 0xffff || vendor == 0x0000)
			continue;

		pci_bus_read_config(bus, bdf, PCI_HEADER_TYPE,
				    &header_type, PCI_SIZE_8);

		if (!PCI_FUNC(bdf))
			found_multi = header_type & 0x80;

		printf("%s: bus %d: found device %x, function %d", __func__,
		      dev_seq(bus), PCI_DEV(bdf), PCI_FUNC(bdf));
		pci_bus_read_config(bus, bdf, PCI_DEVICE_ID, &device,
				    PCI_SIZE_16);
		pci_bus_read_config(bus, bdf, PCI_CLASS_REVISION, &class,
				    PCI_SIZE_32);
		class >>= 8;

		/* Find this device in the device tree */
		//ret = pci_bus_find_devfn(bus, PCI_MASK_BUS(bdf), &dev);
		//printf(": find ret=%d\n", ret);

		/* If nothing in the device tree, bind a device */
		//if (ret == -ENODEV) 
		{
			struct pci_device_id find_id;
			unsigned long val;

			memset(&find_id, '\0', sizeof(find_id));
			find_id.vendor = vendor;
			find_id.device = device;
			find_id.class = class;
			if ((header_type & 0x7f) == PCI_HEADER_TYPE_NORMAL) {
				pci_bus_read_config(bus, bdf,
						    PCI_SUBSYSTEM_VENDOR_ID,
						    &val, PCI_SIZE_32);
				find_id.subvendor = val & 0xffff;
				find_id.subdevice = val >> 16;
			}
			//ret = pci_find_and_bind_driver(bus, &find_id, bdf,
			//			       &dev);
		}
		//if (ret == -EPERM)
			//continue;
		//else if (ret)
			//return ret;

		/* Update the platform data */
		pplat = dev_get_parent_plat(bus);
		pplat->devfn = PCI_MASK_BUS(bdf);
		pplat->vendor = vendor;
		pplat->device = device;
		pplat->class = class;

#if 0
		if (IS_ENABLED(CONFIG_PCI_ARID)) {
			ari_off = dm_pci_find_ext_capability(dev,
							     PCI_EXT_CAP_ID_ARI);
			if (ari_off) {
				u16 ari_cap;

				/*
				 * Read Next Function number in ARI Cap
				 * Register
				 */
				dm_pci_read_config16(dev, ari_off + 4,
						     &ari_cap);
				/*
				 * Update next scan on this function number,
				 * subtract 1 in BDF to satisfy loop increment.
				 */
				if (ari_cap & 0xff00) {
					bdf = PCI_BDF(PCI_BUS(bdf),
						      PCI_DEV(ari_cap),
						      PCI_FUNC(ari_cap));
					bdf = bdf - 0x100;
				}
			}
		}

		board_pci_fixup_dev(bus, dev);
#endif
	}

	return 0;
error:
	printf("Cannot read bus configuration: %d\n", ret);

	return ret;
}

int pci_auto_config_devices(struct udevice *bus)
{
	struct pci_controller *hose = dev_get_uclass_priv(bus);
	struct pci_child_plat *pplat;
	//unsigned int sub_bus;
	//struct udevice *dev;
	int ret;

	//sub_bus = dev_seq(bus);
	printf("%s: start\n", __func__);
	//pciauto_config_init(hose);

	ret = dm_pciauto_config_device(bus);
	if (ret < 0)
		return ret;
#if 0
	for (ret = device_find_first_child(bus, &dev);
	     !ret && dev;
	     ret = device_find_next_child(&dev)) {
		unsigned int max_bus;
		int ret;

		printf("%s: device %s\n", __func__, dev->name);

		ret = dm_pciauto_config_device(dev);
		if (ret < 0)
			return log_msg_ret("auto", ret);
		max_bus = ret;
		sub_bus = max(sub_bus, max_bus);

		if (dev_get_parent(dev) == bus)
			continue;

		//pplat = dev_get_parent_plat(dev);
		//if (pplat->class == (PCI_CLASS_DISPLAY_VGA << 8))
			//set_vga_bridge_bits(dev);
	}
#endif

	printf("%s: done\n", __func__);

	return ret;
	//return log_msg_ret("sub", sub_bus);
}


int pci_probe(struct udevice *bus)
{
	struct pci_controller *hose = dev_get_uclass_priv(bus);
	int ret;

	ret = pci_bind_bus_devices(bus);
	//if (ret)
		//return log_msg_ret("bind", ret);
	if (!ret)
		ret = pci_auto_config_devices(bus);

	bus->msi_dev.dev = bus;
	pcie_init_capabilities(&bus->msi_dev);

	return ret;
}


