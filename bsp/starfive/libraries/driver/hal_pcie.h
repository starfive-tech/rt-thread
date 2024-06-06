
#ifndef HAL_PCIE_H__
#define HAL_PCIE_H__

struct pcie;

struct pcie_cfg {
	void *bridge_base;
	void *cfg_base;
	unsigned long mem32_base;
	unsigned long mem32_log;
	unsigned long mem64_base;
	unsigned long mem64_log;
	int reset_gpio;
	int (*pcie_post_init_cb)(struct pcie *);
};

struct pcie {
    struct pcie_cfg cfg;
    char *name;
    int index;
    int irq;
    int link_up:1;
    int (*irq_handle)(void *priv);
    void *priv;
};

int pcie_init(struct pcie *pcie);
void pcie_plat_init(struct pcie *pcie);

#endif
