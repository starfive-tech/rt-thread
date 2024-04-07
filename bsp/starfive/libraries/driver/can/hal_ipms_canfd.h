
unsigned int canfd_interrupt(void *ipms_priv);
int ipms_canfd_init(struct ipms_canfd *ipms, int ctrl_mode);
int canfd_driver_start_xmit(const void *data, void *ipms_priv, int canfd);
int canfd_rx_poll(void *priv, void *data);

