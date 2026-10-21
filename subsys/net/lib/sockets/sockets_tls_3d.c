/*
 * 3d local: the mbedtls error that ends a TLS read, at WRN.
 *
 * The IO IND remote-admin MQTT session ended every few minutes with
 * mqtt_input -EIO; sockets_tls.c maps every mbedtls read error to EIO and
 * logs the code only through NET_ERR in a module whose level is off on the
 * products (turning NET_LOG on raises every networking module at once).
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_sock_tls_3d, LOG_LEVEL_WRN);

void z_3d_tls_log_error(const char *where, int mbedtls_err)
{
	LOG_WRN("TLS %s error -0x%04x", where, (unsigned int)(-mbedtls_err));
}
