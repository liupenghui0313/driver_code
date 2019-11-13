#ifndef SPI_HOST_H_
#define SPI_HOST_H_

#define SPI_IF_STD			(0x01)
#define SPI_IF_DUAL		(0x02)
#define SPI_IF_QUAD		(0x04)

// struct spi_device {
//     unsigned int msecs;
//     unsigned int csnums;
//     unsigned int iftype;
//     int (*select_bus)(struct spi_device *spi, unsigned int cs);
//     int (*transmit)(struct spi_device *spi, const void *cmd, size_t len, void *buf, size_t send, size_t recv);
//     int (*set_clock)(struct spi_device *spi, unsigned int Hz);
//     int (*set_mode)(struct spi_device *spi, int spo, int sph);
//     int (*wait_ready)(struct spi_device *spi, int msecs);
//     int (*entry_4addr)(struct spi_device *spi, int enable);
//     int (*qe_enable)(struct spi_device *spi);
// };

int spi_host_init(unsigned int msecs);
void spi_host_deinit(struct spi_device *spi);

int spi_host_register(struct spi_device *spi);

#endif /* SPI_HOST_H_ */
