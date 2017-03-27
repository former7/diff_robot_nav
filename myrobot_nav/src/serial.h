#ifndef SERIAL_H_INCLUDED
#define SERIAL_H_INCLUDED

#define DEVICE "/dev/ttyUSB1"

extern int init_serial(void);
extern int uart_send(char *data, int datalen);
extern int uart_recv(char *data, int datalen);

#endif // SERIAL_H_INCLUDED

/************************End********************************/
