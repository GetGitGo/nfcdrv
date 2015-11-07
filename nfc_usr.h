#ifndef NFC_USR_H
#define NFC_USR_H

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */


#define NFC_IOC_CMD_RESET   0x11
#define NFC_IOC_CMD_READID  0x12
#define NFC_IOC_CMD_STATUS  0x13
#define NFC_IOC_WRITE       0x14

typedef struct nfc_ioc_write_s {
    char * data;
    unsigned int size;
    unsigned int addr_l;
    unsigned int addr_h;
}NFC_IOC_WRITE_S;


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

#endif	/* NFC_H */
