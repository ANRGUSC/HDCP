CONTIKI = /home/pradipta/contiki
PROJECT_SOURCEFILES = bcp.c packetstack.c routingtable.c

#Load our own project-conf to employ nullrdc driver
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
DEFINES = RF_CHANNEL=26, NETSTACK_CONF_MAC=csma_driver, NETSTACK_CONF_RDC=nullrdc_driver, TIMESYNCH_CONF_ENABLED=1, QUEUEBUF_CONF_NUM=30

#Turn off the radio's hardware autoack and address recognition (for snooping)
#CFLAGS +="-DCC2420_CONF_AUTOACK=0"

#Turn off nullrdc's address filtering (for snooping)
#CFLAGS +="-DNULLRDC_CONF_ADDRESS_FILTER=0"

all: main  

include $(CONTIKI)/Makefile.include
