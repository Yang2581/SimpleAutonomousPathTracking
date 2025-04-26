#ifndef USBCAN_H
#define USBCAN_H

#include <usbcan/controlcan.h>
#include <optional>
class UsbCan {
public:
    UsbCan();
    virtual ~UsbCan();
    UsbCan(const UsbCan & other) = delete;
    UsbCan(UsbCan && other) = delete;
    UsbCan & operator=(const UsbCan & other) = delete;

public:
    std::optional<VCI_CAN_OBJ> receive(int channel);
    void send(const VCI_CAN_OBJ & vciCanObj, int channel);
};


#endif //USBCAN_H
