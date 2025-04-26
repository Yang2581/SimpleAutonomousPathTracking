#include <usbcan/UsbCan.h>

UsbCan::UsbCan() {
    if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) != 1) {
        throw std::runtime_error("open USBCAN device error");
    }
    VCI_INIT_CONFIG config;
    config.AccCode = 0;
    config.AccMask = 0xFFFFFFFF;
    config.Filter = 1;
    config.Timing0 = 0x00;
    config.Timing1 = 0x1C;
    config.Mode = 0;

    if (VCI_InitCAN(VCI_USBCAN2, 0, 0, &config) != 1) {
        VCI_CloseDevice(VCI_USBCAN2, 0);
        throw std::runtime_error("init CAN1 error");
    }

    if (VCI_StartCAN(VCI_USBCAN2, 0, 0) != 1) {
        VCI_CloseDevice(VCI_USBCAN2, 0);
        throw std::runtime_error("start CAN1 error");

    }
    if (VCI_ClearBuffer(VCI_USBCAN2, 0, 0) != 1) {
        VCI_CloseDevice(VCI_USBCAN2, 0);
        throw std::runtime_error("CAN1 clear buffer error");
    }
    if (VCI_InitCAN(VCI_USBCAN2, 0, 1, &config) != 1) {
        VCI_CloseDevice(VCI_USBCAN2, 0);
        throw std::runtime_error("init CAN2 error");
    }

    if (VCI_StartCAN(VCI_USBCAN2, 0, 1) != 1) {
        VCI_CloseDevice(VCI_USBCAN2, 0);
        throw std::runtime_error("start CAN2 error");

    }
    if (VCI_ClearBuffer(VCI_USBCAN2, 0, 1) != 1) {
        VCI_CloseDevice(VCI_USBCAN2, 0);
        throw std::runtime_error("CAN2 clear buffer error");

    }
}

UsbCan::~UsbCan() {
    VCI_CloseDevice(VCI_USBCAN2, 0);
}

std::optional<VCI_CAN_OBJ> UsbCan::receive(int channel) {
    VCI_CAN_OBJ ret;
    auto n = VCI_Receive(VCI_USBCAN2, 0, channel, &ret, 1, 0);
    if (n < 0) {
        throw std::runtime_error("USBCAN receive error");
    }
    if (0 == n) {
        return std::nullopt;
    }
    return ret;
}

void UsbCan::send(const VCI_CAN_OBJ &vciCanObj, int channel) {
    VCI_CAN_OBJ obj = vciCanObj;
    for (;;) {
        auto n = VCI_Transmit(VCI_USBCAN2, 0, channel, &obj, 1);
        if (n < 0) {
            throw std::runtime_error("USBCAN send error");
        }
        if (1 == n)
            break;

    }
}
