#include "gimbal_ground_station/serial_bridge.hpp"
#include <QDebug>

SerialBridge::SerialBridge(QObject *parent)
    : QObject(parent)
{
    connect(&m_serial, &QSerialPort::readyRead,
            this, &SerialBridge::onReadyRead);
    connect(&m_serial, &QSerialPort::errorOccurred,
            this, &SerialBridge::onErrorOccurred);
}

SerialBridge::~SerialBridge()
{
    if (m_serial.isOpen())
        m_serial.close();
}

QStringList SerialBridge::availablePorts() const
{
    QStringList list;
    for (const auto &info : QSerialPortInfo::availablePorts())
        list << info.systemLocation();  // /dev/ttyUSB0 thay vì ttyUSB0
    return list;
}

void SerialBridge::setPortName(const QString &name)
{
    if (m_portName == name) return;
    m_portName = name;
    emit portNameChanged();
}

void SerialBridge::setBaudRate(int rate)
{
    if (m_baudRate == rate) return;
    m_baudRate = rate;
    emit baudRateChanged();
    if (m_serial.isOpen())
        m_serial.setBaudRate(m_baudRate);
}

void SerialBridge::connectPort()
{
    if (m_serial.isOpen()) {
        setStatus("Đã kết nối rồi");
        return;
    }
    if (m_portName.isEmpty()) {
        setStatus("Chưa chọn cổng COM");
        emit errorOccurred("Chưa chọn cổng COM");
        return;
    }

    m_serial.setPortName(m_portName);
    m_serial.setBaudRate(m_baudRate);
    m_serial.setDataBits(QSerialPort::Data8);
    m_serial.setParity(QSerialPort::NoParity);
    m_serial.setStopBits(QSerialPort::OneStop);
    m_serial.setFlowControl(QSerialPort::NoFlowControl);

    if (m_serial.open(QIODevice::ReadWrite)) {
        setStatus(QString("Đã kết nối: %1 @ %2 baud").arg(m_portName).arg(m_baudRate));
        emit connectionChanged();
    } else {
        setStatus("Lỗi: " + m_serial.errorString());
        emit errorOccurred(m_serial.errorString());
    }
}

void SerialBridge::disconnectPort()
{
    if (m_serial.isOpen()) {
        m_serial.close();
        setStatus("Đã ngắt kết nối");
        emit connectionChanged();
    }
}

void SerialBridge::refreshPorts()
{
    emit availablePortsChanged();
}

void SerialBridge::sendBytes(const QByteArray &data)
{
    if (!m_serial.isOpen()) {
        emit errorOccurred("Chưa kết nối UART");
        return;
    }
    qint64 written = m_serial.write(data);
    if (written < 0)
        emit errorOccurred("Ghi dữ liệu thất bại: " + m_serial.errorString());
}

void SerialBridge::onReadyRead()
{
    QByteArray data = m_serial.readAll();
    if (!data.isEmpty())
        emit dataReceived(data);
}

void SerialBridge::onErrorOccurred(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::NoError) return;

    QString msg = m_serial.errorString();
    setStatus("Lỗi UART: " + msg);
    emit errorOccurred(msg);

    if (error == QSerialPort::ResourceError) {
        m_serial.close();
        emit connectionChanged();
    }
}

void SerialBridge::setStatus(const QString &msg)
{
    if (m_status == msg) return;
    m_status = msg;
    emit statusMessageChanged();
    qDebug() << "[SerialBridge]" << msg;
}
