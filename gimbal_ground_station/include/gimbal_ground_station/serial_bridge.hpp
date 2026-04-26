#pragma once

#include <QObject>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QStringList>
#include <QByteArray>
#include <QTimer>

class SerialBridge : public QObject
{
    Q_OBJECT

    Q_PROPERTY(QString portName   READ portName   WRITE setPortName   NOTIFY portNameChanged)
    Q_PROPERTY(int     baudRate   READ baudRate   WRITE setBaudRate   NOTIFY baudRateChanged)
    Q_PROPERTY(bool    connected  READ isConnected                    NOTIFY connectionChanged)
    Q_PROPERTY(QStringList availablePorts READ availablePorts         NOTIFY availablePortsChanged)
    Q_PROPERTY(QString statusMessage     READ statusMessage           NOTIFY statusMessageChanged)

public:
    explicit SerialBridge(QObject *parent = nullptr);
    ~SerialBridge() override;

    QString    portName()      const { return m_portName; }
    int        baudRate()      const { return m_baudRate; }
    bool       isConnected()   const { return m_serial.isOpen(); }
    QStringList availablePorts() const;
    QString    statusMessage() const { return m_status; }

    void setPortName(const QString &name);
    void setBaudRate(int rate);

public slots:
    void connectPort();
    void disconnectPort();
    void refreshPorts();
    void sendBytes(const QByteArray &data);

signals:
    void portNameChanged();
    void baudRateChanged();
    void connectionChanged();
    void availablePortsChanged();
    void statusMessageChanged();
    void dataReceived(const QByteArray &data);
    void errorOccurred(const QString &error);

private slots:
    void onReadyRead();
    void onErrorOccurred(QSerialPort::SerialPortError error);

private:
    void setStatus(const QString &msg);

    QSerialPort m_serial;
    QString     m_portName;
    int         m_baudRate{115200};
    QString     m_status{"Chưa kết nối"};
};
