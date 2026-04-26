#pragma once

#include <QObject>
#include <QByteArray>
#include <QTimer>

class SerialBridge;

// ── Packet layout (TX 32 bytes) ──────────────────────────────────
// [0-1]  = 0xAA 0x55 (header)
// [2]    = payload length (29)
// [3]    = control_mode
// [4]    = gimbal_mode
// [5]    = ptz_cmd
// [6-9]  = pitch_deg  (float32 LE)
// [10-13]= yaw_deg    (float32 LE)
// [14-17]= pitch_speed(float32 LE)
// [18-21]= yaw_speed  (float32 LE)
// [22]   = enable_flags (bit0=pitch, bit1=yaw)
// [23-26]= pitch_vel  (float32 LE)
// [27-30]= yaw_vel    (float32 LE)
// [31]   = CRC8
//
// ── Packet layout (RX 18 bytes) ─────────────────────────────────
// [0-1]  = 0xBB 0x66
// [2]    = payload length (14)
// [3]    = control_mode
// [4-7]  = yaw_deg   (float32 LE)
// [8-11] = pitch_deg (float32 LE)
// [12-15]= roll_deg  (float32 LE)
// [16]   = is_connected
// [17]   = CRC8

class GimbalController : public QObject
{
    Q_OBJECT

    Q_PROPERTY(int   gimbalMode  READ gimbalMode  WRITE setGimbalMode  NOTIFY gimbalModeChanged)
    Q_PROPERTY(int   controlMode READ controlMode WRITE setControlMode NOTIFY controlModeChanged)
    Q_PROPERTY(float pitchDeg    READ pitchDeg    WRITE setPitchDeg    NOTIFY pitchDegChanged)
    Q_PROPERTY(float yawDeg      READ yawDeg      WRITE setYawDeg      NOTIFY yawDegChanged)
    Q_PROPERTY(float pitchSpeed  READ pitchSpeed  WRITE setPitchSpeed  NOTIFY pitchSpeedChanged)
    Q_PROPERTY(float yawSpeed    READ yawSpeed    WRITE setYawSpeed    NOTIFY yawSpeedChanged)
    Q_PROPERTY(float pitchVel    READ pitchVel                         NOTIFY pitchVelChanged)
    Q_PROPERTY(float yawVel      READ yawVel                           NOTIFY yawVelChanged)
    Q_PROPERTY(bool  enablePitch READ enablePitch WRITE setEnablePitch NOTIFY enableChanged)
    Q_PROPERTY(bool  enableYaw   READ enableYaw   WRITE setEnableYaw   NOTIFY enableChanged)
    Q_PROPERTY(float fbPitch     READ fbPitch                          NOTIFY feedbackUpdated)
    Q_PROPERTY(float fbYaw       READ fbYaw                            NOTIFY feedbackUpdated)
    Q_PROPERTY(float fbRoll      READ fbRoll                           NOTIFY feedbackUpdated)
    Q_PROPERTY(bool  gimbalLinked READ gimbalLinked                    NOTIFY feedbackUpdated)

public:
    enum GimbalMode  { MODE_PTZ = 0, MODE_POSITION = 1, MODE_VELOCITY = 2 };
    enum ControlMode { MANUAL = 0, AUTO = 1 };
    enum PtzCmd { PTZ_STOP=0, PTZ_UP=1, PTZ_DOWN=2, PTZ_CENTER=3, PTZ_FOLLOW=4, PTZ_LOCK=5 };
    Q_ENUM(GimbalMode)
    Q_ENUM(ControlMode)
    Q_ENUM(PtzCmd)

    explicit GimbalController(SerialBridge *serial, QObject *parent = nullptr);

    int   gimbalMode()  const { return m_gimbalMode; }
    int   controlMode() const { return m_controlMode; }
    float pitchDeg()    const { return m_pitchDeg; }
    float yawDeg()      const { return m_yawDeg; }
    float pitchSpeed()  const { return m_pitchSpeed; }
    float yawSpeed()    const { return m_yawSpeed; }
    float pitchVel()    const { return m_pitchVel; }
    float yawVel()      const { return m_yawVel; }
    float fbPitch()     const { return m_fbPitch; }
    float fbYaw()       const { return m_fbYaw; }
    float fbRoll()      const { return m_fbRoll; }
    bool  gimbalLinked()const { return m_gimbalLinked; }

    bool enablePitch() const { return m_enablePitch; }
    bool enableYaw()   const { return m_enableYaw; }

    void setGimbalMode (int v);
    void setControlMode(int v);
    void setPitchDeg   (float v);
    void setYawDeg     (float v);
    void setPitchSpeed (float v);
    void setYawSpeed   (float v);
    void setEnablePitch(bool v);
    void setEnableYaw  (bool v);

public slots:
    void sendPtzCmd(int cmd);
    void sendPositionCmd();
    void resetToCenter();               // pitch=0, yaw=0 rồi gửi ngay
    void setJoystickInput(float x, float y);  // normalized -1..1
    void setFeedbackState(float yawDeg, float pitchDeg, float rollDeg, bool connected);

signals:
    void gimbalModeChanged();
    void controlModeChanged();
    void pitchDegChanged();
    void yawDegChanged();
    void pitchSpeedChanged();
    void yawSpeedChanged();
    void pitchVelChanged();
    void yawVelChanged();
    void enableChanged();
    void feedbackUpdated();

private slots:
    void onDataReceived(const QByteArray &data);
    void onVelocityTick();
    void onHeartbeatTick();

private:
    QByteArray buildPacket();
    static uint8_t crc8(const uint8_t *data, int len);

    SerialBridge *m_serial;
    QTimer        m_velTimer;
    QTimer        m_hbTimer;   // heartbeat 1 Hz
    QByteArray    m_rxBuf;

    int   m_gimbalMode  { MODE_PTZ };
    int   m_controlMode { MANUAL };
    int   m_ptzCmd      { PTZ_STOP };
    float m_pitchDeg    { 0.0f };
    float m_yawDeg      { 0.0f };
    float m_pitchSpeed  { 100.0f };
    float m_yawSpeed    { 100.0f };
    float m_pitchVel    { 0.0f };
    float m_yawVel      { 0.0f };
    bool  m_enablePitch { true };
    bool  m_enableYaw   { true };

    float m_fbPitch     { 0.0f };
    float m_fbYaw       { 0.0f };
    float m_fbRoll      { 0.0f };
    bool  m_gimbalLinked{ false };
};
