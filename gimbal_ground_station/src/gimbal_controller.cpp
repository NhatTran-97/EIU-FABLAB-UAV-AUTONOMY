#include "gimbal_ground_station/gimbal_controller.hpp"
#include "gimbal_ground_station/serial_bridge.hpp"

#include <QDebug>
#include <cstring>
#include <cmath>

static constexpr uint8_t TX_STX1 = 0xAA;
static constexpr uint8_t TX_STX2 = 0x55;
static constexpr uint8_t RX_STX1 = 0xBB;
static constexpr uint8_t RX_STX2 = 0x66;
static constexpr int TX_LEN = 32;
static constexpr int RX_LEN = 18;
static constexpr float VEL_TICK_MS = 50.0f;
static constexpr float POSITION_STEP_DPS = 60.0f;
static constexpr float MAX_JOYSTICK_VEL_DPS = 200.0f;
static constexpr float MIN_SPEED_DPS = 5.0f;
static constexpr float MAX_SPEED_DPS = 200.0f;

GimbalController::GimbalController(SerialBridge *serial, QObject *parent)
    : QObject(parent), m_serial(serial)
{
    connect(m_serial, &SerialBridge::dataReceived,
            this, &GimbalController::onDataReceived);

    m_velTimer.setInterval(static_cast<int>(VEL_TICK_MS));
    connect(&m_velTimer, &QTimer::timeout,
            this, &GimbalController::onVelocityTick);

    // Heartbeat: gửi trạng thái hiện tại 1Hz kể cả khi không có input
    m_hbTimer.setInterval(1000);
    connect(&m_hbTimer, &QTimer::timeout,
            this, &GimbalController::onHeartbeatTick);
    m_hbTimer.start();
}

// ── Setters ──────────────────────────────────────────────────────

void GimbalController::setGimbalMode(int v)
{
    if (m_gimbalMode == v) return;
    m_gimbalMode = v;
    emit gimbalModeChanged();

    m_ptzCmd = PTZ_STOP;  // reset khi đổi mode

    if (v == MODE_VELOCITY)
        m_velTimer.start();
    else {
        m_velTimer.stop();
        m_pitchVel = 0.0f;
        m_yawVel   = 0.0f;
    }
}

void GimbalController::setControlMode(int v)
{
    if (m_controlMode == v) return;
    m_controlMode = v;
    emit controlModeChanged();
}

void GimbalController::setPitchDeg(float v)
{
    v = qBound(-90.0f, v, 90.0f);
    if (qFuzzyCompare(m_pitchDeg, v)) return;
    m_pitchDeg = v;
    emit pitchDegChanged();
}

void GimbalController::setYawDeg(float v)
{
    v = qBound(-90.0f, v, 90.0f);
    if (qFuzzyCompare(m_yawDeg, v)) return;
    m_yawDeg = v;
    emit yawDegChanged();
}

void GimbalController::setPitchSpeed(float v)
{
    v = qBound(MIN_SPEED_DPS, v, MAX_SPEED_DPS);
    if (qFuzzyCompare(m_pitchSpeed, v)) return;
    m_pitchSpeed = v;
    emit pitchSpeedChanged();
}

void GimbalController::setYawSpeed(float v)
{
    v = qBound(MIN_SPEED_DPS, v, MAX_SPEED_DPS);
    if (qFuzzyCompare(m_yawSpeed, v)) return;
    m_yawSpeed = v;
    emit yawSpeedChanged();
}

void GimbalController::setEnablePitch(bool v)
{
    if (m_enablePitch == v) return;
    m_enablePitch = v;
    emit enableChanged();
}

void GimbalController::setEnableYaw(bool v)
{
    if (m_enableYaw == v) return;
    m_enableYaw = v;
    emit enableChanged();
}

// ── Commands ─────────────────────────────────────────────────────

void GimbalController::sendPtzCmd(int cmd)
{
    m_ptzCmd = cmd;
    m_serial->sendBytes(buildPacket());
}

void GimbalController::sendPositionCmd()
{
    m_serial->sendBytes(buildPacket());
}

void GimbalController::resetToCenter()
{
    m_pitchDeg = 0.0f;
    m_yawDeg   = 0.0f;
    emit pitchDegChanged();
    emit yawDegChanged();
    m_serial->sendBytes(buildPacket());
}

void GimbalController::setFeedbackState(float yawDeg, float pitchDeg, float rollDeg, bool connected)
{
    const bool changed =
        !qFuzzyCompare(m_fbYaw, yawDeg) ||
        !qFuzzyCompare(m_fbPitch, pitchDeg) ||
        !qFuzzyCompare(m_fbRoll, rollDeg) ||
        m_gimbalLinked != connected;

    if (!changed) return;

    m_fbYaw = yawDeg;
    m_fbPitch = pitchDeg;
    m_fbRoll = rollDeg;
    m_gimbalLinked = connected;
    emit feedbackUpdated();
}

void GimbalController::setJoystickInput(float x, float y)
{
    static constexpr float DEAD = 0.15f;
    static constexpr float POS_STEP_DEG = POSITION_STEP_DPS * (VEL_TICK_MS / 1000.0f);

    switch (m_gimbalMode) {

    case MODE_VELOCITY:
        m_pitchVel = (std::abs(y) > DEAD) ? -y * MAX_JOYSTICK_VEL_DPS : 0.0f;
        m_yawVel   = (std::abs(x) > DEAD) ?  x * MAX_JOYSTICK_VEL_DPS : 0.0f;
        emit pitchVelChanged();
        emit yawVelChanged();
        break;

    case MODE_POSITION: {
        if (std::abs(x) < DEAD && std::abs(y) < DEAD) break;
        setPitchDeg(m_pitchDeg - y * POS_STEP_DEG);
        setYawDeg  (m_yawDeg   + x * POS_STEP_DEG);
        sendPositionCmd();
        break;
    }

    case MODE_PTZ: {
        if (std::abs(x) < DEAD && std::abs(y) < DEAD) {
            sendPtzCmd(PTZ_STOP);
        } else if (std::abs(y) >= std::abs(x)) {
            sendPtzCmd(y < 0 ? PTZ_UP : PTZ_DOWN);
        }
        // Yaw PTZ (if gimbal supports it — extend ptz_cmd enum on firmware side)
        break;
    }
    }
}

void GimbalController::onHeartbeatTick()
{
    // Chỉ gửi khi UART đã kết nối
    if (!m_serial->isConnected()) return;
    m_serial->sendBytes(buildPacket());
}

void GimbalController::onVelocityTick()
{
    // Không gửi khi cả 2 velocity = 0 để tiết kiệm bandwidth LoRa
    if (m_pitchVel == 0.0f && m_yawVel == 0.0f) return;
    m_serial->sendBytes(buildPacket());
}

// ── Packet builder ────────────────────────────────────────────────

QByteArray GimbalController::buildPacket()
{
    QByteArray pkt(TX_LEN, 0x00);
    uint8_t *p = reinterpret_cast<uint8_t *>(pkt.data());

    p[0] = TX_STX1;
    p[1] = TX_STX2;
    p[2] = TX_LEN - 3;
    p[3] = static_cast<uint8_t>(m_controlMode);
    p[4] = static_cast<uint8_t>(m_gimbalMode);
    p[5] = static_cast<uint8_t>(m_ptzCmd);

    auto wf = [&](int off, float v) { memcpy(&p[off], &v, 4); };
    wf(6,  m_pitchDeg);
    wf(10, m_yawDeg);
    wf(14, m_pitchSpeed);
    wf(18, m_yawSpeed);
    p[22] = static_cast<uint8_t>((m_enablePitch ? 0x01 : 0x00) |
                                  (m_enableYaw   ? 0x02 : 0x00));
    wf(23, m_pitchVel);
    wf(27, m_yawVel);

    p[TX_LEN - 1] = crc8(p + 2, TX_LEN - 3);
    return pkt;
}

// ── RX parser ─────────────────────────────────────────────────────

void GimbalController::onDataReceived(const QByteArray &data)
{
    m_rxBuf.append(data);

    while (m_rxBuf.size() >= RX_LEN) {
        // Tìm header
        int idx = -1;
        for (int i = 0; i <= m_rxBuf.size() - 2; ++i) {
            if ((uint8_t)m_rxBuf[i]   == RX_STX1 &&
                (uint8_t)m_rxBuf[i+1] == RX_STX2) {
                idx = i; break;
            }
        }
        if (idx < 0) { m_rxBuf.clear(); break; }
        if (idx > 0)  m_rxBuf = m_rxBuf.mid(idx);

        // Chưa đủ bytes cho một packet hoàn chỉnh → đợi thêm dữ liệu
        if (m_rxBuf.size() < RX_LEN) break;

        const uint8_t *p = reinterpret_cast<const uint8_t *>(m_rxBuf.constData());
        if (crc8(p + 2, RX_LEN - 3) != p[RX_LEN - 1]) {
            // CRC sai → bỏ byte đầu, tìm lại header
            m_rxBuf = m_rxBuf.mid(1); continue;
        }

        float yaw = 0.0f;
        float pitch = 0.0f;
        float roll = 0.0f;
        memcpy(&yaw,   p + 4,  4);
        memcpy(&pitch, p + 8,  4);
        memcpy(&roll,  p + 12, 4);
        setFeedbackState(yaw, pitch, roll, p[16] != 0);
        m_rxBuf = m_rxBuf.mid(RX_LEN);
    }
}

// ── CRC-8 (polynomial 0x07) ───────────────────────────────────────

uint8_t GimbalController::crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0x00;
    for (int i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j)
            crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1);
    }
    return crc;
}
