#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this); 
    serial = new QSerialPort(this);
    serial->setPortName("COM8");
    serial->setBaudRate(QSerialPort::Baud9600);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);
    connect(serial,&QSerialPort::readyRead,this,&MainWindow::readSerialData);

    if (!serial->open(QIODevice::ReadOnly))
    {
        qWarning() << "Serial open failed:" << serial->portName()
                   << serial->errorString();
    }
    else
    {
        qDebug() << "Serial connected:" << serial->portName();
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::readSerialData()
{
    rxBuffer += serial->readAll();

    const QByteArray startBytes("\xAA\x55", 2);
    constexpr int packetSize = 8;

    while (true)
    {
        const int startIndex = rxBuffer.indexOf(startBytes);

        if (startIndex < 0)
        {
            // 마지막 AA는 다음 수신의 55와 이어질 수 있으므로 보관
            if (rxBuffer.endsWith('\xAA'))
                rxBuffer = rxBuffer.right(1);
            else
                rxBuffer.clear();

            return;
        }

        // 시작 표시 앞의 불필요한 데이터 제거
        rxBuffer.remove(0, startIndex);

        // 나머지 바이트가 도착할 때까지 대기
        if (rxBuffer.size() < packetSize)
            return;

        const quint8 header = static_cast<quint8>(rxBuffer.at(2));

        // 헤더의 상위 6비트는 101000
        if ((header & 0xFC) != 0xA0)
        {
            rxBuffer.remove(0, 1);
            continue;
        }

        const QByteArray packet = rxBuffer.left(packetSize);
        rxBuffer.remove(0, packetSize);

        const bool ultrasonicOk = (header & 0x02) != 0;
        const bool dhtOk        = (header & 0x01) != 0;

        const int tempInt  = static_cast<quint8>(packet.at(3));
        const int tempFrac = static_cast<quint8>(packet.at(4));
        const int humiInt  = static_cast<quint8>(packet.at(5));
        const int humiFrac = static_cast<quint8>(packet.at(6));
        const int distance = static_cast<quint8>(packet.at(7));

        qDebug() << "RX:" << packet.toHex(' ');

        if (dhtOk)
        {
            // 소수부를 소수 첫째 자리 값으로 해석
            const double temp = tempInt + tempFrac / 10.0;
            const double humi = humiInt + humiFrac / 10.0;

            ui->lblTemp->setText(
                QString("%1 °C").arg(temp, 0, 'f', 1));

            ui->lblHumidity->setText(
                QString("%1 %").arg(humi, 0, 'f', 1));

            currentTemp     = temp;
            currentHumidity = humi;
        }
        else
        {
            ui->lblTemp->setText("온습도 오류");
            ui->lblHumidity->setText("온습도 오류");
        }

        if (ultrasonicOk)
        {
            qDebug() << "거리:" << distance << "cm";


            ui->lblDistance->setText(QString("%1 cm").arg(distance));
        }
        else
        {
            qDebug() << "초음파 오류";

            ui->lblDistance->setText("초음파 오류");
        }
    }
}
