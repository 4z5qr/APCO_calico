#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QDir>
#include <QLineEdit>
#include <QImage>
#include <QStatusBar>
#include <QTextStream>
#include <QFileInfo>
#include <QSysInfo>
#include <QUuid>
#include <chrono>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QMessageBox>
#include <QPainter>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QThread>
#include <QVBoxLayout>
#include <QWidget>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <fftw3.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <complex>
#include <memory>
#include <vector>

struct Config {
  double startMHz=865, stopMHz=870, stepMHz=4, rateMHz=5, gain=35, thresholdDb=9;
  int dwellMs=1000, scanSeconds=600, captureSeconds=3;
  QString device="driver=lime", directory="captures";
  int trigger=0; // 0 SNR, 1 C4FM candidate, 2 CQPSK candidate
  QString resultsDirectory="results";
};
Q_DECLARE_METATYPE(Config)

struct Snapshot {
  double centerMHz=0, rateMHz=0, peakMHz=0, peakDb=0;
  QString modulation;
  QVector<float> spectrum;
  bool triggered=false;
};
Q_DECLARE_METATYPE(Snapshot)

class Scanner : public QThread {
  Q_OBJECT
public:
  Config cfg;
  std::atomic_bool stopping{false};
signals:
  void snapshot(Snapshot);
  void log(QString);
protected:
  void run() override {
    const QString runId=QDateTime::currentDateTimeUtc().toString("yyyyMMdd_HHmmss_zzz")+"_"+
      QUuid::createUuid().toString(QUuid::WithoutBraces).left(8);
    const QString runDir=cfg.resultsDirectory+"/"+runId;
    QDir().mkpath(runDir+"/captures");
    QFile eventsFile(runDir+"/events.csv"), observationsFile(runDir+"/observations.csv"),
          candidatesFile(runDir+"/candidates.csv"), capturesFile(runDir+"/captures.csv"),
          configFile(runDir+"/config.csv"), sessionFile(runDir+"/session.csv");
    const auto openCsv=[](QFile &f,const QString &header){
      if(!f.open(QIODevice::WriteOnly|QIODevice::Text)) throw std::runtime_error(("Cannot open "+f.fileName()).toStdString());
      f.write(header.toUtf8()); f.write("\n"); f.flush();
    };
    auto writeRow=[](QFile &f,const QStringList &fields){
      QStringList encoded;
      for(QString field:fields) {
        field.replace('"', "\"\"");
        if(field.contains(',') || field.contains('"') || field.contains('\n') || field.contains('\r'))
          field="\"" + field + "\"";
        encoded << field;
      }
      f.write(encoded.join(',').toUtf8()); f.write("\n"); f.flush();
    };
    const QString startedUtc=QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
    QString outcome="error";
    int observationsCount=0, capturesCount=0, triggerCount=0;
    try {
      openCsv(eventsFile,"timestamp_utc,event,frequency_hz,details");
      openCsv(observationsFile,"timestamp_utc,center_frequency_hz,sample_rate_hz,dwell_ms,fft_frames,peak_frequency_hz,peak_excess_db,noise_floor_relative_db,triggered,modulation,coverage");
      openCsv(candidatesFile,"timestamp_utc,frequency_hz,peak_excess_db,trigger,classification");
      openCsv(capturesFile,"timestamp_utc,path,center_frequency_hz,sample_rate_hz,format,samples,bytes,trigger,peak_excess_db");
      openCsv(configFile,"parameter,value");
      openCsv(sessionFile,"run_id,started_utc,ended_utc,status,observations,captures,triggers,device");
      for(const auto &pair:QVector<QPair<QString,QString>>{
        {"start_mhz",QString::number(cfg.startMHz,'f',6)},
        {"stop_mhz",QString::number(cfg.stopMHz,'f',6)},
        {"step_mhz",QString::number(cfg.stepMHz,'f',6)},
        {"sample_rate_msps",QString::number(cfg.rateMHz,'f',6)},
        {"gain_db",QString::number(cfg.gain)},
        {"snr_proxy_threshold_db",QString::number(cfg.thresholdDb)},
        {"dwell_ms",QString::number(cfg.dwellMs)},
        {"scan_seconds",QString::number(cfg.scanSeconds)},
        {"capture_seconds",QString::number(cfg.captureSeconds)},
        {"trigger_mode",QString::number(cfg.trigger)},
        {"device",cfg.device},
        {"iq_format","CF32 interleaved little-endian host"},
        {"fft_size","8192"},
        {"results_directory",runDir}
      }) writeRow(configFile,{pair.first,pair.second});
      writeRow(eventsFile,{startedUtc,"session_start","",runId});
      emit log("Results: "+runDir);
    } catch(const std::exception &e) {
      emit log(QString("Logging setup ERROR: ")+e.what());
      return;
    }
    SoapySDR::Device *dev=nullptr;
    SoapySDR::Stream *stream=nullptr;
    fftwf_complex *in=nullptr,*out=nullptr;
    fftwf_plan plan=nullptr;
    constexpr int N=8192;
    try {
      dev=SoapySDR::Device::make(cfg.device.toStdString());
      if(!dev) throw std::runtime_error("SDR not found");
      const double rate=cfg.rateMHz*1e6;
      dev->setSampleRate(SOAPY_SDR_RX,0,rate);
      dev->setBandwidth(SOAPY_SDR_RX,0,rate);
      dev->setGain(SOAPY_SDR_RX,0,cfg.gain);
      stream=dev->setupStream(SOAPY_SDR_RX,SOAPY_SDR_CF32);
      in=fftwf_alloc_complex(N); out=fftwf_alloc_complex(N);
      if(!in||!out) throw std::runtime_error("FFTW allocation failed");
      plan=fftwf_plan_dft_1d(N,in,out,FFTW_FORWARD,FFTW_ESTIMATE);
      dev->activateStream(stream);
      std::vector<std::complex<float>> iq(N);
      const auto started=std::chrono::steady_clock::now();
      int scanIndex=0;
      const int steps=std::max(1,1+int(std::ceil((cfg.stopMHz-cfg.startMHz-cfg.rateMHz)/cfg.stepMHz)));
      QDir().mkpath(cfg.directory);
      while(!stopping) {
        if(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now()-started).count()>=cfg.scanSeconds) break;
        const double center=std::min(cfg.stopMHz-cfg.rateMHz*.5,
          cfg.startMHz+cfg.rateMHz*.5+(scanIndex%steps)*cfg.stepMHz);
        ++scanIndex;
        dev->setFrequency(SOAPY_SDR_RX,0,center*1e6);
        std::vector<double> powers(N,0);
        int frames=0;
        std::vector<std::complex<float>> last;
        const auto dwellStart=std::chrono::steady_clock::now();
        while(!stopping && std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now()-dwellStart).count()<cfg.dwellMs) {
          int collected=0;
          while(collected<N && !stopping) {
            void *buff[]={iq.data()+collected};
            int flags=0; long long ns=0;
            int n=dev->readStream(stream,buff,N-collected,flags,ns,100000);
            if(n<0) throw std::runtime_error("SDR read failed: "+std::to_string(n));
            collected+=n;
          }
          if(collected!=N) break;
          last=iq;
          for(int i=0;i<N;i++) {
            float w=.5f-.5f*std::cos(2*M_PI*i/(N-1));
            in[i][0]=iq[i].real()*w; in[i][1]=iq[i].imag()*w;
          }
          fftwf_execute(plan);
          for(int k=0;k<N;k++){
            int j=(k+N/2)%N;
            powers[j]+=double(out[k][0])*out[k][0]+double(out[k][1])*out[k][1];
          }
          ++frames;
        }
        if(!frames || stopping) continue;
        Snapshot snap; snap.centerMHz=center; snap.rateMHz=cfg.rateMHz;
        snap.spectrum.resize(N);
        for(int k=0;k<N;k++) snap.spectrum[k]=10*std::log10(powers[k]/frames/(N*N)+1e-20);
        // Global robust median, used as a provisional noise estimate.
        std::vector<float> sorted(snap.spectrum.begin(),snap.spectrum.end());
        std::nth_element(sorted.begin(),sorted.begin()+N/2,sorted.end());
        const double noise=sorted[N/2];
        int peak=-1;
        for(int k=128;k<N-128;k++){
          if(std::abs(k-N/2)<16) continue;
          if(peak<0 || snap.spectrum[k]>snap.spectrum[peak]) peak=k;
        }
        snap.peakMHz=center+(peak-N/2)*cfg.rateMHz/N;
        snap.peakDb=snap.spectrum[peak]-noise;
        // Modulation classification is NOT implemented: FFT peaks and raw IQ
        // cannot establish C4FM/CQPSK without channelization and symbol recovery.
        snap.modulation="unknown";
        snap.triggered=cfg.trigger==0 && snap.peakDb>=cfg.thresholdDb;
        const QString observedUtc=QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
        ++observationsCount;
        if(snap.triggered) {
          ++triggerCount;
          writeRow(candidatesFile,{observedUtc,QString::number(snap.peakMHz*1e6,'f',0),
            QString::number(snap.peakDb,'f',3),"snr_proxy","unknown"});
        }
        writeRow(observationsFile,{observedUtc,QString::number(center*1e6,'f',0),
          QString::number(rate,'f',0),QString::number(cfg.dwellMs),
          QString::number(frames),QString::number(snap.peakMHz*1e6,'f',0),
          QString::number(snap.peakDb,'f',3),QString::number(noise,'f',3),
          snap.triggered?"1":"0",snap.modulation,"observed"});
        if(snap.triggered && !last.empty()) {
          const QString name=runDir+"/captures/iq_"+QDateTime::currentDateTimeUtc().toString("yyyyMMdd_HHmmss_zzz")+
            "_"+QString::number(center,'f',4)+"MHz.cf32";
          QFile file(name);
          if(file.open(QIODevice::WriteOnly)) {
            // Capture from the current tuned center; duration configurable.
            emit log("Trigger SNR "+QString::number(snap.peakDb,'f',1)+" dB: "+name);
            writeRow(eventsFile,{observedUtc,"capture_started",QString::number(center*1e6,'f',0),name});
            qint64 samplesWritten=0;
            const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(cfg.captureSeconds);
            while(!stopping && std::chrono::steady_clock::now()<until) {
              int flags=0; long long ns=0;
              void *buff[]={iq.data()};
              int n=dev->readStream(stream,buff,N,flags,ns,100000);
              if(n>0) {
                qint64 bytes=file.write(reinterpret_cast<const char*>(iq.data()),n*sizeof(iq[0]));
                if(bytes<0 || bytes!=qint64(n*sizeof(iq[0]))) {
                  writeRow(eventsFile,{QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs),
                    "capture_write_error",QString::number(center*1e6,'f',0),file.errorString()});
                  break;
                }
                samplesWritten+=n;
              }
              if(n<0) {
                writeRow(eventsFile,{QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs),
                  "capture_read_error",QString::number(center*1e6,'f',0),QString::number(n)});
                break;
              }
            }
            file.flush();file.close();
            ++capturesCount;
            writeRow(capturesFile,{observedUtc,name,QString::number(center*1e6,'f',0),
              QString::number(rate,'f',0),"cf32",QString::number(samplesWritten),
              QString::number(samplesWritten*qint64(sizeof(iq[0]))),"snr_proxy",
              QString::number(snap.peakDb,'f',3)});
            writeRow(eventsFile,{QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs),
              "capture_finished",QString::number(center*1e6,'f',0),QString::number(samplesWritten)});
          } else {
            emit log("Cannot write "+name);
            writeRow(eventsFile,{observedUtc,"capture_open_error",QString::number(center*1e6,'f',0),file.errorString()});
          }
        }
        emit snapshot(snap);
      }
      outcome=stopping?"stopped":"completed";
    } catch(const std::exception &e) {
      emit log(QString("ERROR: ")+e.what());
      writeRow(eventsFile,{QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs),
        "error","",QString::fromUtf8(e.what())});
    }
    if(dev && stream){try{dev->deactivateStream(stream);}catch(...){} dev->closeStream(stream);}
    if(plan) fftwf_destroy_plan(plan);
    if(in) fftwf_free(in); if(out) fftwf_free(out);
    if(dev) SoapySDR::Device::unmake(dev);
    writeRow(eventsFile,{QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs),
      "session_end","",outcome});
    writeRow(sessionFile,{runId,startedUtc,QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs),
      outcome,QString::number(observationsCount),QString::number(capturesCount),
      QString::number(triggerCount),cfg.device});
    emit log("Scanner stopped; CSV: "+runDir);
  }
};

class Spectrum : public QWidget {
public:
  bool overview=false;
  double start=865,stop=870;
  QVector<float> current;
  QImage waterfall;
  double center=867.5,rate=5;
  explicit Spectrum(bool isOverview,QWidget *parent=nullptr):QWidget(parent),overview(isOverview) {
    setMinimumHeight(isOverview?130:240);
    waterfall=QImage(1024,180,QImage::Format_RGB32);
    waterfall.fill(QColor(10,18,38));
  }
  static QColor heat(float db){
    float t=std::clamp((db+95)/70.f,0.f,1.f);
    return QColor(int(15+240*t*t),int(20+180*t),int(55+100*(1-t)));
  }
  void accept(const Snapshot &s){
    center=s.centerMHz;rate=s.rateMHz;current=s.spectrum;
    if(!overview && !current.isEmpty()){
      { QImage old=waterfall.copy(); QPainter painter(&waterfall); painter.drawImage(0,1,old,0,0,old.width(),old.height()-1); }
      for(int x=0;x<waterfall.width();x++){
        int i=x*current.size()/waterfall.width();
        waterfall.setPixelColor(x,0,heat(current[i]));
      }
    }
    update();
  }
protected:
  void paintEvent(QPaintEvent*) override {
    QPainter p(this);
    p.fillRect(rect(),QColor(10,18,38));
    const QRect area=rect().adjusted(10,10,-10,-25);
    if(overview){
      if(!current.isEmpty()){
        QPolygonF poly;
        for(int x=0;x<area.width();x++){
          const double f=start+(stop-start)*x/area.width();
          if(f<center-rate*.5 || f>center+rate*.5) continue;
          int i=int((f-(center-rate*.5))/rate*current.size());
          i=std::clamp(i,0,int(current.size()-1));
          const double v=std::clamp((double(current[i])+110)/100.,0.,1.);
          poly<<QPointF(area.left()+x,area.bottom()-v*area.height());
        }
        p.setPen(QPen(QColor(80,220,180),1));p.drawPolyline(poly);
      }
      int x0=area.left()+int((center-rate*.5-start)/(stop-start)*area.width());
      int x1=area.left()+int((center+rate*.5-start)/(stop-start)*area.width());
      p.fillRect(QRect(x0,area.top(),x1-x0,area.height()),QColor(30,140,220,35));
      p.setPen(QColor(50,180,255));p.drawRect(QRect(x0,area.top(),x1-x0,area.height()));
    } else p.drawImage(area,waterfall);
    p.setPen(Qt::white);
    p.drawText(10,height()-6,QString::number(overview?start:center-rate*.5,'f',3)+" MHz");
    p.drawText(width()-105,height()-6,QString::number(overview?stop:center+rate*.5,'f',3)+" MHz");
  }
};

class Window : public QMainWindow {
  Scanner *worker=nullptr;
  Spectrum *top,*bottom;
  QDoubleSpinBox *start,*stop,*step,*rate,*gain,*snr;
  QSpinBox *dwell,*duration,*capture;
  QComboBox *mode;
  QPlainTextEdit *log;
  QPushButton *run,*halt;
  QLineEdit *device,*directory;
  static QDoubleSpinBox *number(double lo,double hi,double val,int decimals=3){
    auto *s=new QDoubleSpinBox;s->setRange(lo,hi);s->setDecimals(decimals);s->setValue(val);return s;
  }
public:
  Window(){
    setWindowTitle("APCO_calico — Scanner");resize(1050,850);
    auto *central=new QWidget;setCentralWidget(central);
    auto *layout=new QVBoxLayout(central);
    auto *settings=new QGroupBox("Scan & capture");auto *form=new QFormLayout(settings);
    start=number(1,6000,865);stop=number(1,6000,870);
    step=number(.01,100,4);rate=number(.1,60,5);
    gain=number(0,73,35,1);snr=number(0,60,9,1);
    dwell=new QSpinBox;dwell->setRange(100,60000);dwell->setValue(1000);
    duration=new QSpinBox;duration->setRange(1,86400);duration->setValue(600);
    capture=new QSpinBox;capture->setRange(1,3600);capture->setValue(3);
    mode=new QComboBox;
    mode->addItem("SNR threshold — operational");
    mode->addItem("C4FM — not implemented");
    mode->addItem("CQPSK — not implemented");
    device=new QLineEdit("driver=lime");directory=new QLineEdit("results");
    form->addRow("Start frequency (MHz)",start);form->addRow("Stop frequency (MHz)",stop);
    form->addRow("Step (MHz)",step);form->addRow("Sample rate (MS/s)",rate);
    form->addRow("RF gain (dB)",gain);form->addRow("Trigger",mode);
    form->addRow("SNR proxy threshold (dB)",snr);
    form->addRow("Dwell per tuning (ms)",dwell);
    form->addRow("Total scan time (s)",duration);
    form->addRow("IQ capture duration (s)",capture);
    form->addRow("SoapySDR device",device);form->addRow("Results folder (CSV + IQ)",directory);
    layout->addWidget(settings);
    auto *buttons=new QHBoxLayout;
    run=new QPushButton("Start scan");halt=new QPushButton("Stop");halt->setEnabled(false);
    buttons->addWidget(run);buttons->addWidget(halt);layout->addLayout(buttons);
    layout->addWidget(new QLabel("Full-range overview (current dwell highlighted)"));
    top=new Spectrum(true);layout->addWidget(top);
    layout->addWidget(new QLabel("Live waterfall — currently tuned passband"));
    bottom=new Spectrum(false);layout->addWidget(bottom);
    log=new QPlainTextEdit;log->setReadOnly(true);log->setMaximumBlockCount(300);
    log->setMaximumHeight(115);layout->addWidget(log);
    connect(mode,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this](int i){
      if(i>0) log->appendPlainText("Modulation trigger requires a future DSP classifier; selection will not trigger IQ capture.");
    });
    connect(run,&QPushButton::clicked,this,[this]{
      if(worker) return;
      if(start->value()>=stop->value() || step->value()>rate->value()*.9 || rate->value()>stop->value()-start->value()) {
        QMessageBox::warning(this,"Invalid scan","Start must be below stop, scan span >= sample rate, and step <= 90% of sample rate.");return;
      }
      Config c;
      c.startMHz=start->value();c.stopMHz=stop->value();c.stepMHz=step->value();
      c.rateMHz=rate->value();c.gain=gain->value();c.thresholdDb=snr->value();
      c.dwellMs=dwell->value();c.scanSeconds=duration->value();c.captureSeconds=capture->value();
      c.trigger=mode->currentIndex();c.device=device->text();c.resultsDirectory=directory->text();
      top->start=c.startMHz;top->stop=c.stopMHz;
      worker=new Scanner;worker->cfg=c;
      connect(worker,&Scanner::snapshot,this,[this](const Snapshot &s){
        top->accept(s);bottom->accept(s);
        statusBar()->showMessage(QString("%1 MHz | peak %2 MHz | excess %3 dB | %4")
          .arg(s.centerMHz,0,'f',4).arg(s.peakMHz,0,'f',4).arg(s.peakDb,0,'f',1).arg(s.modulation));
      });
      connect(worker,&Scanner::log,log,&QPlainTextEdit::appendPlainText);
      connect(worker,&QThread::finished,this,[this]{
        worker->deleteLater();worker=nullptr;run->setEnabled(true);halt->setEnabled(false);
      });
      run->setEnabled(false);halt->setEnabled(true);worker->start();
    });
    connect(halt,&QPushButton::clicked,this,[this]{if(worker) worker->stopping=true;});
  }
  ~Window() override {if(worker){worker->stopping=true;worker->wait();}}
};
int main(int argc,char **argv){
  QApplication app(argc,argv);
  qRegisterMetaType<Snapshot>("Snapshot");
  Window w;w.show();return app.exec();
}
#include "gui.moc"
