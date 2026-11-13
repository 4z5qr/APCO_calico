#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <fftw3.h>
#include <algorithm>
#include <cmath>
#include <complex>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc, char** argv) {
  const double center = argc > 1 ? std::stod(argv[1])*1e6 : 867.5e6;
  const double rate = argc > 2 ? std::stod(argv[2])*1e6 : 5e6;
  const std::string deviceArgs = argc > 3 ? argv[3] : "driver=lime";
  constexpr int N = 8192;
  constexpr int frames = 120;
  SoapySDR::Device* dev = nullptr;
  SoapySDR::Stream* stream = nullptr;
  fftwf_complex* in = nullptr;
  fftwf_complex* out = nullptr;
  fftwf_plan plan = nullptr;
  try {
    dev = SoapySDR::Device::make(deviceArgs);
    if (!dev) throw std::runtime_error("No SDR device");
    dev->setSampleRate(SOAPY_SDR_RX, 0, rate);
    dev->setFrequency(SOAPY_SDR_RX, 0, center);
    dev->setBandwidth(SOAPY_SDR_RX, 0, rate);
    try { dev->setAntenna(SOAPY_SDR_RX, 0, "LNAL"); } catch (...) {}
    dev->setGain(SOAPY_SDR_RX, 0, 35);
    stream = dev->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32);
    in = fftwf_alloc_complex(N);
    out = fftwf_alloc_complex(N);
    if (!in || !out) throw std::runtime_error("FFTW allocation failed");
    plan = fftwf_plan_dft_1d(N, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
    dev->activateStream(stream);
    std::vector<std::complex<float>> samples(N);
    std::vector<double> psd(N, 0.0);
    int completed = 0;
    while (completed < frames) {
      int collected = 0;
      while (collected < N) {
        void* buffs[] = {samples.data() + collected};
        int flags = 0; long long timeNs = 0;
        int got = dev->readStream(stream, buffs, N-collected, flags, timeNs, 1000000);
        if (got < 0) throw std::runtime_error("readStream failed: " + std::to_string(got));
        collected += got;
      }
      for (int i=0;i<N;i++) {
        float w = 0.5f - 0.5f*std::cos(2.0*std::acos(-1.0)*i/(N-1));
        in[i][0] = samples[i].real()*w;
        in[i][1] = samples[i].imag()*w;
      }
      fftwf_execute(plan);
      for (int k=0;k<N;k++) {
        int shifted = (k + N/2)%N;
        double re = out[k][0], im = out[k][1];
        psd[shifted] += (re*re+im*im)/(N*N);
      }
      ++completed;
    }
    dev->deactivateStream(stream);
    std::vector<double> db(N);
    for (int k=0;k<N;k++) db[k] = 10*std::log10(psd[k]/completed + 1e-20);
    std::vector<double> floor(N);
    constexpr int radius = 200;
    for (int k=0;k<N;k++) {
      int lo = std::max(0,k-radius), hi=std::min(N,k+radius+1);
      std::vector<double> local(db.begin()+lo, db.begin()+hi);
      auto mid=local.begin()+local.size()/2;
      std::nth_element(local.begin(),mid,local.end());
      floor[k]=*mid;
    }
    std::ofstream csv("spectrum.csv");
    csv << "frequency_hz,relative_db,noise_floor_db,snr_proxy_db\n";
    csv << std::fixed << std::setprecision(3);
    for (int k=0;k<N;k++) {
      double f=center + (k-N/2)*rate/N;
      csv << f << ',' << db[k] << ',' << floor[k] << ',' << db[k]-floor[k] << '\n';
    }
    std::ofstream candidates("candidates.csv");
    candidates << "center_frequency_hz,peak_excess_db,width_hz\n";
    const double threshold=9.0;
    for (int k=1;k<N-1;) {
      if (std::abs(k-N/2)<8 || db[k]-floor[k]<threshold) { ++k; continue; }
      int start=k, peak=k;
      while (k<N-1 && db[k]-floor[k]>=threshold) {
        if (db[k]-floor[k]>db[peak]-floor[peak]) peak=k;
        ++k;
      }
      int end=k;
      if (end-start<2) continue;
      double f=center+(peak-N/2)*rate/N;
      candidates << std::fixed << std::setprecision(1) << f << ','
                 << std::setprecision(2) << db[peak]-floor[peak] << ','
                 << std::setprecision(1) << (end-start)*rate/N << '\n';
    }
    std::cout << "Saved spectrum.csv and candidates.csv (relative levels only)\n";
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << '\n';
    if (dev && stream) { try { dev->deactivateStream(stream); } catch (...) {} }
    if (dev && stream) dev->closeStream(stream);
    if (plan) fftwf_destroy_plan(plan);
    if (in) fftwf_free(in);
    if (out) fftwf_free(out);
    if (dev) SoapySDR::Device::unmake(dev);
    return 1;
  }
  if (dev && stream) dev->closeStream(stream);
  if (plan) fftwf_destroy_plan(plan);
  if (in) fftwf_free(in);
  if (out) fftwf_free(out);
  if (dev) SoapySDR::Device::unmake(dev);
  return 0;
}
